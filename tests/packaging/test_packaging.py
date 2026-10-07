"""Packaging oracles; real-image cases require both final target build dirs.

PGL_RAM_BUILD=build/runtime-ram PGL_FLASH_BUILD=build/runtime-flash \
    python3 -m unittest discover -s tests/packaging -v

No executable firmware is synthesized. Unit cases exercise metadata structures
and address ledgers; integration cases consume final SDK ELF/bin/map products.
"""
import argparse
import contextlib
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import random
import struct
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from pgl_elf import Elf32, ElfError, Segment
from pgl_image_package import (PackageError, SRAM_BASE, SRAM_END,
                               _load_reservations, _sram_accounting,
                               main as package_main, payload_checksum,
                               validate_manifest)
from pgl_mapfile import MapFile, union_size
from pgl_picobin import ImageDefError, parse_image_def, require_rp2350_arm_exe
from pgl_asset_embed import main as embed_main
from pgl_pair_bundle import main as pair_main
from pgl_budget_report import main as budget_main


class MetadataTests(unittest.TestCase):
    def test_checksum_known_vectors(self):
        self.assertEqual(payload_checksum(b""), 0x811c9dc5)
        self.assertEqual(payload_checksum(b"hello"), 0x4f9f2cab)

    def test_union_matches_independent_byte_coverage(self):
        rng = random.Random(197)
        for _ in range(100):
            ranges = [(rng.randrange(80), rng.randrange(80)) for _ in range(12)]
            covered = set()
            for a, b in ranges:
                covered.update(range(a, b))
            self.assertEqual(union_size(ranges), len(covered))

    def test_sram_scratch_stack_and_bss_count_once(self):
        # Address records only, not an ELF or an executable firmware fixture.
        elf = object.__new__(Elf32)
        segments = []
        for address, size in ((SRAM_BASE, 4096), (SRAM_END - 4096, 4096)):
            seg = Segment()
            seg.vaddr, seg.memsz = address, size
            segments.append(seg)
        elf.load_segments = segments
        errors = []
        result = _sram_accounting(elf, [
            {"name": "already-linked-bss", "address": SRAM_BASE + 32, "bytes": 2048},
            {"name": "additional", "bytes": 1024}], 32768, errors, 64)
        self.assertEqual(errors, [])
        self.assertEqual(result["residentBytes"], 8192)
        self.assertEqual(result["usedBytes"], 9216)
        self.assertEqual(result["headroomBytes"], SRAM_END - SRAM_BASE - 9216)
        self.assertEqual(result["reservations"][0]["addedBytes"], 0)
        self.assertTrue(result["fits"])
        errors = []
        _sram_accounting(elf, [], SRAM_END - SRAM_BASE, errors)
        self.assertTrue(errors)

    def test_wrapped_map_output_section(self):
        text = """Memory Configuration

Name Origin Length Attributes
RAM 0x20000000 0x00080000 xrw

Linker script and memory map
.very_long_output_section
                0x20000020 0x00000080
 .input         0x20000020 0x00000040 obj.o
.text           0x20000100 0x00000020
"""
        parsed = MapFile(text)
        self.assertEqual([(s.name, s.addr, s.size) for s in parsed.sections], [
            (".very_long_output_section", 0x20000020, 128),
            (".text", 0x20000100, 32)])

    def test_reservation_list_and_malformed_types(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "ledger.json"
            path.write_text(json.dumps([{"name": "buffer", "bytes": 128,
                                         "region": "RAM"}]))
            self.assertEqual(_load_reservations(str(path))[0]["bytes"], 128)
            for bad in ([None], [{"name": "bool", "bytes": True}],
                        [{"name": "bad", "bytes": 64, "address": []}]):
                path.write_text(json.dumps(bad))
                with self.assertRaises(PackageError):
                    _load_reservations(str(path))

    @staticmethod
    def block(items, next_offset=0):
        # PICOBIN metadata only: START, items, LAST preceding-item count,
        # signed relative pointer, END. This cannot boot as firmware.
        return struct.pack("<" + "I" * (len(items) + 4),
                           0xffffded3, *items, 0xff | len(items) << 8,
                           next_offset & 0xffffffff, 0xab123579)

    def test_picobin_item_sizes_and_signed_loop(self):
        first = self.block([0x10210142, 0x00000203, SRAM_BASE], 28)
        second = self.block([0x000003fe, 0xaaaaaaaa, 0xbbbbbbbb], -28)
        parsed = parse_image_def(first + second)
        require_rp2350_arm_exe(parsed)
        self.assertEqual(parsed.blocks, 2)
        self.assertEqual(parsed.vector_table, SRAM_BASE)
        self.assertEqual(parsed.size, 28)
        # LAST stores three preceding words, not bytes nor trailer words.
        damaged = bytearray(first)
        struct.pack_into("<I", damaged, 16, 0x000002ff)
        with self.assertRaises(ImageDefError):
            parse_image_def(damaged)
        for item in (0x09, 0x47, 0x4b, 0x0c, 0x06):
            with self.assertRaises(ImageDefError):
                parse_image_def(self.block([0x10210142, 0x100 | item]))
        with self.assertRaises(ImageDefError):
            parse_image_def(self.block([0x10210142], -4))

    def test_picobin_truncation_and_family(self):
        block = self.block([0x10210142])
        for length in range(len(block)):
            with self.assertRaises(ImageDefError):
                parse_image_def(block[:length])
        for image_type in (0x0021, 0x1121, 0x9021, 0x1031):
            with self.assertRaises(ImageDefError):
                require_rp2350_arm_exe(parse_image_def(
                    self.block([image_type << 16 | 0x142])))
        with self.assertRaises(ElfError):
            Elf32(b"not an ELF")


@unittest.skipUnless(os.environ.get("PGL_RAM_BUILD") and os.environ.get("PGL_FLASH_BUILD"),
                     "requires final RAM_HOST and FLASH_LOCAL target builds")
class FinalImageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.builds = {"RAM_HOST": Path(os.environ["PGL_RAM_BUILD"]).resolve(),
                       "FLASH_LOCAL": Path(os.environ["PGL_FLASH_BUILD"]).resolve()}

    def package_args(self, profile, directory, binary=None, identity=None):
        build = self.builds[profile]
        directory.mkdir(parents=True, exist_ok=True)
        return ["--elf", str(build / "protogl_gpu.elf"),
                "--bin", str(binary or build / "protogl_gpu.bin"),
                "--map", str(build / "protogl_gpu.elf.map"),
                "--identity", str(identity or build / "generated/pgl_build_identity.json"),
                "--boot-storage", profile,
                "--manifest", str(directory / "manifest.json"),
                "--manifest-bin", str(directory / "manifest.bin"),
                "--manifest-header", str(directory / "manifest.h"),
                "--report", str(directory / "report.txt")]

    def quiet(self, function, args):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return function(args)

    def packaged(self, profile):
        out = self.out / profile
        self.assertEqual(self.quiet(package_main, self.package_args(profile, out)), 0)
        return out, json.loads((out / "manifest.json").read_text())

    def test_both_final_images_and_wire_schema(self):
        for profile in self.builds:
            with self.subTest(profile=profile):
                out, manifest = self.packaged(profile)
                payload = (self.builds[profile] / "protogl_gpu.bin").read_bytes()
                wire = (out / "manifest.bin").read_bytes()
                self.assertEqual(wire, validate_manifest(manifest, payload))
                budget_json = out / "budget.json"
                budget_args = [
                    "--elf", str(self.builds[profile] / "protogl_gpu.elf"),
                    "--map", str(self.builds[profile] / "protogl_gpu.elf.map"),
                    "--identity", str(self.builds[profile] / "generated/pgl_build_identity.json"),
                    "--boot-storage", profile, "--json", str(budget_json),
                    "--report", str(out / "budget.txt")]
                self.assertEqual(self.quiet(budget_main, budget_args), 0)
                budget = json.loads(budget_json.read_text())
                self.assertEqual(budget["sram"]["usedBytes"], manifest["sram"]["usedBytes"])
                self.assertEqual(budget["sram"]["headroomBytes"],
                                 SRAM_END - SRAM_BASE - budget["sram"]["usedBytes"])
                fields = struct.unpack("<IHHIIIIII", wire)
                self.assertEqual(fields[:3], (0x494c4750, 1, 32))
                self.assertEqual(fields[3], len(payload))
                self.assertEqual(fields[4], (len(payload) + 31) & ~31)
                self.assertEqual(fields[6], 9)
                self.assertEqual(fields[7], manifest["build"]["buildId"])
                self.assertEqual(fields[8], 1 if profile == "RAM_HOST" else 0)
                self.assertEqual(hashlib.sha256(payload).hexdigest(), manifest["image"]["sha256"])
                for path, value in ((["image", "sha256"], "0" * 64),
                                    (["image", "paddedBytes"], len(payload) + 32),
                                    (["image", "base"], "0x30000000"),
                                    (["build", "buildId"], 0),
                                    (["build", "buildIdCompiled"], "0x00000000"),
                                    (["protocol", "major"], 8),
                                    (["host", "manifestFlags"], "0x00000080")):
                    damaged = copy.deepcopy(manifest)
                    damaged[path[0]][path[1]] = value
                    with self.assertRaises(PackageError):
                        validate_manifest(damaged, payload)

    def test_corruption_and_identity_profile_reject_without_output(self):
        for profile, build in self.builds.items():
            payload = bytearray((build / "protogl_gpu.bin").read_bytes())
            payload[len(payload) // 2] ^= 1
            corrupted = self.out / (profile + "-bad.bin")
            corrupted.write_bytes(payload)
            out = self.out / (profile + "-corrupt")
            self.assertEqual(self.quiet(package_main, self.package_args(profile, out, binary=corrupted)), 1)
            self.assertEqual(list(out.iterdir()), [])
            identity = json.loads((build / "generated/pgl_build_identity.json").read_text())
            identity["bootStorage"] = "FLASH_LOCAL" if profile == "RAM_HOST" else "RAM_HOST"
            bad_identity = self.out / (profile + "-identity.json")
            bad_identity.write_text(json.dumps(identity))
            out = self.out / (profile + "-profile")
            self.assertEqual(self.quiet(package_main, self.package_args(profile, out, identity=bad_identity)), 1)
            self.assertEqual(list(out.iterdir()), [])

    def test_real_elf_compiled_id_and_vector_corruption(self):
        for profile, build in self.builds.items():
            original = Elf32.from_file(str(build / "protogl_gpu.elf"))
            mutations = [
                (original.symbols["pgl_build_id"].value,
                 struct.unpack("<I", original.read_vaddr(
                     original.symbols["pgl_build_id"].value, 4))[0] ^ 1),
                (original.symbols["__vectors"].value + 4, 0x30000001)]
            for index, (address, value) in enumerate(mutations):
                data = bytearray(original.data)
                segment = next(s for s in original.load_segments
                               if s.file_contains_vaddr(address, 4))
                struct.pack_into("<I", data, segment.offset + address - segment.vaddr, value)
                corrupted_elf = self.out / f"{profile}-{index}.elf"
                corrupted_bin = self.out / f"{profile}-{index}.bin"
                corrupted_elf.write_bytes(data)
                corrupted_bin.write_bytes(Elf32(bytes(data)).load_image()[1])
                out = self.out / f"{profile}-{index}-rejected"
                args = self.package_args(profile, out, binary=corrupted_bin)
                args[args.index("--elf") + 1] = str(corrupted_elf)
                self.assertEqual(self.quiet(package_main, args), 1)
                self.assertEqual(list(out.iterdir()), [])

    def test_embed_rejects_every_wire_field_corruption_and_flash_profile(self):
        out, _ = self.packaged("RAM_HOST")
        build = self.builds["RAM_HOST"]
        args = ["image", "--bin", str(build / "protogl_gpu.bin"),
                "--manifest", str(out / "manifest.json"), "--manifest-bin", str(out / "bad.bin"),
                "--name", "Firmware", "--out-dir", str(out / "embed")]
        wire = (out / "manifest.bin").read_bytes()
        for index in range(32):
            damaged = bytearray(wire)
            damaged[index] ^= 1
            (out / "bad.bin").write_bytes(damaged)
            self.assertEqual(self.quiet(embed_main, args), 1)
            self.assertFalse((out / "embed").exists())
        flash_out, manifest = self.packaged("FLASH_LOCAL")
        payload = (self.builds["FLASH_LOCAL"] / "protogl_gpu.bin").read_bytes()
        with self.assertRaises(PackageError):
            validate_manifest(manifest, payload, ram_only=True)

    def test_dual_profile_pair_and_artifact_hash_rejection(self):
        # Use actual release manifests beside their recorded artifacts.
        manifests = []
        artifacts = []
        for profile, build in self.builds.items():
            manifests.append(str(build / "pgl_image_manifest.json"))
            artifacts.extend(str(build / name) for name in
                             ("protogl_gpu.bin", "protogl_gpu.elf", "protogl_gpu.elf.map"))
        args = ["--repo", str(ROOT), "--firmware-manifest", *manifests,
                "--firmware-artifacts", *artifacts, "--out-dir", str(self.out / "release")]
        self.assertEqual(self.quiet(pair_main, args), 0)
        # Removing any required recorded artifact must fail before bundle creation.
        missing = artifacts[1:]
        args = ["--repo", str(ROOT), "--firmware-manifest", *manifests,
                "--firmware-artifacts", *missing, "--out-dir", str(self.out / "rejected")]
        self.assertEqual(self.quiet(pair_main, args), 1)
        self.assertFalse((self.out / "rejected").exists())


if __name__ == "__main__":
    unittest.main()
