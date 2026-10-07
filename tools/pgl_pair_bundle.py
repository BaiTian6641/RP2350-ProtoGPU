#!/usr/bin/env python3
# Copyright (c) 2026 ProtoGPU contributors
"""pgl_pair_bundle.py — P12 compatible host/RP release bundle.

Pairs one or both *packaged* RAM_HOST/FLASH_LOCAL firmware images with the
exact ProtoGL host source tree. ELF, binary, map and host/lock hashes must
match; the compiled build ID and RP2350 metadata are checked independently.
Incompatible inputs are rejected before the release directory is created.

Bundle layout:
  <out>/manifest.json          pairing record (firmware + host + pins)
  <out>/firmware/<profile>/    .bin/.elf/.map + exact image manifests
  <out>/firmware/<profile>/reproduce.sh  pinned rebuild/repackage commands
  <out>/host/                  paired ProtoGL headers + Arduino metadata/examples
  <out>/source/                exact inputs, tools, replay/checks/docs and lock

No custom OTA, no OTP programming, no private-signing policy: installation
remains the ordinary SDK USB/SWD path (FLASH_LOCAL) or the documented ROM
UART bootstrap (RAM_HOST).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import shlex
import struct

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pgl_build_identity import (EXCLUDE_DIR_NAMES, IdentityError, _excluded,
                                _hash_file_list, collect_source_files)
from pgl_image_package import (PackageError, SRAM_BASE, SRAM_END, XIP_BASE,
                               _check_no_overlap, _check_vectors, validate_manifest)
from pgl_elf import Elf32, ElfError
from pgl_picobin import ImageDefError, parse_image_def, require_rp2350_arm_exe
from pgl_mapfile import MapFile, MapError


class BundleError(Exception):
    pass


def _hash_host_tree(repo: str, host_dir: str) -> tuple[str, int]:
    """Hash the host (ProtoGL) working tree with the same walker rules as
    the firmware identity, restricted to host_dir."""
    root = os.path.join(repo, host_dir)
    if not os.path.isdir(root):
        raise BundleError(f"host source directory missing: {host_dir}")
    rel_files = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(
            d for d in dirnames if d not in EXCLUDE_DIR_NAMES)
        for name in sorted(filenames):
            full = os.path.join(dirpath, name)
            rel = os.path.relpath(full, repo).replace(os.sep, "/")
            if _excluded(rel, name):
                continue
            rel_files.append(rel)
    digest, _ = _hash_file_list(repo, rel_files)
    return digest, len(rel_files)


def _sha256_file(path: str) -> str:
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def run(args: argparse.Namespace) -> int:
    repo = os.path.abspath(args.repo)

    manifests = args.firmware_manifest
    if isinstance(manifests, str):
        manifests = [manifests]
    records = []
    profiles = set()
    paths = {os.path.abspath(p): p for p in args.firmware_artifacts}
    with open(os.path.join(repo, args.lock), "rb") as fh:
        lock_bytes = fh.read()
    lock = json.loads(lock_bytes)
    protocol = lock.get("protocol_major")
    if protocol != 9:
        raise BundleError("release lock must explicitly pin protocol_major 9")
    for manifest_path in manifests:
        with open(manifest_path, encoding="utf-8") as fh:
            fw = json.load(fh)
        profile = fw["profile"]["bootStorage"]
        if profile in profiles:
            raise BundleError(f"duplicate release profile {profile}")
        profiles.add(profile)
        root = os.path.dirname(os.path.abspath(manifest_path))
        artifacts = []
        selected = []
        for file_key, hash_key in (("file", "sha256"),
                                   ("elfFile", "elfSha256"),
                                   ("mapFile", "mapSha256")):
            name = fw["image"][file_key]
            if not isinstance(name, str) or os.path.basename(name) != name:
                raise BundleError("artifact name must be a basename")
            absolute = os.path.join(root, name)
            if absolute not in paths:
                raise BundleError(f"required artifact not supplied: {absolute}")
            digest = _sha256_file(absolute)
            if digest != fw["image"][hash_key]:
                raise BundleError(f"artifact hash mismatch: {absolute}")
            selected.append(absolute)
            artifacts.append({"file": name, "sha256": digest,
                              "bytes": os.path.getsize(absolute)})
        with open(selected[0], "rb") as fh:
            payload = fh.read()
        wire = validate_manifest(fw, payload, protocol)
        elf = Elf32.from_file(selected[1])
        base, image = elf.load_image()
        if image != payload or base != int(fw["image"]["base"], 16):
            raise BundleError("final ELF load image differs from release binary")
        metadata = parse_image_def(image)
        require_rp2350_arm_exe(metadata)
        vectors = elf.symbols.get("__vectors")
        if vectors is None or (profile == "RAM_HOST" and vectors.value != SRAM_BASE):
            raise BundleError("unproven/misplaced vector table")
        errors = []
        errors.extend(MapFile.from_file(selected[2]).check_elf_sram(
            elf, SRAM_BASE, SRAM_END))
        _check_no_overlap(elf, errors)
        _check_vectors(elf, image, base, vectors.value, errors)
        if metadata.vector_table is not None and metadata.vector_table != vectors.value:
            errors.append("IMAGE_DEF vector table differs from ELF")
        if profile == "RAM_HOST" and metadata.vector_table is None:
            errors.append("RAM_HOST IMAGE_DEF lacks vector table")
        flash_end = XIP_BASE + (fw["flash"]["sizeBytes"] if fw["flash"] else 0)
        for seg in elf.load_segments:
            if profile == "RAM_HOST":
                if seg.memsz and not SRAM_BASE <= seg.vaddr < seg.vaddr + seg.memsz <= SRAM_END:
                    errors.append("RAM_HOST residency outside SRAM")
                if seg.filesz and not SRAM_BASE <= seg.paddr < seg.paddr + seg.filesz <= SRAM_END:
                    errors.append("RAM_HOST load range outside SRAM")
            else:
                if seg.filesz and not XIP_BASE <= seg.paddr < seg.paddr + seg.filesz <= flash_end:
                    errors.append("FLASH_LOCAL load range outside NOR")
                if seg.memsz and not (
                        SRAM_BASE <= seg.vaddr < seg.vaddr + seg.memsz <= SRAM_END or
                        XIP_BASE <= seg.vaddr < seg.vaddr + seg.memsz <= flash_end and
                        seg.vaddr == seg.paddr):
                    errors.append("FLASH_LOCAL residency outside NOR/SRAM")
        if errors:
            raise BundleError("; ".join(errors))
        sym = elf.symbols.get("pgl_build_id")
        if sym is None or sym.size != 4 or struct.unpack(
                "<I", elf.read_vaddr(sym.value, 4))[0] != fw["build"]["buildId"]:
            raise BundleError("final ELF compiled build ID mismatch")
        records.append((manifest_path, fw, wire, selected, artifacts))
    supplied_wire = args.manifest_bin or []
    if isinstance(supplied_wire, str):
        supplied_wire = [supplied_wire]
    unmatched = list(supplied_wire)
    for _, _, wire, _, _ in records:
        for path in list(unmatched):
            with open(path, "rb") as fh:
                if fh.read() == wire:
                    unmatched.remove(path)
                    break
        else:
            if supplied_wire:
                raise BundleError("missing/mismatched binary manifest for profile")
    if unmatched:
        raise BundleError("unexpected binary manifest supplied")
    if len(paths) != len({p for _, _, _, selected, _ in records for p in selected}):
        raise BundleError("unrecorded artifacts refused")
    host_sha256, host_files = _hash_host_tree(repo, args.host_source)
    source_sha256, compiled_inputs = _hash_file_list(repo, collect_source_files(repo))
    for _, fw, _, _, _ in records:
        if fw["build"].get("hostSourceSha256") != host_sha256:
            raise BundleError("host sources differ from firmware identity")
        if fw["build"]["dependencySha256"] != hashlib.sha256(lock_bytes).hexdigest():
            raise BundleError("dependency lock differs from firmware identity")
        if fw["build"]["sourceSha256"] != source_sha256:
            raise BundleError("firmware/allocator sources differ from image identity")
    sdk = lock.get("sources", {}).get("pico_sdk", {})
    sdk_path = sdk.get("path", "third_party/pico-sdk")
    sdk_commit = sdk.get("commit")
    toolchain = lock.get("toolchains", {})
    scripts = {fw["profile"]["bootStorage"]: _render_reproduce(
        fw["profile"], sdk_path, sdk_commit, toolchain) for _, fw, _, _, _ in records}
    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=False)
    host_target = os.path.join(out_dir, "host")
    shutil.copytree(
        os.path.join(repo, args.host_source), os.path.join(host_target, "src"),
        ignore=lambda directory, names: [
            name for name in names if name in EXCLUDE_DIR_NAMES or _excluded(
                os.path.relpath(os.path.join(directory, name), repo).replace(os.sep, "/"),
                name)])
    # Use the standard Arduino library layout, with compiler sources separately
    # identified from consumer metadata/examples.
    host_root = os.path.dirname(os.path.join(repo, args.host_source))
    for name in ("library.properties", "library.json", "README.md"):
        shutil.copy2(os.path.join(host_root, name), os.path.join(host_target, name))
    for name in ("examples", "docs", "shaders"):
        shutil.copytree(os.path.join(host_root, name), os.path.join(host_target, name),
                        ignore=shutil.ignore_patterns("build", "__pycache__", "*.o", "*.d"))
    source_target = os.path.join(out_dir, "source")
    snapshot_inputs = list(compiled_inputs) + [args.lock]
    snapshot_inputs += [
        "tools/" + name for name in sorted(os.listdir(os.path.join(repo, "tools")))
        if name.endswith(".py")]
    for relative_root in ("sim", "tests", "docs", "ProtoGL/examples", "ProtoGL/tests", "ProtoGL/docs", "ProtoGL/shaders"):
        for directory, directories, filenames in os.walk(os.path.join(repo, relative_root)):
            directories[:] = sorted(name for name in directories if name not in EXCLUDE_DIR_NAMES)
            for name in sorted(filenames):
                relative = os.path.relpath(os.path.join(directory, name), repo).replace(os.sep, "/")
                if not _excluded(relative, name):
                    snapshot_inputs.append(relative)
    snapshot_inputs += ["README.md", "CHANGELOG.md", "ProtoGL/README.md",
                        "ProtoGL/library.properties", "ProtoGL/library.json"]
    _, snapshot_hashes = _hash_file_list(repo, snapshot_inputs)
    for relative in sorted(snapshot_hashes):
        destination = os.path.join(source_target, relative)
        os.makedirs(os.path.dirname(destination), exist_ok=True)
        shutil.copy2(os.path.join(repo, relative), destination)
    shutil.copy2(os.path.join(repo, args.lock), os.path.join(out_dir, "dependencies.lock.json"))
    firmware = []
    for manifest_path, fw, wire, selected, artifacts in records:
        profile = fw["profile"]
        fw_dir = os.path.join(out_dir, "firmware", profile["bootStorage"])
        os.makedirs(fw_dir)
        for path in selected:
            shutil.copy2(path, fw_dir)
        shutil.copy2(manifest_path, os.path.join(fw_dir, "pgl_image_manifest.json"))
        with open(os.path.join(fw_dir, "pgl_image_manifest.bin"), "wb") as fh:
            fh.write(wire)
        firmware.append({"profile": profile, "build": fw["build"],
                         "image": fw["image"], "artifacts": artifacts})
        script = os.path.join(fw_dir, "reproduce.sh")
        with open(script, "w", encoding="utf-8") as fh:
            fh.write(scripts[profile["bootStorage"]])
        os.chmod(script, 0o755)
    pairing = {
        "schema": 1, "kind": "pgl-pair-bundle",
        "protocol": {"major": protocol}, "firmware": firmware,
        "host": {"sourceDir": args.host_source, "bundledDir": "host",
                 "headerDir": "src", "workingTreeSha256": host_sha256, "fileCount": host_files,
                 "baseCommit": lock.get("sources", {}).get("ProtoGL", {}).get("base_commit")},
        "source": {"bundledDir": "source", "compiledSourceSha256": source_sha256,
                   "compiledInputs": compiled_inputs, "snapshotFiles": snapshot_hashes,
                   "sdkAcquisition": f"Clone {sdk.get('remote')} into {sdk_path} at {sdk_commit}; SDK is not duplicated in this bundle."},
        "pins": {"picoSdk": sdk, "toolchains": toolchain,
                 "lockSha256": hashlib.sha256(lock_bytes).hexdigest()},
        "installation": {
            "RAM_HOST": "PglBootLoader::LoadRamImage ROM UART bootstrap",
            "FLASH_LOCAL": "ordinary Pico SDK USB/SWD; no custom OTA/OTP"},
        "evidence": {"kind": "software-static-analysis",
                     "hardwareTimingQualified": False,
                     "hardwareExecutionQualified": False},
    }
    with open(os.path.join(out_dir, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump(pairing, fh, indent=2)
        fh.write("\n")
    print(f"pgl_pair_bundle: OK protocol {protocol} profiles {sorted(profiles)}")
    print(f"  host tree sha256 {host_sha256} ({host_files} files)")
    print(f"  bundle {out_dir}")
    return 0


def _render_reproduce(profile: dict, sdk_path: str, sdk_commit,
                      toolchain: dict) -> str:
    boot = profile["bootStorage"]
    display = profile["display"]
    psram = "ON" if profile["psram"] else "OFF"
    diagnostic = "ON" if profile["diagnostic"] else "OFF"
    build_dir = f"build/{boot.lower()}"
    if boot not in ("RAM_HOST", "FLASH_LOCAL") or \
            display not in ("SPI", "HUB", "LED", "CUSTOM", "NONE") or \
            type(profile["psram"]) is not bool or type(profile["diagnostic"]) is not bool:
        raise BundleError("invalid reproduction profile")
    if not sdk_commit or not toolchain.get("arm_gcc"):
        raise BundleError("missing SDK/toolchain pins")
    return f"""\
#!/usr/bin/env bash
# Reproduce from this bundle's source/ snapshot or an exact source checkout.
# Pinned inputs (dependencies.lock.json):
#   pico-sdk   {sdk_path} @ {sdk_commit}
#   arm gcc    {toolchain.get('arm_gcc')} at build/toolchain/usr/bin
#   profile    BOOT_STORAGE={boot} DEFAULT_DISPLAY={display} PSRAM={psram}
#   board BSP  pico2 (RP2350), protocol {9}
# Expected result: profile {profile.get('buildConfig')}, matching compiled
# source/build identity and freshly validated image/budget manifests.
set -euo pipefail

cd "${{1:?usage: reproduce.sh /path/to/bundle/source-or-exact-checkout}}"
REPO_ROOT="$PWD"

# Exact tool path/environment: pinned local toolchain BEFORE any system gcc.
export PATH="$REPO_ROOT/build/toolchain/usr/bin:$PATH"
export PICO_SDK_PATH="$REPO_ROOT"/{shlex.quote(sdk_path)}
unset PICO_PLATFORM PICO_BOARD_HEADER_DIRS  # no ambient environment leakage

test "$(git -C "$PICO_SDK_PATH" rev-parse HEAD)" = {shlex.quote(sdk_commit)}
arm-none-eabi-gcc --version
test "$(arm-none-eabi-gcc -dumpfullversion)" = {shlex.quote(toolchain['arm_gcc'])}

cmake -S . -B {build_dir} \\
    -DPICO_SDK_PATH="$PICO_SDK_PATH" \\
    -DPICO_BOARD=pico2 \\
    -DBOOT_STORAGE={boot} \\
    -DDEFAULT_DISPLAY={display} \\
    -DPSRAM={psram} \\
    -DHOSTLESS_DEMO={diagnostic} \\
    -DCMAKE_BUILD_TYPE=Release

cmake --build {build_dir} --parallel

# Packaging runs as POST_BUILD steps (pgl_image_package.py,
# pgl_budget_report.py).  To re-run validation explicitly:
python3 tools/pgl_image_package.py \\
    --elf {build_dir}/protogl_gpu.elf \\
    --bin {build_dir}/protogl_gpu.bin \\
    --map {build_dir}/protogl_gpu.elf.map \\
    --identity {build_dir}/generated/pgl_build_identity.json \\
    --boot-storage {boot} \\
    --manifest {build_dir}/pgl_image_manifest.json \\
    --manifest-bin {build_dir}/pgl_image_manifest.bin \\
    --manifest-header {build_dir}/generated/pgl_image_manifest.h \\
    --report {build_dir}/pgl_image_report.txt

python3 tools/pgl_budget_report.py \\
    --elf {build_dir}/protogl_gpu.elf \\
    --map {build_dir}/protogl_gpu.elf.map \\
    --identity {build_dir}/generated/pgl_build_identity.json \\
    --boot-storage {boot} \\
    --json {build_dir}/pgl_budget.json \\
    --report {build_dir}/pgl_budget.txt

echo "Compare profile/build/source identity with the bundled manifest; hashes identify the rebuilt artifacts."
"""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Pair a packaged firmware image with host sources "
                    "into a release bundle.")
    ap.add_argument("--firmware-manifest", nargs="+", required=True,
                    help="one or both RAM_HOST/FLASH_LOCAL packaged manifests")
    ap.add_argument("--firmware-artifacts", nargs="+", required=True)
    ap.add_argument("--manifest-bin", nargs="+",
                    help="optional exact binary manifests for all profiles")
    ap.add_argument("--host-source", default="ProtoGL/src")
    ap.add_argument("--lock", default="dependencies.lock.json")
    ap.add_argument("--repo", default=".")
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args(argv)
    try:
        return run(args)
    except (BundleError, PackageError, ImageDefError, MapError, ElfError, OSError,
            json.JSONDecodeError, KeyError, TypeError, ValueError,
            IdentityError) as exc:
        print(f"pgl_pair_bundle: ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
