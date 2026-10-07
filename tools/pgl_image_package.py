#!/usr/bin/env python3
# Copyright (c) 2026 ProtoGPU contributors
"""pgl_image_package.py — validate and package the FINAL firmware image.

Consumes the real SDK post-build products (post-picotool ELF, objcopy flat
.bin, linker .map) — never a raw ELF/UF2 substitute — and:

  * proves the .bin is exactly the ELF load image (holes zero-filled)
  * proves RP2350 ARM EXE identity from the genuine, closed IMAGE_DEF
    (PICOBIN block markers/item types/sizes, not source wording)
  * proves vectors: ELF __vectors at a 512-aligned initialized table,
    valid SRAM SP and Thumb reset PC inside initialized executable bytes
  * proves profile placement: RAM_HOST everything inside
    [0x20000000, 0x20082000) with padded-32 upload bounds; FLASH_LOCAL
    normal NOR XIP addresses with SRAM resident data (never a fake RAM
    manifest)
  * proves the compiled pgl_build_id symbol equals the build identity
  * accounts SRAM residency (BSS/stacks included once, NOLOAD never
    double-counted) and padded upload/reservations against a total-free
    SRAM reserve floor (not a runtime high-water or contiguous-heap claim)
  * hashes the actual binary (SHA-256) and emits the exact 32-byte
    PglImageManifest (PglBootLoader.h) the host validates before upload

Exit 0 = image packaged; exit 1 = every violation listed, nothing emitted
as "valid".  This is software static evidence: hardware timing/execution
qualification is explicitly out of scope and labeled as such.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pgl_elf import Elf32, ElfError  # noqa: E402
from pgl_mapfile import MapFile, MapError, merged, union_size  # noqa: E402
from pgl_picobin import (ImageDefError, parse_image_def,  # noqa: E402
                         require_rp2350_arm_exe)
from pgl_build_identity import _derive_build_id  # noqa: E402

SRAM_BASE = 0x20000000
SRAM_END = 0x20082000          # 520 KiB: 512 KiB main + 2x4 KiB scratch
XIP_BASE = 0x10000000
VECTOR_ALIGN = 512
PAD_MULTIPLE = 32
IMAGE_DEF_WINDOW = 4096

# Exact host-side schema (ProtoGL/src/PglBootLoader.h).
MANIFEST_MAGIC = 0x494C4750  # "PGLI"
MANIFEST_VERSION = 1
MANIFEST_HEADER_BYTES = 32
IMAGE_FLAG_RAM_HOST = 0x01

BOOT_STORAGE_IDS = {"RAM_HOST": 1, "FLASH_LOCAL": 2}


class PackageError(Exception):
    pass


def payload_checksum(data: bytes) -> int:
    """PglRuntime::PayloadChecksum — FNV-1a 32 (corruption precheck only)."""
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def _pad(length: int, multiple: int = PAD_MULTIPLE) -> int:
    return (length + multiple - 1) // multiple * multiple


def validate_manifest(manifest: dict, payload: bytes,
                      protocol: int = 9, ram_only: bool = False) -> bytes:
    """Validate JSON metadata against bytes and return the host wire record."""
    try:
        if manifest["kind"] != "pgl-image-manifest" or manifest["schema"] != 1:
            raise PackageError("unsupported image manifest schema")
        image, build, host = manifest["image"], manifest["build"], manifest["host"]
        profile = manifest["profile"]["bootStorage"]
        if profile not in BOOT_STORAGE_IDS or (ram_only and profile != "RAM_HOST"):
            raise PackageError("image profile is not supported by this consumer")
        if manifest["profile"]["bootStorageId"] != BOOT_STORAGE_IDS[profile]:
            raise PackageError("manifest boot storage ID/profile mismatch")
        flags = IMAGE_FLAG_RAM_HOST if profile == "RAM_HOST" else 0
        if not payload or image["bytes"] != len(payload) or \
                image["paddedBytes"] != _pad(len(payload)):
            raise PackageError("image length/padded length mismatch")
        if image["sha256"] != hashlib.sha256(payload).hexdigest():
            raise PackageError("image SHA-256 mismatch")
        checksum = payload_checksum(payload)
        if int(image["payloadChecksum"], 16) != checksum:
            raise PackageError("image payload checksum mismatch")
        build_id = build["buildId"]
        if type(build_id) is not int or not 0 < build_id <= 0xFFFFFFFF or \
                build["buildIdHex"] != f"{build_id:08x}" or \
                build["buildIdCompiled"] != f"0x{build_id:08x}":
            raise PackageError("manifest compiled build ID mismatch")
        if manifest["protocol"]["major"] != protocol or \
                manifest["protocol"]["minCompatibleMajor"] != protocol:
            raise PackageError("manifest protocol mismatch")
        if int(host["manifestMagic"], 16) != MANIFEST_MAGIC or \
                host["manifestVersion"] != MANIFEST_VERSION or \
                host["manifestHeaderBytes"] != MANIFEST_HEADER_BYTES or \
                int(host["manifestFlags"], 16) != flags:
            raise PackageError("manifest host schema/flags mismatch")
        base = int(image["base"], 16)
        if int(image["top"], 16) != base + len(payload):
            raise PackageError("image range/length mismatch")
        if profile == "RAM_HOST":
            if base != SRAM_BASE or base + image["paddedBytes"] > SRAM_END:
                raise PackageError("RAM_HOST padded image outside SRAM")
        else:
            size = manifest["flash"]["sizeBytes"]
            if type(size) is not int or not 0 < size <= 16 * 1024 * 1024 or \
                    base < XIP_BASE or base + image["paddedBytes"] > XIP_BASE + size:
                raise PackageError("FLASH_LOCAL padded image outside NOR")
        if manifest["security"]["signedImage"] or not \
                manifest["security"]["integrityOnly"] or \
                manifest["imageDef"]["signed"]:
            raise PackageError("unsupported image authentication claim")
        return struct.pack("<IHHIIIIII", MANIFEST_MAGIC, MANIFEST_VERSION,
                           MANIFEST_HEADER_BYTES, len(payload), _pad(len(payload)),
                           checksum, protocol, build_id, flags)
    except (KeyError, TypeError, ValueError, struct.error) as exc:
        raise PackageError(f"malformed image manifest: {exc}") from exc


def _load_identity(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as fh:
        identity = json.load(fh)
    fields = ("buildId", "buildIdHex", "identitySha256", "protocol", "board",
              "bootStorage", "bootStorageId", "sourceSha256",
              "dependencySha256", "toolchain", "display", "psram", "diagnostic")
    if not isinstance(identity, dict) or any(f not in identity for f in fields):
        raise PackageError(f"identity {path}: incomplete identity record")
    if any(not isinstance(identity[f], str) or not identity[f] for f in (
            "board", "bootStorage", "toolchain", "display", "sourceSha256",
            "dependencySha256", "identitySha256")) or type(identity["psram"]) is not bool or type(identity["diagnostic"]) is not bool:
        raise PackageError("identity field types are invalid")
    if type(identity["buildId"]) is not int or not \
            0 < identity["buildId"] <= 0xFFFFFFFF:
        raise PackageError("identity buildId must be nonzero uint32")
    if identity["protocol"] != 9 or identity["bootStorage"] not in BOOT_STORAGE_IDS:
        raise PackageError("identity protocol/profile is unsupported")
    if identity["bootStorageId"] != BOOT_STORAGE_IDS[identity["bootStorage"]]:
        raise PackageError("identity bootStorageId disagrees with profile")
    if identity["buildIdHex"] != f"{identity['buildId']:08x}":
        raise PackageError("identity buildIdHex disagrees with buildId")
    material = {key: identity[key] for key in (
        "sourceSha256", "dependencySha256", "protocol", "board",
        "bootStorage", "display", "psram", "diagnostic", "toolchain")}
    material["schema"] = 1
    digest = hashlib.sha256(json.dumps(
        material, sort_keys=True, separators=(",", ":")).encode()).digest()
    if digest.hex() != identity["identitySha256"] or \
            _derive_build_id(digest) != identity["buildId"]:
        raise PackageError("identity digest/buildId does not match its inputs")
    return identity


def _load_reservations(path: str | None) -> list[dict]:
    if not path:
        return []
    with open(path, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    reservations = doc.get("reservations") if isinstance(doc, dict) else doc
    if not isinstance(reservations, list):
        raise PackageError(f"reservations {path}: expected a list")
    out = []
    seen = set()
    for entry in reservations:
        if not isinstance(entry, dict):
            raise PackageError("reservation must be an object")
        name = entry.get("name")
        size = entry.get("bytes")
        if not isinstance(name, str) or not name or type(size) is not int or size <= 0:
            raise PackageError(
                f"reservations {path}: entry needs name and positive bytes")
        if name in seen:
            raise PackageError(f"duplicate reservation name {name}")
        seen.add(name)
        item = {"name": name, "bytes": size}
        if "region" in entry:
            if not isinstance(entry["region"], str) or not entry["region"]:
                raise PackageError("reservation region must be a name")
            item["region"] = entry["region"]
        if "address" in entry:
            addr = entry["address"]
            if not isinstance(addr, str) and type(addr) is not int:
                raise PackageError("reservation address must be integer/hex")
            try:
                item["address"] = int(addr, 16) if isinstance(addr, str) else addr
            except ValueError as exc:
                raise PackageError("invalid reservation address") from exc
            if item["address"] % 4:
                raise PackageError(
                    f"reservation {name}: address not word aligned")
        out.append(item)
    return out


def _check_vectors(elf: Elf32, image: bytes, base: int, vector_addr: int,
                   errors: list[str]) -> dict:
    """Validate initial SP / Thumb reset PC at *vector_addr*."""
    info: dict = {"address": f"0x{vector_addr:08x}"}
    if vector_addr % VECTOR_ALIGN:
        errors.append(
            f"vector table 0x{vector_addr:08x} is not {VECTOR_ALIGN}-byte "
            "aligned")
    if vector_addr < base or vector_addr + 8 > base + len(image):
        errors.append(
            f"vector table 0x{vector_addr:08x} outside load image "
            f"[0x{base:08x}, 0x{base + len(image):08x})")
        return info
    off = vector_addr - base
    sp, pc = struct.unpack_from("<II", image, off)
    info["initialSp"] = f"0x{sp:08x}"
    info["resetHandler"] = f"0x{pc:08x}"
    if sp % 8:
        errors.append(f"initial SP 0x{sp:08x} is not 8-byte aligned")
    if not (SRAM_BASE < sp <= SRAM_END):
        errors.append(
            f"initial SP 0x{sp:08x} outside SRAM "
            f"(0x{SRAM_BASE:08x}, 0x{SRAM_END:08x}]")
    if not (pc & 1):
        errors.append(
            f"reset PC 0x{pc:08x} lacks the Thumb bit (not ARM Thumb code)")
    else:
        target = pc & ~1
        if not any(seg.file_contains_vaddr(target, 2)
                   for seg in elf.exec_segments()):
            errors.append(
                f"reset PC 0x{pc:08x} (0x{target:08x}) is not inside any "
                "executable LOAD segment")
    return info


def _check_no_overlap(elf: Elf32, errors: list[str]) -> None:
    for label, key, size in (("virtual", "vaddr", "memsz"),
                             ("load", "paddr", "filesz")):
        spans = sorted((getattr(s, key),
                        getattr(s, key) + getattr(s, size), s.index)
                       for s in elf.load_segments if getattr(s, size))
        for (a0, a1, ia), (b0, b1, ib) in zip(spans, spans[1:]):
            if b0 < a1:
                errors.append(
                    f"LOAD segments {ia} and {ib} overlap in {label} "
                    f"addresses (0x{a0:08x}-0x{a1:08x} vs "
                    f"0x{b0:08x}-0x{b1:08x})")


def _sram_accounting(elf: Elf32, reservations: list[dict],
                     floor: int, errors: list[str], upload_bytes: int = 0) -> dict:
    """Union of SRAM-resident LOAD memsz (bss/stacks counted exactly once)
    plus fixed reservations; size-only reservations add bytes."""
    intervals = [(s.vaddr, s.vaddr + s.memsz)
                 for s in elf.load_segments
                 if s.memsz and s.vaddr < SRAM_END and
                 s.vaddr + s.memsz > SRAM_BASE]
    resident = union_size(intervals)
    if any(a < SRAM_BASE or b > SRAM_END for a, b in intervals):
        errors.append("SRAM LOAD residency extends outside SRAM")
    if upload_bytes:
        intervals.append((SRAM_BASE, SRAM_BASE + upload_bytes))
    live_top = max((e for _, e in merged(intervals)), default=SRAM_BASE)
    counted = []
    extra = 0
    for res in reservations:
        entry = {"name": res["name"], "bytes": res["bytes"]}
        if "address" in res:
            a0, a1 = res["address"], res["address"] + res["bytes"]
            if a0 < SRAM_BASE or a1 > SRAM_END:
                errors.append(
                    f"reservation {res['name']} "
                    f"[0x{a0:08x}, 0x{a1:08x}) outside SRAM")
                continue
            entry["address"] = f"0x{a0:08x}"
            before = union_size(intervals)
            intervals.append((a0, a1))
            after = union_size(intervals)
            entry["addedBytes"] = after - before  # overlaps counted once
            live_top = max(live_top, a1)
        else:
            entry["addedBytes"] = res["bytes"]
            extra += res["bytes"]
        counted.append(entry)
    used = union_size(intervals) + extra
    headroom = SRAM_END - SRAM_BASE - used
    fits = headroom >= floor and not errors
    if headroom < floor:
        errors.append(
            f"SRAM headroom {headroom} bytes below reserve floor {floor} "
            f"(live top 0x{live_top:08x}, sized reservations {extra} bytes)")
    return {
        "base": f"0x{SRAM_BASE:08x}",
        "end": f"0x{SRAM_END:08x}",
        "capacityBytes": SRAM_END - SRAM_BASE,
        "residentBytes": resident,
        "liveTop": f"0x{live_top:08x}",
        "reservations": counted,
        "usedBytes": used,
        "headroomBytes": headroom,
        "reserveFloorBytes": floor,
        "fits": fits,
    }


def package(args: argparse.Namespace) -> int:
    errors: list[str] = []
    boot_storage = args.boot_storage
    flash_end = XIP_BASE + args.flash_size
    if args.reserve_floor < 0 or not 0 < args.flash_size <= 16 * 1024 * 1024:
        print("pgl_image_package: ERROR: invalid floor/flash size", file=sys.stderr)
        return 1

    # ── Load inputs ───────────────────────────────────────────────────
    try:
        identity = _load_identity(args.identity)
    except (OSError, json.JSONDecodeError, PackageError) as exc:
        print(f"pgl_image_package: ERROR: {exc}", file=sys.stderr)
        return 1
    if identity["bootStorage"] != boot_storage:
        print(f"pgl_image_package: ERROR: identity profile "
              f"{identity['bootStorage']} != --boot-storage {boot_storage}",
              file=sys.stderr)
        return 1
    try:
        reservations = _load_reservations(args.reservations)
    except (OSError, json.JSONDecodeError, PackageError) as exc:
        print(f"pgl_image_package: ERROR: {exc}", file=sys.stderr)
        return 1
    try:
        elf = Elf32.from_file(args.elf)
    except (OSError, ElfError) as exc:
        print(f"pgl_image_package: ERROR: {exc}", file=sys.stderr)
        return 1
    try:
        with open(args.bin, "rb") as fh:
            bin_bytes = fh.read()
    except OSError as exc:
        print(f"pgl_image_package: ERROR: {exc}", file=sys.stderr)
        return 1
    if not bin_bytes:
        print("pgl_image_package: ERROR: empty .bin", file=sys.stderr)
        return 1
    try:
        mapfile = MapFile.from_file(args.map) if args.map else None
    except (OSError, MapError) as exc:
        print(f"pgl_image_package: ERROR: {exc}", file=sys.stderr)
        return 1
    if mapfile:
        errors.extend(mapfile.check_elf_sram(elf, SRAM_BASE, SRAM_END))
        sram_regions = {r.name for r in mapfile.regions
                        if SRAM_BASE <= r.origin and r.end <= SRAM_END}
        region_names = {r.name for r in mapfile.regions}
        filtered = []
        for res in reservations:
            if "address" in res:
                if not mapfile.region_for(res["address"], res["bytes"]):
                    errors.append(f"reservation {res['name']} outside linker regions")
                if SRAM_BASE <= res["address"] < SRAM_END:
                    filtered.append(res)
            else:
                if res.get("region") not in region_names:
                    errors.append(f"reservation {res['name']} needs a valid region")
                if res.get("region") in sram_regions:
                    filtered.append(res)
        reservations = filtered

    # ── bin must be the actual ELF load image (post-picotool objcopy) ─
    try:
        base, reconstructed = elf.load_image()
    except ElfError as exc:
        print(f"pgl_image_package: ERROR: {exc}", file=sys.stderr)
        return 1
    if reconstructed != bin_bytes:
        errors.append(
            f".bin ({len(bin_bytes)} bytes, sha256 "
            f"{hashlib.sha256(bin_bytes).hexdigest()[:16]}…) is not the "
            f"exact load image of the ELF ({len(reconstructed)} bytes); "
            "package the final post-picotool objcopy output, not a raw or "
            "stale binary")

    # ── Segment placement per profile ─────────────────────────────────
    _check_no_overlap(elf, errors)
    if boot_storage == "RAM_HOST":
        if base != SRAM_BASE:
            errors.append(
                f"RAM_HOST image base 0x{base:08x} != SRAM base "
                f"0x{SRAM_BASE:08x} (wrong binary type? need no_flash)")
        for seg in elf.load_segments:
            for label, addr, span in (("VMA", seg.vaddr, seg.memsz),
                                      ("LMA", seg.paddr, seg.filesz)):
                if span and (addr < SRAM_BASE or addr + span > SRAM_END):
                    errors.append(
                        f"RAM_HOST LOAD segment {seg.index} {label} "
                        f"[0x{addr:08x}, 0x{addr + span:08x}) outside SRAM "
                        f"[0x{SRAM_BASE:08x}, 0x{SRAM_END:08x})")
        vector_addr = SRAM_BASE
    else:  # FLASH_LOCAL
        if not (XIP_BASE <= base < flash_end):
            errors.append(
                f"FLASH_LOCAL image base 0x{base:08x} outside NOR XIP "
                f"[0x{XIP_BASE:08x}, 0x{flash_end:08x})")
        xip_vaddrs = []
        for seg in elf.load_segments:
            if seg.filesz and not (XIP_BASE <= seg.paddr and
                                   seg.paddr + seg.filesz <= flash_end):
                errors.append(
                    f"FLASH_LOCAL LOAD segment {seg.index} LMA "
                    f"[0x{seg.paddr:08x}, 0x{seg.paddr + seg.filesz:08x}) "
                    "outside NOR flash window")
            if XIP_BASE <= seg.vaddr < flash_end:
                if seg.vaddr + seg.memsz > flash_end:
                    errors.append("FLASH_LOCAL XIP segment exceeds NOR window")
                if seg.vaddr != seg.paddr:
                    errors.append(
                        f"FLASH_LOCAL segment {seg.index} XIP VMA "
                        f"0x{seg.vaddr:08x} != LMA 0x{seg.paddr:08x}")
                xip_vaddrs.append(seg.vaddr)
            elif SRAM_BASE <= seg.vaddr and seg.vaddr + seg.memsz <= SRAM_END:
                pass  # SRAM-resident initialized data with flash LMA
            else:
                errors.append(
                    f"FLASH_LOCAL segment {seg.index} VMA "
                    f"[0x{seg.vaddr:08x}, 0x{seg.vaddr + seg.memsz:08x}) "
                    "is neither XIP nor SRAM")
        if not xip_vaddrs:
            errors.append("FLASH_LOCAL image has no XIP-resident segment")
            vector_addr = base
        else:
            vector_addr = min(xip_vaddrs)
        if base + _pad(len(bin_bytes)) > flash_end:
            errors.append(
                f"FLASH_LOCAL padded image 0x{base:08x}+"
                f"{_pad(len(bin_bytes))} exceeds flash window end "
                f"0x{flash_end:08x}")

    # ── Vectors ───────────────────────────────────────────────────────
    vector_symbol = elf.symbols.get("__vectors")
    if vector_symbol is None:
        errors.append("ELF lacks __vectors; vector table location is unproven")
    else:
        vector_addr = vector_symbol.value
        try:
            actual_vectors = elf.read_vaddr(vector_addr, 8)
            if actual_vectors != reconstructed[vector_addr - base:vector_addr - base + 8]:
                errors.append("vector table VMA does not map to the load image")
        except ElfError as exc:
            errors.append(f"vector table unreadable: {exc}")
    if boot_storage == "RAM_HOST" and vector_addr != SRAM_BASE:
        errors.append("RAM_HOST vectors must begin at SRAM base")
    vectors = _check_vectors(elf, reconstructed, base, vector_addr, errors)

    # ── IMAGE_DEF (genuine closed PICOBIN metadata) ───────────────────
    image_def: dict = {}
    try:
        idef = parse_image_def(reconstructed, IMAGE_DEF_WINDOW)
        require_rp2350_arm_exe(idef)
        if idef.vector_table is not None and \
                idef.vector_table != vector_addr:
            errors.append(
                f"IMAGE_DEF VECTOR_TABLE 0x{idef.vector_table:08x} != "
                f"actual vector base 0x{vector_addr:08x}")
        if boot_storage == "RAM_HOST" and idef.vector_table is None:
            errors.append(
                "RAM_HOST IMAGE_DEF lacks a VECTOR_TABLE item (SDK "
                "no_flash images always carry one)")
        if idef.entry_point is not None:
            ep = idef.entry_point
            if not (ep & 1) or not any(
                    s.file_contains_vaddr(ep & ~1, 2) for s in elf.exec_segments()):
                errors.append(
                    f"IMAGE_DEF ENTRY_POINT 0x{ep:08x} outside executable "
                    "segments")
            if idef.entry_sp is not None and not \
                    (SRAM_BASE < idef.entry_sp <= SRAM_END):
                errors.append(
                    f"IMAGE_DEF entry SP 0x{idef.entry_sp:08x} outside "
                    "SRAM")
            if idef.entry_sp is not None and idef.entry_sp % 8:
                errors.append("IMAGE_DEF entry SP is not 8-byte aligned")
        t = idef.image_type
        image_def = {
            "offset": idef.offset,
            "size": idef.size,
            "blocks": idef.blocks,
            "type": t.describe(),
            "security": {0: "unspecified", 1: "non-secure",
                         2: "secure"}[t.security],
            "vectorTable": (f"0x{idef.vector_table:08x}"
                            if idef.vector_table is not None else None),
            "entryPoint": (f"0x{idef.entry_point:08x}"
                           if idef.entry_point is not None else None),
            "signed": False,
        }
    except ImageDefError as exc:
        errors.append(f"IMAGE_DEF invalid: {exc}")

    # ── Compiled build identity tie ───────────────────────────────────
    build_id_compiled = None
    sym = elf.symbols.get("pgl_build_id")
    if sym is None:
        errors.append(
            "ELF has no pgl_build_id symbol; compile `const uint32_t "
            "pgl_build_id = PGL_FIRMWARE_BUILD_ID;` so the identity "
            "determines the image-compiled value")
    else:
        if sym.size != 4 or sym.value % 4 or (sym.info & 15) != 1:
            errors.append("pgl_build_id is not an aligned uint32 object symbol")
        try:
            build_id_compiled = struct.unpack(
                "<I", elf.read_vaddr(sym.value, 4))[0]
            if build_id_compiled != identity["buildId"]:
                errors.append(
                    f"compiled pgl_build_id 0x{build_id_compiled:08x} != "
                    f"identity buildId 0x{identity['buildId']:08x} "
                    "(stale generated header or mismatched build)")
        except ElfError as exc:
            errors.append(f"pgl_build_id unreadable: {exc}")

    # ── Upload bound + SRAM accounting ────────────────────────────────
    padded = _pad(len(bin_bytes))
    if boot_storage == "RAM_HOST" and SRAM_BASE + padded > SRAM_END:
        errors.append(
            f"RAM_HOST padded upload {padded} bytes overruns SRAM: "
            f"0x{SRAM_BASE:08x}+{padded} > 0x{SRAM_END:08x}")
    sram = _sram_accounting(elf, reservations, args.reserve_floor, errors,
                            padded if boot_storage == "RAM_HOST" else 0)

    sha256 = hashlib.sha256(bin_bytes).hexdigest()
    checksum = payload_checksum(bin_bytes)

    # ── Emit or reject ────────────────────────────────────────────────
    if errors:
        for line in errors:
            print(f"pgl_image_package: REJECT: {line}", file=sys.stderr)
        return 1

    flags = IMAGE_FLAG_RAM_HOST if boot_storage == "RAM_HOST" else 0
    manifest_bin = struct.pack(
        "<IHHIIIIII",
        MANIFEST_MAGIC, MANIFEST_VERSION, MANIFEST_HEADER_BYTES,
        len(bin_bytes), padded, checksum,
        identity["protocol"], identity["buildId"], flags)
    assert len(manifest_bin) == MANIFEST_HEADER_BYTES

    with open(args.elf, "rb") as fh:
        elf_sha256 = hashlib.sha256(fh.read()).hexdigest()
    map_sha256 = None
    if args.map:
        with open(args.map, "rb") as fh:
            map_sha256 = hashlib.sha256(fh.read()).hexdigest()

    segments = [{
        "index": s.index,
        "vma": f"0x{s.vaddr:08x}",
        "lma": f"0x{s.paddr:08x}",
        "fileBytes": s.filesz,
        "memBytes": s.memsz,
        "flags": ("R" if s.flags & 4 else "") + ("W" if s.flags & 2 else "")
                 + ("X" if s.flags & 1 else ""),
    } for s in elf.load_segments]

    manifest = {
        "schema": 1,
        "kind": "pgl-image-manifest",
        "profile": {
            "bootStorage": boot_storage,
            "bootStorageId": BOOT_STORAGE_IDS[boot_storage],
            "board": identity["board"],
            "display": identity.get("display"),
            "psram": identity.get("psram"),
            "diagnostic": identity["diagnostic"],
            "buildConfig": identity.get("buildConfig"),
        },
        "protocol": {
            "major": identity["protocol"],
            "minCompatibleMajor": identity["protocol"],
        },
        "build": {
            "buildId": identity["buildId"],
            "buildIdHex": identity["buildIdHex"],
            "buildIdCompiled": (f"0x{build_id_compiled:08x}"
                                if build_id_compiled is not None else None),
            "sourceSha256": identity["sourceSha256"],
            "hostSourceSha256": identity.get("hostSourceSha256"),
            "dependencySha256": identity["dependencySha256"],
            "toolchain": identity["toolchain"],
            "sdk": identity.get("dependencies", {}).get("picoSdk"),
        },
        "image": {
            "file": os.path.basename(args.bin),
            "bytes": len(bin_bytes),
            "paddedBytes": padded,
            "padMultiple": PAD_MULTIPLE,
            "sha256": sha256,
            "payloadChecksum": f"0x{checksum:08x}",
            "base": f"0x{base:08x}",
            "top": f"0x{base + len(bin_bytes):08x}",
            "elfFile": os.path.basename(args.elf),
            "elfSha256": elf_sha256,
            "elfEntryPoint": f"0x{elf.entry:08x}",
            "mapFile": os.path.basename(args.map) if args.map else None,
            "mapSha256": map_sha256,
            "loadSegments": segments,
        },
        "imageDef": image_def,
        "vectors": vectors,
        "sram": sram,
        "flash": ({
            "xipBase": f"0x{XIP_BASE:08x}",
            "sizeBytes": args.flash_size,
            "imageBytes": len(bin_bytes),
            "note": "configured NOR size; actual part geometry is a board "
                    "property, not inferred here",
        } if boot_storage == "FLASH_LOCAL" else None),
        "host": {
            "manifestMagic": f"0x{MANIFEST_MAGIC:08x}",
            "manifestVersion": MANIFEST_VERSION,
            "manifestHeaderBytes": MANIFEST_HEADER_BYTES,
            "manifestFlags": f"0x{flags:08x}",
        },
        "clock": {
            "defaultProfile": 0,
            "mhz": 150,
            "note": "software default profile; hardware timing is NOT "
                    "qualified by this packaging",
        },
        "security": {
            "integrityOnly": True,
            "secureBoot": False,
            "otpProgrammed": False,
            "signedImage": False,
            "note": "content hash is integrity, not authentication; no "
                    "secure-boot/OTP claim",
        },
        "evidence": {
            "kind": "software-static-analysis",
            "hardwareTimingQualified": False,
            "hardwareExecutionQualified": False,
        },
    }

    _write(args.manifest, json.dumps(manifest, indent=2) + "\n")
    _write_bytes(args.manifest_bin, manifest_bin)
    _write(args.manifest_header, _manifest_header(
        manifest_bin, identity, len(bin_bytes), padded, sha256, checksum,
        flags))
    _write(args.report, _report(manifest, mapfile))
    print(f"pgl_image_package: OK {boot_storage} image "
          f"{len(bin_bytes)} bytes (padded {padded}) base "
          f"0x{base:08x} sha256 {sha256[:16]}… buildId "
          f"0x{identity['buildId']:08x}")
    print(f"  SRAM resident {sram['residentBytes']} B, used "
          f"{sram['usedBytes']} B, headroom {sram['headroomBytes']} B "
          f"(floor {sram['reserveFloorBytes']} B)")
    print(f"  manifest {args.manifest}")
    print(f"  manifest.bin {args.manifest_bin} (32B PglImageManifest)")
    print(f"  report {args.report}")
    return 0


def _write(path: str, text: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)


def _write_bytes(path: str, data: bytes) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "wb") as fh:
        fh.write(data)


def _manifest_header(manifest_bin: bytes, identity: dict, image_bytes: int,
                     padded: int, sha256: str, checksum: int,
                     flags: int) -> str:
    array = ", ".join(f"0x{b:02x}" for b in manifest_bin)
    lines = [
        "// Generated by tools/pgl_image_package.py — exact 32-byte",
        "// PglImageManifest (ProtoGL/src/PglBootLoader.h) for this image.",
        "#pragma once",
        "#include <stdint.h>",
        "",
        f"#define PGL_IMAGE_BYTES          {image_bytes}u",
        f"#define PGL_IMAGE_PADDED_BYTES   {padded}u",
        f"#define PGL_IMAGE_SHA256         \"{sha256}\"",
        f"#define PGL_IMAGE_CHECKSUM       UINT32_C(0x{checksum:08x})",
        f"#define PGL_IMAGE_FLAGS          UINT32_C(0x{flags:08x})",
        f"#define PGL_IMAGE_ABI_VERSION    {identity['protocol']}u",
        f"#define PGL_IMAGE_BUILD_ID       "
        f"UINT32_C(0x{identity['buildId']:08x})",
        "",
        "// memcpy into PglImageManifest or cast after alignment check.",
        "static const uint8_t PGL_IMAGE_MANIFEST_BYTES[32] = {",
        f"    {array}",
        "};",
        "",
    ]
    return "\n".join(lines)


def _report(manifest: dict, mapfile) -> str:
    img = manifest["image"]
    sram = manifest["sram"]
    out = []
    out.append("ProtoGPU image packaging report (software static evidence)")
    out.append("=" * 60)
    out.append(f"profile        {manifest['profile']['bootStorage']} "
               f"board {manifest['profile']['board']} "
               f"display {manifest['profile']['display']}")
    out.append(f"buildId        {manifest['build']['buildIdHex']} "
               f"(compiled {manifest['build']['buildIdCompiled']})")
    out.append(f"source sha256  {manifest['build']['sourceSha256']}")
    out.append(f"dependency     {manifest['build']['dependencySha256']}")
    out.append(f"toolchain      {manifest['build']['toolchain']}")
    sdk = manifest["build"].get("sdk") or {}
    out.append(f"pico-sdk       {sdk.get('version')} @ {sdk.get('commit')}")
    out.append("")
    out.append(f"image          {img['file']}: {img['bytes']} bytes, "
               f"padded {img['paddedBytes']} (multiple "
               f"{img['padMultiple']})")
    out.append(f"  sha256       {img['sha256']}")
    out.append(f"  range        [{img['base']}, {img['top']})")
    out.append(f"  elf entry    {img['elfEntryPoint']} "
               "(ROM/debug trampoline; vectors are authoritative)")
    out.append("  LOAD segments (final ranges):")
    for seg in img["loadSegments"]:
        out.append(f"    #{seg['index']} VMA {seg['vma']} LMA {seg['lma']} "
                   f"file {seg['fileBytes']:>7} B mem {seg['memBytes']:>7} B "
                   f"{seg['flags']}")
    idef = manifest["imageDef"]
    out.append(f"IMAGE_DEF      offset {idef['offset']} size {idef['size']} "
               f"{idef['type']}")
    out.append(f"  vector table {idef['vectorTable']} "
               f"entry item {idef['entryPoint']}")
    vec = manifest["vectors"]
    out.append(f"vectors        @{vec['address']} SP {vec['initialSp']} "
               f"reset {vec['resetHandler']} (Thumb)")
    out.append("")
    out.append(f"SRAM           [{sram['base']}, {sram['end']}) capacity "
               f"{sram['capacityBytes']} B")
    out.append(f"  resident     {sram['residentBytes']} B (ELF LOAD memsz "
               "union; BSS/NOLOAD counted once)")
    for res in sram["reservations"]:
        addr = f" @{res['address']}" if "address" in res else ""
        out.append(f"  reserve      {res['name']}: {res['bytes']} B{addr} "
                   f"(+{res['addedBytes']} B new)")
    out.append(f"  used total   {sram['usedBytes']} B")
    out.append(f"  headroom     {sram['headroomBytes']} B "
               f"(reserve floor {sram['reserveFloorBytes']} B) — "
               f"{'FITS' if sram['fits'] else 'OVER'}")
    if mapfile is not None:
        out.append("")
        out.append("linker memory regions (map cross-reference):")
        for region in mapfile.regions:
            used = union_size(
                [(s.addr, s.addr + s.size) for s in mapfile.sections
                 if s.size and region.contains(s.addr, s.size)])
            out.append(f"  {region.name:<12} "
                       f"[0x{region.origin:08x}, 0x{region.end:08x}) "
                       f"used {used} / {region.length} B")
    out.append("")
    out.append("security       integrity-only hash; no OTP, no secure-boot "
               "or signing claim")
    out.append("clock          default profile 0 (150 MHz) — software "
               "default, hardware timing NOT qualified")
    out.append("evidence       software-static-analysis; no hardware "
               "execution/timing qualification")
    out.append("")
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Validate and package the final ProtoGPU firmware "
                    "image.")
    ap.add_argument("--elf", required=True)
    ap.add_argument("--bin", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--identity", required=True,
                    help="generated/pgl_build_identity.json")
    ap.add_argument("--boot-storage", required=True,
                    choices=sorted(BOOT_STORAGE_IDS))
    ap.add_argument("--reservations",
                    help="JSON reservations ledger (counted once vs ELF)")
    ap.add_argument("--reserve-floor", type=int, default=32768,
                    help="minimum free SRAM bytes after residency + "
                         "reservations (default 32768)")
    ap.add_argument("--flash-size", type=int, default=4 * 1024 * 1024,
                    help="FLASH_LOCAL NOR window in bytes (default 4 MiB)")
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--manifest-bin", required=True)
    ap.add_argument("--manifest-header", required=True)
    ap.add_argument("--report", required=True)
    args = ap.parse_args(argv)
    return package(args)


if __name__ == "__main__":
    sys.exit(main())
