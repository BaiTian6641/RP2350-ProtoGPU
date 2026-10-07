#!/usr/bin/env python3
# Copyright (c) 2026 ProtoGPU contributors
"""pgl_budget_report.py — P02 SRAM/flash accounting ledger.

Builds ONE non-double-counted byte/address ledger per profile from:
  * the linker map (Memory Configuration + every output section's final
    address/size — real final ranges, alignment costs included)
  * the ELF LOAD segments (memsz covers .bss/NOLOAD exactly once)
  * a reservations ledger (scene arena, frame/output buffers, queues,
    transport slots, stacks) whose fixed-address entries are unioned with
    section intervals — overlapping reservations are counted once

Exit 1 if any region is over capacity or the SRAM headroom floor is not
met.  Output is labeled software evidence: static map accounting is not a
runtime high-water measurement, and hardware timing cannot be qualified by
a map file.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pgl_elf import Elf32, ElfError  # noqa: E402
from pgl_mapfile import MapFile, MapError, union_size  # noqa: E402
from pgl_image_package import (PackageError, _load_identity,
                               _load_reservations, _sram_accounting)

SRAM_BASE = 0x20000000
SRAM_END = 0x20082000


def run(args: argparse.Namespace) -> int:
    errors: list[str] = []
    if args.reserve_floor < 0:
        print("pgl_budget_report: ERROR: negative reserve floor", file=sys.stderr)
        return 1
    try:
        elf = Elf32.from_file(args.elf)
    except (OSError, ElfError) as exc:
        print(f"pgl_budget_report: ERROR: {exc}", file=sys.stderr)
        return 1
    try:
        mapfile = MapFile.from_file(args.map)
    except (OSError, MapError) as exc:
        print(f"pgl_budget_report: ERROR: {exc}", file=sys.stderr)
        return 1
    try:
        reservations = _load_reservations(args.reservations)
    except (OSError, json.JSONDecodeError, PackageError, ValueError, TypeError) as exc:
        print(f"pgl_budget_report: ERROR: {exc}", file=sys.stderr)
        return 1
    identity = None
    if args.identity:
        try:
            identity = _load_identity(args.identity)
        except (OSError, json.JSONDecodeError, PackageError) as exc:
            print(f"pgl_budget_report: ERROR: {exc}", file=sys.stderr)
            return 1
        if identity["bootStorage"] != args.boot_storage:
            errors.append("identity boot storage disagrees with requested profile")
    names = {r.name for r in mapfile.regions}
    for res in reservations:
        if "address" in res:
            if not mapfile.region_for(res["address"], res["bytes"]):
                errors.append(f"reservation {res['name']} outside linker regions")
        elif res.get("region") not in names:
            errors.append(f"reservation {res['name']} needs a valid target region")
    for seg in elf.load_segments:
        if seg.memsz and SRAM_BASE <= seg.vaddr < SRAM_END and \
                seg.vaddr + seg.memsz > SRAM_END:
            errors.append("ELF SRAM segment exceeds SRAM end")
    errors.extend(mapfile.check_elf_sram(elf, SRAM_BASE, SRAM_END))

    # ── Region usage from the map (sections unioned per region) ──────
    region_report = []
    for region in mapfile.regions:
        section_intervals = []
        section_rows = []
        for seg in elf.load_segments:
            if seg.memsz and region.contains(seg.vaddr, seg.memsz):
                section_intervals.append((seg.vaddr, seg.vaddr + seg.memsz))
        for sec in mapfile.sections:
            if not sec.size:
                continue
            if region.contains(sec.addr, sec.size):
                section_intervals.append((sec.addr, sec.addr + sec.size))
                section_rows.append({
                    "name": sec.name,
                    "address": f"0x{sec.addr:08x}",
                    "bytes": sec.size,
                })
        fixed_res = []
        sized_res = []
        for res in reservations:
            if "address" in res:
                a0, a1 = res["address"], res["address"] + res["bytes"]
                if region.contains(a0, res["bytes"]):
                    before = union_size(section_intervals)
                    section_intervals.append((a0, a1))
                    added = union_size(section_intervals) - before
                    fixed_res.append({
                        "name": res["name"], "bytes": res["bytes"],
                        "address": f"0x{a0:08x}", "addedBytes": added,
                    })
            elif res.get("region") == region.name:
                sized_res.append({"name": res["name"],
                                  "bytes": res["bytes"]})
        section_bytes = union_size(
            [(s.addr, s.addr + s.size) for s in mapfile.sections
             if s.size and region.contains(s.addr, s.size)])
        union_bytes = union_size(section_intervals)
        sized_bytes = sum(r["bytes"] for r in sized_res)
        used = union_bytes + sized_bytes
        free = region.length - used
        if free < 0:
            errors.append(
                f"region {region.name} over capacity: used {used} > "
                f"{region.length} bytes")
        region_report.append({
            "name": region.name,
            "origin": f"0x{region.origin:08x}",
            "length": region.length,
            "sectionBytes": section_bytes,
            "fixedReservations": fixed_res,
            "sizedReservations": sized_res,
            "usedBytes": used,
            "freeBytes": free,
            "sections": section_rows,
        })

    # ── Cross-check: ELF LOAD memsz vs map sections (SRAM side) ──────
    elf_sram = union_size([(s.vaddr, s.vaddr + s.memsz)
                           for s in elf.load_segments
                           if SRAM_BASE <= s.vaddr < SRAM_END and s.memsz])
    elf_lma = union_size([(s.paddr, s.paddr + s.filesz)
                          for s in elf.load_segments if s.filesz])

    # ELF residency is authoritative; map and reservations are unioned, not
    # added a second time. Scratch stacks at SRAM_END do not consume gaps.
    sram_res = []
    sram_regions = {r.name for r in mapfile.regions
                    if SRAM_BASE <= r.origin and r.end <= SRAM_END}
    for res in reservations:
        if ("address" in res and SRAM_BASE <= res["address"] < SRAM_END) or \
                ("address" not in res and res.get("region") in sram_regions):
            sram_res.append(res)
    upload_bytes = 0
    if args.boot_storage == "RAM_HOST":
        try:
            _, image = elf.load_image()
            upload_bytes = (len(image) + 31) // 32 * 32
        except ElfError as exc:
            errors.append(str(exc))
    sram = _sram_accounting(elf, sram_res, args.reserve_floor, errors, upload_bytes)
    sram_used = sram["usedBytes"]
    sram_capacity = sram["capacityBytes"]
    headroom = sram["headroomBytes"]

    ledger = {
        "schema": 1,
        "kind": "pgl-budget-ledger",
        "profile": {
            "bootStorage": args.boot_storage,
            "buildId": (f"0x{identity['buildId']:08x}"
                        if identity else None),
            "buildConfig": identity.get("buildConfig") if identity else None,
        },
        "regions": region_report,
        "sram": {
            "base": f"0x{SRAM_BASE:08x}",
            "end": f"0x{SRAM_END:08x}",
            "capacityBytes": sram_capacity,
            "usedBytes": sram_used,
            "headroomBytes": headroom,
            "reserveFloorBytes": args.reserve_floor,
            "elfLoadMemszBytes": elf_sram,
        },
        "elfLoadLmaBytes": elf_lma,
        "fits": not errors,
        "evidence": {
            "kind": "software-static-map-accounting",
            "runtimeHighWaterMeasured": False,
            "hardwareTimingQualified": False,
            "note": "linker map/ELF static accounting with reservations; "
                    "not a measured runtime high-water mark",
        },
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
    with open(args.json, "w", encoding="utf-8") as fh:
        json.dump(ledger, fh, indent=2)
        fh.write("\n")
    os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
    with open(args.report, "w", encoding="utf-8") as fh:
        fh.write(_render(ledger))

    for line in errors:
        print(f"pgl_budget_report: REJECT: {line}", file=sys.stderr)
    if errors:
        return 1
    print(f"pgl_budget_report: OK — SRAM used {sram_used}/"
          f"{sram_capacity} B, headroom {headroom} B "
          f"(floor {args.reserve_floor} B)")
    print(f"  json   {args.json}")
    print(f"  report {args.report}")
    return 0


def _render(ledger: dict) -> str:
    out = ["ProtoGPU SRAM/flash budget ledger",
           "evidence: software static map accounting — NOT a measured "
           "runtime high-water; hardware timing NOT qualified by a map",
           "=" * 72]
    profile = ledger["profile"]
    out.append(f"profile {profile['bootStorage']} buildId "
               f"{profile['buildId']} config {profile['buildConfig']}")
    out.append("")
    for region in ledger["regions"]:
        out.append(f"{region['name']:<12} origin {region['origin']} "
                   f"length {region['length']} B  used "
                   f"{region['usedBytes']} B  free {region['freeBytes']} B")
        out.append(f"  sections (union, alignment included): "
                   f"{region['sectionBytes']} B")
        for sec in region["sections"]:
            out.append(f"    {sec['name']:<20} {sec['address']} "
                       f"{sec['bytes']:>8} B")
        for res in region["fixedReservations"]:
            out.append(f"    [reserve] {res['name']:<16} {res['address']} "
                       f"{res['bytes']:>8} B (+{res['addedBytes']} new, "
                       "overlap counted once)")
        for res in region["sizedReservations"]:
            out.append(f"    [reserve] {res['name']:<16} (unplaced)   "
                       f"{res['bytes']:>8} B")
        out.append("")
    sram = ledger["sram"]
    out.append(f"SRAM total     capacity {sram['capacityBytes']} B  used "
               f"{sram['usedBytes']} B  headroom "
               f"{sram['headroomBytes']} B (floor "
               f"{sram['reserveFloorBytes']} B)")
    out.append(f"ELF LOAD memsz (SRAM) {sram['elfLoadMemszBytes']} B; "
               f"ELF LOAD file bytes {ledger['elfLoadLmaBytes']} B")
    out.append(f"verdict        {'FITS' if ledger['fits'] else 'OVER'}")
    out.append("")
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="ProtoGPU P02 budget ledger from linker map + ELF + "
                    "reservations.")
    ap.add_argument("--elf", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--identity",
                    help="generated/pgl_build_identity.json (optional tie)")
    ap.add_argument("--boot-storage", required=True,
                    choices=("RAM_HOST", "FLASH_LOCAL"))
    ap.add_argument("--reservations", help="reservations JSON ledger")
    ap.add_argument("--reserve-floor", type=int, default=32768)
    ap.add_argument("--json", required=True)
    ap.add_argument("--report", required=True)
    args = ap.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
