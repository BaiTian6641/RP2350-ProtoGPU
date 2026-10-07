# Copyright (c) 2026 ProtoGPU contributors
# GNU ld map-file parser ("Memory Configuration" + top-level output section
# placement).  Used by the packaging/budget tools for real final ranges.
# Standard library only; strict about the records it consumes.

from __future__ import annotations

import re


class MapError(Exception):
    pass


class MemoryRegion:
    __slots__ = ("name", "origin", "length", "attrs")

    @property
    def end(self) -> int:
        return self.origin + self.length

    def contains(self, addr: int, size: int = 1) -> bool:
        return self.origin <= addr and addr + size <= self.end


class OutputSection:
    __slots__ = ("name", "addr", "size")


_MEM_LINE = re.compile(
    r"^(\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s*(\S*)\s*$")
_SEC_LINE = re.compile(
    r"^(\.[\w.]*)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)(?:\s|$)")


class MapFile:
    def __init__(self, text: str, source: str = "<map>"):
        self.source = source
        self.regions: list[MemoryRegion] = []
        self.sections: list[OutputSection] = []
        self._parse(text)

    @classmethod
    def from_file(cls, path: str) -> "MapFile":
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            return cls(fh.read(), path)

    def _parse(self, text: str) -> None:
        lines = text.splitlines()
        i = 0
        # ── Memory Configuration block ────────────────────────────────
        while i < len(lines) and lines[i].strip() != "Memory Configuration":
            i += 1
        if i == len(lines):
            raise MapError(
                f"{self.source}: no 'Memory Configuration' block (not a "
                "GNU ld map with a MEMORY command?)")
        i += 1
        # skip blank + header line
        while i < len(lines) and not _MEM_LINE.match(lines[i]):
            if lines[i].strip().startswith("Linker script"):
                raise MapError(
                    f"{self.source}: empty Memory Configuration block")
            i += 1
        while i < len(lines):
            m = _MEM_LINE.match(lines[i])
            if not m:
                break
            name, origin, length, attrs = m.groups()
            if name != "*default*":
                region = MemoryRegion()
                region.name = name
                region.origin = int(origin, 16)
                region.length = int(length, 16)
                region.attrs = attrs
                self.regions.append(region)
            i += 1
        if not self.regions:
            raise MapError(
                f"{self.source}: Memory Configuration lists no regions")

        # ── Top-level output sections ─────────────────────────────────
        pending = None
        for line in lines[i:]:
            if pending is not None:
                match = re.match(
                    r"^\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)(?:\s|$)", line)
                if match:
                    sec = OutputSection()
                    sec.name = pending
                    sec.addr, sec.size = (int(v, 16) for v in match.groups())
                    self.sections.append(sec)
                pending = None
            if line[:1] in (" ", "\t"):
                continue
            m = _SEC_LINE.match(line)
            if m:
                name, addr, size = m.groups()
                sec = OutputSection()
                sec.name = name
                sec.addr = int(addr, 16)
                sec.size = int(size, 16)
                self.sections.append(sec)
            elif re.fullmatch(r"\.[\w.]+", line):
                pending = line
        if not self.sections:
            raise MapError(f"{self.source}: no output sections found")

    def region_for(self, addr: int, size: int) -> MemoryRegion | None:
        for region in self.regions:
            if region.contains(addr, max(size, 1)):
                return region
        return None

    def check_elf_sram(self, elf, base: int, end: int) -> list[str]:
        """Compare linked SRAM output sections, not input-section details.

        Post-picotool flash metadata may change; SRAM section placement may
        not. A stale map must not stand in for the final residency ledger.
        """
        expected = {(s.name, s.addr, s.size) for s in elf.sections
                    if s.flags & 2 and s.size and base <= s.addr < end}
        recorded = {(s.name, s.addr, s.size) for s in self.sections
                    if s.size and base <= s.addr < end}
        if expected != recorded:
            return ["SRAM output sections in linker map differ from final ELF"]
        return []

    def section_intervals(self) -> list[tuple[int, int, str]]:
        """(start, end, name) for every nonzero-size output section."""
        return [(s.addr, s.addr + s.size, s.name)
                for s in self.sections if s.size]


def union_size(intervals: list[tuple[int, int]]) -> int:
    """Total bytes covered by half-open intervals, overlaps counted once."""
    total = 0
    cur_start = cur_end = None
    for start, end in sorted(intervals):
        if end <= start:
            continue
        if cur_start is None:
            cur_start, cur_end = start, end
        elif start <= cur_end:
            cur_end = max(cur_end, end)
        else:
            total += cur_end - cur_start
            cur_start, cur_end = start, end
    if cur_start is not None:
        total += cur_end - cur_start
    return total


def merged(intervals: list[tuple[int, int]]) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    for start, end in sorted(intervals):
        if end <= start:
            continue
        if out and start <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], end))
        else:
            out.append((start, end))
    return out
