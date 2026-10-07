# Copyright (c) 2026 ProtoGPU contributors
# Minimal, strict ELF32 little-endian parser for RP2350 (EM_ARM) firmware
# images.  Standard library only; used by the packaging and budget tools.
#
# This is a validation parser, not a general ELF toolkit: every structure
# read is bounds-checked so truncated or malformed files are rejected with
# a precise error instead of producing a plausible-looking manifest.

from __future__ import annotations

import struct

ELF_MAGIC = b"\x7fELF"
ELFCLASS32 = 1
ELFDATA2LSB = 1
ET_EXEC = 2
EM_ARM = 40
EV_CURRENT = 1

PT_LOAD = 1
PF_X = 0x1
PF_W = 0x2
PF_R = 0x4

SHT_SYMTAB = 2
SHT_STRTAB = 3
SHT_NOBITS = 8


class ElfError(Exception):
    """Raised for any malformed, truncated or unexpected ELF content."""


def _need(buf: bytes, off: int, size: int, what: str) -> None:
    if off < 0 or size < 0 or off + size > len(buf):
        raise ElfError(
            f"truncated ELF: {what} at offset {off} needs {size} bytes, "
            f"file has {len(buf)}"
        )


class Segment:
    __slots__ = ("index", "type", "offset", "vaddr", "paddr", "filesz",
                 "memsz", "flags", "align")

    def contains_vaddr(self, addr: int, length: int = 1) -> bool:
        return self.vaddr <= addr and addr + length <= self.vaddr + self.memsz

    def file_contains_vaddr(self, addr: int, length: int = 1) -> bool:
        return self.vaddr <= addr and addr + length <= self.vaddr + self.filesz

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return (f"Segment(#{self.index} type={self.type} off=0x{self.offset:x} "
                f"vaddr=0x{self.vaddr:08x} paddr=0x{self.paddr:08x} "
                f"filesz=0x{self.filesz:x} memsz=0x{self.memsz:x} "
                f"flags={self.flags})")


class Section:
    __slots__ = ("index", "name", "type", "flags", "addr", "offset", "size",
                 "link", "entsize")


class Symbol:
    __slots__ = ("name", "value", "size", "info", "shndx")

    def __repr__(self) -> str:  # pragma: no cover
        return f"Symbol({self.name} value=0x{self.value:08x} size={self.size})"


class Elf32:
    """Parsed 32-bit little-endian ARM executable."""

    def __init__(self, data: bytes, source: str = "<bytes>"):
        self.data = data
        self.source = source
        self._parse_header()
        self._parse_segments()
        self._parse_sections()
        self._parse_symbols()

    @classmethod
    def from_file(cls, path: str) -> "Elf32":
        with open(path, "rb") as fh:
            data = fh.read()
        return cls(data, path)

    def _parse_header(self) -> None:
        d = self.data
        _need(d, 0, 52, "ELF header")
        if d[0:4] != ELF_MAGIC:
            raise ElfError(f"{self.source}: not an ELF file (bad magic)")
        if d[4] != ELFCLASS32:
            raise ElfError(f"{self.source}: not a 32-bit ELF (class {d[4]})")
        if d[5] != ELFDATA2LSB:
            raise ElfError(f"{self.source}: not little-endian (data {d[5]})")
        if d[6] != EV_CURRENT:
            raise ElfError(f"{self.source}: bad ident version {d[6]}")
        (self.e_type, self.e_machine, self.e_version, self.entry, self.phoff,
         self.shoff, self.flags, self.ehsize, self.phentsize, self.phnum,
         self.shentsize, self.shnum, self.shstrndx) = struct.unpack_from(
            "<HHIIIIIHHHHHH", d, 16)
        if self.e_type != ET_EXEC:
            raise ElfError(
                f"{self.source}: ELF type {self.e_type}, expected ET_EXEC "
                "(a fully linked firmware image)")
        if self.e_machine != EM_ARM:
            raise ElfError(
                f"{self.source}: ELF machine {self.e_machine}, expected "
                f"EM_ARM (40); wrong target family")
        if self.e_version != EV_CURRENT:
            raise ElfError(f"{self.source}: bad ELF version {self.e_version}")
        if self.ehsize < 52:
            raise ElfError(f"{self.source}: bad ELF header size {self.ehsize}")

    def _parse_segments(self) -> None:
        d = self.data
        phnum = self.phnum
        if phnum == 0xFFFF:  # PN_XNUM: real count in section header 0
            if self.shentsize < 40:
                raise ElfError("invalid extended program-header count")
            _need(d, self.shoff, 40, "section header 0")
            phnum = struct.unpack_from("<I", d, self.shoff + 28)[0]
        if phnum == 0:
            raise ElfError(f"{self.source}: no program headers")
        if self.phentsize < 32:
            raise ElfError(
                f"{self.source}: bad program header entry size "
                f"{self.phentsize}")
        self.segments = []
        for i in range(phnum):
            off = self.phoff + i * self.phentsize
            _need(d, off, 32, f"program header {i}")
            (p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags,
             p_align) = struct.unpack_from("<IIIIIIII", d, off)
            seg = Segment()
            seg.index, seg.type, seg.offset = i, p_type, p_offset
            seg.vaddr, seg.paddr = p_vaddr, p_paddr
            seg.filesz, seg.memsz = p_filesz, p_memsz
            seg.flags, seg.align = p_flags, p_align
            if p_type == PT_LOAD:
                if p_filesz > p_memsz:
                    raise ElfError(
                        f"{self.source}: LOAD segment {i} filesz "
                        f"0x{p_filesz:x} > memsz 0x{p_memsz:x}")
                if p_vaddr + p_memsz > 0x100000000 or \
                        p_paddr + p_filesz > 0x100000000:
                    raise ElfError(f"{self.source}: LOAD address wraps u32")
                if p_align not in (0, 1) and (p_align & (p_align - 1)):
                    raise ElfError(f"{self.source}: invalid LOAD alignment")
                if p_filesz:
                    _need(d, p_offset, p_filesz, f"LOAD segment {i} data")
            self.segments.append(seg)
        self.load_segments = [s for s in self.segments if s.type == PT_LOAD]
        if not self.load_segments:
            raise ElfError(f"{self.source}: no PT_LOAD segments")

    def _parse_sections(self) -> None:
        d = self.data
        self.sections = []
        if self.shoff == 0 or self.shnum == 0:
            return
        if self.shentsize < 40:
            raise ElfError(
                f"{self.source}: bad section header entry size "
                f"{self.shentsize}")
        _need(d, self.shoff, self.shentsize * self.shnum, "section headers")
        raw = []
        shstr_off = None
        for i in range(self.shnum):
            off = self.shoff + i * self.shentsize
            (name, s_type, s_flags, s_addr, s_offset, s_size, s_link, _info,
             _align, s_entsize) = struct.unpack_from("<IIIIIIIIII", d, off)
            raw.append((name, s_type, s_flags, s_addr, s_offset, s_size,
                        s_link, s_entsize))
            if i == self.shstrndx and s_type == SHT_STRTAB:
                shstr_off = (s_offset, s_size)
        strtab = b""
        if shstr_off is not None:
            _need(d, shstr_off[0], shstr_off[1], "section string table")
            strtab = d[shstr_off[0]:shstr_off[0] + shstr_off[1]]
        for i, (name, s_type, s_flags, s_addr, s_offset, s_size, s_link,
                s_entsize) in enumerate(raw):
            sec = Section()
            sec.index = i
            sec.name = _read_cstr(strtab, name) if strtab else ""
            sec.type, sec.flags = s_type, s_flags
            sec.addr, sec.offset, sec.size = s_addr, s_offset, s_size
            sec.link, sec.entsize = s_link, s_entsize
            if s_type != SHT_NOBITS and s_size:
                _need(d, s_offset, s_size, f"section {sec.name or i}")
            self.sections.append(sec)

    def _parse_symbols(self) -> None:
        self.symbols = {}
        d = self.data
        for sec in self.sections:
            if sec.type != SHT_SYMTAB:
                continue
            if sec.entsize < 16:
                raise ElfError(
                    f"{self.source}: bad symbol entry size {sec.entsize}")
            if sec.link >= len(self.sections):
                raise ElfError(
                    f"{self.source}: symtab string link {sec.link} out of "
                    "range")
            strsec = self.sections[sec.link]
            if strsec.type != SHT_STRTAB or sec.size % sec.entsize:
                raise ElfError("invalid symbol/string table layout")
            _need(d, strsec.offset, strsec.size, "symbol string table")
            strtab = d[strsec.offset:strsec.offset + strsec.size]
            count = sec.size // sec.entsize
            for i in range(count):
                off = sec.offset + i * sec.entsize
                _need(d, off, 16, f"symbol {i}")
                (st_name, st_value, st_size, st_info, _other,
                 st_shndx) = struct.unpack_from("<IIIBBH", d, off)
                name = _read_cstr(strtab, st_name)
                if not name:
                    continue
                sym = Symbol()
                sym.name, sym.value, sym.size = name, st_value, st_size
                sym.info, sym.shndx = st_info, st_shndx
                if st_shndx == 0:
                    continue  # undefined references are not definitions
                old = self.symbols.get(name)
                if old is not None and (old.value, old.size, old.shndx) != \
                        (sym.value, sym.size, sym.shndx):
                    if name == "pgl_build_id":
                        raise ElfError("ambiguous pgl_build_id definitions")
                self.symbols.setdefault(name, sym)

    def load_image(self, base: int | None = None) -> tuple[int, bytes]:
        """Reconstruct objcopy's allocated, initialized section load image.

        BSS/NOLOAD sections never extend the binary; section LMA is derived
        from the containing PT_LOAD. Segment padding is not file content.
        A bounded span prevents malformed addresses causing huge allocations.
        """
        chunks = []
        for sec in self.sections:
            if not sec.size or not (sec.flags & 2) or sec.type == SHT_NOBITS:
                continue
            matches = [s for s in self.load_segments
                       if s.file_contains_vaddr(sec.addr, sec.size) and
                       s.offset + sec.addr - s.vaddr == sec.offset]
            if len(matches) != 1:
                raise ElfError(
                    f"{self.source}: section {sec.name} has no unique LOAD")
            seg = matches[0]
            chunks.append((seg.paddr + sec.addr - seg.vaddr,
                           self.data[sec.offset:sec.offset + sec.size]))
        if not chunks:
            raise ElfError(f"{self.source}: no initialized allocated sections")
        chunks.sort()
        if base is None:
            base = chunks[0][0]
        top = max(addr + len(data) for addr, data in chunks)
        if base < 0 or top <= base or top - base > 16 * 1024 * 1024:
            raise ElfError(f"{self.source}: flat load image span is invalid")
        image = bytearray(top - base)
        previous_end = base
        for addr, data in chunks:
            if addr < previous_end:
                raise ElfError(f"{self.source}: initialized load ranges overlap")
            start = addr - base
            image[start:start + len(data)] = data
            previous_end = addr + len(data)
        return base, bytes(image)

    def read_vaddr(self, addr: int, length: int) -> bytes:
        """Read *initialized* file bytes mapped at a virtual address."""
        for seg in self.load_segments:
            if seg.file_contains_vaddr(addr, length):
                off = seg.offset + (addr - seg.vaddr)
                return self.data[off:off + length]
        raise ElfError(
            f"{self.source}: address 0x{addr:08x}+{length} not covered by "
            "any initialized LOAD range")

    def exec_segments(self):
        return [s for s in self.load_segments if s.flags & PF_X]


def _read_cstr(buf: bytes, off: int) -> str:
    if off >= len(buf):
        return ""
    end = buf.find(b"\0", off)
    if end < 0:
        end = len(buf)
    return buf[off:end].decode("utf-8", "replace")
