# Copyright (c) 2026 ProtoGPU contributors
# RP2350 PICOBIN block / IMAGE_DEF parser used by the packaging tool.
# Standard library only.
#
# Constants mirror the pinned SDK header
# third_party/pico-sdk/src/common/boot_picobin_headers/include/boot/picobin.h
# and the block layout emitted by
# third_party/pico-sdk/src/rp2_common/pico_crt0/embedded_start_block.inc.S.

from __future__ import annotations

import struct

BLOCK_MARKER_START = 0xFFFFDED3
BLOCK_MARKER_END = 0xAB123579

ITEM_1BS_IMAGE_TYPE = 0x42
ITEM_1BS_VECTOR_TABLE = 0x03
ITEM_1BS_ENTRY_POINT = 0x44
ITEM_2BS_IGNORED = 0xFE
ITEM_2BS_LAST = 0xFF

MAX_BLOCK_SIZE = 0x280
MAX_IMAGE_DEF_BLOCK_SIZE = 0x180

# IMAGE_TYPE value fields
IMAGE_TYPE_EXE = 0x1
EXE_SECURITY_UNSPECIFIED = 0x0
EXE_SECURITY_NS = 0x1
EXE_SECURITY_S = 0x2
EXE_CPU_ARM = 0x0
EXE_CPU_RISCV = 0x1
EXE_CHIP_RP2040 = 0x0
EXE_CHIP_RP2350 = 0x1

_SECURITY_NAMES = {0: "unspecified", 1: "non-secure", 2: "secure"}
_CPU_NAMES = {0: "ARM", 1: "RISC-V", 2: "VARMULET"}
_CHIP_NAMES = {0: "RP2040", 1: "RP2350"}


class ImageDefError(Exception):
    """Raised for missing, truncated, malformed or inconsistent IMAGE_DEF."""


class ImageType:
    __slots__ = ("raw", "kind", "security", "cpu", "chip", "tbyb")

    def __init__(self, raw: int):
        self.raw = raw
        self.kind = raw & 0xF
        self.security = (raw >> 4) & 0x3
        self.cpu = (raw >> 8) & 0x7
        self.chip = (raw >> 12) & 0x7
        self.tbyb = bool(raw & 0x8000)

    def describe(self) -> str:
        kind = {0: "INVALID", 1: "EXE", 2: "DATA"}.get(self.kind,
                                                       f"unknown({self.kind})")
        return (f"{kind} chip={_CHIP_NAMES.get(self.chip, self.chip)} "
                f"cpu={_CPU_NAMES.get(self.cpu, self.cpu)} "
                f"security={_SECURITY_NAMES.get(self.security, self.security)}"
                f"{' TBYB' if self.tbyb else ''}")


class ImageDef:
    """Parsed result for the image-definition block loop at the image
    start."""

    def __init__(self):
        self.offset = 0            # byte offset of START marker in image
        self.size = 0              # bytes START..END inclusive (first block)
        self.image_type: ImageType | None = None
        self.vector_table: int | None = None
        self.entry_point: int | None = None
        self.entry_sp: int | None = None
        self.blocks = 0
        self.signed = False


def _u32(image: bytes, off: int) -> int:
    return struct.unpack_from("<I", image, off)[0]


def parse_image_def(image: bytes, window: int = 4096) -> ImageDef:
    """Locate and strictly validate the PICOBIN block loop at the start of
    a flat load image (bytes, with image base at offset 0).

    Raises ImageDefError on any malformed/truncated/unclosed structure.
    """
    result = ImageDef()
    limit = min(window, len(image))
    start = None
    for off in range(0, limit - 3, 4):
        if _u32(image, off) == BLOCK_MARKER_START:
            start = off
            break
    if start is None:
        raise ImageDefError(
            f"no PICOBIN block start marker 0x{BLOCK_MARKER_START:08x} in "
            f"first {limit} bytes of image")
    result.offset = start

    block_start = start
    visited = set()
    blocks = 0
    while True:
        if block_start in visited:
            if block_start == start:
                break
            raise ImageDefError("PICOBIN loop closes on a noninitial block")
        visited.add(block_start)
        blocks += 1
        if blocks > 8:
            raise ImageDefError("PICOBIN block loop does not terminate")
        if block_start < 0 or block_start + 4 > len(image):
            raise ImageDefError("PICOBIN block pointer outside image")
        if _u32(image, block_start) != BLOCK_MARKER_START:
            raise ImageDefError(
                f"PICOBIN block {blocks} at offset {block_start} lacks "
                "start marker")
        pos = block_start + 4
        block_limit = min(block_start + MAX_BLOCK_SIZE, len(image))
        while True:
            if pos + 4 > block_limit:
                raise ImageDefError("no LAST item before PICOBIN block bound")
            word = _u32(image, pos)
            item = word & 0xFF
            size_words = (word >> 8) & (0xFFFF if item & 0x80 else 0xFF)
            if item == ITEM_2BS_LAST:
                if word >> 24:
                    raise ImageDefError("LAST item has reserved bits set")
                items_bytes = pos - (block_start + 4)
                if size_words * 4 != items_bytes:
                    raise ImageDefError(
                        "PICOBIN LAST size does not match preceding items")
                pos += 4
                break
            if not size_words:
                raise ImageDefError("PICOBIN item has zero size")
            item_end = pos + size_words * 4
            if item_end > block_limit:
                raise ImageDefError("PICOBIN item overruns block bound")
            if item == ITEM_1BS_IMAGE_TYPE:
                if size_words != 1 or result.image_type is not None:
                    raise ImageDefError("invalid or duplicate IMAGE_TYPE")
                if blocks != 1 or pos != block_start + 4:
                    raise ImageDefError("IMAGE_TYPE must begin first block")
                result.image_type = ImageType((word >> 16) & 0xFFFF)
            elif item == ITEM_1BS_VECTOR_TABLE:
                if size_words != 2 or result.vector_table is not None:
                    raise ImageDefError("invalid or duplicate VECTOR_TABLE")
                result.vector_table = _u32(image, pos + 4)
            elif item == ITEM_1BS_ENTRY_POINT:
                if size_words != 3 or result.entry_point is not None:
                    raise ImageDefError("invalid or duplicate ENTRY_POINT")
                result.entry_point = _u32(image, pos + 4)
                result.entry_sp = _u32(image, pos + 8)
            elif item in (0x09, 0x47, 0x4B, 0x0C):
                raise ImageDefError(
                    "signed/hashed PICOBIN metadata is unsupported; "
                    "cryptographic verification is not implemented")
            elif item not in (ITEM_2BS_IGNORED, 0x48):
                raise ImageDefError(
                    f"unsupported boot-affecting PICOBIN item 0x{item:02x}")
            pos = item_end

        # After LAST: next-block offset word, then END marker.
        if pos + 8 > block_limit:
            raise ImageDefError(
                "truncated PICOBIN block: missing next-offset/END marker")
        next_offset = _u32(image, pos)
        end_marker = _u32(image, pos + 4)
        if end_marker != BLOCK_MARKER_END:
            raise ImageDefError(
                f"PICOBIN block at offset {block_start} is not closed: "
                f"END marker 0x{end_marker:08x} != "
                f"0x{BLOCK_MARKER_END:08x}")
        block_end = pos + 8
        # The LAST size was checked before consuming the trailer.
        if blocks == 1:
            result.size = block_end - block_start
            if result.size > MAX_IMAGE_DEF_BLOCK_SIZE:
                raise ImageDefError(
                    f"IMAGE_DEF block size {result.size} exceeds "
                    f"0x{MAX_IMAGE_DEF_BLOCK_SIZE:x}")
            if block_end > window:
                raise ImageDefError(
                    f"IMAGE_DEF block ends at offset {block_end}, beyond "
                    f"the first {window} bytes")
        if next_offset == 0:
            if blocks != 1:
                raise ImageDefError("PICOBIN loop closes on a noninitial block")
            break
        if next_offset % 4:
            raise ImageDefError("PICOBIN next-block offset not word aligned")
        # ROM offsets are signed 32-bit relative to this block's START.
        delta = struct.unpack("<i", struct.pack("<I", next_offset))[0]
        block_start += delta
        if block_start < 0 or block_start >= len(image):
            raise ImageDefError("PICOBIN next-block pointer outside image")

    result.blocks = blocks
    if result.image_type is None:
        raise ImageDefError(
            "first PICOBIN block has no IMAGE_TYPE item (not an IMAGE_DEF)")
    return result


def require_rp2350_arm_exe(idef: ImageDef) -> None:
    """Enforce the boot family this firmware targets: RP2350 ARM EXE."""
    t = idef.image_type
    assert t is not None
    if t.kind != IMAGE_TYPE_EXE:
        raise ImageDefError(
            f"IMAGE_TYPE kind {t.kind}, expected EXE (1): {t.describe()}")
    if t.chip != EXE_CHIP_RP2350:
        raise ImageDefError(
            f"IMAGE_TYPE wrong chip family: {t.describe()} (expected "
            "RP2350)")
    if t.cpu != EXE_CPU_ARM:
        raise ImageDefError(
            f"IMAGE_TYPE wrong CPU: {t.describe()} (expected ARM)")
    if t.security not in _SECURITY_NAMES or t.raw & 0x08C0:
        raise ImageDefError("IMAGE_TYPE has invalid security/reserved bits")
    if t.tbyb:
        raise ImageDefError(
            "IMAGE_TYPE has TBYB (try-before-you-buy update) flag; not a "
            "normally bootable image")
