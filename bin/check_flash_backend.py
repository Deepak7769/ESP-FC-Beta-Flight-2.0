#!/usr/bin/env python3
"""Compile the real ESP-FC ESP32 flash backend with a fault-injecting SDK.

The stub SDK models NOR programming and supports read/write/erase failures.
No firmware implementation is duplicated in this harness: the tested C++
translation unit is lib/Espfc/src/Blackbox/BlackboxFlashfs.cpp.
"""
from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "lib/Espfc/src/Blackbox/BlackboxFlashfs.cpp"
INCLUDES = ROOT / "lib/Espfc/src"

PLATFORM_H = r"""
#pragma once
#include <cstdint>
#include <cstddef>
#define FLASHFS_WRITE_BUFFER_SIZE 128u
#define FLASHFS_FLUSH_BUFFER_SIZE 64u
#define FLASHFS_JOURNAL_ITEMS 32u
#define FLASHFS_JOURNAL_SIZE (FLASHFS_JOURNAL_ITEMS * sizeof(FlashfsJournalItem))
#define SPI_FLASH_SEC_SIZE 4096u
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t logBegin;
    uint32_t logEnd;
} __attribute__((packed)) FlashfsJournalItem;
typedef struct {
    const void* partition;
    void* buffer;
    uint32_t address;
    FlashfsJournalItem journal[FLASHFS_JOURNAL_ITEMS];
    uint32_t journalIdx;
} FlashfsRuntime;
const FlashfsRuntime* flashfsGetRuntime();
void flashfsJournalLoad(FlashfsJournalItem*, size_t, size_t);
void flashfsJournalFix(FlashfsJournalItem*, size_t);
int flashfsInit(void);
uint32_t flashfsGetSize(void);
uint32_t flashfsGetOffset(void);
uint32_t flashfsGetSectors(void);
void flashfsWriteByte(uint8_t);
void flashfsWrite(const uint8_t*, unsigned int, bool);
void flashfsWriteAbs(uint32_t, const uint8_t*, unsigned int);
int flashfsReadAbs(uint32_t, uint8_t*, unsigned int);
bool flashfsFlushAsync(bool);
bool flashfsIsSupported(void);
bool flashfsIsReady(void);
bool flashfsIsEOF(void);
void flashfsEraseCompletely(void);
void flashfsClose(void);
uint32_t flashfsGetWriteBufferFreeSpace(void);
uint32_t flashfsGetWriteBufferSize(void);
#ifdef __cplusplus
}
#endif
"""

ESP_PARTITION_H = r"""
#pragma once
#include <cstddef>
#include <cstdint>
typedef int esp_err_t;
static constexpr esp_err_t ESP_OK = 0;
static constexpr esp_err_t ESP_FAIL = -1;
static constexpr int ESP_PARTITION_TYPE_ANY = 255;
static constexpr int ESP_PARTITION_SUBTYPE_ANY = 255;
static constexpr int ESP_PARTITION_TYPE_DATA = 1;
static constexpr int ESP_PARTITION_SUBTYPE_DATA_SPIFFS = 130;
struct esp_partition_t {
    int type;
    int subtype;
    uint32_t address;
    uint32_t size;
    const char* label;
};
using esp_partition_iterator_t = const esp_partition_t*;
esp_partition_iterator_t esp_partition_find(int, int, const char*);
esp_partition_iterator_t esp_partition_next(esp_partition_iterator_t);
const esp_partition_t* esp_partition_get(esp_partition_iterator_t);
const esp_partition_t* esp_partition_find_first(int, int, const char*);
esp_err_t esp_partition_read_raw(const esp_partition_t*, size_t, void*, size_t);
esp_err_t esp_partition_write_raw(const esp_partition_t*, size_t, const void*, size_t);
esp_err_t esp_partition_erase_range(const esp_partition_t*, size_t, size_t);
"""

PRINTER_H = r"""
#pragma once
namespace Espfc { namespace Stream {
class Printer {
public:
    template<typename... T> void printf(const char*, T...) {}
};
} }
"""

TEST_CPP = r"""
#include <platform.h>
#include <esp_partition.h>
#include "Device/FlashDevice.h"
#include <array>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

static constexpr size_t SIZE = 16384;
static std::array<uint8_t, SIZE> chip;
static esp_partition_t partition{1, 130, 0, uint32_t(SIZE), "blackbox"};
static bool present = true;
static bool readFails = false;
static bool eraseFails = false;
static int failWriteAt = -1;
static int partialWriteAt = -1;
static int writeCalls = 0;
static int cases = 0;

esp_partition_iterator_t esp_partition_find(int, int, const char*) { return nullptr; }
esp_partition_iterator_t esp_partition_next(esp_partition_iterator_t) { return nullptr; }
const esp_partition_t* esp_partition_get(esp_partition_iterator_t p) { return p; }
const esp_partition_t* esp_partition_find_first(int, int, const char*) {
    return present ? &partition : nullptr;
}
esp_err_t esp_partition_read_raw(const esp_partition_t* p, size_t off, void* dst, size_t n) {
    if (readFails || !p || !dst || off > p->size || n > p->size - off) return ESP_FAIL;
    std::memcpy(dst, chip.data() + off, n);
    return ESP_OK;
}
esp_err_t esp_partition_write_raw(const esp_partition_t* p, size_t off, const void* data, size_t n) {
    ++writeCalls;
    if (!p || !data || off > p->size || n > p->size - off) return ESP_FAIL;
    const auto* src = static_cast<const uint8_t*>(data);
    if (writeCalls == partialWriteAt) {
        if (n) chip[off] &= src[0];
        return ESP_FAIL;
    }
    if (writeCalls == failWriteAt) return ESP_FAIL;
    for (size_t i = 0; i < n; ++i) chip[off + i] &= src[i];
    return ESP_OK;
}
esp_err_t esp_partition_erase_range(const esp_partition_t* p, size_t off, size_t n) {
    if (eraseFails || !p || off > p->size || n > p->size - off) return ESP_FAIL;
    std::fill(chip.begin() + off, chip.begin() + off + n, 0xff);
    return ESP_OK;
}

static void check(bool ok, const char* detail) {
    if (!ok) throw std::runtime_error(detail);
}
static void fresh() {
    chip.fill(0xff);
    partition.size = SIZE;
    present = true;
    readFails = eraseFails = false;
    failWriteAt = partialWriteAt = -1;
    writeCalls = 0;
    check(flashfsInit() == 1, "fresh init");
}
template<typename F>
static void test(const char* name, F action) {
    fresh();
    try { action(); }
    catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL %s: %s\n", name, e.what());
        std::exit(1);
    }
    ++cases;
    std::printf("PASS %s\n", name);
}
static uint32_t journalWord(size_t index) {
    uint32_t v;
    std::memcpy(&v, chip.data() + SIZE - FLASHFS_JOURNAL_SIZE + index, sizeof(v));
    return v;
}
static void writeWord(size_t offset, uint32_t value) {
    std::memcpy(chip.data() + SIZE - FLASHFS_JOURNAL_SIZE + offset, &value, sizeof(value));
}

int main() {
    test("blank partition initializes ready", [] {
        check(flashfsIsReady() && flashfsGetOffset() == 0, "blank readiness");
        check(flashfsGetSize() == SIZE - FLASHFS_JOURNAL_SIZE, "data capacity");
    });
    test("empty flush/close leaves journal erased", [] {
        check(flashfsFlushAsync(true), "empty flush");
        flashfsClose();
        check(journalWord(0) == 0xffffffffu, "empty journal");
    });
    test("successful payload flush advances offset", [] {
        flashfsWriteByte(0x23);
        check(flashfsFlushAsync(true), "payload flush");
        check(chip[0] == 0x23 && flashfsGetOffset() == 1, "written payload");
    });
    test("normal close seals journal and reboot recovers", [] {
        flashfsWriteByte(0x31);
        flashfsClose();
        check(journalWord(0) == 0 && journalWord(4) == 1, "journal start/end");
        check(flashfsInit() == 1 && flashfsGetOffset() == 1, "restart recovery");
    });
    test("journal begin write failure blocks recording", [] {
        failWriteAt = 1;
        flashfsWriteByte(0x41);
        check(!flashfsFlushAsync(true), "begin should fail");
        check(!flashfsIsReady() && flashfsGetOffset() == 0, "fault latched");
    });
    test("payload write failure preserves pending bytes and offset", [] {
        failWriteAt = 2;
        flashfsWriteByte(0x42);
        check(!flashfsFlushAsync(true), "payload should fail");
        check(flashfsGetOffset() == 0 && chip[0] == 0xff, "not consumed");
        check(!flashfsIsReady() && flashfsGetWriteBufferFreeSpace() == 0, "no further writes");
    });
    test("partial NOR write error prohibits in-place retry", [] {
        partialWriteAt = 2;
        flashfsWriteByte(0x43);
        check(!flashfsFlushAsync(true), "partial write should fail");
        const int before = writeCalls;
        check(!flashfsFlushAsync(true), "retry prohibited");
        check(writeCalls == before && chip[0] == 0x43 && flashfsGetOffset() == 0,
              "in-place partial write retry");
    });
    test("failed journal end remains unfinished across reboot", [] {
        failWriteAt = 3;
        flashfsWriteByte(0x44);
        flashfsClose();
        check(!flashfsIsReady(), "end write failure");
        check(journalWord(0) == 0 && journalWord(4) == 0xffffffffu, "unfinished journal");
        failWriteAt = -1;
        check(flashfsInit() == 0 && !flashfsIsReady(), "must not auto recover");
        uint8_t read = 0;
        check(flashfsReadAbs(0, &read, 1) == 1 && read == 0x44, "unfinished data readable");
    });
    test("failed flash erase retains fault and data", [] {
        flashfsWriteByte(0x45);
        flashfsClose();
        eraseFails = true;
        flashfsEraseCompletely();
        check(!flashfsIsReady() && chip[0] == 0x45, "erase failure reported");
    });
    test("explicit successful erase recovers fault and journal", [] {
        failWriteAt = 2;
        flashfsWriteByte(0x46);
        check(!flashfsFlushAsync(true), "fault created");
        failWriteAt = -1;
        flashfsEraseCompletely();
        check(flashfsIsReady() && journalWord(0) == 0xffffffffu, "erase reinitializes");
    });
    test("invalid journal load bounds latch a fault", [] {
        FlashfsJournalItem out[2]{};
        flashfsJournalLoad(out, FLASHFS_JOURNAL_ITEMS - 1, 2);
        check(!flashfsIsReady(), "journal bounds");
    });
    test("corrupt journal end is rejected at startup", [] {
        writeWord(0, 0);
        writeWord(4, uint32_t(flashfsGetSize() + 1));
        check(flashfsInit() == 0 && !flashfsIsReady(), "journal corruption");
    });
    test("nonempty journal beyond first gap is rejected", [] {
        writeWord(8, 0);
        writeWord(12, 1);
        check(flashfsInit() == 0 && !flashfsIsReady(), "journal gap");
    });
    test("flash partition write rejects SDK failure and overflow", [] {
        Espfc::Device::FlashPartition api(&partition);
        uint8_t b = 0x5a;
        failWriteAt = 1;
        check(api.write(0, &b, 1) == 0, "SDK write failure");
        failWriteAt = -1;
        check(api.write(SIZE, &b, 1) == 0, "out-of-range write");
        check(api.write(0, &b, 1) == 1 && chip[0] == 0x5a, "valid write");
    });
    test("flash partition read rejects SDK failure and overflow", [] {
        Espfc::Device::FlashPartition api(&partition);
        uint8_t b = 0x5a;
        readFails = true;
        check(api.read(0, &b, 1) == 0, "SDK read failure");
        readFails = false;
        check(api.read(SIZE, &b, 1) == 0, "out-of-range read");
        check(api.read(0, &b, 1) == 1 && b == 0xff, "valid read");
    });
    test("flash init detects SDK read failure", [] {
        readFails = true;
        check(flashfsInit() == 0 && !flashfsIsReady(), "failed journal read");
    });
    test("too-small partition is rejected", [] {
        partition.size = FLASHFS_JOURNAL_SIZE;
        check(flashfsInit() == 0 && !flashfsIsReady(), "journal exceeds partition");
    });
    std::printf("Flash backend fault-injection: %d cases passed\n", cases);
    return 0;
}
"""

def main() -> None:
    with tempfile.TemporaryDirectory(prefix="espfc-flash-fault-") as folder:
        temp = Path(folder)
        stubs = temp / "stubs"
        (stubs / "Stream").mkdir(parents=True)
        (stubs / "Arduino.h").write_text("#pragma once\n", encoding="utf-8")
        (stubs / "platform.h").write_text(PLATFORM_H, encoding="utf-8")
        (stubs / "esp_partition.h").write_text(ESP_PARTITION_H, encoding="utf-8")
        (stubs / "Stream" / "Printer.hpp").write_text(PRINTER_H, encoding="utf-8")
        test_source = temp / "flash_fault_tests.cpp"
        test_source.write_text(TEST_CPP, encoding="utf-8")
        executable = temp / "check_flash_backend"
        cmd = [
            os.environ.get("CXX", "c++"),
            "-std=c++17", "-Wall", "-Wextra", "-Werror=return-type",
            "-DUSE_FLASHFS", "-DUNIT_TEST", "-DIRAM_ATTR=",
            "-I", str(stubs), "-I", str(INCLUDES),
            str(SOURCE), str(test_source), "-o", str(executable),
        ]
        subprocess.run(cmd, cwd=ROOT, check=True)
        subprocess.run([str(executable)], cwd=ROOT, check=True)

if __name__ == "__main__":
    main()
