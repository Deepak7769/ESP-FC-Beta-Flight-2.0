#!/usr/bin/env python3
"""Fault-inject GPIO into the actual EspWire implementation; no hardware used."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
ARDUINO = r'''
#pragma once
#include <cstdint>
#include <cstddef>
#define IRAM_ATTR
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define SDA 0
#define SCL 1
void pinMode(int, int);
int digitalRead(int);
class Print {
public:
 virtual ~Print() = default;
 virtual size_t write(uint8_t) { return 0; }
 virtual size_t write(const uint8_t*, size_t) { return 0; }
 void setWriteError() {}
};
class Stream : public Print {
public:
 virtual int available() { return 0; }
 virtual int read() { return -1; }
 virtual int peek() { return -1; }
 virtual void flush() {}
};
'''
TEST = r'''
#include <cassert>
#include <deque>
#include <cstdio>
#include "EspWire.h"
#include "esp_twi.h"
static int sdaMode, sclMode, reads, sclReads, sclFaultAt;
static bool sdaLow, sclLow;
static std::deque<int> sdaBits;
void pinMode(int pin, int mode) { (pin == SDA ? sdaMode : sclMode) = mode; }
int digitalRead(int pin) {
 assert(++reads < 10000); // Catch unbounded recovery, independently of a test timeout.
 if (pin == SCL) return !sclLow && !(sclFaultAt && ++sclReads >= sclFaultAt);
 if (sdaLow || sdaMode == OUTPUT) return 0;
 if (sdaBits.empty()) return 1;
 int value = sdaBits.front(); sdaBits.pop_front(); return value;
}
static void reset() {
 reads = sclReads = sclFaultAt = 0; sdaLow = sclLow = false; sdaBits.clear();
 esp_twi_init(SDA, SCL); esp_twi_setClockStretchLimit(1);
}
int main() {
 uint8_t data[65]; for (auto& b : data) b = 0xa5;
 reset();
 assert(esp_twi_readFrom(0x68, data, 0, true) == 0 && data[0] == 0xa5 && reads == 0);
 assert(esp_twi_readFrom(0x68, nullptr, 1, true) == 4);
 assert(esp_twi_writeTo(0x68, nullptr, 1, true) == 4);
 reset(); sclLow = true;
 assert(esp_twi_readFrom(0x68, data, 64, true) == 4 && reads <= 11 && data[0] == 0xa5);
 reset(); sdaLow = true;
 assert(esp_twi_writeTo(0x68, data, 64, true) == 4 && reads <= 2);
 reset();
 assert(esp_twi_readFrom(0x68, data, 6, true) == 2); // No responding device: NACK.
 reset(); sdaBits = {1,0,0,1};
 assert(esp_twi_writeTo(0x68, data, 1, true) == 0);
 reset(); sdaBits = {1,0};
 for (int b=0; b<6; ++b) for (int bit=7; bit>=0; --bit) sdaBits.push_back((b>>bit)&1);
 sdaBits.push_back(1);
 EspTwoWire wire;
 assert(wire.requestFrom(uint8_t(0x68), uint8_t(6)) == 6);
 for (int b=0;b<6;++b) assert(wire.read()==b);
 assert(wire.available()==0 && wire.read()==-1);
 // A clock line failing in the data phase must not publish partial/fake data.
 reset(); sdaBits={1,0}; sclFaultAt=15;
 assert(wire.requestFrom(uint8_t(0x68), uint8_t(64))==0);
 assert(wire.available()==0 && reads<100);
 reset(); sdaLow=true;
 assert(esp_twi_status()==I2C_SDA_HELD_LOW && reads<100);
 // A later transaction can succeed; no permanent bus-wide error latch.
 reset(); sdaBits={1,0};
 for (int i=0;i<8;++i) sdaBits.push_back(1);
 sdaBits.push_back(1);
 assert(wire.requestFrom(uint8_t(0x68),uint8_t(1))==1 && wire.read()==255);
 assert(sdaMode==INPUT_PULLUP && sclMode==INPUT_PULLUP);
 puts("EspWire: empty/invalid requests, NACK, stuck SDA/SCL, mid-read fault, finite recovery, successful reads and subsequent recovery passed");
}
'''

with tempfile.TemporaryDirectory(prefix="espfc-i2c-") as tmp:
    work = Path(tmp)
    (work / "Arduino.h").write_text(ARDUINO)
    (work / "test.cpp").write_text(TEST)
    sources = ROOT / "lib/EspWire/src"
    subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra",
                    "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                    "-DUNIT_TEST", "-DF_CPU=240000000L", "-I"+str(work), "-I"+str(sources),
                    str(work / "test.cpp"), str(sources / "EspWire.cpp"),
                    str(sources / "esp_twi.cpp"), "-o", str(work / "check")], check=True)
    subprocess.run([str(work / "check")], check=True, timeout=10)
