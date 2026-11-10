#include <Arduino.h>
#include <platform.h>

#ifdef USE_FLASHFS

#include "Device/FlashDevice.h"
#include "Utils/RingBuf.h"
#include "Utils/Math.hpp"

static constexpr uint32_t FLASHFS_ERASED_VAL = 0xffffffff;

using BufferType = Espfc::Utils::RingBuf<uint8_t, FLASHFS_WRITE_BUFFER_SIZE>;

static BufferType buff;

static FlashfsRuntime flashfs;
static bool flashfsFault = false;

static bool IRAM_ATTR flashfsWriteChecked(
    uint32_t address, const uint8_t* data, unsigned int len)
{
    const auto p = reinterpret_cast<const esp_partition_t*>(flashfs.partition);
    if (flashfsFault || !p || !data || address > p->size || len > p->size - address)
    {
        flashfsFault = true;
        return false;
    }
    if (len && esp_partition_write_raw(p, address, data, len) != ESP_OK)
    {
        // Never retry a potentially partially programmed NOR region in place.
        flashfsFault = true;
        return false;
    }
    return true;
}

const FlashfsRuntime * flashfsGetRuntime()
{
    return &flashfs;
}

static uint32_t IRAM_ATTR flashfsJournalAddress(size_t index, bool getEnd = false)
{
    uint32_t journalBase = reinterpret_cast<const esp_partition_t*>(flashfs.partition)->size - FLASHFS_JOURNAL_SIZE;
    return journalBase + (index * sizeof(FlashfsJournalItem)) + (getEnd * sizeof(uint32_t));
}

void flashfsJournalLoad(FlashfsJournalItem * data, size_t start, size_t num)
{
    if (!flashfs.partition || !data || start >= FLASHFS_JOURNAL_ITEMS ||
        num > FLASHFS_JOURNAL_ITEMS - start)
    {
        flashfsFault = true;
        return;
    }
    size_t size = num * sizeof(FlashfsJournalItem);
    if (flashfsReadAbs(flashfsJournalAddress(start), (uint8_t*)data, size) != int(size))
        flashfsFault = true;
}

static bool IRAM_ATTR flashfsLogBegin()
{
    uint32_t beginAddr = flashfs.address;
    size_t idx = flashfs.journalIdx;
    if(idx >= FLASHFS_JOURNAL_ITEMS)
{
    return false;
}
    

    if(!flashfs.partition) return false;

    size_t address = flashfsJournalAddress(idx);

    if (!flashfsWriteChecked(address, (uint8_t*)&beginAddr, sizeof(uint32_t))) return false;
    flashfs.journal[idx].logBegin = beginAddr;
    return true;
}

static bool IRAM_ATTR flashfsLogStarted()
{
    if(flashfs.journalIdx >= FLASHFS_JOURNAL_ITEMS)
    {
        return false;
    }

    return
        flashfs.journal[flashfs.journalIdx].logBegin !=
        FLASHFS_ERASED_VAL;
}
static bool IRAM_ATTR flashfsLogEnd()
{
    uint32_t endAddr = flashfs.address;
    size_t idx = flashfs.journalIdx;
    if(idx >= FLASHFS_JOURNAL_ITEMS)
{
    return false;
}
    if(!flashfs.partition) return false;

    size_t address = flashfsJournalAddress(idx, true);

    if (!flashfsWriteChecked(address, (uint8_t*)&endAddr, sizeof(uint32_t))) return false;
    flashfs.journal[idx].logEnd = endAddr;
    flashfs.journalIdx++;
    return true;
}

void flashfsJournalFix(FlashfsJournalItem * data, size_t num)
{
    if (!data || num > FLASHFS_JOURNAL_ITEMS) { flashfsFault = true; return; }
    // An unfinished log has no authenticated end offset. Preserve it for
    // download; require an explicit disarmed erase before writing again.
    for(size_t i = 0; i < num; i++)
    {
        if(data[i].logBegin != FLASHFS_ERASED_VAL && data[i].logEnd == FLASHFS_ERASED_VAL)
        {
            flashfsFault = true;
            return;
        }
    }
}

int flashfsInit(void)
{
    flashfsFault = false;
    buff.clear();
    flashfs.partition = Espfc::Device::FlashDevice::findPartition();
    if(!flashfs.partition) return 0;
    if (reinterpret_cast<const esp_partition_t*>(flashfs.partition)->size <= FLASHFS_JOURNAL_SIZE)
    {
        flashfsFault = true;
        return 0;
    }

    flashfs.buffer = (void*)&buff;
    flashfs.journalIdx = 0;
    flashfs.address = 0;

    flashfsJournalLoad(flashfs.journal, 0, FLASHFS_JOURNAL_ITEMS);
    if (flashfsFault) return 0;
    for(size_t i = 0; i < FLASHFS_JOURNAL_ITEMS; i++)
    {
        const auto& it = flashfs.journal[i];
        if (it.logBegin == FLASHFS_ERASED_VAL && it.logEnd == FLASHFS_ERASED_VAL)
        {
            for (size_t j = i + 1; j < FLASHFS_JOURNAL_ITEMS; ++j)
                if (flashfs.journal[j].logBegin != FLASHFS_ERASED_VAL ||
                    flashfs.journal[j].logEnd != FLASHFS_ERASED_VAL) flashfsFault = true;
            break;
        }
        if (it.logBegin != flashfs.address || it.logBegin >= flashfsGetSize() ||
            (it.logEnd != FLASHFS_ERASED_VAL &&
             (it.logEnd < it.logBegin || it.logEnd > flashfsGetSize())))
        {
            flashfsFault = true;
            break;
        }
        if(it.logEnd == FLASHFS_ERASED_VAL) break;
        flashfs.address = it.logEnd;
        flashfs.journalIdx++;
    }
    flashfsJournalFix(flashfs.journal, FLASHFS_JOURNAL_ITEMS);

    return flashfsFault ? 0 : 1;
}

bool IRAM_ATTR flashfsFlushAsync(bool force)
{
    (void)force;
    auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
    if (flashfsFault || !flashfs.partition || !buffer) return false;
    const size_t size = buffer->size();
    if (!size) return true; // Empty means drained; do not create an empty log.
    if(!flashfsLogStarted() && !flashfsLogBegin()) return false;
    if(flashfs.partition && size > 0)
    {
        //uint32_t newAddress = force ? (flashfs.address + size) : Espfc::Utils::alignAddressToWrite(flashfs.address, size, FLASHFS_FLUSH_BUFFER_SIZE);
        //size_t toWrite = newAddress - flashfs.address;
        uint8_t tmp[FLASHFS_FLUSH_BUFFER_SIZE];
        size_t chunks = (size / FLASHFS_FLUSH_BUFFER_SIZE) + 1;
        while(chunks--)
        {
            size_t len = buffer->peek(tmp, FLASHFS_FLUSH_BUFFER_SIZE);
            if (!len) break;
            if (flashfs.address > flashfsGetSize() || len > flashfsGetSize() - flashfs.address)
            {
                flashfsFault = true;
                return false;
            }
            if (!flashfsWriteChecked(flashfs.address, tmp, len)) return false;
            buffer->pop(tmp, len);
            flashfs.address += len;
        }
        return true;
    }
    return false;
}

void IRAM_ATTR flashfsWriteByte(uint8_t byte)
{
    if(flashfsIsEOF()) return;

    auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
    if (!buffer || !buffer->push(byte)) { flashfsFault = true; return; }
    const size_t size = buffer->size();

    if(flashfs.address + size >= flashfsGetSize())
    {
        flashfsClose();
        return;
    }

    if((flashfs.address + size) % SPI_FLASH_SEC_SIZE == 0)
    {
        flashfsFlushAsync(true);
        return;
    }

    if(size >= FLASHFS_FLUSH_BUFFER_SIZE)
    {
        flashfsFlushAsync(false);
    }
}

void IRAM_ATTR flashfsWrite(const uint8_t *data, unsigned int len, bool sync)
{
    (void)sync;
    while(len)
    {
        flashfsWriteByte(*data);
        data++;
        len--;
    }
}

void IRAM_ATTR flashfsWriteAbs(uint32_t address, const uint8_t *data, unsigned int len)
{
    (void)flashfsWriteChecked(address, data, len);
}

int IRAM_ATTR flashfsReadAbs(
    uint32_t address,
    uint8_t *data,
    unsigned int len)
{
    if(!flashfs.partition || !data || len == 0)
    {
        return 0;
    }

    const auto p =
        reinterpret_cast<
            const esp_partition_t*>(
                flashfs.partition);

    if(address >= p->size)
    {
        return 0;
    }

    len =
        std::min<uint32_t>(
            len,
            p->size - address);

    if(esp_partition_read_raw(
           p,
           address,
           data,
           len) == ESP_OK)
    {
        return len;
    }

    return 0;
}

bool IRAM_ATTR flashfsIsSupported(void)
{
    return (bool)flashfs.partition;
}

bool IRAM_ATTR flashfsIsReady(void)
{
    return flashfsIsSupported() && !flashfsFault;
}

bool IRAM_ATTR flashfsIsEOF(void)
{
    return flashfsFault || !flashfs.partition || flashfs.journalIdx >= FLASHFS_JOURNAL_ITEMS || flashfs.address >= flashfsGetSize();
}

void flashfsEraseCompletely(void)
{
    if(!flashfs.partition)
    {
        return;
    }

    const auto p =
        reinterpret_cast<
            const esp_partition_t*>(
                flashfs.partition);

    if (esp_partition_erase_range(
        p,
        0,
        p->size) != ESP_OK)
    {
        flashfsFault = true;
        return;
    }

    flashfsInit();
}

void IRAM_ATTR flashfsClose(void)
{
    if (flashfsFlushAsync(true) && flashfsLogStarted()) (void)flashfsLogEnd();
}

uint32_t IRAM_ATTR flashfsGetWriteBufferFreeSpace(void)
{
    auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
    return buffer && !flashfsFault ? buffer->available() : 0;
}

uint32_t IRAM_ATTR flashfsGetWriteBufferSize(void)
{
    return FLASHFS_WRITE_BUFFER_SIZE;
}

uint32_t IRAM_ATTR flashfsGetSize(void)
{
    const auto p = reinterpret_cast<const esp_partition_t*>(flashfs.partition);
    return p && p->size > FLASHFS_JOURNAL_SIZE ? p->size - FLASHFS_JOURNAL_SIZE : 0;
}

uint32_t flashfsGetOffset(void)
{
    return flashfs.address;
}

uint32_t flashfsGetSectors(void)
{
    return flashfsGetSize() / SPI_FLASH_SEC_SIZE;
}

#endif
