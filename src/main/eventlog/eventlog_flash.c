/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#if defined(USE_EVENTLOG) && defined(USE_FLASHFS)

#include <string.h>

#include "common/maths.h"
#include "drivers/flash/flash.h"

#include "pg/eventlog.h"

#include "eventlog/eventlog_flash.h"

#define EVENTLOG_FREE_BLOCK_SIZE 2048
#define EVENTLOG_FREE_BLOCK_TEST_SIZE_INTS 4
#define EVENTLOG_FREE_BLOCK_TEST_SIZE_BYTES (EVENTLOG_FREE_BLOCK_TEST_SIZE_INTS * sizeof(uint32_t))
#define EVENTLOG_FREE_SCAN_SIZE 128

static const flashPartition_t *eventlogPartition = NULL;
static const flashGeometry_t *eventlogFlashGeometry = NULL;
static uint32_t eventlogStartAddress = 0;
static uint32_t eventlogSize = 0;
static uint32_t eventlogTailAddress = 0;

static uint32_t eventlogFlashFindStartOfFreeSpace(void)
{
    STATIC_DMA_DATA_AUTO union {
        uint8_t bytes[EVENTLOG_FREE_BLOCK_TEST_SIZE_BYTES];
        uint32_t ints[EVENTLOG_FREE_BLOCK_TEST_SIZE_INTS];
    } testBuffer;

    uint32_t left = 0;
    uint32_t right = eventlogSize / EVENTLOG_FREE_BLOCK_SIZE;
    uint32_t result = right;

    while (left < right) {
        const uint32_t mid = (left + right) / 2;

        const int bytesRead = flashReadBytes(eventlogStartAddress + (mid * EVENTLOG_FREE_BLOCK_SIZE), testBuffer.bytes, EVENTLOG_FREE_BLOCK_TEST_SIZE_BYTES);
        if (bytesRead < (int)EVENTLOG_FREE_BLOCK_TEST_SIZE_BYTES) {
            break;
        }

        bool blockErased = true;
        for (int i = 0; i < EVENTLOG_FREE_BLOCK_TEST_SIZE_INTS; i++) {
            if (testBuffer.ints[i] != 0xFFFFFFFF) {
                blockErased = false;
                break;
            }
        }

        if (blockErased) {
            result = mid;
            right = mid;
        } else {
            left = mid + 1;
        }
    }

    if (result == 0) {
        return 0;
    }

    const uint32_t scanStart = (result - 1) * EVENTLOG_FREE_BLOCK_SIZE;
    const uint32_t scanEnd = MIN(result * EVENTLOG_FREE_BLOCK_SIZE, eventlogSize);
    STATIC_DMA_DATA_AUTO uint8_t scanBuffer[EVENTLOG_FREE_SCAN_SIZE];

    for (uint32_t offset = scanStart; offset < scanEnd; offset += sizeof(scanBuffer)) {
        const uint32_t bytesToRead = MIN((uint32_t)sizeof(scanBuffer), scanEnd - offset);
        const int bytesRead = flashReadBytes(eventlogStartAddress + offset, scanBuffer, bytesToRead);
        if (bytesRead <= 0) {
            return scanEnd;
        }

        for (int i = 0; i < bytesRead; i++) {
            if (scanBuffer[i] == 0xFF) {
                return offset + (uint32_t)i;
            }
        }
    }

    return scanEnd;
}

bool eventlogFlashInit(void)
{
    eventlogPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_EVENTLOG);
    eventlogFlashGeometry = flashGetGeometry();

    if (!eventlogPartition || !eventlogFlashGeometry || eventlogFlashGeometry->sectorSize == 0 || eventlogFlashGeometry->pageSize == 0) {
        eventlogStartAddress = 0;
        eventlogSize = 0;
        eventlogTailAddress = 0;
        return false;
    }

    eventlogStartAddress = eventlogPartition->startSector * eventlogFlashGeometry->sectorSize;
    const uint32_t partitionSize = FLASH_PARTITION_SECTOR_COUNT(eventlogPartition) * eventlogFlashGeometry->sectorSize;
    const uint32_t configuredSize = eventlogConfig()->sizeKb * 1024UL;
    eventlogSize = configuredSize > 0 ? MIN(partitionSize, configuredSize) : partitionSize;
    eventlogTailAddress = eventlogFlashFindStartOfFreeSpace();

    return eventlogSize > 0;
}

bool eventlogFlashErase(void)
{
    if (!eventlogPartition || !eventlogFlashGeometry || eventlogFlashGeometry->sectorSize == 0) {
        return false;
    }

    for (flashSector_t sector = eventlogPartition->startSector; sector <= eventlogPartition->endSector; sector++) {
        flashEraseSector(sector * eventlogFlashGeometry->sectorSize);
    }

    eventlogTailAddress = 0;
    flashFlush();

    return true;
}

bool eventlogFlashWrite(const uint8_t *data, uint32_t len)
{
    if (!eventlogSize || !data || !len || eventlogTailAddress >= eventlogSize) {
        return false;
    }

    uint32_t remaining = MIN(len, eventlogSize - eventlogTailAddress);

    while (remaining > 0) {
        if (!flashIsReady()) {
            return false;
        }

        const uint32_t absoluteAddress = eventlogStartAddress + eventlogTailAddress;
        const uint32_t pageRemaining = eventlogFlashGeometry->pageSize - (absoluteAddress % eventlogFlashGeometry->pageSize);
        const uint32_t chunkSize = MIN(remaining, pageRemaining);

        flashPageProgram(absoluteAddress, data, chunkSize, NULL);
        flashWaitForReady();

        eventlogTailAddress += chunkSize;
        data += chunkSize;
        remaining -= chunkSize;
    }

    return true;
}

void eventlogFlashFlush(void)
{
    if (eventlogSize) {
        flashFlush();
    }
}

void eventlogFlashClose(void)
{
    eventlogFlashFlush();
}

uint32_t eventlogFlashGetSize(void)
{
    return eventlogSize;
}

uint32_t eventlogFlashGetUsedSize(void)
{
    return eventlogTailAddress;
}

int eventlogFlashReadAbs(uint32_t offset, uint8_t *data, uint32_t len)
{
    if (!eventlogSize || !data || offset >= eventlogTailAddress) {
        return 0;
    }

    const uint32_t readable = MIN(len, eventlogTailAddress - offset);
    memset(data, 0, len);
    return flashReadBytes(eventlogStartAddress + offset, data, readable);
}

#endif // USE_EVENTLOG && USE_FLASHFS
