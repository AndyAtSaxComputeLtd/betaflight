#pragma once

#include <stdbool.h>
#include <stdint.h>

bool eventlogFlashInit(void);
bool eventlogFlashErase(void);
bool eventlogFlashWrite(const uint8_t *data, uint32_t len);
void eventlogFlashFlush(void);
void eventlogFlashClose(void);

uint32_t eventlogFlashGetSize(void);
uint32_t eventlogFlashGetUsedSize(void);
int eventlogFlashReadAbs(uint32_t offset, uint8_t *data, uint32_t len);
