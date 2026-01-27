/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#ifndef FREERTOS_APP_HOOKS_H
#define FREERTOS_APP_HOOKS_H

#ifdef __cplusplus
extern "C" {
#endif
#include "mcu.h"
#include "FreeRTOS.h"
#include "task.h"

void vHeapRegionsInt(void);
// 堆疊溢出 Hook
void __attribute__((weak)) vApplicationStackOverflowHook(TaskHandle_t xTask,
                                                         char* pcTaskName);

// 記憶體分配失敗 Hook
void __attribute__((weak)) vApplicationMallocFailedHook(void);

// Idle 任務 Hook
void __attribute__((weak)) vApplicationIdleHook(void);

// Tick Hook（需 configUSE_TICK_HOOK 設為 1）
#if (configUSE_TICK_HOOK != 0)
void __attribute__((weak)) vApplicationTickHook(void);
#endif

// Tickless idle 模式的睡眠函式（需 configUSE_TICKLESS_IDLE 設為 1）
#if (configUSE_TICKLESS_IDLE != 0)
void __attribute__((weak)) vApplicationSleep(TickType_t xExpectedIdleTime);
#endif

// 提供 Idle 任務的記憶體空間（需 configSUPPORT_STATIC_ALLOCATION = 1）
void __attribute__((weak))
vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                              StackType_t** ppxIdleTaskStackBuffer,
                              uint32_t* pulIdleTaskStackSize);

// 提供 Timer 任務的記憶體空間（需 configSUPPORT_STATIC_ALLOCATION && configUSE_TIMERS = 1）
void __attribute__((weak))
vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                StackType_t** ppxTimerTaskStackBuffer,
                                uint32_t* pulTimerTaskStackSize);

// vAssertCalled 用於除錯（可選）
void __attribute__((weak)) vAssertCalled(const char* const pcFileName,
                                         unsigned long ulLine);



uint8_t __attribute__((weak)) SystemReadyToSleep(void);


#ifdef __cplusplus
}
#endif

#endif /* FREERTOS HOOK */