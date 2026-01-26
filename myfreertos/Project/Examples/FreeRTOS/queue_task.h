/**
 * queue_task.h
 * Queue task interface: 创建队列与发送接口
 */
#ifndef QUEUE_TASK_H
#define QUEUE_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void KeyQueue_Task(void *pvParameters);

/* 在任务上下文里发送 */
BaseType_t queue_task_send(uint8_t val, TickType_t xTicksToWait);

/* 在 ISR 中发送，传入 xHigherPriorityTaskWoken 指针 */
BaseType_t queue_task_send_from_isr(uint8_t val, BaseType_t *pxHigherPriorityTaskWoken);

#endif /* QUEUE_TASK_H */
