/** @file eventgrp_handler.h
 *
 * @brief Event group handler module header.
 */

#ifndef EVENTGRP_HANDLER_H
#define EVENTGRP_HANDLER_H

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/**************************************************************************************************
 *    DEFINES
 *************************************************************************************************/
#define EVENT_BIT_0    (1 << 0)
#define EVENT_BIT_1    (1 << 1)

/**************************************************************************************************
 *    GLOBALS
 *************************************************************************************************/
extern EventGroupHandle_t event_group;
extern TaskHandle_t EventTask_Handle;

/**************************************************************************************************
 *    FUNCTIONS
 *************************************************************************************************/
void Event_Task(void *params);
void EventGroup_Init(void);
void EventTask1(void *params);
void EventTask2(void *params);
void EventTask3(void *params);

#endif /* EVENTGRP_HANDLER_H */
