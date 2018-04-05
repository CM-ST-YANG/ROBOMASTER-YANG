#pragma once
#pragma once
#ifndef __OS_TASK_H__
#define __OS_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>

	void os_task_init(void);
	void os_task_start(void);


#ifdef __cplusplus
}
#endif

#endif
