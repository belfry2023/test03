#include "startup.h"
#include "bsp_dwt.h"
#include "car_task.h"
#include "cmd.h"
#include "car.h"

void CarInit()
{  
    __disable_irq();
    
    DWT_Init(168);
    cmd_task_init();
	car_task_init();
    OSTaskInit();

    __enable_irq();
}

void CarTask()
{
    cmd_task();
    car_task();
}