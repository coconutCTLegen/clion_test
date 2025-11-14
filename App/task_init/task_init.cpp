//
// Created by 26278 on 25-11-1.
//

#include "task_init.hpp"



void task_init(void)
{
    // 初始化开始，关中断
    __disable_irq();
    // Cap_Ctrl_n::Cap_Ctrl_c::getInstance()->init();          //初始化超级电容
    RoboCmd_c::GetInstance()->RoboCmdInit();
    ChassisCtrl_c::getInstance()->ChassisInit();

    // 初始化完成,开启中断
    __enable_irq();
}
