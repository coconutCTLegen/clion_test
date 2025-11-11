//
// Created by 26278 on 25-11-1.
//

#include "chassis_task.hpp"

void chassis_task(void const *argument)
{
    while (1) {
        ChassisCtrl_c::getInstance()->ChassisMotorEnable();
        ChassisCtrl_c::getInstance()->ChassisBehaviorChoose();
        ChassisCtrl_c::getInstance()->ChassisModeCalculation();

        Motor_n::DjiMotor_n::DjiMotorControl();
        // Motor_n::DjiMotor_n::set_GiveCurrent();
        // Motor_n::DjiMotor_n::MotorTransmit();
        vTaskDelay(1);
    }
}

