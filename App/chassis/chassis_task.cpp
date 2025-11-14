// namespace Motor_n::MotorBaseDef_n {
//     class MotorBase_c;
// }
// Motor_n::MotorBaseDef_n::MotorBase_c * motorbase;
#include "chassis_task.hpp"

void chassis_task(void const *argument)
{
    while (1) {
        ChassisCtrl_c::getInstance()->loop();
        // motorbase = &ChassisCtrl_c::getInstance()->AGV_Rudder_Motor[0]->get_base();

        Motor_n::DjiMotor_n::DjiMotorControl();
        Motor_n::DjiMotor_n::set_GiveCurrent();
        Motor_n::DjiMotor_n::MotorTransmit();
        vTaskDelay(1);
    }
}

