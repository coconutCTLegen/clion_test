#include "chassis_ctrl.hpp"

// 创建单例
ChassisCtrl_c ChassisCtrl_c::ChassisInstance = ChassisCtrl_c();
namespace MM = Motor_n::MotorBaseDef_n;

void ChassisCtrl_c::ChassisInit()
{
    /* ------------------------裁判系统框架测试用----------------------------*/
    rc_text = Dr16_n::dr16_c::GetInstance();
    referee_text = Referee_n::referee_c::GetInstance();
    robocmd_text = RoboCmd_c::GetInstance();
    /* ------------------------裁判系统框架测试用----------------------------*/

     MM::Motor_Base_Config_t WheelConfig = MM::Motor_Base_Config_t("WheelMotor", MM::Motor_Type_euc::M3508)
     .SetControlSetting(MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::SPEED_LOOP})
     .SetPIDConfig(
     alg_n::PidInitConfig_t{
            .Kp = CHASSIS_M3508_PositionLock_Kp,
            .Ki = CHASSIS_M3508_PositionLock_Ki,
            .Kd = CHASSIS_M3508_PositionLock_Kd,
            },
       alg_n::PidInitConfig_t{
           .Kp = 10.0f,
           .Ki = 0.0f,
           .Kd = 0.0f,
           .mode = OutputFilter | Integral_Limit | StepIn,
           .max_Ierror = 1000,
           .out_filter_num = 0.5,
           .stepIn = 30,
       },
       alg_n::PidInitConfig_t{
         .Kp = 0.0f,
         .Ki = 0.0f,
         .Kd = 0.0f,
         .Kfa = 0.0f,
         .Kfb = 0.0f,
         .ActualValueSource = nullptr})
     .SetCANConfig(BSP_n::CanInitConfig_s{
       .fdcan_handle = &hfdcan2,
         })
     .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{
       .zero_offset = 0.0f,
       .radius = 0.0f,
       .ecd2length = 0.0f,
       .ratio = 1.0f})
     .SetOutputLimit(Current_limit_H_3508, Current_limit_L_3508);

    MM::Motor_Base_Config_t SteerConfig = MM::Motor_Base_Config_t("SteerMotor", MM::Motor_Type_euc::GM6020)
     .SetControlSetting(MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::ANGLE_AND_SPEED_LOOP})
     .SetPIDConfig(
     alg_n::PidInitConfig_t{
            .Kp = 15.0f,
            .Ki = 1.0f,
            .Kd = 0.0f,
            .mode = Integral_Limit,
            .max_Ierror = 50,
            },
       alg_n::PidInitConfig_t{
           .Kp = 50.0f,
           .Ki = 0.0f,
           .Kd = 0.0f,
           .mode = OutputFilter | StepIn,
           .max_Ierror = 1000,
           .out_filter_num = 0.2,
           .stepIn = 100,
       },
       alg_n::PidInitConfig_t{
         .Kp = 0.0f,
         .Ki = 0.0f,
         .Kd = 0.0f,
         .Kfa = 0.0f,
         .Kfb = 0.0f,
         .ActualValueSource = nullptr})
     .SetCANConfig(BSP_n::CanInitConfig_s{
       .fdcan_handle = &hfdcan1,
         })
     .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{
       .zero_offset = 0.0f,
       .radius = 0.0f,
       .ecd2length = 0.0f,
       .ratio = 1.0f})
     .SetOutputLimit(Voltage_limit_H_6020, Voltage_limit_L_6020);

 /* 设置电调ID */
    WheelConfig.can_init_config.tx_id = 1;
    AGV_Drive_Motor[LF] = new Motor_n::DjiMotor_n::DjiDriver_c(WheelConfig);
    WheelConfig.can_init_config.tx_id = 2;
    AGV_Drive_Motor[LB] = new Motor_n::DjiMotor_n::DjiDriver_c(WheelConfig);
    WheelConfig.can_init_config.tx_id = 3;
    AGV_Drive_Motor[RB] = new Motor_n::DjiMotor_n::DjiDriver_c(WheelConfig);
    WheelConfig.can_init_config.tx_id = 4;
    AGV_Drive_Motor[RF] = new Motor_n::DjiMotor_n::DjiDriver_c(WheelConfig);

    SteerConfig.can_init_config.tx_id = 1;
    SteerConfig.zero_offset = 6787;
    AGV_Rudder_Motor[LF] = new Motor_n::DjiMotor_n::DjiDriver_c(SteerConfig);
    AGV_Rudder_Motor[LF]->motor_controller_.SetAngleFeedbackPtr(nullptr);
    SteerConfig.can_init_config.tx_id = 2;
    SteerConfig.zero_offset = 6818;
    AGV_Rudder_Motor[LB] = new Motor_n::DjiMotor_n::DjiDriver_c(SteerConfig);
    AGV_Rudder_Motor[LB]->motor_controller_.SetAngleFeedbackPtr(nullptr);
    SteerConfig.can_init_config.tx_id = 3;
    SteerConfig.zero_offset = 660;
    AGV_Rudder_Motor[RB] = new Motor_n::DjiMotor_n::DjiDriver_c(SteerConfig);
    AGV_Rudder_Motor[RB]->motor_controller_.SetAngleFeedbackPtr(nullptr);
    SteerConfig.can_init_config.tx_id = 4;
    SteerConfig.zero_offset = 6578;
    AGV_Rudder_Motor[RF] = new Motor_n::DjiMotor_n::DjiDriver_c(SteerConfig);
    AGV_Rudder_Motor[RF]->motor_controller_.SetAngleFeedbackPtr(nullptr);

    ChassisMotorEnable();

    /*功率控制初始化*/
    // std::vector<const Motor_n::DjiMotor_n::DjiDriver_c *> motors_;        //创建一个电机向量，用于储存底盘所有大疆电机的实例
    // //轮电机
    // motors_.push_back(AGV_Drive_Motor[LF]);
    // motors_.push_back(AGV_Drive_Motor[LB]);
    // motors_.push_back(AGV_Drive_Motor[RB]);
    // motors_.push_back(AGV_Drive_Motor[RF]);
    // //舵电机
    // motors_.push_back(AGV_Rudder_Motor[LF]);
    // motors_.push_back(AGV_Rudder_Motor[LB]);
    // motors_.push_back(AGV_Rudder_Motor[RB]);
    // motors_.push_back(AGV_Rudder_Motor[RF]);
    //
    // Core::Control::Power::Manager manager(motors_, Core::Control::Power::Division::SENTRY);// 功率管理器实例
    // Core::Control::Power::init(manager);
}

void ChassisCtrl_c::loop()
{
    ChassisMotorEnable();
    CorrectChassisRudderZeroOffset();
    ChassisBehaviorChoose();
    ChassisModeCalculation();
}

void ChassisCtrl_c::ChassisMotorEnable()
{
     AGV_Drive_Motor[LF]->Enable();
     AGV_Drive_Motor[LB]->Enable();
     AGV_Drive_Motor[RB]->Enable();
     AGV_Drive_Motor[RF]->Enable();
     AGV_Rudder_Motor[LF]->Enable();
     AGV_Rudder_Motor[LB]->Enable();
     AGV_Rudder_Motor[RB]->Enable();
     AGV_Rudder_Motor[RF]->Enable();
}

void ChassisCtrl_c::ChassisModeChangeClearPid()
{
    for (int i = 0; i < 4; i++)
    {
        AGV_Drive_Motor[i]->motor_data_.clear();
        AGV_Rudder_Motor[i]->motor_data_.clear();

        AGV_Drive_Motor[i]->motor_controller_.angle_PID->Clear();
        AGV_Rudder_Motor[i]->motor_controller_.angle_PID->Clear();

        AGV_Drive_Motor[i]->motor_controller_.speed_PID->Clear();
        AGV_Rudder_Motor[i]->motor_controller_.speed_PID->Clear();
    }
}

/**
 * 舵轮电机零点修正
 * @note 该函数需要在底盘电机初始化之后调用一次，目的就是为了修正舵轮电机的零点
 *       该函数会将舵轮电机的当前角度进行计算，并存储在Calibrated_Rudder_Angle数组中
 */
void ChassisCtrl_c::CorrectChassisRudderZeroOffset()
{
    double Current_Angle;          //当前角度
    int16_t  Encoder;             //修正的编码值

    for (int i = 0; i < 4; i++)
    {

        Encoder = AGV_Rudder_Motor[i]->motor_data_.motor_raw_data.feedback_ecd - Rudder_Offset_[i];

        if (Encoder < 0)
            Encoder += 8192;

        Current_Angle = (double)(Encoder * 360.0f / 8192.0f);

        Calibrated_Rudder_Angle_[i] = Current_Angle;
    }
}

    /**
     * 底盘模式选择
     * @note 该函数需要在底盘控制任务中调用
     *       该函数会根据遥控器的开关位置选择底盘的工作模式
     *       该函数会调用SwitchChassisMode函数进行模式切换
     */
    void ChassisCtrl_c::ChassisBehaviorChoose()
    {
        static ChassisBehavior_e Behavior;
        /*手柄----------------------------------------------------------------------------- */
        /*左上开关*/
        switch (this->robocmd_text->dt7_data_->s1)
        {
            case DT7_SW_MID:
                Behavior = ChassisOmniMove;       //全向移动
                break;
            default:   //数据有误直接无力
                Behavior = ChassisZeroForce;       //无力模式
                break;
        }
    
        // 防止拨杆拨到一点点，让哨兵乱撞
        if ((((user_abs(this->robocmd_text->dt7_data_->ch[2])) < 5) && ((user_abs(this->robocmd_text->dt7_data_->ch[3])) < 5) &&
                 (this->robocmd_text->dt7_data_->s1 != DT7_SW_UP) && (this->robocmd_text->dt7_data_->s1 != DT7_SW_DOWN) && (this->robocmd_text->dt7_data_->s2 != DT7_SW_MID) &&
                 (this->robocmd_text->dt7_data_->s2 != DT7_SW_DOWN) &&
                 ((user_abs(AGV_Drive_Motor[LF]->motor_data_.motor_raw_data.feedback_ecd) + user_abs(AGV_Drive_Motor[LB]->motor_data_.motor_raw_data.feedback_ecd) + user_abs(AGV_Drive_Motor[RB]->motor_data_.motor_raw_data.feedback_ecd) + user_abs(AGV_Drive_Motor[RF]->motor_data_.motor_raw_data.feedback_ecd)) < 7500)))
        {
            Behavior = ChassisPositionLock; // 底盘自锁
        }

        SwitchChassisMode(Behavior);
    }

    /**
     * 底盘模式切换
     * @note 该函数会根据传入的模式参数进行模式切换
     *       该函数会根据不同的模式进行不同的初始化操作
     *       该函数会调用ChassisModeChangeClearPid函数清除PID数据
     *       该函数会根据不同的模式设置不同的PID参数
     *       该函数会根据不同的模式设置不同的电机状态
     */
    void ChassisCtrl_c::SwitchChassisMode(ChassisBehavior_e behavior)
    {
        if (behavior != Rc_Behavior)   //如果这次的底盘模式和哨兵现在执行的模式不一样
        {
            if (behavior == ChassisPositionLock)     //如果底盘现在要自锁
            {
                ChassisModeChangeClearPid();  // 清除PID数据
                /*3508 自锁最外层变换并添加角度环   6020 位置环的反馈设置为MOTOR_FEED  */
                for (int i = 0; i < 4; i++)
                {
                    // 修改位置环的电机反馈类型
                    AGV_Rudder_Motor[i]->motor_controller_.SetAngleFeedbackPtr(&AGV_Rudder_Motor[i]->motor_data_.motor_processed_data.total_ecd); //舵电机用电机反馈过来的角度进行自锁
                    AGV_Rudder_Motor[i]->motor_controller_.speed_PID->GetMeasure()->Kp = 0.5;
                    Target_Speed_[i] = 0; //自锁时速度设为0
                    AGV_Drive_Motor[i]->DjiMotorSelfLock();
                    AGV_Rudder_Motor[i]->DjiMotorSelfLock();
                }
            }

            //之前是自锁要解锁
            if (Rc_Behavior == ChassisPositionLock)
            {
                ChassisModeChangeClearPid();  // 清除PID数据
                /*3508 删除角度环、取消自锁，设置状态为Enable   6020 位置环反馈设置成NONE_FEED  */
                for (int i = 0; i < 4; i++)
                {
                    AGV_Rudder_Motor[i]->motor_controller_.SetAngleFeedbackPtr(nullptr);         //舵电机用之前已经计算好的目标之间的差值即目标角度-实际角度
                    AGV_Rudder_Motor[i]->motor_controller_.speed_PID->GetMeasure()->Kp = 50;
                    AGV_Drive_Motor[i]->DjiMotorClearSelfLock();
                    AGV_Rudder_Motor[i]->DjiMotorClearSelfLock();
                }
            }
        }
        Rc_Behavior = behavior;
    }

/**
 * 底盘模式计算
 * @note 该函数需要在底盘控制任务中调用
 *       该函数会根据当前的底盘模式进行不同的计算操作
 *       该函数会调用ChassisSpeedPidCalculate函数进行速度计算
 *       该函数会根据不同的模式调用不同的行为函数
 *       该函数会将计算得到的目标速度和角度发送给电机
 */
void ChassisCtrl_c::ChassisModeCalculation()
{
    Chassis_Rc_x_ =  static_cast<float>(robocmd_text->dt7_data_->ch[2]) ; //-660~660  左右
    Chassis_Rc_y_ = - static_cast<float>(robocmd_text->dt7_data_->ch[3]) ; //-660~660 前后

    user_value_limit(Chassis_Rc_x_, -660.0f, 660.0f);
    user_value_limit(Chassis_Rc_y_, -660.0f, 660.0f);

    Chassis_Vx_ = Chassis_Rc_x_*13.63f;
    Chassis_Vy_ = Chassis_Rc_y_*13.63f;

    switch (Rc_Behavior)
    {
        case ChassisZeroForce:
            F_Chassis_Zero_Force();
            break;
        case ChassisOmniMove:
            F_Chassis_Omni_Move();
            break;
        default:
            break;
    }

    for (int i = 0; i < 4; i++)
    {
        AGV_Drive_Motor[i]->DjiMotorSetRef( Target_Parameters[i].Vh);
        AGV_Rudder_Motor[i]->DjiMotorSetRef( Target_Parameters[i].Wheel_Angle);
    }
}


/***************************************************** 哨兵哨兵底盘用户层文件****************************************************
*                                                     数学解算文件❤❤❤                                                      */

    // 速度合成  舵轮解算核心速度代码
    void ChassisCtrl_c::Calculate_FourWheel_Vh(WheelParameters_t *Target, float VX, float VY, float VW)
    {
        Target[LF].Vh = sqrt(pow(VX - VW * 0.707107f, 2) + pow(VY - VW * 0.707107f, 2)); // 左前
        Target[LB].Vh = sqrt(pow(VX + VW * 0.707107f, 2) + pow(VY - VW * 0.707107f, 2)); // 左后
        Target[RB].Vh = sqrt(pow(VX - VW * 0.707107f, 2) + pow(VY + VW * 0.707107f, 2)); // 右后
        Target[RF].Vh = sqrt(pow(VX + VW * 0.707107f, 2) + pow(VY + VW * 0.707107f, 2)); // 右前
    }

    void ChassisCtrl_c::Calculate_FourWheel_Angle(double *target_angle, float VX, float VY, float VW)
    {
        target_angle[LF] = atan2(-(VX - VW * 0.707107f), (VY - VW * 0.707107f)) * 57.29577951308232f; // 左前
        target_angle[LB] = atan2(-(VX + VW * 0.707107f), (VY - VW * 0.707107f)) * 57.29577951308232f; // 左后
        target_angle[RB] = atan2(-(VX - VW * 0.707107f), (VY + VW * 0.707107f)) * 57.29577951308232f; // 右前
        target_angle[RF] = atan2(-(VX + VW * 0.707107f), (VY + VW * 0.707107f)) * 57.29577951308232f; // 右后
    }


    void ChassisCtrl_c::Angle_to_0_360(double *angle)
    {
        for (int i = 0; i < 4; i++)
        {
            if (angle[i] < 0)
            {
                angle[i] += 360.0f;
            }
            else if (angle[i] >= 360.0f)
            {
                angle[i] -= 360.0f;
            }
        }
    }

    // 将角度差值进行180度回环处理
    double ChassisCtrl_c::Angle_Reduce_0_180(double D_value)
    {
        while (D_value > 180)
        {
            D_value -= 360;
        }
        while (D_value < -180)
        {
            D_value += 360;
        }
        return D_value;
    }

    // 选择角度调整的最佳路径
    double ChassisCtrl_c::Angle_Reduce_0_90(double D_value)
    {
        if (D_value > 90)
        {
            D_value = D_value - 180;
        }

        if (D_value < -90)
        {
            D_value = D_value + 180;
        }
        return D_value;
    }

    /**
     *  无力模式
     *  直接将电机关闭，设定值也为0
     */
    void ChassisCtrl_c::F_Chassis_Zero_Force()
    {
        for (int i = 0; i < MotorsPerWheelGroup; i++)
        {
            AGV_Drive_Motor[i]->Disable();     //只是关了标志位
            AGV_Rudder_Motor[i]->Disable();
            AGV_Drive_Motor[i]->DjiMotorSetRef(0);   //以防万一，先写一个0
            AGV_Rudder_Motor[i]->DjiMotorSetRef(0);
        }
    }

    /**
     *  全向移动模式
     *  无旋转角速度，目前未加上云台差角，所以不能实现坐标转换
     */
    void ChassisCtrl_c::F_Chassis_Omni_Move()
    {
        double D_Angle[MotorsPerWheelGroup];
        double Mid_Angle [MotorsPerWheelGroup];
        Calculate_FourWheel_Vh(Target_Parameters, Chassis_Vx_, Chassis_Vy_, 0);
        Calculate_FourWheel_Angle(Target_Angle_, Chassis_Vx_, Chassis_Vy_, 0);

        Angle_to_0_360(Target_Angle_);       //转化为0~360

        for (int i = 0; i < 4; i++)
        {
            D_Angle[i] = Target_Angle_[i] - Calibrated_Rudder_Angle_[i];
            D_Angle[i] = Angle_Reduce_0_180(D_Angle[i]);
            Mid_Angle[i] = Angle_Reduce_0_90(D_Angle[i]);

            if (IsReverseCurrent_[i] == false && D_Angle[i] != Mid_Angle[i])    //如果当前电机现在没有反转，但是角度不一样，说明需要反转
            {
                IsReverseCurrent_[i] = true;
                if(AGV_Drive_Motor[i]->motor_controller_.motor_setting.motor_reverse_flag == MM::Motor_Reverse_Flag_e::MOTOR_DIRECTION_NORMAL)
                {
                    AGV_Drive_Motor[i]->motor_controller_.motor_setting.motor_reverse_flag = MM::Motor_Reverse_Flag_e::MOTOR_DIRECTION_REVERSE;
                }
                else
                {
                    AGV_Drive_Motor[i]->motor_controller_.motor_setting.motor_reverse_flag == MM::Motor_Reverse_Flag_e::MOTOR_DIRECTION_NORMAL;
                }
            }

            if (IsReverseCurrent_[i] == true && D_Angle[i] == Mid_Angle[i])     //如果当前电机现在是反向电流，但是角度一样，说明不需要反转
            {
                IsReverseCurrent_[i] = false;
                if(AGV_Drive_Motor[i]->motor_controller_.motor_setting.motor_reverse_flag == MM::Motor_Reverse_Flag_e::MOTOR_DIRECTION_NORMAL)
                {
                    AGV_Drive_Motor[i]->motor_controller_.motor_setting.motor_reverse_flag = MM::Motor_Reverse_Flag_e::MOTOR_DIRECTION_REVERSE;
                }
                else
                {
                    AGV_Drive_Motor[i]->motor_controller_.motor_setting.motor_reverse_flag == MM::Motor_Reverse_Flag_e::MOTOR_DIRECTION_NORMAL;
                }
            }

            Target_Parameters[i].Wheel_Angle = Mid_Angle[i];     //最终给PID的参数
            if (AGV_Drive_Motor[i]->motor_controller_.motor_setting.motor_reverse_flag == MM::Motor_Reverse_Flag_e::MOTOR_DIRECTION_REVERSE)
            {
                Target_Speed_[i] = -Target_Parameters[i].Vh;   //速度反转 仅调试时方便观看，不参与计算
            }
            else
            {
                Target_Speed_[i] = Target_Parameters[i].Vh;   //速度正转 仅调试时方便观看，不参与计算
            }
        }
    }


