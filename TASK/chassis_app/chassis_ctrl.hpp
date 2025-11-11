#ifndef CHASSIS_CTRL_HPP
#define CHASSIS_CTRL_HPP

#include "dji_driver.hpp"
#include "Alg_PID.hpp"
#include "filter.hpp"
#include "safe_task.hpp"
#include "robo_cmd.hpp"
#ifdef STM32H723xx
#include "bsp_fdcan.hpp"
#else
#include "bsp_can.hpp"
#endif

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "string.h"
#ifdef __cplusplus
}

/************遥控速度pid************/
// 9000(电机最大转速) / 660 (摇杆最大值) = 13.63
/* X方向 ----------------------------------------------------*/
#define CHASSIS_SPEED_X_KP 3600 //导航控制需要改成这个值
#define CHASSIS_SPEED_X_KI 0.0f
#define CHASSIS_SPEED_X_KD 0.0f
/* Y方向 ----------------------------------------------------*/
#define CHASSIS_SPEED_Y_KP 3600 //导航控制需要改成这个值
#define CHASSIS_SPEED_Y_KI 0.0f
#define CHASSIS_SPEED_Y_KD 0.0f
/* 低通滤波比例 ----------------------------------------------*/
#define DT7_FIRST_ORDER_FILTER_num 0.01f//1.0f//0.0410f // 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高

// 3508 自锁用的PID参数
#define CHASSIS_M3508_PositionLock_Kp 0.02f
#define CHASSIS_M3508_PositionLock_Ki 0.0f
#define CHASSIS_M3508_PositionLock_Kd 0.0f


// 功率控制相关参数

#define RLS_K1 0.251f
#define RLS_K2 1.784f
#define RLS_K3 7.03f


// 机器人底盘修改的参数,单位为mm(毫米)
// 轮距 正方形:277毫米  外围直径：540 毫米的圆    轮子半径：60毫米
#define WHEEL_BASE   277f              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH  277f             // 横向轮距(左右平移方向)
#define RADIUS_WHEEL 60              // 轮子半径
// 电机电流值限幅
#define MOTOR_3508_CURRENT_LIMIT 16000 //3508最大电流值
#define MOTOR_6020_CURRENT_LIMIT 22000 //6020最大电压值(6020是电压控制的)
#define MOTOR_3508_SPEED_LIMIT   9000  //3508最大速度值
#define MOTOR_6020_SPEED_LIMIT   320   //6020最大速度值


class ChassisCtrl_c {
public:
    /*底盘轮组编号*/
    typedef enum{
        LF = 0,
        LB = 1,
        RB = 2,
        RF = 3,
        MotorsPerWheelGroup = 4
    }WheelNum_e;

    //舵轮轮组信息结构体
    typedef struct
    {
        float  Vh;               //沿轮子方向的合速度
        float Wheel_Angle;       //轮子的角度差值，PID的error
    }WheelParameters_t;

    /*底盘模式*/
    typedef enum
    {
        ChassisZeroForce,     // 无力模式
        ChassisOmniMove,      // 全向移动
        ChassisPositionLock   // 自锁
    } ChassisBehavior_e;

public:
    Motor_n::DjiMotor_n::DjiDriver_c *AGV_Drive_Motor[MotorsPerWheelGroup];    //底盘轮驱动电机 DJI-3508
    Motor_n::DjiMotor_n::DjiDriver_c *AGV_Rudder_Motor[MotorsPerWheelGroup];
    WheelParameters_t Target_Parameters[MotorsPerWheelGroup];
    double Target_Angle_[MotorsPerWheelGroup]; //目标角度  仅调试时方便观看，不参与计算
    float Target_Speed_[MotorsPerWheelGroup]; //目标速度   仅调试时方便观看，不参与计算
    bool IsReverseCurrent_[MotorsPerWheelGroup] = {0,0,0,0}; //电机是否反转标志   1表示现在是反向电流
    /* ------------------------裁判系统框架测试用----------------------------*/
    Dr16_n::dr16_c *rc;                             //遥控器指针
    Referee_n::referee_c *referee_text;             //框架测试用的裁判系统指针
    RoboCmd_c *robocmd_text;                        //框架测试用的机器人命令指针
    /* ------------------------裁判系统框架测试用----------------------------*/
    float Chassis_Rc_x_ = 0.0f;                      //原始遥控器关于底盘X和Y方向上的值*/
    float Chassis_Rc_y_ = 0.0f;
    uint16_t Rudder_Offset_[4] = {6787,6818,660,6578};            //舵电机相对于0点位置的偏移量
    double Calibrated_Rudder_Angle_[4];                           //校准后的舵电机角度
    ChassisBehavior_e Rc_Behavior ;               //上一次底盘模式
    /*底盘目标速度（19:1）RPM   用于计算速度  */
    float Chassis_Vx_ = 0.0f;    //底盘x方向速度
    float Chassis_Vy_ = 0.0f;
    float Chassis_Omega_ = 0.0f; //底盘角速度

    user_maths_c math;

public:
    void ChassisInit();
    void ChassisMotorEnable();
    void ChassisModeChangeClearPid();
    void CorrectChassisRudderZeroOffset();
    void ChassisBehaviorChoose();
    void SwitchChassisMode(ChassisBehavior_e behavior);
    void ChassisModeCalculation();

    void F_Chassis_Omni_Move();
    void Calculate_FourWheel_Vh(WheelParameters_t *Target, float VX, float VY, float VW);
    void Calculate_FourWheel_Angle(double *target_angle, float VX, float VY, float VW);
    void Angle_to_0_360(double *angle);
    double Angle_Reduce_0_180(double D_value);
    double Angle_Reduce_0_90(double D_value);
    void F_Chassis_Zero_Force();

    static ChassisCtrl_c *getInstance()
    {
        return &ChassisInstance; // 返回静态成员变量 instance 的地址
    }
public:
    ChassisCtrl_c() = default; // 私有构造函数
    static ChassisCtrl_c ChassisInstance;//唯一实例指针
};


#endif

#endif
