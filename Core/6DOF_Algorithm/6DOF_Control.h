/**
 * @file 6DOF_Control.h
 * @brief 六自由度机械臂控制头文件
 * @details 包含机械臂运动学计算、控制接口和相关数据结构的定义
 */

#ifndef INC_6DOF_CONTROL_H
#define INC_6DOF_CONTROL_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "DM_motor.h"
#include "ZDT_Control.h"
#include "main.h"
#define RAD_TO_DEG 57.295777754771045f  /**< 弧度转角度：180/π ≈57.2958 */
#define L_BASE_M     0.1015f              // 基座高度（米）
#define D_BASE_M     0.0f                 // 基座偏移（米，实际为0）
#define L_ARM_M      0.093f               // 大臂长度（米）
#define L_FOREARM_M  0.174f               // 小臂长度（米）
#define D_ELBOW_M    0.0f                 // 肘部偏移（米，实际为0）
#define L_WRIST_M    0.088f               // 腕部长度（米）

#define MOTOR1_MIN_LIMIT    -180.0f
#define MOTOR1_MAX_LIMIT     180.0f
#define MOTOR1_MAX_VEL       90.0f          //（°/s）

#define MOTOR2_MIN_LIMIT    -70.0f
#define MOTOR2_MAX_LIMIT     70.0f
#define MOTOR2_MAX_VEL       90.0f

#define MOTOR3_MIN_LIMIT    -90.0f
#define MOTOR3_MAX_LIMIT     50.0f
#define MOTOR3_MAX_VEL       90.0f

#define MOTOR4_MIN_LIMIT    -180.0f
#define MOTOR4_MAX_LIMIT     180.0f
#define MOTOR4_MAX_VEL       90.0f

#define MOTOR5_MIN_LIMIT    -90.0f
#define MOTOR5_MAX_LIMIT     90.0f
#define MOTOR5_MAX_VEL       90.0f

#define MOTOR6_MIN_LIMIT    -180.0f
#define MOTOR6_MAX_LIMIT     180.0f
#define MOTOR6_MAX_VEL       90.0f
/**
 * @enum MOTOR_t
 * @brief 电机枚举类型
 * @details 定义各关节对应的电机编号
 */
typedef enum MOTOR_NAME {
    MOTOR_1 = DAM_MOTOR1,  /**< 关节1电机 - 达妙电机1 */
    MOTOR_2 = DAM_MOTOR2,  /**< 关节2电机 - 达妙电机2 */
    MOTOR_3 = DAM_MOTOR3,  /**< 关节3电机 - 达妙电机3 */
    MOTOR_4 = ZDT_MOTOR1,  /**< 关节4电机 - 张大头电机1 */
    MOTOR_5 = ZDT_MOTOR2,  /**< 关节5电机 - 张大头电机2 */
    MOTOR_6 = ZDT_MOTOR3,  /**< 关节6电机 - 张大头电机3 */
} MOTOR_t;

/**
 * @struct Joint6D_t
 * @brief 六关节角度结构体
 * @details 存储六个关节的角度值，单位为度(°)
 */
typedef struct {
    float a[6];  /**< J1~J6 关节角度数组，单位：度 */
} Joint6D_t;

/**
 * @struct Pose6D_t
 * @brief 六维位姿结构体
 * @details 包含末端执行器的位置和姿态信息
 */
typedef struct {
    float X, Y, Z;    /**< 位置坐标，单位：毫米 */
    float A, B, C;    /**< 欧拉角姿态，单位：度，XYZ旋转顺序 */
    float R[9];       /**< 3x3 旋转矩阵，按行优先存储 */
    bool hasR;        /**< 标志位：是否已计算旋转矩阵 */
} Pose6D_t;

/**
 * @struct ArmConfig_t
 * @brief 机械臂结构参数配置
 * @details 基于DH参数法的机械臂连杆结构参数定义
 */
typedef struct {
    float L_BASE;     /**< 基座长度，单位：米 */
    float D_BASE;     /**< 基座偏移，单位：米 */
    float L_ARM;      /**< 大臂长度，单位：米 */
    float L_FOREARM;  /**< 小臂长度，单位：米 */
    float D_ELBOW;    /**< 肘部偏移，单位：米 */
    float L_WRIST;    /**< 腕部长度，单位：米 */
    float DH[6][4];   /**< 6关节DH参数矩阵：[theta_home, d, a, alpha] */
} ArmConfig_t;

/**
 * @struct DOF6Kinematic
 * @brief 六自由度运动学结构体
 * @details 存储机械臂运动学计算所需的所有参数和中间变量
 */
typedef struct {
    ArmConfig_t armConfig;  /**< 机械臂结构配置 */
    float DH_matrix[6][4];  /**< 当前DH参数矩阵 */
    float L1_base[3];       /**< 基座连杆向量，单位：米 */
    float L2_arm[3];        /**< 大臂连杆向量，单位：米 */
    float L3_elbow[3];      /**< 肘部连杆向量，单位：米 */
    float L6_wrist[3];      /**< 腕部连杆向量，单位：米 */
    float l_se_2;           /**< 大臂长度平方，用于逆运动学计算 */
    float l_se;             /**< 大臂长度，单位：米 */
    float l_ew_2;           /**< 前臂有效长度平方（考虑肘部偏移） */
    float l_ew;             /**< 前臂有效长度，单位：米 */
    float atan_e;           /**< 肘部偏移角度：atan(D_ELBOW/L_FOREARM) */
} DOF6Kinematic;

/**
 * @struct IKSolves_t
 * @brief 逆运动学解集结构体
 * @details 存储逆运动学计算得到的所有可能关节角解
 */
typedef struct {
    Joint6D_t config[8];   /**< 8组可能的关节角解 */
    char solFlag[8][3];    /**< 解的有效性标志矩阵，8行对应8组解，3列分别对应：
                            * [0]: J1有效性 (1有效, 0无效, -1奇异)
                            * [1]: J2,J3有效性
                            * [2]: J4,J5,J6有效性 */
} IKSolves_t;


bool Joint6D_CheckLimit(const Joint6D_t* joints, const float* min_angle, const float* max_angle);
bool IKSolves_SelectBest(const IKSolves_t* solves, const Joint6D_t* current_joints,const float* min_angle, const float* max_angle, Joint6D_t* best_joints);
float Joint6D_CalcSyncVel(const Joint6D_t* current_joints, const Joint6D_t* target_joints,const float* max_vel, float* target_vel);
bool DOF6_MoveJ(DOF6Kinematic* kinematic, const Joint6D_t* current_joints,const Joint6D_t* target_joints, const float* max_vel,const float* min_angle, const float* max_angle);
bool DOF6_MoveL(DOF6Kinematic* kinematic, const Joint6D_t* current_joints,const Pose6D_t* target_pose, const float* max_vel,const float* min_angle, const float* max_angle);
bool Joint6D_ReadFromMotor(Joint6D_t* current_joints);
void MOTOR_SetPosVal(MOTOR_t motor, float target_pos, float target_vel);
void DOF6Kinematic_Init(DOF6Kinematic* kinematic, float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);
bool DOF6_SolveFK(const DOF6Kinematic* kinematic, const Joint6D_t* _inputJoint6D, Pose6D_t* _outputPose6D);
bool DOF6_SolveIK(DOF6Kinematic* kinematic, const Pose6D_t* _inputPose6D,const Joint6D_t* _lastJoint6D, IKSolves_t* _outputSolves);
Joint6D_t Joint6D_Subtract(const Joint6D_t* joints1, const Joint6D_t* joints2);
void Joints_FK(float angle1,float angle2,float angle3,float angle4,float angle5,float angle6);
void Joints_IK(float x,float y,float z) ;
#endif // INC_6DOF_CONTROL_H