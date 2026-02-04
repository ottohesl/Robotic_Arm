/**
 * @file 6DOF_Control.c
 * @brief 六自由度机械臂控制实现文件
 * @details 包含正逆运动学计算、电机控制等具体实现
 */

#include "6DOF_Control.h"

/**
 * @brief 优化三角函数实现（使用ARM CMSIS-DSP库）
 * @param x 角度值，弧度
 * @return 余弦值
 */
inline float cosf(float x) { return arm_cos_f32(x); }

/**
 * @brief 优化三角函数实现（使用ARM CMSIS-DSP库）
 * @param x 角度值，弧度
 * @return 正弦值
 */
inline float sinf(float x) { return arm_sin_f32(x); }

/**
 * @brief 矩阵乘法运算
 * @param _matrix1 输入矩阵1，维度为 _m × _l
 * @param _matrix2 输入矩阵2，维度为 _l × _n
 * @param _matrixOut 输出矩阵，维度为 _m × _n
 * @param _m 矩阵1的行数
 * @param _l 矩阵1的列数（矩阵2的行数）
 * @param _n 矩阵2的列数
 * @return void
 * @note 输出矩阵必须预先分配足够的内存空间
 */
static void MatMultiply(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _l, const int _n)
{
    float tmp;
    int i, j, k;
    
    for (i = 0; i < _m; i++) {
        for (j = 0; j < _n; j++) {
            tmp = 0.0f;
            for (k = 0; k < _l; k++) {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}

/**
 * @brief 旋转矩阵转换为欧拉角（XYZ顺序）
 * @param _rotationM 输入旋转矩阵（3×3，行优先）
 * @param _eulerAngles 输出欧拉角数组[Z, Y, X]，单位：弧度
 * @return void
 * @note 使用XYZ固定角顺序（横滚-俯仰-偏航）
 */
static void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles)
{
    float A, B, C, cb;

    /* 处理俯仰角为±90°的奇异情况 */
    if (fabs(_rotationM[6]) >= 1.0f - 0.0001f) {
        if (_rotationM[6] < 0) {  /* 俯仰角为+90° */
            A = 0.0f;
            B = (float)M_PI_2;
            C = atan2f(_rotationM[1], _rotationM[4]);
        } else {  /* 俯仰角为-90° */
            A = 0.0f;
            B = -(float)M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    } else {  /* 非奇异情况 */
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        cb = cosf(B);
        A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }

    /* 输出欧拉角：C=偏航(Z), B=俯仰(Y), A=横滚(X) */
    _eulerAngles[0] = C;
    _eulerAngles[1] = B;
    _eulerAngles[2] = A;
}

/**
 * @brief 欧拉角转换为旋转矩阵（XYZ顺序）
 * @param _eulerAngles 输入欧拉角数组[Z, Y, X]，单位：弧度
 * @param _rotationM 输出旋转矩阵（3×3，行优先）
 * @return void
 * @note 使用XYZ固定角顺序（横滚-俯仰-偏航）
 */
static void EulerAngleToRotMat(const float* _eulerAngles, float* _rotationM)
{
    float ca, cb, cc, sa, sb, sc;

    /* 计算各角度的三角函数值 */
    cc = cosf(_eulerAngles[0]);  /* 偏航角cos */
    cb = cosf(_eulerAngles[1]);  /* 俯仰角cos */
    ca = cosf(_eulerAngles[2]);  /* 横滚角cos */
    sc = sinf(_eulerAngles[0]);  /* 偏航角sin */
    sb = sinf(_eulerAngles[1]);  /* 俯仰角sin */
    sa = sinf(_eulerAngles[2]);  /* 横滚角sin */

    /* 计算旋转矩阵元素 */
    _rotationM[0] = ca * cb;
    _rotationM[1] = ca * sb * sc - sa * cc;
    _rotationM[2] = ca * sb * cc + sa * sc;
    _rotationM[3] = sa * cb;
    _rotationM[4] = sa * sb * sc + ca * cc;
    _rotationM[5] = sa * sb * cc - ca * sc;
    _rotationM[6] = -sb;
    _rotationM[7] = cb * sc;
    _rotationM[8] = cb * cc;
}

/**
 * @brief 设置电机位置和速度
 * @param motor 电机编号
 * @param target_pos 目标位置，单位：度
 * @param target_vel 目标速度，单位：度/秒
 * @return void
 * @note 根据电机类型调用不同的控制函数
 */
void MOTOR_SetPosVal(MOTOR_t motor, float target_pos, float target_vel)
{
    //度/秒 -》 转/秒
    float target_vel_RPS = target_vel/360.0f;
    /* 达妙电机控制（关节1-3） */
    if (motor == DAM_MOTOR1 || motor == DAM_MOTOR2 || motor == DAM_MOTOR3) {
        DAM_MOTOR_POS(motor, target_pos, target_vel_RPS);
    }
    
    /* 张大头电机控制（关节4-6） */
    if (motor == ZDT_MOTOR1 || motor == ZDT_MOTOR2 || motor == ZDT_MOTOR3) {
        Dir dir;
        if (target_pos < 0) {  /* 负角度转换为正角度+方向 */
            target_pos = -target_pos;
            dir = CCW;  /* 逆时针 */
        } else {
            dir = CW;   /* 顺时针 */
        }
        ZDT_MOTOR_POSITION(motor, dir, 0, target_vel_RPS, target_pos);
    }
}


/**
 * @brief 关节角度限位检查
 * @param joints 待检查的关节角度
 * @param min_angle 各关节最小限位（°），数组长度6
 * @param max_angle 各关节最大限位（°），数组长度6
 * @return bool true=所有关节在限位内，false=存在超限位关节
 */
bool Joint6D_CheckLimit(const Joint6D_t* joints, const float* min_angle, const float* max_angle)
{
    for (int i = 0; i < 6; i++) {
        if (joints->a[i] < min_angle[i] || joints->a[i] > max_angle[i]) {
            return false; // 超限位
        }
    }
    return true;
}

/**
 * @brief 从IK8组解中选择最优解（与当前关节角差值最小）
 * @param solves IK求解结果
 * @param current_joints 当前关节角度
 * @param min_angle 关节最小限位
 * @param max_angle 关节最大限位
 * @param best_joints 输出最优关节解
 * @return bool true=找到有效最优解，false=无有效解
 */
bool IKSolves_SelectBest(const IKSolves_t* solves, const Joint6D_t* current_joints,
                         const float* min_angle, const float* max_angle, Joint6D_t* best_joints)
{
    float min_diff = 10000.0f; // 初始化为极大值
    int best_idx = -1;

    // 遍历8组解，筛选有效且差值最小的解
    for (int i = 0; i < 8; i++) {
        // 跳过无效/奇异解
        if (solves->solFlag[i][0] != 1 || solves->solFlag[i][1] != 1 || solves->solFlag[i][2] != 1) {
            continue;
        }
        // 检查限位
        if (!Joint6D_CheckLimit(&solves->config[i], min_angle, max_angle)) {
            continue;
        }

        // 计算与当前关节角的总差值（绝对值和）
        float diff = 0.0f;
        for (int j = 0; j < 6; j++) {
            diff += fabsf(solves->config[i].a[j] - current_joints->a[j]);
        }

        // 更新最优解
        if (diff < min_diff) {
            min_diff = diff;
            best_idx = i;
        }
    }

    // 无有效解
    if (best_idx == -1) {
        return false;
    }

    // 复制最优解
    memcpy(best_joints, &solves->config[best_idx], sizeof(Joint6D_t));
    return true;
}

//计算最大差值角度，以求同一时间到达位置
float Joint6D_CalcSyncVel(const Joint6D_t* current_joints, const Joint6D_t* target_joints,
                          const float* max_vel, float* target_vel)
{
    float delta[6];
    float max_delta = 0.0f;

    // 计算各关节角度差，找到最大差值（决定总运动时间）
    for (int i = 0; i < 6; i++) {
        delta[i] = fabsf(target_joints->a[i] - current_joints->a[i]);
        if (delta[i] > max_delta) {
            max_delta = delta[i];
        }
    }

    // 无运动需求
    if (max_delta < 0.001f) {
        memset(target_vel, 0, 6 * sizeof(float));
        return 0.0f;
    }

    // 计算总运动时间（按最大差值和对应关节的最大速度）
    int max_idx = 0;
    for (int i = 0; i < 6; i++) {
        if (delta[i] == max_delta) {
            max_idx = i;
            break;
        }
    }
    float total_time = max_delta / max_vel[max_idx];

    // 计算各关节同步速度（保证同时到达）
    for (int i = 0; i < 6; i++) {
        if (delta[i] < 0.001f) {
            target_vel[i] = 0.0f;
        } else {
            target_vel[i] = delta[i] / total_time;
            // 限制速度不超过最大值
            if (target_vel[i] > max_vel[i]) {
                target_vel[i] = max_vel[i];
            }
        }
    }

    return total_time;
}

/* ========== 新增：从电机读取当前关节角度 ========== */
bool Joint6D_ReadFromMotor(Joint6D_t* current_joints)
{
    // 对接你的电机底层：读取每个电机的当前位置（°）
    // 示例：需替换为实际的电机位置读取接口
    current_joints->a[0] = ((float)DAM_get_motor1()->para.pos)* 180/ M_PI; // 关节1（达妙1）
    current_joints->a[1] = ((float)DAM_get_motor2()->para.pos)* 180/ M_PI; // 关节2（达妙2）
    current_joints->a[2] = ((float)DAM_get_motor3()->para.pos)* 180/ M_PI; // 关节3（达妙3）
    current_joints->a[3] = get_motor1()->S_Cpos; // 关节4（张大头1）
    current_joints->a[4] = get_motor2()->S_Cpos; // 关节5（张大头2）
    current_joints->a[5] = get_motor3()->S_Cpos; // 关节6（张大头3）

    return true;
}

/**
 * @brief 关节空间运动（MoveJ）：直接控制6轴到目标关节角
 * @param kinematic 运动学结构体
 * @param current_joints 当前关节角度（需从电机读取）
 * @param target_joints 目标关节角度
 * @param max_vel 各关节最大速度（°/s）
 * @param min_angle 关节最小限位（°）
 * @param max_angle 关节最大限位（°）
 * @return bool true=运动指令下发成功，false=参数错误/超限位
 */
bool DOF6_MoveJ(DOF6Kinematic* kinematic, const Joint6D_t* current_joints,
                const Joint6D_t* target_joints, const float* max_vel,
                const float* min_angle, const float* max_angle)
{
    // // 1. 检查目标关节角是否超限位
    // if (!Joint6D_CheckLimit(target_joints, min_angle, max_angle)) {
    //     return false; // 超限位，拒绝下发
    // }

    // 2. 计算6轴同步速度
    float target_vel[6];
    Joint6D_CalcSyncVel(current_joints, target_joints, max_vel, target_vel);
    // for (int i = 0; i < 6; i++) {
    //     OTTO_uart(&huart_debug,"target_vel:%f" ,target_vel[i]);
    // }

    // 3. 下发每个关节的目标位置和速度到电机
    MOTOR_SetPosVal(MOTOR_1, target_joints->a[0], target_vel[0]);
    MOTOR_SetPosVal(MOTOR_2, target_joints->a[1], target_vel[1]);
    MOTOR_SetPosVal(MOTOR_3, target_joints->a[2], target_vel[2]);
    MOTOR_SetPosVal(MOTOR_4, target_joints->a[3], target_vel[3]);
    MOTOR_SetPosVal(MOTOR_5, target_joints->a[4], target_vel[4]);
    MOTOR_SetPosVal(MOTOR_6, target_joints->a[5], target_vel[5]);

    return true;
}

/**
 * @brief 笛卡尔空间直线运动（MoveL）：控制末端到目标位姿
 * @param kinematic 运动学结构体
 * @param current_joints 当前关节角度（需从电机读取）
 * @param target_pose 目标末端位姿（X/Y/Z/A/B/C）
 * @param max_vel 各关节最大速度（°/s）
 * @param min_angle 关节最小限位（°）
 * @param max_angle 关节最大限位（°）
 * @return bool true=运动指令下发成功，false=无解/超限位
 */
bool DOF6_MoveL(DOF6Kinematic* kinematic, const Joint6D_t* current_joints,
                const Pose6D_t* target_pose, const float* max_vel,
                const float* min_angle, const float* max_angle)
{
    // 1. IK求解：目标位姿→8组关节解
    IKSolves_t ik_solves;
    if (!DOF6_SolveIK(kinematic, target_pose, current_joints, &ik_solves)) {
        return false; // IK求解失败
    }

    // 2. 选择最优解（限位内+差值最小）
    Joint6D_t best_joints;
    if (!IKSolves_SelectBest(&ik_solves, current_joints, min_angle, max_angle, &best_joints)) {
        return false; // 无有效解
    }

    // 3. 调用MoveJ执行运动
    return DOF6_MoveJ(kinematic, current_joints, &best_joints, max_vel, min_angle, max_angle);
}



/**
 * @brief 六自由度运动学初始化
 * @param kinematic 运动学结构体指针
 * @param L_BS 基座长度，单位：米
 * @param D_BS 基座偏移，单位：米
 * @param L_AM 大臂长度，单位：米
 * @param L_FA 小臂长度，单位：米
 * @param D_EW 肘部偏移，单位：米
 * @param L_WT 腕部长度，单位：米
 * @return void
 * @note 初始化DH参数和连杆向量，为运动学计算做准备
 */
void DOF6Kinematic_Init(DOF6Kinematic* kinematic, float L_BS, float D_BS, 
                        float L_AM, float L_FA, float D_EW, float L_WT)
{
    /* 初始化机械臂结构参数 */
    kinematic->armConfig.L_BASE = L_BS;
    kinematic->armConfig.D_BASE = D_BS;
    kinematic->armConfig.L_ARM = L_AM;
    kinematic->armConfig.L_FOREARM = L_FA;
    kinematic->armConfig.D_ELBOW = D_EW;
    kinematic->armConfig.L_WRIST = L_WT;
    
    /* 初始化DH参数矩阵（标准DH表示法） */
    float tmp_DH_matrix[6][4] = {
        /* theta_home(rad), d(m), a(m), alpha(rad) */
        {0.0f,            kinematic->armConfig.L_BASE,    kinematic->armConfig.D_BASE, -(float)M_PI_2},
        {-(float)M_PI_2,  0.0f,                kinematic->armConfig.L_ARM,    0.0f},
        {(float)M_PI_2,   kinematic->armConfig.D_ELBOW,  0.0f,                (float)M_PI_2},
        {0.0f,            kinematic->armConfig.L_FOREARM, 0.0f,               -(float)M_PI_2},
        {0.0f,            0.0f,                0.0f,                (float)M_PI_2},
        {0.0f,            kinematic->armConfig.L_WRIST,  0.0f,                0.0f}
    };
    memcpy(kinematic->DH_matrix, tmp_DH_matrix, sizeof(tmp_DH_matrix));
    
    /* 初始化连杆向量（在各自坐标系中的表示） */
    float tmp_L1_bs[3] = {kinematic->armConfig.D_BASE, -kinematic->armConfig.L_BASE, 0.0f};
    memcpy(kinematic->L1_base, tmp_L1_bs, sizeof(tmp_L1_bs));
    
    float tmp_L2_se[3] = {kinematic->armConfig.L_ARM, 0.0f, 0.0f};
    memcpy(kinematic->L2_arm, tmp_L2_se, sizeof(tmp_L2_se));
    
    float tmp_L3_ew[3] = {-kinematic->armConfig.D_ELBOW, 0.0f, kinematic->armConfig.L_FOREARM};
    memcpy(kinematic->L3_elbow, tmp_L3_ew, sizeof(tmp_L3_ew));
    
    float tmp_L6_wt[3] = {0.0f, 0.0f, kinematic->armConfig.L_WRIST};
    memcpy(kinematic->L6_wrist, tmp_L6_wt, sizeof(tmp_L6_wt));
    
    /* 初始化中间计算变量 */
    kinematic->l_se_2 = kinematic->armConfig.L_ARM * kinematic->armConfig.L_ARM;
    kinematic->l_se = kinematic->armConfig.L_ARM;
    kinematic->l_ew_2 = kinematic->armConfig.L_FOREARM * kinematic->armConfig.L_FOREARM + 
                        kinematic->armConfig.D_ELBOW * kinematic->armConfig.D_ELBOW;
    kinematic->l_ew = 0.0f;   /* 延迟计算，避免重复计算 */
    kinematic->atan_e = 0.0f; /* 延迟计算，避免重复计算 */
}

/**
 * @brief 六自由度正运动学计算
 * @param kinematic 运动学结构体指针（常量）
 * @param _inputJoint6D 输入关节角度
 * @param _outputPose6D 输出末端位姿
 * @return bool 计算成功标志（总是返回true）
 * @note 使用DH参数法和矩阵乘法计算末端位姿
 */
bool DOF6_SolveFK(const DOF6Kinematic* kinematic, const Joint6D_t* _inputJoint6D, 
                  Pose6D_t* _outputPose6D)
{
    float q_in[6];      /* 输入关节角度（弧度） */
    float q[6];         /* 实际关节角度（包含偏移） */
    float cosq, sinq;   /* 关节角度三角函数值 */
    float cosa, sina;   /* DH参数alpha的三角函数值 */
    float P06[6];       /* 末端位置+欧拉角（前3个为位置，后3个为欧拉角） */
    float R06[9];       /* 末端旋转矩阵 */
    float R[6][9];      /* 各关节变换矩阵 */
    float R02[9];       /* 基座到关节2的变换 */
    float R03[9];       /* 基座到关节3的变换 */
    float R04[9];       /* 基座到关节4的变换 */
    float R05[9];       /* 基座到关节5的变换 */
    float L0_bs[3];     /* 基座连杆在基座坐标系中的向量 */
    float L0_se[3];     /* 大臂连杆在基座坐标系中的向量 */
    float L0_ew[3];     /* 肘部连杆在基座坐标系中的向量 */
    float L0_wt[3];     /* 腕部连杆在基座坐标系中的向量 */
    
    /* 将关节角度从度转换为弧度 */
    for (int i = 0; i < 6; i++) {
        q_in[i] = _inputJoint6D->a[i] / RAD_TO_DEG;
    }
    
    /* 计算各关节的变换矩阵 */
    for (int i = 0; i < 6; i++) {
        q[i] = q_in[i] + kinematic->DH_matrix[i][0];  /* 加上theta_home偏移 */
        cosq = cosf(q[i]);
        sinq = sinf(q[i]);
        cosa = cosf(kinematic->DH_matrix[i][3]);  /* alpha角度 */
        sina = sinf(kinematic->DH_matrix[i][3]);
        
        /* 标准DH变换矩阵 */
        R[i][0] = cosq;
        R[i][1] = -cosa * sinq;
        R[i][2] = sina * sinq;
        R[i][3] = sinq;
        R[i][4] = cosa * cosq;
        R[i][5] = -sina * cosq;
        R[i][6] = 0.0f;
        R[i][7] = sina;
        R[i][8] = cosa;
    }
    
    /* 累积变换矩阵，得到从基座到各关节的变换 */
    MatMultiply(R[0], R[1], R02, 3, 3, 3);
    MatMultiply(R02, R[2], R03, 3, 3, 3);
    MatMultiply(R03, R[3], R04, 3, 3, 3);
    MatMultiply(R04, R[4], R05, 3, 3, 3);
    MatMultiply(R05, R[5], R06, 3, 3, 3);
    
    /* 计算各连杆在基座坐标系中的位置 */
    MatMultiply(R[0], kinematic->L1_base, L0_bs, 3, 3, 1);
    MatMultiply(R02, kinematic->L2_arm, L0_se, 3, 3, 1);
    MatMultiply(R03, kinematic->L3_elbow, L0_ew, 3, 3, 1);
    MatMultiply(R06, kinematic->L6_wrist, L0_wt, 3, 3, 1);
    
    /* 计算末端位置：各连杆向量之和 */
    for (int i = 0; i < 3; i++) {
        P06[i] = L0_bs[i] + L0_se[i] + L0_ew[i] + L0_wt[i];
    }
    
    /* 从旋转矩阵提取欧拉角 */
    RotMatToEulerAngle(R06, &(P06[3]));
    
    /* 输出结果（位置单位：米 -> 毫米，角度：弧度 -> 度） */
    _outputPose6D->X = P06[0] * 1000.0f;  /* 米转毫米 */
    _outputPose6D->Y = P06[1] * 1000.0f;
    _outputPose6D->Z = P06[2] * 1000.0f;
    _outputPose6D->A = P06[3] * RAD_TO_DEG;
    _outputPose6D->B = P06[4] * RAD_TO_DEG;
    _outputPose6D->C = P06[5] * RAD_TO_DEG;
    memcpy(_outputPose6D->R, R06, 9 * sizeof(float));
    _outputPose6D->hasR = true;
    
    return true;
}

/**
 * @brief 角度标准化到[-π, π]范围
 * @param angle 输入角度，弧度
 * @return float 标准化后的角度，范围[-π, π]
 * @note 通过加减2π将角度限制在主值区间
 */
static float NormalizeAngle(float angle)
{
    while (angle > (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle <= -(float)M_PI) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

/**
 * @brief 六自由度逆运动学计算
 * @param kinematic 运动学结构体指针
 * @param _inputPose6D 输入末端位姿
 * @param _lastJoint6D 上一次关节角度（用于解的选择和奇异处理）
 * @param _outputSolves 输出逆解集（8组可能的解）
 * @return bool 计算成功标志（总是返回true）
 * @note 基于解析法求解，考虑奇异位置和多重解
 */
bool DOF6_SolveIK(DOF6Kinematic* kinematic, const Pose6D_t* _inputPose6D,
                  const Joint6D_t* _lastJoint6D, IKSolves_t* _outputSolves)
{
    float qs[2];        /* 关节1的两个可能解 */
    float qa[2][2];     /* 关节2和3的四个可能解 [解编号][关节2,关节3] */
    float qw[2][3];     /* 关节4,5,6的可能解 [解编号][关节4,5,6] */
    float cosqs, sinqs; /* 关节1的三角函数值 */
    float cosqa[2], sinqa[2]; /* 关节2,3的三角函数值 */
    float cosqw, sinqw; /* 关节4,5,6的三角函数值 */
    float P06[6];       /* 末端位置和姿态（弧度） */
    float R06[9];       /* 末端旋转矩阵 */
    float P0_w[3];      /* 腕部点在基座坐标系中的位置 */
    float P1_w[3];      /* 腕部点在关节1坐标系中的位置 */
    float L0_wt[3];     /* 腕部连杆在基座坐标系中的向量 */
    float L1_sw[3];     /* 从关节1到腕部的向量（在关节1坐标系中） */
    float R10[9];       /* 从基座到关节1的旋转矩阵（关节1坐标系） */
    float R31[9];       /* 从关节1到关节3的旋转矩阵 */
    float R30[9];       /* 从基座到关节3的旋转矩阵 */
    float R36[9];       /* 从关节3到末端的旋转矩阵 */
    float l_sw_2, l_sw; /* 关节1到腕部距离的平方和实际距离 */
    float atan_a, acos_a, acos_e; /* 用于求解关节2,3的中间角度 */
    
    int ind_arm, ind_elbow, ind_wrist; /* 循环索引：臂型、肘型、腕型 */
    int i; /* 通用循环索引 */
    
    /* 延迟计算中间变量（避免重复计算） */
    if (kinematic->l_ew == 0.0f) {
        kinematic->l_ew = sqrtf(kinematic->l_ew_2);
        kinematic->atan_e = atanf(kinematic->armConfig.D_ELBOW / kinematic->armConfig.L_FOREARM);
    }
    
    /* 位置单位转换：毫米 -> 米 */
    P06[0] = _inputPose6D->X / 1000.0f;
    P06[1] = _inputPose6D->Y / 1000.0f;
    P06[2] = _inputPose6D->Z / 1000.0f;
    
    /* 获取末端旋转矩阵（如果未提供则从欧拉角计算） */
    if (!_inputPose6D->hasR) {
        P06[3] = _inputPose6D->A / RAD_TO_DEG;
        P06[4] = _inputPose6D->B / RAD_TO_DEG;
        P06[5] = _inputPose6D->C / RAD_TO_DEG;
        EulerAngleToRotMat(&(P06[3]), R06);
    } else {
        memcpy(R06, _inputPose6D->R, 9 * sizeof(float));
    }
    
    /* 初始化关节角度候选解（从上一周期角度开始） */
    for (i = 0; i < 2; i++) {
        qs[i] = _lastJoint6D->a[0] / RAD_TO_DEG;
        qa[i][0] = _lastJoint6D->a[1] / RAD_TO_DEG;
        qa[i][1] = _lastJoint6D->a[2] / RAD_TO_DEG;
        qw[i][0] = _lastJoint6D->a[3] / RAD_TO_DEG;
        qw[i][1] = _lastJoint6D->a[4] / RAD_TO_DEG;
        qw[i][2] = _lastJoint6D->a[5] / RAD_TO_DEG;
    }
    
    /* 计算腕部点位置（从末端减去腕部连杆） */
    MatMultiply(R06, kinematic->L6_wrist, L0_wt, 3, 3, 1);
    for (i = 0; i < 3; i++) {
        P0_w[i] = P06[i] - L0_wt[i];
    }
    
    /* ========== 求解关节1 ========== */
    /* 检查是否在奇异位置（腕部点在Z轴上） */
    if (sqrtf(P0_w[0] * P0_w[0] + P0_w[1] * P0_w[1]) <= 0.000001f) {
        /* 奇异位置：关节1任意，使用上一周期的值 */
        qs[0] = _lastJoint6D->a[0] / RAD_TO_DEG;
        qs[1] = _lastJoint6D->a[0] / RAD_TO_DEG;
        for (i = 0; i < 4; i++) {
            _outputSolves->solFlag[0 + i][0] = -1;  /* 奇异标志 */
            _outputSolves->solFlag[4 + i][0] = -1;
        }
    } else {
        /* 正常情况：两个可能的解（相差180°） */
        qs[0] = atan2f(P0_w[1], P0_w[0]);
        qs[1] = atan2f(-P0_w[1], -P0_w[0]);
        for (i = 0; i < 4; i++) {
            _outputSolves->solFlag[0 + i][0] = 1;
            _outputSolves->solFlag[4 + i][0] = 1;
        }
    }
    
    /* ========== 求解关节2和关节3 ========== */
    for (ind_arm = 0; ind_arm < 2; ind_arm++) {
        /* 计算关节1旋转矩阵（基座到关节1） */
        cosqs = cosf(qs[ind_arm] + kinematic->DH_matrix[0][0]);
        sinqs = sinf(qs[ind_arm] + kinematic->DH_matrix[0][0]);
        
        /* 关节1坐标系相对于基座坐标系的旋转矩阵（实际上是逆变换） */
        R10[0] = cosqs;
        R10[1] = sinqs;
        R10[2] = 0.0f;
        R10[3] = 0.0f;
        R10[4] = 0.0f;
        R10[5] = -1.0f;
        R10[6] = -sinqs;
        R10[7] = cosqs;
        R10[8] = 0.0f;
        
        /* 将腕部点转换到关节1坐标系 */
        MatMultiply(R10, P0_w, P1_w, 3, 3, 1);
        for (i = 0; i < 3; i++) {
            L1_sw[i] = P1_w[i] - kinematic->L1_base[i];
        }
        
        /* 计算关节1到腕部的距离 */
        l_sw_2 = L1_sw[0] * L1_sw[0] + L1_sw[1] * L1_sw[1];
        l_sw = sqrtf(l_sw_2);
        
        /* 情况1：完全伸展或收缩（大臂+前臂长度等于距离） */
        if (fabsf(kinematic->l_se + kinematic->l_ew - l_sw) <= 0.000001f) {
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            qa[0][1] = 0.0f;
            qa[1][1] = 0.0f;
            
            /* 检查可达性 */
            if (l_sw > kinematic->l_se + kinematic->l_ew) {
                for (i = 0; i < 2; i++) {
                    _outputSolves->solFlag[4 * ind_arm + 0 + i][1] = 0;  /* 不可达 */
                    _outputSolves->solFlag[4 * ind_arm + 2 + i][1] = 0;
                }
            } else {
                for (i = 0; i < 2; i++) {
                    _outputSolves->solFlag[4 * ind_arm + 0 + i][1] = 1;  /* 可达 */
                    _outputSolves->solFlag[4 * ind_arm + 2 + i][1] = 1;
                }
            }
        }
        /* 情况2：特殊角度情况（距离等于两臂长度差） */
        else if (fabsf(l_sw - fabsf(kinematic->l_se - kinematic->l_ew)) <= 0.000001f) {
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            
            /* 根据臂型确定关节3角度 */
            if (ind_arm == 0) {
                qa[0][1] = (float)M_PI;
                qa[1][1] = -(float)M_PI;
            } else {
                qa[0][1] = -(float)M_PI;
                qa[1][1] = (float)M_PI;
            }
            
            /* 检查可达性 */
            if (l_sw < fabsf(kinematic->l_se - kinematic->l_ew)) {
                for (i = 0; i < 2; i++) {
                    _outputSolves->solFlag[4 * ind_arm + 0 + i][1] = 0;  /* 不可达 */
                    _outputSolves->solFlag[4 * ind_arm + 2 + i][1] = 0;
                }
            } else {
                for (i = 0; i < 2; i++) {
                    _outputSolves->solFlag[4 * ind_arm + 0 + i][1] = 1;  /* 可达 */
                    _outputSolves->solFlag[4 * ind_arm + 2 + i][1] = 1;
                }
            }
        }
        /* 情况3：一般情况（使用余弦定理） */
        else {
            /* 计算中间角度 */
            atan_a = atan2f(L1_sw[1], L1_sw[0]);
            acos_a = 0.5f * (kinematic->l_se_2 + l_sw_2 - kinematic->l_ew_2) /
                     (kinematic->l_se * l_sw);
            acos_a = fmaxf(-1.0f, fminf(1.0f, acos_a));  /* 限制范围防止数值误差 */
            acos_a = acosf(acos_a);
            
            acos_e = 0.5f * (kinematic->l_se_2 + kinematic->l_ew_2 - l_sw_2) /
                     (kinematic->l_se * kinematic->l_ew);
            acos_e = fmaxf(-1.0f, fminf(1.0f, acos_e));  /* 限制范围防止数值误差 */
            acos_e = acosf(acos_e);
            
            /* 根据臂型（左手/右手）计算两组解 */
            if (ind_arm == 0) {  /* 左手型 */
                qa[0][0] = atan_a - acos_a + (float)M_PI_2;
                qa[0][1] = acos_e - kinematic->atan_e + (float)M_PI;
                qa[1][0] = atan_a + acos_a + (float)M_PI_2;
                qa[1][1] = -acos_e + kinematic->atan_e - (float)M_PI;
            } else {  /* 右手型 */
                qa[0][0] = atan_a + acos_a + (float)M_PI_2;
                qa[0][1] = -acos_e + kinematic->atan_e - (float)M_PI;
                qa[1][0] = atan_a - acos_a + (float)M_PI_2;
                qa[1][1] = acos_e - kinematic->atan_e + (float)M_PI;
            }
            
            for (i = 0; i < 2; i++) {
                _outputSolves->solFlag[4 * ind_arm + 0 + i][1] = 1;
                _outputSolves->solFlag[4 * ind_arm + 2 + i][1] = 1;
            }
        }
        
        /* ========== 求解关节4,5,6 ========== */
        for (ind_elbow = 0; ind_elbow < 2; ind_elbow++) {
            /* 计算关节2和3的变换矩阵 */
            cosqa[0] = cosf(qa[ind_elbow][0] + kinematic->DH_matrix[1][0]);
            sinqa[0] = sinf(qa[ind_elbow][0] + kinematic->DH_matrix[1][0]);
            cosqa[1] = cosf(qa[ind_elbow][1] + kinematic->DH_matrix[2][0]);
            sinqa[1] = sinf(qa[ind_elbow][1] + kinematic->DH_matrix[2][0]);
            
            /* 从关节1到关节3的旋转矩阵 */
            R31[0] = cosqa[0] * cosqa[1] - sinqa[0] * sinqa[1];
            R31[1] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
            R31[2] = 0.0f;
            R31[3] = 0.0f;
            R31[4] = 0.0f;
            R31[5] = 1.0f;
            R31[6] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
            R31[7] = -cosqa[0] * cosqa[1] + sinqa[0] * sinqa[1];
            R31[8] = 0.0f;
            
            /* 计算从基座到关节3的旋转矩阵 */
            MatMultiply(R31, R10, R30, 3, 3, 3);
            /* 计算从关节3到末端的旋转矩阵 */
            MatMultiply(R30, R06, R36, 3, 3, 3);
            
            /* 求解关节5（根据旋转矩阵的特定元素） */
            if (R36[8] >= 1.0f - 0.000001f) {  /* 关节5接近0° */
                cosqw = 1.0f;
                qw[0][1] = 0.0f;
                qw[1][1] = 0.0f;
            } else if (R36[8] <= -1.0f + 0.000001f) {  /* 关节5接近180° */
                cosqw = -1.0f;
                if (ind_arm == 0) {
                    qw[0][1] = (float)M_PI;
                    qw[1][1] = -(float)M_PI;
                } else {
                    qw[0][1] = -(float)M_PI;
                    qw[1][1] = (float)M_PI;
                }
            } else {  /* 关节5一般角度 */
                cosqw = R36[8];
                if (ind_arm == 0) {
                    qw[0][1] = acosf(cosqw);
                    qw[1][1] = -acosf(cosqw);
                } else {
                    qw[0][1] = -acosf(cosqw);
                    qw[1][1] = acosf(cosqw);
                }
            }
            
            /* 求解关节4和关节6（检查奇异位置） */
            if (fabsf(fabsf(cosqw) - 1.0f) <= 0.000001f) {
                /* 奇异位置：关节5为0或180度，关节4和6耦合 */
                if (ind_arm == 0) {
                    qw[0][0] = _lastJoint6D->a[3] / RAD_TO_DEG;
                    cosqw = cosf(_lastJoint6D->a[3] / RAD_TO_DEG + kinematic->DH_matrix[3][0]);
                    sinqw = sinf(_lastJoint6D->a[3] / RAD_TO_DEG + kinematic->DH_matrix[3][0]);
                    qw[0][2] = atan2f(cosqw * R36[3] - sinqw * R36[0],
                                      cosqw * R36[0] + sinqw * R36[3]);
                    qw[1][2] = _lastJoint6D->a[5] / RAD_TO_DEG;
                    cosqw = cosf(_lastJoint6D->a[5] / RAD_TO_DEG + kinematic->DH_matrix[5][0]);
                    sinqw = sinf(_lastJoint6D->a[5] / RAD_TO_DEG + kinematic->DH_matrix[5][0]);
                    qw[1][0] = atan2f(cosqw * R36[3] - sinqw * R36[0],
                                      cosqw * R36[0] + sinqw * R36[3]);
                } else {
                    qw[0][2] = _lastJoint6D->a[5] / RAD_TO_DEG;
                    cosqw = cosf(_lastJoint6D->a[5] / RAD_TO_DEG + kinematic->DH_matrix[5][0]);
                    sinqw = sinf(_lastJoint6D->a[5] / RAD_TO_DEG + kinematic->DH_matrix[5][0]);
                    qw[0][0] = atan2f(cosqw * R36[3] - sinqw * R36[0],
                                      cosqw * R36[0] + sinqw * R36[3]);
                    qw[1][0] = _lastJoint6D->a[3] / RAD_TO_DEG;
                    cosqw = cosf(_lastJoint6D->a[3] / RAD_TO_DEG + kinematic->DH_matrix[3][0]);
                    sinqw = sinf(_lastJoint6D->a[3] / RAD_TO_DEG + kinematic->DH_matrix[3][0]);
                    qw[1][2] = atan2f(cosqw * R36[3] - sinqw * R36[0],
                                      cosqw * R36[0] + sinqw * R36[3]);
                }
                _outputSolves->solFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = -1;  /* 奇异 */
                _outputSolves->solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;
            } else {
                /* 非奇异位置：可以独立求解关节4和6 */
                if (ind_arm == 0) {
                    qw[0][0] = atan2f(R36[5], R36[2]);
                    qw[1][0] = atan2f(-R36[5], -R36[2]);
                    qw[0][2] = atan2f(R36[7], -R36[6]);
                    qw[1][2] = atan2f(-R36[7], R36[6]);
                } else {
                    qw[0][0] = atan2f(-R36[5], -R36[2]);
                    qw[1][0] = atan2f(R36[5], R36[2]);
                    qw[0][2] = atan2f(-R36[7], R36[6]);
                    qw[1][2] = atan2f(R36[7], -R36[6]);
                }
                _outputSolves->solFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = 1;  /* 有效 */
                _outputSolves->solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;
            }
            
            /* ========== 存储结果 ========== */
            for (ind_wrist = 0; ind_wrist < 2; ind_wrist++) {
                int sol_index = 4 * ind_arm + 2 * ind_elbow + ind_wrist;
                
                /* 关节1：角度标准化并转换为度 */
                _outputSolves->config[sol_index].a[0] = NormalizeAngle(qs[ind_arm]) * RAD_TO_DEG;
                
                /* 关节2和3：角度标准化并转换为度 */
                for (i = 0; i < 2; i++) {
                    _outputSolves->config[sol_index].a[1 + i] =
                        NormalizeAngle(qa[ind_elbow][i]) * RAD_TO_DEG;
                }
                
                /* 关节4,5,6：角度标准化并转换为度 */
                for (i = 0; i < 3; i++) {
                    _outputSolves->config[sol_index].a[3 + i] =
                        NormalizeAngle(qw[ind_wrist][i]) * RAD_TO_DEG;
                }
            }
        }
    }
    
    return true;
}

/**
 * @brief 关节角度差值计算
 * @param joints1 第一组关节角度
 * @param joints2 第二组关节角度
 * @return Joint6D_t 差值结果：joints1 - joints2
 * @note 计算两组关节角度的逐元素差值
 */
Joint6D_t Joint6D_Subtract(const Joint6D_t* joints1, const Joint6D_t* joints2)
{
    Joint6D_t result;
    
    for (int i = 0; i < 6; i++) {
        result.a[i] = joints1->a[i] - joints2->a[i];
    }
    
    return result;
}
#define DEBUG_JOINTS 1
void Joints_FK(float angle1,float angle2,float angle3,float angle4,float angle5,float angle6) {
    DOF6Kinematic robot;
    Joint6D_t joints;
    Pose6D_t fk_pose;
    DOF6Kinematic_Init(&robot, L_BASE_M, D_BASE_M, L_ARM_M, L_FOREARM_M, D_ELBOW_M, L_WRIST_M);
#if DEBUG_JOINTS
    if(angle1>=MOTOR1_MIN_LIMIT&&angle1<=MOTOR1_MAX_LIMIT) joints.a[0] = angle1;
    else {OTTO_uart(&huart_debug,"关节角1超限");}
    if (angle2>=MOTOR2_MIN_LIMIT&&angle2<=MOTOR2_MAX_LIMIT) joints.a[1] = angle2;
    else {OTTO_uart(&huart_debug,"关节角2超限");}
    if(angle3>=MOTOR3_MIN_LIMIT&&angle3<=MOTOR3_MAX_LIMIT) joints.a[2] = angle3;
    else {OTTO_uart(&huart_debug,"关节角3超限");}
    if (angle4>=MOTOR4_MIN_LIMIT&&angle4<=MOTOR4_MAX_LIMIT) joints.a[3] = angle4;
    else {OTTO_uart(&huart_debug,"关节角4超限");}
    if(angle5>=MOTOR5_MIN_LIMIT&&angle5<=MOTOR5_MAX_LIMIT) joints.a[4] = angle5;
    else {OTTO_uart(&huart_debug,"关节角5超限");}
    if (angle6>=MOTOR6_MIN_LIMIT&&angle6<=MOTOR6_MAX_LIMIT) joints.a[5] = angle6;
    else {OTTO_uart(&huart_debug,"关节角6超限");}
#endif
    float joint[6]      = {angle1,angle2,angle3,angle4,angle5,angle6};
    float joint_min[6]  = {MOTOR1_MIN_LIMIT, MOTOR2_MIN_LIMIT, MOTOR3_MIN_LIMIT, MOTOR4_MIN_LIMIT, MOTOR5_MIN_LIMIT, MOTOR6_MIN_LIMIT}; // 最小限位
    float joint_max[6]  = {MOTOR1_MAX_LIMIT, MOTOR2_MAX_LIMIT, MOTOR3_MAX_LIMIT, MOTOR4_MAX_LIMIT, MOTOR5_MAX_LIMIT, MOTOR6_MAX_LIMIT};       // 最大限位
    float max_vel[6]    = {MOTOR1_MAX_VEL, MOTOR2_MAX_VEL, MOTOR3_MAX_VEL, MOTOR4_MAX_VEL, MOTOR5_MAX_VEL, MOTOR6_MAX_VEL};          // 最大速度（°/s）


    if (Joint6D_CheckLimit(joint,joint_min,joint_max)) {
        DOF6_SolveFK(&robot, &joints, &fk_pose);
        if (fk_pose.hasR) {
            OTTO_uart(&huart_debug, "=========================");
            OTTO_uart(&huart_debug,"正解计算完成，末端姿态为：");
            OTTO_uart(&huart_debug, "位置：X=%.2f mm | Y=%.2f mm | Z=%.2f mm", fk_pose.X, fk_pose.Y, fk_pose.Z);
            OTTO_uart(&huart_debug, "姿态：A=%.2f ° | B=%.2f ° | C=%.2f °", fk_pose.A, fk_pose.B, fk_pose.C);
            OTTO_uart(&huart_debug, "=========================");
            Joint6D_t current_joints;
            Joint6D_ReadFromMotor(&current_joints);
            if (DOF6_MoveJ(&robot,&current_joints,&joints,&max_vel,&joint_min,&joint_max)) {
                OTTO_uart(&huart_debug, "开启电机运动到指定位置");
            }
        }
    }else {
        OTTO_uart(&huart_debug,"关节角超限");
    }
}
void Joints_IK(float x,float y,float z) {
    DOF6Kinematic robot;
    Joint6D_t current_joints,best_ik_joints;
    Pose6D_t fk_pose;
    fk_pose.X = x;
    fk_pose.Y = y;
    fk_pose.Z = z;
    fk_pose.A = -10.15;
    fk_pose.B = 48.43;
    fk_pose.C = -16.52;
    fk_pose.hasR = false;
    IKSolves_t ik_solves;
    DOF6Kinematic_Init(&robot, L_BASE_M, D_BASE_M, L_ARM_M, L_FOREARM_M, D_ELBOW_M, L_WRIST_M);
    Joint6D_ReadFromMotor(&current_joints);
    DOF6_SolveIK(&robot, &fk_pose, &current_joints, &ik_solves);
    float joint_min[6]  = {MOTOR1_MIN_LIMIT, MOTOR2_MIN_LIMIT, MOTOR3_MIN_LIMIT, MOTOR4_MIN_LIMIT, MOTOR5_MIN_LIMIT, MOTOR6_MIN_LIMIT}; // 最小限位
    float joint_max[6]  = {MOTOR1_MAX_LIMIT, MOTOR2_MAX_LIMIT, MOTOR3_MAX_LIMIT, MOTOR4_MAX_LIMIT, MOTOR5_MAX_LIMIT, MOTOR6_MAX_LIMIT};       // 最大限位
    float max_vel[6]    = {MOTOR1_MAX_VEL, MOTOR2_MAX_VEL, MOTOR3_MAX_VEL, MOTOR4_MAX_VEL, MOTOR5_MAX_VEL, MOTOR6_MAX_VEL};          // 最大速度（°/s）
    if (IKSolves_SelectBest(&ik_solves, &current_joints, joint_min, joint_max, &best_ik_joints)) {
        OTTO_uart(&huart_debug, "=========================");
        OTTO_uart(&huart_debug,"逆解计算完成，最佳关节角：");
        OTTO_uart(&huart_debug, "J1: %.2f ° | J2: %.2f ° | J3: %.2f °", best_ik_joints.a[0], best_ik_joints.a[1], best_ik_joints.a[2]);
        OTTO_uart(&huart_debug, "J4: %.2f ° | J5: %.2f ° | J6: %.2f °", best_ik_joints.a[3], best_ik_joints.a[4], best_ik_joints.a[5]);
        OTTO_uart(&huart_debug, "=========================");
        if (DOF6_MoveJ(&robot, &current_joints, &best_ik_joints, max_vel, &joint_min, &joint_max)) {
            OTTO_uart(&huart_debug, "开启电机运动到指定位置");
        }
    }else {
        OTTO_uart(&huart_debug, "❌ IK无有效最优解！");
    }

}