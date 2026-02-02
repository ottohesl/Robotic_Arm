#include "DM_Motor_LOG.h"
void DAM_Motor_ALL_State(motor_t *motor) {
    if (motor->valid) {
        uint8_t state = motor->para.state;
        uint8_t motor_id= motor->id;
        float rad_speed=motor->para.vel;
        float position=motor->para.pos;
        float tor=motor->para.tor;
        float temperature_mos=motor->para.Tmos;
        float temperature_coil=motor->para.Tcoil;
        float kp=motor->para.Kp;
        float kd=motor->para.Kd;
        float angle_speed=rad_speed * (180.0f / M_PI);
        float angle_position=position * (180.0f / M_PI);

        OTTO_uart(&huart_debug,"++++++++++++++ 电机[0x%02X] ++++++++++++++", motor_id);
        OTTO_uart(&huart_debug,"驱动器MOS温度: %.2f,电机线圈温度:%.2f,kp:%.2f,kd:%.2f",
           temperature_mos,temperature_coil,kp,kd);
        OTTO_uart(&huart_debug, "电机状态:%d,实时位置:%.3f,实时速度:%.3f,实时力矩: %.2f",
                         state, angle_position,angle_speed,tor);
        OTTO_uart(&huart_debug,"+++++++++++++++++++++++++++++++++++++++++");

        motor->valid=false;
    }
}
void DAM_Motor_PV_State(motor_t *motor) {

}