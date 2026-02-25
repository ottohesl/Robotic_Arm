#include "OLED_Menu.h"
#include <stdlib.h>
#include "ottohesl.h"
#include "usart.h"

// 全局菜单句柄（扩展后）
Menu_Handle current_menu={
    .current_level=MENU_LEVEL_1,
    .selected_idx=1,
    .idx_max=0,
    .level2_idx=0,  // 初始化二级菜单历史索引
    .level3_idx=0   // 初始化三级菜单历史索引
};

// 按键读取函数（保持你的原有逻辑，这里仅补全返回）
Key_Event get_key() {
    // 你的按键扫描逻辑
    // if (HAL_GPIO_ReadPin(KEY_EN_GPIO_Port, KEY_EN_Pin) == GPIO_PIN_RESET) {
    //     HAL_Delay(20); // 消抖
    //     if (HAL_GPIO_ReadPin(KEY_EN_GPIO_Port, KEY_EN_Pin) == GPIO_PIN_RESET) {
    //         while(HAL_GPIO_ReadPin(KEY_EN_GPIO_Port, KEY_EN_Pin) == GPIO_PIN_RESET);
    //         return KEY_EVENT_ENTER;
    //     }
    // }
    // if (HAL_GPIO_ReadPin(KEY_NE_GPIO_Port, KEY_NE_Pin) == GPIO_PIN_RESET) {
    //     HAL_Delay(20); // 消抖
    //     if (HAL_GPIO_ReadPin(KEY_NE_GPIO_Port, KEY_NE_Pin) == GPIO_PIN_RESET) {
    //         while(HAL_GPIO_ReadPin(KEY_NE_GPIO_Port, KEY_NE_Pin) == GPIO_PIN_RESET);
    //         return KEY_EVENT_NEXT;
    //     }
    // }
    if (Get_Key_Enter) {
        Get_Key_Enter=0;
        return KEY_EVENT_ENTER;
    }
    if (Get_Key_Enter_Num) {
        Get_Key_Enter_Num=0;
        return KEY_EVENT_NEXT;
    }
    return KEY_EVENT_NONE;
}

void level_1(Menu_Handle* current_menu) {
    if (current_menu->current_level==MENU_LEVEL_1) {
        OLED_Clear();
        Key_Event key_event;
        do {
            key_event = get_key();
            OLED_ShowPicture(14,0,100,64,Robotic_picture_Data,OLED_NORMAL);
            OLED_Printf(85,39 , FONT_8, OLED_NORMAL, "ROBOTIC  ARM");
            OLED_ShowFrame();
        }while (key_event==KEY_EVENT_NONE);
        //检查到任意按键按下后，跳转到第二菜单
        current_menu->current_level = MENU_LEVEL_2;
    }
}

void OLED_Serial_Printf(uint8_t Ser, OLED_DisplayMode mode,const char *fmt, ...) {
    if(fmt == NULL) return;
    char buf[128] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Ser = (Ser - 1) % 4 + 1;
    OLED_Printf(0, (Ser-1)*FONT_16, FONT_16, mode, buf); // 修复：原代码传fmt，应该传buf
}

void level_2(Menu_Handle* current_menu) {
    current_menu->idx_max = 8;  //表示二级菜单总共菜单数量
    if (current_menu->current_level == MENU_LEVEL_2) {
        OLED_Clear();
        Key_Event key_event;
        while (1){
            // 显示二级菜单
            if(current_menu->selected_idx>0&&current_menu->selected_idx<5) {
                OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "实时监视            ");
                OLED_Serial_Printf(2, (current_menu->selected_idx == 2) ? OLED_HIGHLIGHT : OLED_NORMAL, "参数设置            ");
                OLED_Serial_Printf(3, (current_menu->selected_idx == 3) ? OLED_HIGHLIGHT : OLED_NORMAL, "快捷操作            ");
                OLED_Serial_Printf(4, (current_menu->selected_idx == 4) ? OLED_HIGHLIGHT : OLED_NORMAL, "系统信息            ");
            }
            else if (current_menu->selected_idx>4&&current_menu->selected_idx<9) {
                OLED_Serial_Printf(5, (current_menu->selected_idx == 5) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂回零位        ");
                OLED_Serial_Printf(6, (current_menu->selected_idx == 6) ? OLED_HIGHLIGHT : OLED_NORMAL, "数据              ");
                OLED_Serial_Printf(7, (current_menu->selected_idx == 7) ? OLED_HIGHLIGHT : OLED_NORMAL, "type5            ");
                OLED_Serial_Printf(8, (current_menu->selected_idx == 8) ? OLED_HIGHLIGHT : OLED_NORMAL, "return           ");
            }
            OLED_ShowFrame();

            //按键扫描
            key_event = get_key();
            if (key_event==KEY_EVENT_NEXT) {
                //让索引在规定范围循环
                current_menu->selected_idx++;
                if (current_menu->selected_idx > current_menu->idx_max) {
                    current_menu->selected_idx = 1;
                }
            }
            if (key_event==KEY_EVENT_ENTER) {
                if (current_menu->selected_idx==8) {
                    current_menu->current_level = MENU_LEVEL_1;
                    current_menu->selected_idx = 1; // 重置选中索引
                } else {
                    // 保存二级菜单选中的索引
                    current_menu->level2_idx = current_menu->selected_idx;
                    current_menu->current_level = MENU_LEVEL_3;
                    current_menu->selected_idx = 1; // 重置三级菜单起始索引
                }
                break;
            }
        }
    }
}

void level_3(Menu_Handle* current_menu) {
    if (current_menu->current_level==MENU_LEVEL_3) {
        OLED_Clear();
        Key_Event key_event;
        // 根据二级菜单选中的索引设置三级菜单最大索引
        switch (current_menu->level2_idx) {
            case 1: // 实时监视
                current_menu->idx_max = 5;
                break;
            case 2: // 参数设置
                current_menu->idx_max = 6;
                break;
            case 3: // 快捷操作
                current_menu->idx_max = 5;
                break;
            default: // 其他
                current_menu->idx_max = 4;
                break;
        }

        while (1) {
            // 根据二级菜单选中的索引显示对应三级菜单
            switch (current_menu->level2_idx) {
                case 1: //实时监视页面
                    OLED_Clear();
                    if(current_menu->selected_idx>0&&current_menu->selected_idx<5) {
                        OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return        ");
                        OLED_Serial_Printf(2, (current_menu->selected_idx == 2) ? OLED_HIGHLIGHT : OLED_NORMAL, "查看关节角度    ");
                        OLED_Serial_Printf(3, (current_menu->selected_idx == 3) ? OLED_HIGHLIGHT : OLED_NORMAL, "查看末端位姿    ");
                        OLED_Serial_Printf(4, (current_menu->selected_idx == 4) ? OLED_HIGHLIGHT : OLED_NORMAL, "查看运动状态    ");
                    }
                    else if (current_menu->selected_idx>4&&current_menu->selected_idx<9) {
                        OLED_Serial_Printf(5, OLED_HIGHLIGHT, "查看电机状态    ");
                    }
                    break;
                case 2: //参数设置页面
                    OLED_Clear();
                    if(current_menu->selected_idx>0&&current_menu->selected_idx<5) {
                        OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return          ");
                        OLED_Serial_Printf(2, (current_menu->selected_idx == 2) ? OLED_HIGHLIGHT : OLED_NORMAL, "设置关节角度      ");
                        OLED_Serial_Printf(3, (current_menu->selected_idx == 3) ? OLED_HIGHLIGHT : OLED_NORMAL, "设置关节长度      ");
                        OLED_Serial_Printf(4, (current_menu->selected_idx == 4) ? OLED_HIGHLIGHT : OLED_NORMAL, "设置关节速度      ");
                    }
                    else if (current_menu->selected_idx>4&&current_menu->selected_idx<9) {
                        OLED_Serial_Printf(5, (current_menu->selected_idx == 5) ? OLED_HIGHLIGHT : OLED_NORMAL, "设置工作空间限制  ");
                        OLED_Serial_Printf(6, (current_menu->selected_idx == 6) ? OLED_HIGHLIGHT : OLED_NORMAL, "end         ");
                    }
                    break;
                case 3: //快捷操作页面
                    OLED_Clear();
                    if(current_menu->selected_idx>0&&current_menu->selected_idx<5) {
                        OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return          ");
                        OLED_Serial_Printf(2, (current_menu->selected_idx == 2) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂回零位     ");
                        OLED_Serial_Printf(3, (current_menu->selected_idx == 3) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂校准      ");
                        OLED_Serial_Printf(4, (current_menu->selected_idx == 4) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂画正方形      ");
                    }
                    else if (current_menu->selected_idx>4&&current_menu->selected_idx<9) {
                        OLED_Serial_Printf(5, (current_menu->selected_idx == 5) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂示教      ");
                    }
                    break;
                default:
                    OLED_Serial_Printf(1, OLED_NORMAL, "未知菜单        ");
                    break;
            }
            OLED_ShowFrame();

            //按键扫描
            key_event = get_key();
            if (key_event==KEY_EVENT_NEXT) {
                //让索引在规定范围循环
                current_menu->selected_idx++;
                if (current_menu->selected_idx > current_menu->idx_max) {
                    current_menu->selected_idx = 1;
                }
                OTTO_uart(&huart1,"当前三级索引：%d",current_menu->selected_idx);
            }
            if (key_event==KEY_EVENT_ENTER) {
                if (current_menu->selected_idx==1) {
                    // 返回二级菜单
                    current_menu->current_level = MENU_LEVEL_2;
                    current_menu->selected_idx = current_menu->level2_idx; // 恢复二级菜单选中状态
                } else {
                    // 保存三级菜单选中的索引，进入四级菜单
                    current_menu->level3_idx = current_menu->selected_idx;
                    current_menu->current_level = MENU_LEVEL_4;
                    current_menu->selected_idx = 1; // 重置四级菜单起始索引
                }
                break;
            }
        }
    }
}

void level_4(Menu_Handle* current_menu) {
    if (current_menu->current_level==MENU_LEVEL_4) {
        OLED_Clear();
        Key_Event key_event;
        // 根据二级+三级菜单索引设置四级菜单最大索引
        switch (current_menu->level2_idx) {
            case 1: // 实时监视下的四级菜单
                current_menu->idx_max = 2;
                break;
            case 2: // 参数设置下的四级菜单
                current_menu->idx_max = 6;
                break;
            case 3: // 快捷操作下的四级菜单
                current_menu->idx_max = 5;
                break;
            default:
                current_menu->idx_max = 1;
                break;
        }

        while (1) {
            // 根据二级+三级菜单索引显示对应四级菜单
            switch (current_menu->level2_idx) {
                case 1: // 实时监视页面的四级菜单
                    Joint6D_t OLED_Joints;
                    Joint6D_ReadFromMotor(&OLED_Joints);
                    switch (current_menu->level3_idx) {
                        case 2: // 查看关节角度
                            OLED_Clear();
                            OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL,  "    关节角度    ");
                            OLED_Printf(0, 0*FONT_16+16, FONT_16,OLED_NORMAL, "J1：%.2f", OLED_Joints.a[0]);
                            OLED_Printf(0, 1*FONT_16+16, FONT_16,OLED_NORMAL, "J2：%.2f", OLED_Joints.a[1]);
                            OLED_Printf(0, 2*FONT_16+16, FONT_16,OLED_NORMAL, "J3：%.2f", OLED_Joints.a[2]);
                            OLED_Printf(64, 0*FONT_16+16, FONT_16,OLED_NORMAL, "J4：%.2f", OLED_Joints.a[3]);
                            OLED_Printf(64, 1*FONT_16+16, FONT_16,OLED_NORMAL, "J5：%.2f", OLED_Joints.a[4]);
                            OLED_Printf(64, 2*FONT_16+16, FONT_16,OLED_NORMAL, "J6：%.2f", OLED_Joints.a[5]);
                            break;
                        case 3: // 查看末端位姿
                            DOF6Kinematic OLED_robot;
                            Pose6D_t fk_pose;
                            DOF6Kinematic_Init(&OLED_robot, L_BASE_M, D_BASE_M, L_ARM_M, L_FOREARM_M, D_ELBOW_M, L_WRIST_M);
                            DOF6_SolveFK(&OLED_robot, &OLED_Joints, &fk_pose);
                            OLED_Clear();
                            OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "位置 mm|姿态(度)");
                            OLED_Printf(0, 0*FONT_16+16, FONT_16,OLED_NORMAL, "X：%.1f", fk_pose.X);
                            OLED_Printf(0, 1*FONT_16+16, FONT_16,OLED_NORMAL, "Y：%.1f", fk_pose.Y);
                            OLED_Printf(0, 2*FONT_16+16, FONT_16,OLED_NORMAL, "Z：%.1f", fk_pose.Z);
                            OLED_Printf(64, 0*FONT_16+16, FONT_16,OLED_NORMAL, "A：%.1f", fk_pose.A);
                            OLED_Printf(64, 1*FONT_16+16, FONT_16,OLED_NORMAL, "B：%.1f", fk_pose.B);
                            OLED_Printf(64, 2*FONT_16+16, FONT_16,OLED_NORMAL, "C：%.1f", fk_pose.C);
                            break;
                        case 4: // 查看运动状态
                            OLED_Clear();
                            OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "    运动状态    ");
                            if (Flag_Solve_FK==1) {
                                OLED_Printf(0, 0*FONT_8+16, FONT_16,OLED_HIGHLIGHT, "正解 Success");
                            }else if (Flag_Solve_FK==-1) {
                                OLED_Printf(0, 0*FONT_16+16, FONT_16,OLED_HIGHLIGHT, "正解 超限位");
                            }
                            if (Flag_Solve_IK==1) {
                                OLED_Printf(0, 0*FONT_8+16, FONT_16,OLED_HIGHLIGHT, "逆解 Success");
                            }else if (Flag_Solve_IK==-1) {
                                OLED_Printf(0, 0*FONT_16+16, FONT_16,OLED_HIGHLIGHT, "逆解 无有效解");
                            }
                                DOF6Kinematic OLED_run;
                                Pose6D_t fk_pose_run;
                                DOF6Kinematic_Init(&OLED_robot, L_BASE_M, D_BASE_M, L_ARM_M, L_FOREARM_M, D_ELBOW_M, L_WRIST_M);
                                DOF6_SolveFK(&OLED_run, &OLED_Joints, &fk_pose_run);
                                float delta_x=fk_pose_run.X-target_x;
                                float delta_y=fk_pose_run.Y-target_y;
                                float delta_z=fk_pose_run.Z-target_z;
                                float total_err = sqrtf(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
                                OLED_Printf(0, 1*FONT_16+16, FONT_16,OLED_NORMAL, "机械臂运动中>>>");
                                OLED_Printf(0, 2*FONT_16+16, FONT_16,OLED_NORMAL, "误差: %.3f mm", total_err);
                            break;
                    }
                    break;
                case 2: // 参数设置页面的四级菜单
                    OLED_Clear();
                    OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return        ");
                    OLED_Serial_Printf(2, OLED_NORMAL, "设置值：%d     ", current_menu->selected_idx);
                    break;
                case 3: // 快捷操作页面的四级菜单
                    OLED_Clear();
                    OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return        ");
                    OLED_Serial_Printf(2, OLED_NORMAL, "执行：%s      ",
                        current_menu->level3_idx==2?"回零位":
                        current_menu->level3_idx==3?"校准":
                        current_menu->level3_idx==4?"画正方形":"示教");
                    break;
                default:
                    OLED_Serial_Printf(1, OLED_NORMAL, "无数据        ");
                    break;
            }
            OLED_ShowFrame();

            //按键扫描
            key_event = get_key();
            if (key_event==KEY_EVENT_NEXT) {
                current_menu->selected_idx++;
                if (current_menu->selected_idx > current_menu->idx_max) {
                    current_menu->selected_idx = 1;
                }

            }
            if (key_event==KEY_EVENT_ENTER) {
                if (current_menu->selected_idx==1) {
                    // 返回三级菜单
                    current_menu->current_level = MENU_LEVEL_3;
                    current_menu->selected_idx = current_menu->level3_idx; // 恢复三级菜单选中状态
                } else {
                    // 可扩展：执行具体操作
                    OTTO_uart(&huart_debug,"执行四级菜单操作：%d",current_menu->selected_idx);
                }
                break;
            }
        }
    }
}

void oled_loop() {
    // 根据当前层级执行对应菜单逻辑（原代码顺序执行会导致层级切换异常）
    switch (current_menu.current_level) {
        case MENU_LEVEL_1:
            level_1(&current_menu);
            break;
        case MENU_LEVEL_2:
            level_2(&current_menu);
            break;
        case MENU_LEVEL_3:
            level_3(&current_menu);
            break;
        case MENU_LEVEL_4:
            level_4(&current_menu);
            break;
        default:
            current_menu.current_level = MENU_LEVEL_1; // 异常层级重置
            break;
    }
}