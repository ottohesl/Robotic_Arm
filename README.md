# 机械臂
<img width="1260" height="2800" alt="5a6d42bf1186d4bdcb4b519284446994" src="https://github.com/user-attachments/assets/26d284b1-2696-4069-bd63-37a1cccc0a44" />
<img width="1260" height="2800" alt="116149094b39a1e006b885732848cf5e" src="https://github.com/user-attachments/assets/8289fef9-57ef-4fe1-8fdc-fd0e10cc572f" />

*目前搁置了，因为3d打印件太松散，然后没有太多的资金用于cnc加工（😭电机和减速器好贵）*

---
## 硬件
整机机械臂控制板采用 STM32H723 作为核心运算 MCU，搭载 ESP32 实现无线交互，依托 FreeRTOS 实时操作系统搭建多任务调度框架，完成全套硬件电路与运动控制逻辑开发。  

<img width="907" height="742" alt="image" src="https://github.com/user-attachments/assets/229ed812-2c08-4c60-9e56-02c6605ff90f" />

<img width="1433" height="906" alt="image" src="https://github.com/user-attachments/assets/8a741bc7-088b-46d7-80cf-2192bf11883f" />

### 1.选型
为了机械臂需要有足够的精度、力矩，我们本想直接6个张大头的步进电机，但是光电机不够还要加谐波减速器，所以直接超预算了。但是手上刚好有3个达妙电机，于是买了3个zdt的步进电机加一个谐波减速器。  
当前的热门是数字孪生，所以光一个能动的机械臂不够，需要在电脑上利用各种方式连接到机械臂，比如实时获取机械臂的姿态、每个电机角度，甚至直接实时电脑控制机械臂。那么在原本设定的主控上面需要再加一个ESP32，使用网络连接。
### 2.电路
· 因为电机驱动电压需要24v，所以电源输入就24v的输入，然后使用dcdc降压芯片降成5v，ldo降成3.3v给各模块供电。
· 因为电机都是使用can、fdcan控制，所以电路设计上can路线就是重点；其他的比如usart接口引出、ESP32下载口和USB应用typec口引出也完成了。  

*总的来说外设并不多，但是重点在于PCB的防串扰，需保证地平面完整、电源分层合理、fdcan的信号完整性*
