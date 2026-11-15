#ifndef DT7_CONTROL_H
#define DT7_CONTROL_H

#include "dt7_remote_read.h"

#define DT7_COMPETITION_TOGGLE_KEY KEY_PRESSED_OFFSET_R

typedef enum
{
    DT7_GIMBAL_OFF = 0,
    DT7_GIMBAL_MANUAL,
    DT7_GIMBAL_AUTO
} dt7_gimbal_mode_t;

typedef enum
{
    DT7_SHOOT_BLOCKED = 0,
    DT7_SHOOT_NORMAL,
    DT7_SHOOT_COMPETITION
} dt7_shoot_mode_t;

// 0x307 data[5] bit 7..6 仍使用原底盘模式编码。
enum
{
    DT7_DRIVE_STOP = 0,
    DT7_DRIVE_NO_FOLLOW = 2,
    DT7_DRIVE_FOLLOW = 3
};

/*
 * 0x307 data[6] 的位布局
 * bit 1..0  机构选择，0 前组，1 麦轮，2 后组
 * bit 2..5  依次为上台阶、底盘停止、DBUS 有效、协议布局标记
 * bit 7..6  侧别，00 双侧，01 左侧，10 右侧
 */
enum
{
    DT7_CLIMB_FRONT = 0,
    DT7_CLIMB_WHEELS = 1,
    DT7_CLIMB_REAR = 2,
    DT7_CAN_CLIMB = 1U << 2,
    DT7_CAN_STOP = 1U << 3,
    DT7_CAN_DBUS_VALID = 1U << 4,
    DT7_CAN_LAYOUT_V1 = 1U << 5,
    DT7_CAN_SIDE_SHIFT = 6
};

typedef enum
{
    DT7_SIDE_BOTH = 0,
    DT7_SIDE_LEFT = 1,
    DT7_SIDE_RIGHT = 2
} dt7_side_t;

typedef struct
{
    int16_t forward;    // data[0..1]，麦轮前后量，高字节在前
    int16_t lateral;    // data[2..3]，麦轮横移量，高字节在前
    uint8_t drive_mode; // data[5] bit 7..6，原底盘模式
    uint8_t cap_mode;   // data[5] bit 5，超电请求
    uint8_t status_flags; // data[6]，协议状态、机构和侧别
    int8_t joint_motion_request; // data[7]，机构伸收量 -100..100，0 表示保持
} dt7_chassis_command_t;

typedef struct
{
    RC_ctrl_t input;
    dt7_gimbal_mode_t gimbal_mode;
    dt7_shoot_mode_t shoot_mode;
    dt7_chassis_command_t chassis;
} dt7_control_snapshot_t;

// 由底盘发包任务更新，云台和发射任务分别读取同一份快照。
void dt7_update_control_snapshot(void);
void dt7_read_control_snapshot(dt7_control_snapshot_t *snapshot);

#endif
