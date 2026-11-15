#include "dt7_control.h"
#include "main.h"

#include <stdbool.h>

#define DT7_LINK_TIMEOUT_MS 60U        // 从最后一帧有效 DBUS 开始计时
#define DT7_SELECTION_STABLE_MS 30U    // 换档或换侧后所需的有效帧时间跨度

// 死区和拨轮阈值使用减去中值 1024 后的通道量。
#define DT7_DRIVE_DEADBAND 10
#define DT7_CLIMB_WHEEL_DEADBAND 30
#define DT7_JOINT_STICK_DEADBAND 100
#define DT7_JOINT_MOTION_REQUEST_MAX 100
#define DT7_THUMBWHEEL_ENTER_THRESHOLD 300
#define DT7_THUMBWHEEL_EXIT_THRESHOLD 200
#define DT7_THUMBWHEEL_CENTER_BAND 100
#define DT7_SUPERCAPACITOR_REQUEST_THRESHOLD 300

typedef struct
{
    uint32_t selection_started_at_ms;
    uint32_t last_frame_received_at_ms;
    uint8_t left_switch;
    uint8_t right_switch;
    dt7_side_t motor_side;
    bool motion_ready;  // 本次选择已等到左摇杆回中
    bool shoot_ready;   // 本次选择已等到拨轮和相关按键释放
} dt7_mode_selection_t;

static dt7_mode_selection_t mode_selection;
static volatile dt7_control_snapshot_t latest_control_snapshot;

static bool is_centered(int16_t value, int16_t deadband)
{
    return value >= -deadband && value <= deadband;
}

static dt7_side_t select_climb_motor_side(int16_t thumbwheel_input, dt7_side_t previous_side)
{
    // 拨轮达到 ±300 进入单侧，回到 ±200 以内退出。滞回区间沿用同侧选择。
    if (thumbwheel_input >= DT7_THUMBWHEEL_ENTER_THRESHOLD ||
        (previous_side == DT7_SIDE_LEFT && thumbwheel_input > DT7_THUMBWHEEL_EXIT_THRESHOLD))
    {
        return DT7_SIDE_LEFT;
    }
    if (thumbwheel_input <= -DT7_THUMBWHEEL_ENTER_THRESHOLD ||
        (previous_side == DT7_SIDE_RIGHT && thumbwheel_input < -DT7_THUMBWHEEL_EXIT_THRESHOLD))
    {
        return DT7_SIDE_RIGHT;
    }
    return DT7_SIDE_BOTH;
}

static int8_t scale_joint_motion_request(int16_t value)
{
    // 扣除死区后把剩余行程映射到 ±100，DM 底盘再按机构方向和限位执行。
    const int32_t full_scale = RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET;
    const int32_t magnitude = value < 0 ? -(int32_t)value : value;
    int32_t joint_motion_request;

    if (magnitude <= DT7_JOINT_STICK_DEADBAND)
    {
        return 0;
    }
    joint_motion_request = (magnitude - DT7_JOINT_STICK_DEADBAND) * DT7_JOINT_MOTION_REQUEST_MAX /
          (full_scale - DT7_JOINT_STICK_DEADBAND);
    if (joint_motion_request > DT7_JOINT_MOTION_REQUEST_MAX)
    {
        joint_motion_request = DT7_JOINT_MOTION_REQUEST_MAX;
    }
    return (int8_t)(value < 0 ? -joint_motion_request : joint_motion_request);
}

/*
 * 左上在普通行驶档选自瞄并保留原 DT7 发射入口，双上使用原比赛入口。
 * 左中选手动云台。左上或左中时，右中跟随，右下不跟随，右上停车。
 * 左下进入上台阶。右上选前组，右中选麦轮低速挪车，右下选后组。
 */
static void build_control_snapshot(const dt7_received_frame_t *received_frame,
                                   uint32_t now_ms, dt7_control_snapshot_t *next_snapshot)
{
    const uint8_t left_switch = received_frame->input.rc.s[DT7_SWITCH_LEFT];
    const uint8_t right_switch = received_frame->input.rc.s[DT7_SWITCH_RIGHT];
    const int16_t forward = received_frame->input.rc.ch[DT7_CHANNEL_LEFT_Y];
    const int16_t lateral = received_frame->input.rc.ch[DT7_CHANNEL_LEFT_X];
    const int16_t thumbwheel_input = received_frame->input.rc.ch[DT7_CHANNEL_THUMBWHEEL];
    const bool climb_mode_selected = switch_is_down(left_switch);
    const bool joint_control_selected = climb_mode_selected && !switch_is_mid(right_switch);
    const bool competition_mode_selected = switch_is_up(left_switch) && switch_is_up(right_switch);
    const int16_t deadband = climb_mode_selected ? DT7_CLIMB_WHEEL_DEADBAND : DT7_DRIVE_DEADBAND;
    dt7_side_t motor_side = DT7_SIDE_BOTH;

    // 默认底盘停止且动作量为零。通过有效性和换档检查后再写入动作。
    next_snapshot->chassis.status_flags = DT7_CAN_LAYOUT_V1 | DT7_CAN_STOP;
    if (!received_frame->valid || now_ms - received_frame->received_at_ms >= DT7_LINK_TIMEOUT_MS)
    {
        mode_selection = (dt7_mode_selection_t){0};
        return;
    }

    next_snapshot->chassis.status_flags |= DT7_CAN_DBUS_VALID;
    next_snapshot->input = received_frame->input;
    next_snapshot->gimbal_mode = DT7_GIMBAL_MANUAL;
    if (climb_mode_selected)
    {
        // 上台阶时右摇杆直接控制云台，沿用现有 IMU 稳定。
        next_snapshot->input.mouse.x = 0;
        next_snapshot->input.mouse.y = 0;
    }

    if (joint_control_selected)
    {
        if (!received_frame->thumbwheel_valid)
        {
            mode_selection = (dt7_mode_selection_t){0};
            return;
        }
        const dt7_side_t previous_side =
            (mode_selection.left_switch == left_switch && mode_selection.right_switch == right_switch) ?
            mode_selection.motor_side : DT7_SIDE_BOTH;
        motor_side = select_climb_motor_side(thumbwheel_input, previous_side);
    }

    // 选择变化或丢过一段帧时，清掉旧动作并重新等待。
    if (mode_selection.left_switch != left_switch ||
        mode_selection.right_switch != right_switch ||
        mode_selection.motor_side != motor_side ||
        received_frame->received_at_ms - mode_selection.last_frame_received_at_ms >= DT7_LINK_TIMEOUT_MS)
    {
        mode_selection.selection_started_at_ms = received_frame->received_at_ms;
        mode_selection.left_switch = left_switch;
        mode_selection.right_switch = right_switch;
        mode_selection.motor_side = motor_side;
        mode_selection.motion_ready = false;
        mode_selection.shoot_ready = false;
    }
    mode_selection.last_frame_received_at_ms = received_frame->received_at_ms;
    // 用帧的接收时刻判断持续时间，重复读取同一帧时计时量保持原值。
    if (received_frame->received_at_ms - mode_selection.selection_started_at_ms < DT7_SELECTION_STABLE_MS)
    {
        return;
    }

    if (switch_is_up(left_switch) && (!competition_mode_selected || next_snapshot->input.mouse.press_r == 1U))
    {
        next_snapshot->gimbal_mode = DT7_GIMBAL_AUTO;
    }

    // 发射许可单独判断。左上档等待拨轮回中、鼠标左键和比赛按键松开。
    if (switch_is_up(left_switch))
    {
        if (is_centered(thumbwheel_input, DT7_THUMBWHEEL_CENTER_BAND) &&
            next_snapshot->input.mouse.press_l == 0U &&
            (next_snapshot->input.key.v & DT7_COMPETITION_TOGGLE_KEY) == 0U)
        {
            mode_selection.shoot_ready = true;
        }
        if (mode_selection.shoot_ready)
        {
            next_snapshot->shoot_mode = competition_mode_selected ? DT7_SHOOT_COMPETITION : DT7_SHOOT_NORMAL;
        }
    }

    // 新档位或侧别先等左摇杆两个轴回中，之后才响应新的偏转。
    if (!mode_selection.motion_ready)
    {
        if (!is_centered(forward, deadband) || !is_centered(lateral, deadband))
        {
            return;
        }
        mode_selection.motion_ready = true;
    }
    if (!climb_mode_selected && switch_is_up(right_switch))
    {
        return;
    }

    next_snapshot->chassis.status_flags = DT7_CAN_LAYOUT_V1 | DT7_CAN_DBUS_VALID;
    if (climb_mode_selected)
    {
        // 爬阶帧用 data[6] 分派动作。旧模式字段保持 0，兼容旧底盘的零电流分支。
        next_snapshot->chassis.status_flags |= DT7_CAN_CLIMB;
        if (joint_control_selected)
        {
            const uint8_t group = switch_is_up(right_switch) ? DT7_CLIMB_FRONT : DT7_CLIMB_REAR;
            next_snapshot->chassis.status_flags |= group | (uint8_t)(motor_side << DT7_CAN_SIDE_SHIFT);
            next_snapshot->chassis.joint_motion_request = scale_joint_motion_request(forward);
            return;
        }
        next_snapshot->chassis.status_flags |= DT7_CLIMB_WHEELS;
    }
    else
    {
        next_snapshot->chassis.drive_mode = switch_is_mid(right_switch) ? DT7_DRIVE_FOLLOW : DT7_DRIVE_NO_FOLLOW;
        // 保留普通跟随档的超电请求，其他档不继承旧值。
        next_snapshot->chassis.cap_mode =
            switch_is_mid(right_switch) && thumbwheel_input > DT7_SUPERCAPACITOR_REQUEST_THRESHOLD;
    }

    next_snapshot->chassis.forward = is_centered(forward, deadband) ? 0 : forward;
    next_snapshot->chassis.lateral = is_centered(lateral, deadband) ? 0 : lateral;
}

void dt7_update_control_snapshot(void)
{
    dt7_received_frame_t received_frame;
    dt7_control_snapshot_t next_snapshot = {0};
    uint32_t saved_interrupt_mask;

    dt7_read_received_frame(&received_frame);
    build_control_snapshot(&received_frame, HAL_GetTick(), &next_snapshot);

    // 在短临界区内发布整份快照，任务读到的档位和摇杆量属于同一轮判定。
    saved_interrupt_mask = __get_PRIMASK();
    __disable_irq();
    latest_control_snapshot = next_snapshot;
    __set_PRIMASK(saved_interrupt_mask);
}

void dt7_read_control_snapshot(dt7_control_snapshot_t *snapshot)
{
    const uint32_t saved_interrupt_mask = __get_PRIMASK();
    __disable_irq();
    *snapshot = latest_control_snapshot;
    __set_PRIMASK(saved_interrupt_mask);
}
