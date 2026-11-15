#include "chassis_data_task.h"
#include "can_user.h"
#include "dt7_control.h"
#include "FreeRTOS.h"
#include "task.h"

#define CHASSIS_DATA_DELAY 300U
#define REMOTE_SWITCH_PERIOD_MS 10U

void Chassis_Data_Task(void const *argument)
{
    const uint32_t start_ms = HAL_GetTick();
    dt7_control_snapshot_t control;
    uint32_t last_switch_send_ms = start_ms;

    (void)argument;

    for (;;)
    {
        const uint32_t now = HAL_GetTick();
        // 任务启动后立即刷新遥控快照，CAN 发包仍等待原来的 300 ms。
        dt7_update_control_snapshot();
        if (now - start_ms >= CHASSIS_DATA_DELAY)
        {
            dt7_read_control_snapshot(&control);
            CAN_gimbal_send_chassis(&control.chassis);

            if (now - last_switch_send_ms >= REMOTE_SWITCH_PERIOD_MS)
            {
                // 旧 0x401 定时转发左拨杆。爬阶指令随 0x307 发送。
                CAN_send_remote_to_chassis(control.input.rc.s[DT7_SWITCH_LEFT]);
                last_switch_send_ms = now;
            }
        }

        vTaskDelay(1);
    }
}
