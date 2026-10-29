#include "debug_task.h"
#include "ins_task.h"
#include "dt7_remote_read.h"
#include "can_user.h"

extern gimbal_c_send_data_t gimbal_send_chassis_data;
// extern UART_HandleTypeDef huart1;
// extern DMA_HandleTypeDef hdma_usart6_tx;

void Crg_test_Task(void const * argument)
{
    int b=0;
	fp32 channels[CH_COUNT];
	const RC_ctrl_t *remote = get_dt7_remote_control_point();
    const motor_measure_t *yaw_motor = get_motor_gimbal_yaw_measure_point();
	while(1)
	{
		channels[0] = (fp32)remote->rc.ch[0];
        channels[1] = (fp32)remote->rc.ch[1];
        channels[2] = (fp32)remote->rc.ch[2];
        channels[3] = (fp32)remote->rc.ch[3];
		channels[4] = (fp32)remote->rc.ch[4];
        ist8310_data[0]=b++;
		channels[5] = (fp32)yaw_motor->ecd;
        channels[6] = (fp32)gimbal_send_chassis_data.top_mode;
        channels[7] = (fp32)remote->rc.s[0];
		debug_to_vofa(channels, CH_COUNT);
		vTaskDelay(10); //发送频率100hz
	}
}



#include "debug_task.h"
#include <math.h>

/* 可以在 Keil Watch 中观察 */
volatile uint32_t vofa_send_ok = 0;
volatile uint32_t vofa_send_fail = 0;
int a=0;

// void Crg_test_Task(void const *argument)
// {
//     fp32 phase = 0.0f;
//     fp32 channels[CH_COUNT];

//     (void)argument;

//     while (1)
//     {
//         a++;
//         channels[0] = sinf(phase);  /* 正弦波 */
//         channels[1] = cosf(phase);  /* 余弦波 */
//         channels[2] = phase;        /* 锯齿波 */
//         channels[3] = 1.0f;         /* 水平线 */

//         if (debug_to_vofa(channels, CH_COUNT))
//         {
//             vofa_send_ok++;
//         }
//         else
//         {
//             vofa_send_fail++;
//         }

//         phase += 0.06283185f;
//         if (phase >= 6.2831853f)
//         {
//             phase -= 6.2831853f;
//         }

//         osDelay(10);  /* 约每 10 ms 发送一次 */
//     }
// }
