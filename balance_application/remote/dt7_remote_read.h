
/*读取DT7的遥控器数据*/
/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef DT7_REMOTE_READ_H
#define DT7_REMOTE_READ_H
#include <stdint.h>

/**********************遥控器用宏定义*************************/
#define RC_FRAME_LENGTH 18u  //DT7一帧数据长度

#define RC_CH_VALUE_MIN         ((uint16_t)364)  
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024) //遥控器中值
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
#define DT7_CHANNEL_MARGIN      40

#define DT7_CHANNEL_LEFT_X      2U
#define DT7_CHANNEL_LEFT_Y      3U
#define DT7_CHANNEL_THUMBWHEEL  4U
#define DT7_SWITCH_RIGHT        0U
#define DT7_SWITCH_LEFT         1U

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

typedef struct
{
    struct
    {
        int16_t ch[5];
        //右横 右纵 左横 左纵
        uint8_t s[2]; // 拨杆，右侧在前
    } rc; //遥控器

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
        uint8_t press_l_last;
        uint8_t press_r_last;
    } mouse; //鼠标

    struct
    {
        uint16_t v;
    } key; //键盘

} RC_ctrl_t;
typedef struct
{
    RC_ctrl_t input;
    uint32_t received_at_ms;   // 最近一帧通过校验的接收时刻
    uint8_t valid;          // 最新接收结果的整帧有效标志
    uint8_t thumbwheel_valid; // 拨轮通道单独校验的结果
} dt7_received_frame_t;

extern volatile RC_ctrl_t dt7;
extern volatile uint16_t wheel_raw_dbg;

void dt7_remote_read_init(void);
void dt7_read_received_frame(dt7_received_frame_t *received_frame);

#endif
