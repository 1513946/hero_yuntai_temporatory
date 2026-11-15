#include "dt7_remote_read.h"
#include "double_dma_init.h"
#include "usart.h"

// 两块 DMA 缓冲各开 36 字节。初始化和每次重装都用这个容量。
#define DBUS_DMA_BUFFER_LENGTH (2U * RC_FRAME_LENGTH)
#define DBUS_RECEIVE_ERROR_FLAGS (UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE)

volatile RC_ctrl_t dt7;
volatile uint16_t wheel_raw_dbg;
static volatile uint32_t received_at_ms;
static volatile uint8_t frame_valid;
static volatile uint8_t thumbwheel_valid;

static uint8_t dbus_receive_buffers[2][DBUS_DMA_BUFFER_LENGTH];

static uint16_t read_little_endian_uint16(const volatile uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static void decode_frame(const volatile uint8_t *data)
{
    RC_ctrl_t input = {0};
    uint16_t axes[4];
    uint8_t index;

    // data[0..5] 打包了四路 11 位摇杆量和两路 2 位拨杆值。
    axes[0] = read_little_endian_uint16(&data[0]) & 0x07FFU;
    axes[1] = (read_little_endian_uint16(&data[1]) >> 3U) & 0x07FFU;
    axes[2] = ((data[2] >> 6U) | (data[3] << 2U) | (data[4] << 10U)) & 0x07FFU;
    axes[3] = (read_little_endian_uint16(&data[4]) >> 1U) & 0x07FFU;
    input.rc.s[DT7_SWITCH_RIGHT] = (data[5] >> 4U) & 0x03U;
    input.rc.s[DT7_SWITCH_LEFT] = (data[5] >> 6U) & 0x03U;

    // 旧工程使用末两字节作为拨轮，单独校验，避免保留字节被当作单侧选择。
    wheel_raw_dbg = read_little_endian_uint16(&data[16]);
    frame_valid = 0U;
    if (input.rc.s[DT7_SWITCH_LEFT] == 0U || input.rc.s[DT7_SWITCH_RIGHT] == 0U)
    {
        return;
    }
    for (index = 0U; index < 4U; ++index)
    {
        if (axes[index] < RC_CH_VALUE_MIN - DT7_CHANNEL_MARGIN ||
            axes[index] > RC_CH_VALUE_MAX + DT7_CHANNEL_MARGIN)
        {
            return;
        }
        input.rc.ch[index] = (int16_t)axes[index] - RC_CH_VALUE_OFFSET;
    }

    thumbwheel_valid = (wheel_raw_dbg >= RC_CH_VALUE_MIN - DT7_CHANNEL_MARGIN &&
                   wheel_raw_dbg <= RC_CH_VALUE_MAX + DT7_CHANNEL_MARGIN);
    if (thumbwheel_valid)
    {
        input.rc.ch[DT7_CHANNEL_THUMBWHEEL] = (int16_t)wheel_raw_dbg - RC_CH_VALUE_OFFSET;
    }
    input.mouse.x = (int16_t)read_little_endian_uint16(&data[6]);
    input.mouse.y = (int16_t)read_little_endian_uint16(&data[8]);
    input.mouse.z = (int16_t)read_little_endian_uint16(&data[10]);
    input.mouse.press_l_last = dt7.mouse.press_l;
    input.mouse.press_r_last = dt7.mouse.press_r;
    input.mouse.press_l = data[12];
    input.mouse.press_r = data[13];
    input.key.v = read_little_endian_uint16(&data[14]);

    // 完成校验后一起发布输入和接收时间，超时从这帧开始计时。
    dt7 = input;
    received_at_ms = HAL_GetTick();
    frame_valid = 1U;
}

void dt7_remote_read_init(void)
{
    const uint32_t saved_interrupt_mask = __get_PRIMASK();
    __disable_irq();
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    Double_dma_init(&huart3, huart3.hdmarx,
                    dbus_receive_buffers[0], dbus_receive_buffers[1], DBUS_DMA_BUFFER_LENGTH);
    __set_PRIMASK(saved_interrupt_mask);
}

void dt7_read_received_frame(dt7_received_frame_t *received_frame)
{
    // 复制时暂时屏蔽中断，让遥控量、时间戳和有效标志来自同一帧。
    const uint32_t saved_interrupt_mask = __get_PRIMASK();
    __disable_irq();
    received_frame->input = dt7;
    received_frame->received_at_ms = received_at_ms;
    received_frame->valid = frame_valid;
    received_frame->thumbwheel_valid = thumbwheel_valid;
    __set_PRIMASK(saved_interrupt_mask);
}

void USART3_IRQHandler(void)
{
    const uint32_t status = huart3.Instance->SR;
    DMA_HandleTypeDef *const receive_dma = huart3.hdmarx;
    DMA_Stream_TypeDef *const dma = receive_dma->Instance;
    uint32_t length;
    uint8_t buffer;

    if ((status & (UART_FLAG_IDLE | DBUS_RECEIVE_ERROR_FLAGS)) == 0U)
    {
        return;
    }

    // 读取 SR 后读取 DR 清除空闲及接收错误标志。
    (void)huart3.Instance->DR;
    __HAL_DMA_DISABLE(receive_dma);
    while ((dma->CR & DMA_SxCR_EN) != 0U)
    {
    }
    // NDTR 是剩余接收数，用缓冲容量减去它得到本次实际长度。
    buffer = (dma->CR & DMA_SxCR_CT) != 0U;
    length = DBUS_DMA_BUFFER_LENGTH - dma->NDTR;
    dma->NDTR = DBUS_DMA_BUFFER_LENGTH;
    // 先切到另一块缓冲继续接收，再解包刚结束的缓冲区。
    dma->CR ^= DMA_SxCR_CT;
    __HAL_DMA_ENABLE(receive_dma);

    if ((status & DBUS_RECEIVE_ERROR_FLAGS) == 0U && length == RC_FRAME_LENGTH)
    {
        decode_frame(dbus_receive_buffers[buffer]);
    }
    else
    {
        frame_valid = 0U;
    }
}
