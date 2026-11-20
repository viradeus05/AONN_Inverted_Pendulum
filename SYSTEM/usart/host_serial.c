#include "host_serial.h"
#include "stm32f10x.h"

/* USART1 uses DMA1 channel 4 for TX and channel 5 for RX on STM32F103. */
#define RX_CAPACITY 256u
#define RX_BOUNDARIES (DMA_ISR_HTIF5 | DMA_ISR_TCIF5)

static volatile uint8_t rx_buffer[RX_CAPACITY];
static uint8_t tx_frame[HOST_STATE_SIZE];
static unsigned rx_read_pos;
static unsigned rx_last_write_pos;
static unsigned rx_count;

static void start_receiver(void)
{
    DMA1_Channel5->CCR = 0;
    /* The SR then DR reads clear any framing/overrun error before DMA restarts. */
    (void)USART1->SR;
    (void)USART1->DR;
    DMA1->IFCR = DMA_IFCR_CGIF5;
    DMA1_Channel5->CPAR = (uint32_t)(uintptr_t)&USART1->DR;
    DMA1_Channel5->CMAR = (uint32_t)(uintptr_t)rx_buffer;
    DMA1_Channel5->CNDTR = RX_CAPACITY;
    rx_read_pos = 0;
    rx_last_write_pos = 0;
    rx_count = 0;
    DMA1_Channel5->CCR = DMA_CCR1_PL_1 | DMA_CCR1_MINC | DMA_CCR1_CIRC | DMA_CCR1_EN;
}

void host_serial_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_DeInit(USART1);
    USART_StructInit(&uart); /* 8 data bits, no parity, one stop bit, RX and TX. */
    uart.USART_BaudRate = baud;
    USART_Init(USART1, &uart);
    DMA1_Channel4->CCR = 0;
    DMA1_Channel4->CPAR = (uint32_t)(uintptr_t)&USART1->DR;
    DMA1_Channel4->CMAR = (uint32_t)(uintptr_t)tx_frame;
    start_receiver();
    USART_DMACmd(USART1, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

static void update_receiver(void)
{
    if ((DMA1->ISR & DMA_ISR_TEIF5) ||
        (USART1->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))) {
        start_receiver();
        return;
    }

    uint32_t boundaries = DMA1->ISR & RX_BOUNDARIES;
    DMA1->IFCR = boundaries;
    unsigned write_pos = (RX_CAPACITY - DMA1_Channel5->CNDTR) % RX_CAPACITY;
    unsigned advanced = (write_pos + RX_CAPACITY - rx_last_write_pos) % RX_CAPACITY;
    rx_last_write_pos = write_pos;
    rx_count += advanced;

    /* At 115200 baud a 5 ms tick receives at most 58 bytes. If both halves
       turned over between polls, or unread data approaches a lap, discard the
       backlog. This leaves room for DMA to advance while a frame is copied. */
    if (boundaries == RX_BOUNDARIES || rx_count > RX_CAPACITY - HOST_COMMAND_SIZE) {
        rx_read_pos = write_pos;
        rx_count = 0;
    }
}

int host_read_command(HostCommand *command)
{
    uint8_t frame[HOST_COMMAND_SIZE];
    update_receiver();
    /* rx_count is a snapshot. Incoming data cannot extend this scan. */
    while (rx_count >= HOST_COMMAND_SIZE) {
        if (rx_buffer[rx_read_pos] == HOST_SYNC_0) {
            for (unsigned i = 0; i < HOST_COMMAND_SIZE; ++i) {
                frame[i] = rx_buffer[(rx_read_pos + i) % RX_CAPACITY];
            }
            if (host_decode_command(frame, command)) {
                rx_read_pos = (rx_read_pos + HOST_COMMAND_SIZE) % RX_CAPACITY;
                rx_count -= HOST_COMMAND_SIZE;
                return 1;
            }
        }
        rx_read_pos = (rx_read_pos + 1) % RX_CAPACITY;
        --rx_count;
    }
    return 0;
}

void host_send_state(const HostState *state)
{
    /* DMA owns tx_frame until the last byte has been loaded into USART1. If
       a transfer is still going, drop this sample instead of delaying control. */
    if ((DMA1_Channel4->CCR & DMA_CCR1_EN) && DMA1_Channel4->CNDTR != 0 &&
        !(DMA1->ISR & DMA_ISR_TEIF4)) {
        return;
    }
    DMA1_Channel4->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF4;
    host_encode_state(state, tx_frame);
    __DMB();
    DMA1_Channel4->CNDTR = HOST_STATE_SIZE;
    DMA1_Channel4->CCR = DMA_CCR1_MINC | DMA_CCR1_DIR | DMA_CCR1_EN;
}
