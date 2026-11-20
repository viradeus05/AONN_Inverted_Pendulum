#include "sys.h"
#include "host_control.h"
#include "host_serial.h"

#define SERIAL_BAUD 115200u
#define MOTOR_PWM_PERIOD 7200u
#define MOTOR_PWM_LIMIT 6900
#define CONTROL_PERIOD_US 5000u
/* One tick receives at most 58 bytes at 115200 8N1, about five commands. */
#define MAX_COMMANDS_PER_TICK 8u

static HostControl control;
static uint32_t tick;
static uint32_t time_us;
static uint16_t last_timer_count;

/* TIM2 is a free-running 1 MHz clock. Extend its 16-bit count at each sample. */
static void start_sample_clock(void)
{
    TIM_TimeBaseInitTypeDef timer;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000u - 1u);
    timer.TIM_Period = UINT16_MAX;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_Cmd(TIM2, ENABLE);
    last_timer_count = TIM_GetCounter(TIM2);
}

void TIM1_UP_IRQHandler(void)
{
    HostCommand command;
    HostState state;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == RESET)
        return;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    uint16_t now = TIM_GetCounter(TIM2);
    time_us += (uint16_t)(now - last_timer_count);
    last_timer_count = now;
    state.tick = ++tick;
    state.time_us = time_us;
    state.encoder = Read_Encoder(4);
    state.angle_adc = Get_Adc(3);
    state.voltage_cv = (uint16_t)Get_battery_volt();
    host_control_tick(&control, tick, state.encoder, state.voltage_cv, KEY5 == 0);

    for (unsigned i = 0; i < MAX_COMMANDS_PER_TICK && host_read_command(&command); ++i)
        host_control_command(&control, tick, &command);

    Set_Pwm(control.applied_effort * MOTOR_PWM_LIMIT / 1000);
    LED = (control.flags & HOST_FLAG_ARMED) == 0;  /* Active-low board LED. */
    state.flags = control.flags;
    state.applied_effort = control.applied_effort;
    state.last_command_tick = control.last_command_tick;
    host_send_state(&state);
}

int main(void)
{
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    MiniBalance_PWM_Init(MOTOR_PWM_PERIOD - 1u, 0);
    Set_Pwm(0);
    LED_Init();
    KEY_Init();
    Encoder_Init_TIM4();
    Adc_Init();
    host_control_init(&control);
    host_serial_init(SERIAL_BAUD);
    start_sample_clock();
    Timer1_Init(CONTROL_PERIOD_US - 1u, (uint16_t)(SystemCoreClock / 1000000u - 1u));
    for (;;)
        __WFI();
}
