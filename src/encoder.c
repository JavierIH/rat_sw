#include "encoder.h"
#include "stm32f1xx_hal.h"
#include "error.h"

// Input filter: 6 samples at fDTS/4 (333 ns) must agree. Rejects motor PWM
// noise spikes, orders of magnitude shorter than real encoder edges.
#define ENCODER_INPUT_FILTER 6

static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim2;

static volatile int32_t total_l, total_r;
static volatile uint32_t last_motion_ms;
static uint16_t last_l, last_r;

static void encoder_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance){
    TIM_Encoder_InitTypeDef config = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim->Instance = instance;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    config.EncoderMode = TIM_ENCODERMODE_TI12;
    config.IC1Polarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC1Prescaler = TIM_ICPSC_DIV1;
    config.IC1Filter = ENCODER_INPUT_FILTER;
    config.IC2Polarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC2Prescaler = TIM_ICPSC_DIV1;
    config.IC2Filter = ENCODER_INPUT_FILTER;
    if(HAL_TIM_Encoder_Init(htim, &config) != HAL_OK) Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(htim, &master) != HAL_OK) Error_Handler();
}

void ENCODER_Init(void){
    encoder_timer_init(&htim1, TIM1);
    encoder_timer_init(&htim2, TIM2);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    last_l = (uint16_t)TIM1->CNT;
    last_r = (uint16_t)TIM2->CNT;
}

// Called every ms: a 16-bit counter cannot move 32768 ticks in that time, so
// the signed difference is always the true increment.
void encoder_tick(void){
    uint16_t l = (uint16_t)TIM1->CNT;
    uint16_t r = (uint16_t)TIM2->CNT;
    if(l != last_l || r != last_r) last_motion_ms = HAL_GetTick();
    total_l -= (int16_t)(uint16_t)(l - last_l);     // left counter runs backwards when driving forward
    total_r += (int16_t)(uint16_t)(r - last_r);
    last_l = l;
    last_r = r;
}

uint32_t encoder_idle_ms(void){
    return HAL_GetTick() - last_motion_ms;
}

int32_t encoder_total(encoder_t encoder){
    return encoder == ENCODER_L ? total_l : total_r;
}

uint16_t encoder_raw(encoder_t encoder){
    if(encoder == ENCODER_L) return (uint16_t)(0u - TIM1->CNT);
    return (uint16_t)TIM2->CNT;
}
