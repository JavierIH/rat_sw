#include "infrared.h"
#include "stm32f1xx_hal.h"
#include "error.h"

#define IR_MAX_MM 400.0f

DMA_HandleTypeDef hdma_adc1;
static ADC_HandleTypeDef hadc1;

// Conversion k of scan position p lands in adc_buf[k * IR_COUNT + p].
static volatile uint16_t adc_buf[IR_OVERSAMPLE * IR_COUNT];

// Scan order is CH6 (PA6), CH7 (PA7), CH8 (PB0), CH9 (PB1).
static const uint32_t SCAN_CHANNEL[IR_COUNT] = {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9};
static const uint8_t SCAN_POS[IR_COUNT] = {
    [IR_FL] = 3,    // CH9
    [IR_FR] = 0,    // CH6
    [IR_SL] = 2,    // CH8
    [IR_SR] = 1,    // CH7
};

// Calibration: mm = a*x^3 + b*x^2 + c*x + d, x = raw ADC counts (fitted on
// the robot; raw tables in calib.txt). Monotonic over the whole ADC range.
typedef struct { float a, b, c, d; } cubic_t;
static const cubic_t CALIBRATION[IR_COUNT] = {
    [IR_FL] = {-0.00000002278f, 0.000132f,  -0.2627f, 237.7f},
    [IR_FR] = {-0.00000003535f, 0.0001995f, -0.3834f, 317.6f},
    [IR_SL] = {-0.00000005219f, 0.0002629f, -0.4566f, 325.6f},
    [IR_SR] = {-0.00000003241f, 0.0001505f, -0.25f,   189.0f},
};

void IR_Init(void){
    ADC_ChannelConfTypeDef channel = {0};

    __HAL_RCC_DMA1_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = IR_COUNT;
    if(HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();     // pins + DMA: HAL_ADC_MspInit

    channel.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    for(uint8_t i = 0; i < IR_COUNT; i++){
        channel.Channel = SCAN_CHANNEL[i];
        channel.Rank = i + 1u;
        if(HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) Error_Handler();
    }
    if(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler();

    // Circular DMA refreshes the buffer every ~450 us forever. Its interrupts
    // are left disabled in the NVIC: nothing needs them, and they used to
    // fire ~71000 times per second.
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, IR_OVERSAMPLE * IR_COUNT) != HAL_OK) Error_Handler();
}

static uint32_t raw_sum(ir_sensor_t ir){
    uint32_t sum = 0;
    for(uint8_t k = 0; k < IR_OVERSAMPLE; k++) sum += adc_buf[k * IR_COUNT + SCAN_POS[ir]];
    return sum;
}

uint16_t ir_raw(ir_sensor_t ir){
    if((unsigned)ir >= IR_COUNT) return 0;
    return (uint16_t)((raw_sum(ir) + IR_OVERSAMPLE / 2) / IR_OVERSAMPLE);
}

// Single precision on purpose: the Cortex-M3 has no FPU and double math is
// several times slower in software. This runs in the 100 Hz control loop.
float ir_mm(ir_sensor_t ir){
    if((unsigned)ir >= IR_COUNT) return IR_MAX_MM;
    const cubic_t *k = &CALIBRATION[ir];
    float x = (float)raw_sum(ir) * (1.0f / IR_OVERSAMPLE);
    float mm = ((k->a * x + k->b) * x + k->c) * x + k->d;
    if(mm < 0.0f) return 0.0f;
    return mm > IR_MAX_MM ? IR_MAX_MM : mm;
}
