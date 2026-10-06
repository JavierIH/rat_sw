#include "sysclock.h"
#include "error.h"
#include "motor.h"

// Crystal x 9 = 72 MHz, else HSI / 2 x 16 = 64 MHz (the CSS watches the crystal); the HSI is then stopped (flash_store.c).

static volatile clock_source_t source = CLOCK_HSE;
static volatile uint8_t crystal_failed;     // CSS fired: the main context must bring the PLL back

static uint8_t pll_from_hsi(void){
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_OFF;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    return HAL_RCC_OscConfig(&osc) == HAL_OK;
}

// System clock from the PLL, buses and SysTick (1 ms, highest priority).
static uint8_t clocks_from_pll(void){
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) return 0;
    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    periph.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if(HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) return 0;
    SystemCoreClockUpdate();
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000u);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    return 1;
}

void SystemClock_Config(void){
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    if(HAL_RCC_OscConfig(&osc) == HAL_OK) source = CLOCK_HSE;
    else if(pll_from_hsi()) source = CLOCK_HSI_BOOT;
    else Error_Handler();
    if(!clocks_from_pll()) Error_Handler();
    if(source == CLOCK_HSE){
        HAL_RCC_EnableCSS();
        __HAL_RCC_HSI_DISABLE();    // a stopped HSI cannot be left crawling: see above
    }
}

clock_source_t sysclock_source(void){
    return source;
}

// Reaction to a crystal failure, from the NMI: motion.c stops the control and aborts the run.
__weak void clock_failure_hook(void){
    motor_emergency_stop();
}

// NMI, clock security system: the crystal stopped; the hardware switched to the HSI (timing 9x off until recovered).
void HAL_RCC_CSSCallback(void){
    source = CLOCK_HSI_FAILED;
    crystal_failed = 1;
    clock_failure_hook();
}

uint8_t sysclock_recover(void){
    if(!crystal_failed) return 0;
    crystal_failed = 0;
    if(pll_from_hsi()) clocks_from_pll();   // else it stays at 8 MHz: slow, but alive
    return 1;
}
