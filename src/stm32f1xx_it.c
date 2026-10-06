/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "motor.h"

extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;

// Hook run every 1 ms from SysTick (main.c overrides it).
__weak void app_systick(void){}

// Faults: the motors stopped at register level and a slow LED blink (Error_Handler blinks fast).
static void fault_halt(void){
    motor_emergency_stop();
    for(;;){
        GPIOB->ODR ^= (1u << 13) | (1u << 14) | (1u << 15);
        GPIOA->ODR ^= (1u << 3) | (1u << 4) | (1u << 5);
        for(volatile uint32_t i = 0; i < 2000000u; i++){}
    }
}

// Clock security system: the crystal failed (sysclock.c).
void NMI_Handler(void){ HAL_RCC_NMI_IRQHandler(); }
void HardFault_Handler(void){ fault_halt(); }
void MemManage_Handler(void){ fault_halt(); }
void BusFault_Handler(void){ fault_halt(); }
void UsageFault_Handler(void){ fault_halt(); }
void SVC_Handler(void){}
void DebugMon_Handler(void){}
void PendSV_Handler(void){}

// SysTick with the stacked frame (health.c looks at where the main program was).
__weak void app_stall_check(const uint32_t *frame){ (void)frame; }
__weak void app_tick_time(uint32_t cycles){ (void)cycles; }     // how long app_systick() took

__attribute__((used)) void systick_with_frame(const uint32_t *frame){
    HAL_IncTick();
    app_stall_check(frame);
    const uint32_t t0 = DWT->CYCCNT;
    app_systick();
    app_tick_time(DWT->CYCCNT - t0);
}

__attribute__((naked)) void SysTick_Handler(void){
    __asm volatile(
        "tst lr, #4         \n"
        "ite eq             \n"
        "mrseq r0, msp      \n"
        "mrsne r0, psp      \n"
        "b systick_with_frame \n");
}

// USART3 TX DMA.
void DMA1_Channel2_IRQHandler(void){
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

void USART3_IRQHandler(void){
    HAL_UART_IRQHandler(&huart3);
}
