#include "uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "error.h"

#define UART_TX_QUEUE_LEN   8
#define UART_TX_MSG_LEN     140
#define UART_RX_QUEUE_LEN   4
#define UART_RX_LINE_LEN    64
#define PRINT_BUFFER_LEN    120

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

static char tx_queue[UART_TX_QUEUE_LEN][UART_TX_MSG_LEN];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_count;
static volatile uint8_t tx_active;

static uint8_t rx_byte;
static char rx_build[UART_RX_LINE_LEN];
static volatile uint8_t rx_index;
static char rx_queue[UART_RX_QUEUE_LEN][UART_RX_LINE_LEN];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint8_t rx_count;

// Called with interrupts disabled or from the UART/DMA interrupts.
static void tx_drop_head(void){
    tx_head = (uint8_t)((tx_head + 1) % UART_TX_QUEUE_LEN);
    tx_count--;
}

static void uart_start_next_tx(void){
    if(tx_active || tx_count == 0) return;
    tx_active = 1;
    const char *msg = tx_queue[tx_head];
    if(HAL_UART_Transmit_DMA(&huart3, (uint8_t *)msg, (uint16_t)strlen(msg)) != HAL_OK){
        tx_active = 0;
        tx_drop_head();
    }
}

void UART_Init(void){
    huart3.Instance = USART3;
    huart3.Init.BaudRate = UART_BAUDRATE;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart3) != HAL_OK){   // pins, DMA and IRQs: HAL_UART_MspInit (msp.c)
        Error_Handler();
    }
}

void uart_send(const char *text){
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if(tx_count < UART_TX_QUEUE_LEN){
        strncpy(tx_queue[tx_tail], text, UART_TX_MSG_LEN - 1);
        tx_queue[tx_tail][UART_TX_MSG_LEN - 1] = '\0';
        tx_tail = (uint8_t)((tx_tail + 1) % UART_TX_QUEUE_LEN);
        tx_count++;
        uart_start_next_tx();
    }
    if(!primask) __enable_irq();
}

void print(const char *format, ...){
    char text[PRINT_BUFFER_LEN];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);

    // Expand \n to \r\n: raw terminals (picocom, screen...) need the \r.
    char crlf[UART_TX_MSG_LEN];
    size_t j = 0;
    for(size_t i = 0; text[i] != '\0' && j < sizeof(crlf) - 2; i++){
        if(text[i] == '\n') crlf[j++] = '\r';
        crlf[j++] = text[i];
    }
    crlf[j] = '\0';
    uart_send(crlf);
}

uint8_t uart_tx_idle(void){
    return tx_count == 0 && !tx_active;
}

uint8_t uart_tx_full(void){
    return tx_count >= UART_TX_QUEUE_LEN;
}

void uart_flush(uint32_t timeout_ms){
    uint32_t start = HAL_GetTick();
    while(!uart_tx_idle() && HAL_GetTick() - start < timeout_ms){}
}

// Called while waiting: the application says it is alive (health.c), so a
// long printout is not taken for a stalled program.
__weak void uart_waiting(void){
}

void uart_wait_space(uint32_t timeout_ms){
    uint32_t start = HAL_GetTick();
    while(tx_count >= UART_TX_QUEUE_LEN && HAL_GetTick() - start < timeout_ms) uart_waiting();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance != USART3) return;
    if(tx_count > 0) tx_drop_head();
    tx_active = 0;
    uart_start_next_tx();
}

// ---- Line receive (commands) -------------------------------------------------------

void uart_retime(void){
    huart3.Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), UART_BAUDRATE);
}

void uart_start_receive(void){
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance != USART3) return;
    if(rx_byte == '\n' || rx_byte == '\r'){
        if(rx_index > 0){
            rx_build[rx_index] = '\0';
            if(rx_count < UART_RX_QUEUE_LEN){
                memcpy(rx_queue[rx_tail], rx_build, (size_t)rx_index + 1);
                rx_tail = (uint8_t)((rx_tail + 1) % UART_RX_QUEUE_LEN);
                rx_count++;
            }
            rx_index = 0;
        }
    }
    else if(rx_index < sizeof(rx_build) - 1){
        rx_build[rx_index++] = (char)rx_byte;
    }
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if(huart->Instance != USART3) return;
    // An overrun (e.g. interrupts held off while flash is written, or a
    // Bluetooth reconnect burst) makes the HAL abort reception for good.
    // Restart it, dropping the damaged partial line; HAL_BUSY here just
    // means reception survived the error.
    rx_index = 0;
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    // A DMA error ends the transmit without a completion callback: drop that
    // message and keep the queue moving instead of stalling it forever.
    if(tx_active && huart->gState == HAL_UART_STATE_READY){
        tx_active = 0;
        if(tx_count > 0) tx_drop_head();
        uart_start_next_tx();
    }
}

uint8_t uart_read_line(char *buffer, uint8_t buffer_size){
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if(rx_count == 0){
        if(!primask) __enable_irq();
        return 0;
    }
    strncpy(buffer, rx_queue[rx_head], (size_t)buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    rx_head = (uint8_t)((rx_head + 1) % UART_RX_QUEUE_LEN);
    rx_count--;
    if(!primask) __enable_irq();
    return 1;
}
