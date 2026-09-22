#include <stdarg.h>
#include <stdio.h>
#include "uart.h"
#include "gpio.h"
#include "error.h"

#define UART_TX_QUEUE_LEN 8
#define UART_TX_MSG_LEN 140
#define UART_RX_QUEUE_LEN 4
#define UART_RX_LINE_LEN 64

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

static void uart_start_next_tx(void){
    if(tx_active || tx_count == 0) return;
    tx_active = 1;
    if(HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_queue[tx_head], strlen(tx_queue[tx_head])) != HAL_OK){
        tx_active = 0;
        tx_head = (uint8_t)((tx_head + 1) % UART_TX_QUEUE_LEN);
        tx_count--;
    }
}

static void uart_enqueue(const char *data){
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if(tx_count < UART_TX_QUEUE_LEN){
        strncpy(tx_queue[tx_tail], data, UART_TX_MSG_LEN - 1);
        tx_queue[tx_tail][UART_TX_MSG_LEN - 1] = '\0';
        tx_tail = (uint8_t)((tx_tail + 1) % UART_TX_QUEUE_LEN);
        tx_count++;
        uart_start_next_tx();
    }
    if(!primask) __enable_irq();
}

void UART_Init(){
    huart3.Instance = USART3;
    huart3.Init.BaudRate = UART_BAUDRATE;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK){
        Error_Handler();
    }

    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_USART3_CLK_ENABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void send_uart(char* data){
    uart_enqueue(data);
}

void print(const char *format, ...) {
    char print_buffer[100];
    va_list args;
    va_start(args, format);
    vsnprintf(print_buffer, sizeof(print_buffer), format, args);
    va_end(args);

    // Expand bare \n to \r\n: raw terminals (picocom, screen, ...) don't move
    // the cursor back to column 0 on \n alone, only a real UART "cooked" tty does.
    char crlf_buffer[UART_TX_MSG_LEN];
    size_t j = 0;
    for(size_t i = 0; print_buffer[i] != '\0' && j < sizeof(crlf_buffer) - 2; i++){
        if(print_buffer[i] == '\n'){
            crlf_buffer[j++] = '\r';
        }
        crlf_buffer[j++] = print_buffer[i];
    }
    crlf_buffer[j] = '\0';

    send_uart(crlf_buffer);
}

uint8_t uart_tx_idle(void){
    return tx_count == 0 && !tx_active;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance != USART3) return;
    if(tx_count > 0){
        tx_head = (uint8_t)((tx_head + 1) % UART_TX_QUEUE_LEN);
        tx_count--;
    }
    tx_active = 0;
    uart_start_next_tx();
}

// --- Non-blocking line receive (for live tuning over Bluetooth) ---

void uart_start_receive(void){
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART3){
        if(rx_byte == '\n' || rx_byte == '\r'){
            if(rx_index > 0){
                rx_build[rx_index] = '\0';
                if(rx_count < UART_RX_QUEUE_LEN){
                    strncpy(rx_queue[rx_tail], rx_build, UART_RX_LINE_LEN);
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
}

uint8_t uart_read_line(char *buffer, uint8_t buffer_size){
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if(rx_count == 0){
        if(!primask) __enable_irq();
        return 0;
    }
    strncpy(buffer, rx_queue[rx_head], buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    rx_head = (uint8_t)((rx_head + 1) % UART_RX_QUEUE_LEN);
    rx_count--;
    if(!primask) __enable_irq();
    return 1;
}

