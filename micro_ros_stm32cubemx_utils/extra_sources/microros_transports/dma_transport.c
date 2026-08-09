#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

#define UART_DMA_BUFFER_SIZE 2048

uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
volatile size_t dma_head = 0;
volatile size_t dma_tail = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    HAL_UART_DMAStop(uart);
    dma_head = 0;
    dma_tail = 0;

    __HAL_UART_CLEAR_OREFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);

    if (HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE) != HAL_OK) {
        return false;
    }
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    dma_head = 0;
    dma_tail = 0;
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                              uint8_t * buf, size_t len, uint8_t * err)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    if (uart->gState != HAL_UART_STATE_READY) {
        HAL_UART_DMAStop(uart);
        __HAL_UART_CLEAR_OREFLAG(uart);
        __HAL_UART_CLEAR_NEFLAG(uart);
        __HAL_UART_CLEAR_FEFLAG(uart);
        __HAL_UART_CLEAR_PEFLAG(uart);
        uart->gState = HAL_UART_STATE_READY;
    }

    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(uart, buf, len);

    uint32_t start = osKernelGetTickCount();
    while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY) {
        osDelay(1);
        if ((osKernelGetTickCount() - start) > 100) {
            HAL_UART_DMAStop(uart);
            uart->gState = HAL_UART_STATE_READY;
            return 0;
        }
    }

    return (ret == HAL_OK) ? len : 0;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport,
                             uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    int ms_used = 0;

    do {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        dma_tail = (UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx))
                   % UART_DMA_BUFFER_SIZE;
        __set_PRIMASK(primask);

        if (dma_head == dma_tail) {
            osDelay(1);
            ms_used++;
        } else {
            break;
        }
    } while (ms_used < timeout);

    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)) {
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }

    return wrote;
}

#endif
