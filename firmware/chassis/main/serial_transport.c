// CP2102 Type-C data connector -> UART0 GPIO43/44, distinct from Pi power USB-C.
#include <stdbool.h>
#include <stdint.h>
#include "driver/uart.h"
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

static bool serial_open(struct uxrCustomTransport *transport)
{
    const uart_config_t config = {
        .baud_rate = CONFIG_CHASSIS_SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(UART_NUM_0, &config) != ESP_OK ||
        uart_set_pin(UART_NUM_0, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
        return false;
    return uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0) == ESP_OK;
}

static bool serial_close(struct uxrCustomTransport *transport)
{
    return uart_driver_delete(UART_NUM_0) == ESP_OK;
}

static size_t serial_write(struct uxrCustomTransport *transport,
                          const uint8_t *buf, size_t len, uint8_t *error)
{
    int count = uart_write_bytes(UART_NUM_0, buf, len);
    *error = count < 0;
    return count < 0 ? 0 : (size_t)count;
}

static size_t serial_read(struct uxrCustomTransport *transport,
                         uint8_t *buf, size_t len, int timeout, uint8_t *error)
{
    int count = uart_read_bytes(UART_NUM_0, buf, len, pdMS_TO_TICKS(timeout));
    *error = count < 0;
    return count < 0 ? 0 : (size_t)count;
}

bool chassis_serial_init(void)
{
    return rmw_uros_set_custom_transport(true, NULL, serial_open, serial_close,
                                         serial_write, serial_read) == RMW_RET_OK;
}
