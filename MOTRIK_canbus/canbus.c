/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/

/*!
 * @file canbus.c
 * @brief CAN Bus Click Driver.
 */

#include "canbus.h"
#include "usart.h"

uint8_t _canbusDataLen(char *str);

uint8_t _canbusDataLen(char *str)
{
    uint8_t len = 0;
    while (str[len])
    {
        len++;
    }
    return len;
}

void canbus_cfg_setup(void)
{
    /**/
}

void canbus_init(void)
{
    usartInit(_CANBUS_BAUDRATE, _CANBUS_FCPU);
}

int8_t canbus_default_cfg(uint8_t defaultMode)
{
    canbus_set_high_speed_mode(defaultMode);
    return CANBUS_OK;
}

uint8_t canbus_generic_write(char *data_buf, uint8_t len)
{
    return usartSendBytes(data_buf, len);
}

void canbus_generic_read(/*canbus_t *ctx, char *data_buf, uint16_t max_len*/)
{
    //return uart_read(&ctx->uart, data_buf, max_len);
    //usa
}

void canbus_set_low_current_standby_mode(uint8_t standbyMode)
{
    SS_SET(standbyMode);
}

void canbus_set_high_speed_mode(uint8_t highSpeedMode)
{
    SS_SET(!highSpeedMode);
}

int canbus_send_data(char *tx_data)
{
    char tmp_buf[100];
    uint8_t len;
    int error;

    memset(tmp_buf, 0, 100);
    len = strlen(tx_data);
    strncpy(tmp_buf, tx_data, len);

    canbus_generic_write(tmp_buf, len);
    /* for (uint8_t cnt = 0; cnt < len; cnt++)
    {
        error |= canbus_generic_write(ctx, &tmp_buf[cnt], 1);
    } */

    return error;
}

// ------------------------------------------------------------------------- END
