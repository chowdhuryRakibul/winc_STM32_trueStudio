/**
 *
 * \file
 *
 * \brief WINC1500 configuration.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_WINC_H_INCLUDED
#define CONF_WINC_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
   ---------------------------------
   ---------- PIN settings ---------
   ---------------------------------
*/
extern SPI_HandleTypeDef hspi2;

#define CONF_WINC_RESET_PIN				GPIO_PIN_14
#define CONF_WINC_ENABLE_PIN			GPIO_PIN_1
#define CONF_WINC_WAKE_PIN				GPIO_PIN_13
#define CONF_WINC_IRQ_PIN				GPIO_PIN_15
#define CONF_WINC_RESET_PORT			GPIOB
#define CONF_WINC_ENABLE_PORT			GPIOB
#define CONF_WINC_WAKE_PORT				GPIOB
#define CONF_WINC_IRQ_PORT				GPIOB

#define CONF_WINC_EXTI_IRQN             EXTI15_10_IRQn

/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WINC_USE_SPI				(1)

/** SPI pin and instance settings. */

/** SPI interrupt pin. */
#define CONF_WINC_SPI_INT_PIN			CONF_WINC_IRQ_PIN
#define CONF_WINC_SPI_INT_PORT			CONF_WINC_IRQ_PORT

#define SPI_WIFI_CS_PORT				GPIOC
#define SPI_WIFI_CS_PIN					GPIO_PIN_1

#define SPI_WIFI_MISO_PIN				GPIO_PIN_2
#define SPI_WIFI_MISO_PORT				GPIOC

#define SPI_WIFI_MOSI_PIN				GPIO_PIN_3
#define SPI_WIFI_MOSI_PORT				GPIOC

#define SPI_WIFI_SCK_PIN				GPIO_PIN_10
#define SPI_WIFI_SCK_PORT				GPIOB

#define SPI_WIFI						SPI2


/** SPI clock. */
#define CONF_WINC_SPI_CLOCK				(12000000)

/*
   ---------------------------------
   --------- Debug Options ---------
   ---------------------------------
*/

#define CONF_WINC_PRINTF				printf

#ifdef __cplusplus
}
#endif

#endif /* CONF_WINC_H_INCLUDED */
