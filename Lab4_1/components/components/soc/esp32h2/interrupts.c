/*
 * SPDX-FileCopyrightText: 2020-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/interrupts.h"

const char *const esp_isr_names[] = {
    [0] = "PMU",
    [1] = "EFUSE",
    [2] = "LP_RTC_TIMER",
    [3] = "LP_BLE_TIMER",
    [4] = "LP_WDT",
    [5] = "LP_PERI_TIMEOUT",
    [6] = "LP_APM_M0",
    [7] = "CPUFROM_CPU_0",
    [8] = "CPUFROM_CPU_1",
    [9] = "CPUFROM_CPU_2",
    [10] = "CPUFROM_CPU_3",
    [11] = "ASSIST_DEBUG",
    [12] = "TRACE",
    [13] = "CACHE",
    [14] = "CPU_PERI_TIMEOUT",
    [15] = "BT_MAC",
    [16] = "BT_BB",
    [17] = "BT_BB_NMI_",
    [18] = "COEX",
    [19] = "BLE_TIMER",
    [20] = "BLE_SEC",
    [21] = "ZB_MAC",
    [22] = "GPIO_INTERRUPT_PRO_",
    [23] = "GPIO_INTERRUPT_PRO_NMI",
    [24] = "PAU",
    [25] = "HP_PERI_TIMEOUT",
    [26] = "HP_APM_M0",
    [27] = "HP_APM_M1",
    [28] = "HP_APM_M2",
    [29] = "HP_APM_M3",
    [30] = "MSPI",
    [31] = "I2S0",
    [32] = "UHCI0",
    [33] = "UART0",
    [34] = "UART1",
    [35] = "LEDC",
    [36] = "CAN0",
    [37] = "USB",
    [38] = "RMT",
    [39] = "I2C_EXT0",
    [40] = "I2C_EXT1",
    [41] = "TG0_T0",
    [42] = "TG0_WDT",
    [43] = "TG1_T0",
    [44] = "TG1_WDT",
    [45] = "SYSTIMER_TARGET0",
    [46] = "SYSTIMER_TARGET1",
    [47] = "SYSTIMER_TARGET2",
    [48] = "APB_ADC",
    [49] = "PWM",
    [50] = "PCNT",
    [51] = "PARL_IO_TX",
    [52] = "PARL_IO_RX",
    [53] = "DMA_IN_CH0",
    [54] = "DMA_IN_CH1",
    [55] = "DMA_IN_CH2",
    [56] = "DMA_OUT_CH0",
    [57] = "DMA_OUT_CH1",
    [58] = "DMA_OUT_CH2",
    [59] = "GPSPI2",
    [60] = "AES",
    [61] = "SHA",
    [62] = "RSA",
    [63] = "ECC",
    [64] = "ECDSA",
};