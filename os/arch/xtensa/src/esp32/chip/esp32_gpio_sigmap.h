/******************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************/

/****************************************************************************
 * arch/xtensa/src/esp32/chip/esp32_gpio_sigmap.h
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic originally provided by Espressif Systems:
 *
 *   Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32_CHIP_ESP32_GPIO_SIGMAP_H
#define __ARCH_XTENSA_SRC_ESP32_CHIP_ESP32_GPIO_SIGMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPICLK_IN_IDX                 0
#define SPICLK_OUT_IDX                0
#define SPIQ_IN_IDX                   1
#define SPIQ_OUT_IDX                  1
#define SPID_IN_IDX                   2
#define SPID_OUT_IDX                  2
#define SPIHD_IN_IDX                  3
#define SPIHD_OUT_IDX                 3
#define SPIWP_IN_IDX                  4
#define SPIWP_OUT_IDX                 4
#define SPICS0_IN_IDX                 5
#define SPICS0_OUT_IDX                5
#define SPICS1_IN_IDX                 6
#define SPICS1_OUT_IDX                6
#define SPICS2_IN_IDX                 7
#define SPICS2_OUT_IDX                7
#define HSPICLK_IN_IDX                8
#define HSPICLK_OUT_IDX               8
#define HSPIQ_IN_IDX                  9
#define HSPIQ_OUT_IDX                 9
#define HSPID_IN_IDX                  10
#define HSPID_OUT_IDX                 10
#define HSPICS0_IN_IDX                11
#define HSPICS0_OUT_IDX               11
#define HSPIHD_IN_IDX                 12
#define HSPIHD_OUT_IDX                12
#define HSPIWP_IN_IDX                 13
#define HSPIWP_OUT_IDX                13
#define U0RXD_IN_IDX                  14
#define U0TXD_OUT_IDX                 14
#define U0CTS_IN_IDX                  15
#define U0RTS_OUT_IDX                 15
#define U0DSR_IN_IDX                  16
#define U0DTR_OUT_IDX                 16
#define U1RXD_IN_IDX                  17
#define U1TXD_OUT_IDX                 17
#define U1CTS_IN_IDX                  18
#define U1RTS_OUT_IDX                 18
#define I2CM_SCL_O_IDX                19
#define I2CM_SDA_I_IDX                20
#define I2CM_SDA_O_IDX                20
#define EXT_I2C_SCL_O_IDX             21
#define EXT_I2C_SDA_O_IDX             22
#define EXT_I2C_SDA_I_IDX             22
#define I2S0O_BCK_IN_IDX              23
#define I2S0O_BCK_OUT_IDX             23
#define I2S1O_BCK_IN_IDX              24
#define I2S1O_BCK_OUT_IDX             24
#define I2S0O_WS_IN_IDX               25
#define I2S0O_WS_OUT_IDX              25
#define I2S1O_WS_IN_IDX               26
#define I2S1O_WS_OUT_IDX              26
#define I2S0I_BCK_IN_IDX              27
#define I2S0I_BCK_OUT_IDX             27
#define I2S0I_WS_IN_IDX               28
#define I2S0I_WS_OUT_IDX              28
#define I2CEXT0_SCL_IN_IDX            29
#define I2CEXT0_SCL_OUT_IDX           29
#define I2CEXT0_SDA_IN_IDX            30
#define I2CEXT0_SDA_OUT_IDX           30
#define PWM0_SYNC0_IN_IDX             31
#define SDIO_TOHOST_INT_OUT_IDX       31
#define PWM0_SYNC1_IN_IDX             32
#define PWM0_OUT0A_IDX                32
#define PWM0_SYNC2_IN_IDX             33
#define PWM0_OUT0B_IDX                33
#define PWM0_F0_IN_IDX                34
#define PWM0_OUT1A_IDX                34
#define PWM0_F1_IN_IDX                35
#define PWM0_OUT1B_IDX                35
#define PWM0_F2_IN_IDX                36
#define PWM0_OUT2A_IDX                36
#define GPIO_BT_ACTIVE_IDX            37
#define PWM0_OUT2B_IDX                37
#define GPIO_BT_PRIORITY_IDX          38
#define PCNT_SIG_CH0_IN0_IDX          39
#define PCNT_SIG_CH1_IN0_IDX          40
#define GPIO_WLAN_ACTIVE_IDX          40
#define PCNT_CTRL_CH0_IN0_IDX         41
#define BB_DIAG0_IDX                  41
#define PCNT_CTRL_CH1_IN0_IDX         42
#define BB_DIAG1_IDX                  42
#define PCNT_SIG_CH0_IN1_IDX          43
#define BB_DIAG2_IDX                  43
#define PCNT_SIG_CH1_IN1_IDX          44
#define BB_DIAG3_IDX                  44
#define PCNT_CTRL_CH0_IN1_IDX         45
#define BB_DIAG4_IDX                  45
#define PCNT_CTRL_CH1_IN1_IDX         46
#define BB_DIAG5_IDX                  46
#define PCNT_SIG_CH0_IN2_IDX          47
#define BB_DIAG6_IDX                  47
#define PCNT_SIG_CH1_IN2_IDX          48
#define BB_DIAG7_IDX                  48
#define PCNT_CTRL_CH0_IN2_IDX         49
#define BB_DIAG8_IDX                  49
#define PCNT_CTRL_CH1_IN2_IDX         50
#define BB_DIAG9_IDX                  50
#define PCNT_SIG_CH0_IN3_IDX          51
#define BB_DIAG10_IDX                 51
#define PCNT_SIG_CH1_IN3_IDX          52
#define BB_DIAG11_IDX                 52
#define PCNT_CTRL_CH0_IN3_IDX         53
#define BB_DIAG12_IDX                 53
#define PCNT_CTRL_CH1_IN3_IDX         54
#define BB_DIAG13_IDX                 54
#define PCNT_SIG_CH0_IN4_IDX          55
#define BB_DIAG14_IDX                 55
#define PCNT_SIG_CH1_IN4_IDX          56
#define BB_DIAG15_IDX                 56
#define PCNT_CTRL_CH0_IN4_IDX         57
#define BB_DIAG16_IDX                 57
#define PCNT_CTRL_CH1_IN4_IDX         58
#define BB_DIAG17_IDX                 58
#define BB_DIAG18_IDX                 59
#define BB_DIAG19_IDX                 60
#define HSPICS1_IN_IDX                61
#define HSPICS1_OUT_IDX               61
#define HSPICS2_IN_IDX                62
#define HSPICS2_OUT_IDX               62
#define VSPICLK_IN_IDX                63
#define VSPICLK_OUT_MUX_IDX           63
#define VSPIQ_IN_IDX                  64
#define VSPIQ_OUT_IDX                 64
#define VSPID_IN_IDX                  65
#define VSPID_OUT_IDX                 65
#define VSPIHD_IN_IDX                 66
#define VSPIHD_OUT_IDX                66
#define VSPIWP_IN_IDX                 67
#define VSPIWP_OUT_IDX                67
#define VSPICS0_IN_IDX                68
#define VSPICS0_OUT_IDX               68
#define VSPICS1_IN_IDX                69
#define VSPICS1_OUT_IDX               69
#define VSPICS2_IN_IDX                70
#define VSPICS2_OUT_IDX               70
#define PCNT_SIG_CH0_IN5_IDX          71
#define LEDC_HS_SIG_OUT0_IDX          71
#define PCNT_SIG_CH1_IN5_IDX          72
#define LEDC_HS_SIG_OUT1_IDX          72
#define PCNT_CTRL_CH0_IN5_IDX         73
#define LEDC_HS_SIG_OUT2_IDX          73
#define PCNT_CTRL_CH1_IN5_IDX         74
#define LEDC_HS_SIG_OUT3_IDX          74
#define PCNT_SIG_CH0_IN6_IDX          75
#define LEDC_HS_SIG_OUT4_IDX          75
#define PCNT_SIG_CH1_IN6_IDX          76
#define LEDC_HS_SIG_OUT5_IDX          76
#define PCNT_CTRL_CH0_IN6_IDX         77
#define LEDC_HS_SIG_OUT6_IDX          77
#define PCNT_CTRL_CH1_IN6_IDX         78
#define LEDC_HS_SIG_OUT7_IDX          78
#define PCNT_SIG_CH0_IN7_IDX          79
#define LEDC_LS_SIG_OUT0_IDX          79
#define PCNT_SIG_CH1_IN7_IDX          80
#define LEDC_LS_SIG_OUT1_IDX          80
#define PCNT_CTRL_CH0_IN7_IDX         81
#define LEDC_LS_SIG_OUT2_IDX          81
#define PCNT_CTRL_CH1_IN7_IDX         82
#define LEDC_LS_SIG_OUT3_IDX          82
#define RMT_SIG_IN0_IDX               83
#define LEDC_LS_SIG_OUT4_IDX          83
#define RMT_SIG_IN1_IDX               84
#define LEDC_LS_SIG_OUT5_IDX          84
#define RMT_SIG_IN2_IDX               85
#define LEDC_LS_SIG_OUT6_IDX          85
#define RMT_SIG_IN3_IDX               86
#define LEDC_LS_SIG_OUT7_IDX          86
#define RMT_SIG_IN4_IDX               87
#define RMT_SIG_OUT0_IDX              87
#define RMT_SIG_IN5_IDX               88
#define RMT_SIG_OUT1_IDX              88
#define RMT_SIG_IN6_IDX               89
#define RMT_SIG_OUT2_IDX              89
#define RMT_SIG_IN7_IDX               90
#define RMT_SIG_OUT3_IDX              90
#define RMT_SIG_OUT4_IDX              91
#define RMT_SIG_OUT5_IDX              92
#define EXT_ADC_START_IDX             93
#define RMT_SIG_OUT6_IDX              93
#define CAN_RX_IDX                    94
#define RMT_SIG_OUT7_IDX              94
#define I2CEXT1_SCL_IN_IDX            95
#define I2CEXT1_SCL_OUT_IDX           95
#define I2CEXT1_SDA_IN_IDX            96
#define I2CEXT1_SDA_OUT_IDX           96
#define HOST_CARD_DETECT_N_1_IDX      97
#define HOST_CCMD_OD_PULLUP_EN_N_IDX  97
#define HOST_CARD_DETECT_N_2_IDX      98
#define HOST_RST_N_1_IDX              98
#define HOST_CARD_WRITE_PRT_1_IDX     99
#define HOST_RST_N_2_IDX              99
#define HOST_CARD_WRITE_PRT_2_IDX     100
#define GPIO_SD0_OUT_IDX              100
#define HOST_CARD_INT_N_1_IDX         101
#define GPIO_SD1_OUT_IDX              101
#define HOST_CARD_INT_N_2_IDX         102
#define GPIO_SD2_OUT_IDX              102
#define PWM1_SYNC0_IN_IDX             103
#define GPIO_SD3_OUT_IDX              103
#define PWM1_SYNC1_IN_IDX             104
#define GPIO_SD4_OUT_IDX              104
#define PWM1_SYNC2_IN_IDX             105
#define GPIO_SD5_OUT_IDX              105
#define PWM1_F0_IN_IDX                106
#define GPIO_SD6_OUT_IDX              106
#define PWM1_F1_IN_IDX                107
#define GPIO_SD7_OUT_IDX              107
#define PWM1_F2_IN_IDX                108
#define PWM1_OUT0A_IDX                108
#define PWM0_CAP0_IN_IDX              109
#define PWM1_OUT0B_IDX                109
#define PWM0_CAP1_IN_IDX              110
#define PWM1_OUT1A_IDX                110
#define PWM0_CAP2_IN_IDX              111
#define PWM1_OUT1B_IDX                111
#define PWM1_CAP0_IN_IDX              112
#define PWM1_OUT2A_IDX                112
#define PWM1_CAP1_IN_IDX              113
#define PWM1_OUT2B_IDX                113
#define PWM1_CAP2_IN_IDX              114
#define PWM2_OUT1H_IDX                114
#define PWM2_FLTA_IDX                 115
#define PWM2_OUT1L_IDX                115
#define PWM2_FLTB_IDX                 116
#define PWM2_OUT2H_IDX                116
#define PWM2_CAP1_IN_IDX              117
#define PWM2_OUT2L_IDX                117
#define PWM2_CAP2_IN_IDX              118
#define PWM2_OUT3H_IDX                118
#define PWM2_CAP3_IN_IDX              119
#define PWM2_OUT3L_IDX                119
#define PWM3_FLTA_IDX                 120
#define PWM2_OUT4H_IDX                120
#define PWM3_FLTB_IDX                 121
#define PWM2_OUT4L_IDX                121
#define PWM3_CAP1_IN_IDX              122
#define PWM3_CAP2_IN_IDX              123
#define CAN_TX_IDX                    123
#define PWM3_CAP3_IN_IDX              124
#define CAN_BUS_OFF_ON_IDX            124
#define CAN_CLKOUT_IDX                125
#define SPID4_IN_IDX                  128
#define SPID4_OUT_IDX                 128
#define SPID5_IN_IDX                  129
#define SPID5_OUT_IDX                 129
#define SPID6_IN_IDX                  130
#define SPID6_OUT_IDX                 130
#define SPID7_IN_IDX                  131
#define SPID7_OUT_IDX                 131
#define HSPID4_IN_IDX                 132
#define HSPID4_OUT_IDX                132
#define HSPID5_IN_IDX                 133
#define HSPID5_OUT_IDX                133
#define HSPID6_IN_IDX                 134
#define HSPID6_OUT_IDX                134
#define HSPID7_IN_IDX                 135
#define HSPID7_OUT_IDX                135
#define VSPID4_IN_IDX                 136
#define VSPID4_OUT_IDX                136
#define VSPID5_IN_IDX                 137
#define VSPID5_OUT_IDX                137
#define VSPID6_IN_IDX                 138
#define VSPID6_OUT_IDX                138
#define VSPID7_IN_IDX                 139
#define VSPID7_OUT_IDX                139
#define I2S0I_DATA_IN0_IDX            140
#define I2S0O_DATA_OUT0_IDX           140
#define I2S0I_DATA_IN1_IDX            141
#define I2S0O_DATA_OUT1_IDX           141
#define I2S0I_DATA_IN2_IDX            142
#define I2S0O_DATA_OUT2_IDX           142
#define I2S0I_DATA_IN3_IDX            143
#define I2S0O_DATA_OUT3_IDX           143
#define I2S0I_DATA_IN4_IDX            144
#define I2S0O_DATA_OUT4_IDX           144
#define I2S0I_DATA_IN5_IDX            145
#define I2S0O_DATA_OUT5_IDX           145
#define I2S0I_DATA_IN6_IDX            146
#define I2S0O_DATA_OUT6_IDX           146
#define I2S0I_DATA_IN7_IDX            147
#define I2S0O_DATA_OUT7_IDX           147
#define I2S0I_DATA_IN8_IDX            148
#define I2S0O_DATA_OUT8_IDX           148
#define I2S0I_DATA_IN9_IDX            149
#define I2S0O_DATA_OUT9_IDX           149
#define I2S0I_DATA_IN10_IDX           150
#define I2S0O_DATA_OUT10_IDX          150
#define I2S0I_DATA_IN11_IDX           151
#define I2S0O_DATA_OUT11_IDX          151
#define I2S0I_DATA_IN12_IDX           152
#define I2S0O_DATA_OUT12_IDX          152
#define I2S0I_DATA_IN13_IDX           153
#define I2S0O_DATA_OUT13_IDX          153
#define I2S0I_DATA_IN14_IDX           154
#define I2S0O_DATA_OUT14_IDX          154
#define I2S0I_DATA_IN15_IDX           155
#define I2S0O_DATA_OUT15_IDX          155
#define I2S0O_DATA_OUT16_IDX          156
#define I2S0O_DATA_OUT17_IDX          157
#define I2S0O_DATA_OUT18_IDX          158
#define I2S0O_DATA_OUT19_IDX          159
#define I2S0O_DATA_OUT20_IDX          160
#define I2S0O_DATA_OUT21_IDX          161
#define I2S0O_DATA_OUT22_IDX          162
#define I2S0O_DATA_OUT23_IDX          163
#define I2S1I_BCK_IN_IDX              164
#define I2S1I_BCK_OUT_IDX             164
#define I2S1I_WS_IN_IDX               165
#define I2S1I_WS_OUT_IDX              165
#define I2S1I_DATA_IN0_IDX            166
#define I2S1O_DATA_OUT0_IDX           166
#define I2S1I_DATA_IN1_IDX            167
#define I2S1O_DATA_OUT1_IDX           167
#define I2S1I_DATA_IN2_IDX            168
#define I2S1O_DATA_OUT2_IDX           168
#define I2S1I_DATA_IN3_IDX            169
#define I2S1O_DATA_OUT3_IDX           169
#define I2S1I_DATA_IN4_IDX            170
#define I2S1O_DATA_OUT4_IDX           170
#define I2S1I_DATA_IN5_IDX            171
#define I2S1O_DATA_OUT5_IDX           171
#define I2S1I_DATA_IN6_IDX            172
#define I2S1O_DATA_OUT6_IDX           172
#define I2S1I_DATA_IN7_IDX            173
#define I2S1O_DATA_OUT7_IDX           173
#define I2S1I_DATA_IN8_IDX            174
#define I2S1O_DATA_OUT8_IDX           174
#define I2S1I_DATA_IN9_IDX            175
#define I2S1O_DATA_OUT9_IDX           175
#define I2S1I_DATA_IN10_IDX           176
#define I2S1O_DATA_OUT10_IDX          176
#define I2S1I_DATA_IN11_IDX           177
#define I2S1O_DATA_OUT11_IDX          177
#define I2S1I_DATA_IN12_IDX           178
#define I2S1O_DATA_OUT12_IDX          178
#define I2S1I_DATA_IN13_IDX           179
#define I2S1O_DATA_OUT13_IDX          179
#define I2S1I_DATA_IN14_IDX           180
#define I2S1O_DATA_OUT14_IDX          180
#define I2S1I_DATA_IN15_IDX           181
#define I2S1O_DATA_OUT15_IDX          181
#define I2S1O_DATA_OUT16_IDX          182
#define I2S1O_DATA_OUT17_IDX          183
#define I2S1O_DATA_OUT18_IDX          184
#define I2S1O_DATA_OUT19_IDX          185
#define I2S1O_DATA_OUT20_IDX          186
#define I2S1O_DATA_OUT21_IDX          187
#define I2S1O_DATA_OUT22_IDX          188
#define I2S1O_DATA_OUT23_IDX          189
#define I2S0I_H_SYNC_IDX              190
#define PWM3_OUT1H_IDX                190
#define I2S0I_V_SYNC_IDX              191
#define PWM3_OUT1L_IDX                191
#define I2S0I_H_ENABLE_IDX            192
#define PWM3_OUT2H_IDX                192
#define I2S1I_H_SYNC_IDX              193
#define PWM3_OUT2L_IDX                193
#define I2S1I_V_SYNC_IDX              194
#define PWM3_OUT3H_IDX                194
#define I2S1I_H_ENABLE_IDX            195
#define PWM3_OUT3L_IDX                195
#define PWM3_OUT4H_IDX                196
#define PWM3_OUT4L_IDX                197
#define U2RXD_IN_IDX                  198
#define U2TXD_OUT_IDX                 198
#define U2CTS_IN_IDX                  199
#define U2RTS_OUT_IDX                 199
#define EMAC_MDC_I_IDX                200
#define EMAC_MDC_O_IDX                200
#define EMAC_MDI_I_IDX                201
#define EMAC_MDO_O_IDX                201
#define EMAC_CRS_I_IDX                202
#define EMAC_CRS_O_IDX                202
#define EMAC_COL_I_IDX                203
#define EMAC_COL_O_IDX                203
#define PCMFSYNC_IN_IDX               204
#define BT_AUDIO0_IRQ_IDX             204
#define PCMCLK_IN_IDX                 205
#define BT_AUDIO1_IRQ_IDX             205
#define PCMDIN_IDX                    206
#define BT_AUDIO2_IRQ_IDX             206
#define BLE_AUDIO0_IRQ_IDX            207
#define BLE_AUDIO1_IRQ_IDX            208
#define BLE_AUDIO2_IRQ_IDX            209
#define PCMFSYNC_OUT_IDX              210
#define PCMCLK_OUT_IDX                211
#define PCMDOUT_IDX                   212
#define BLE_AUDIO_SYNC0_P_IDX         213
#define BLE_AUDIO_SYNC1_P_IDX         214
#define BLE_AUDIO_SYNC2_P_IDX         215
#define ANT_SEL0_IDX                  216
#define ANT_SEL1_IDX                  217
#define ANT_SEL2_IDX                  218
#define ANT_SEL3_IDX                  219
#define ANT_SEL4_IDX                  220
#define ANT_SEL5_IDX                  221
#define ANT_SEL6_IDX                  222
#define ANT_SEL7_IDX                  223
#define SIG_IN_FUNC224_IDX            224
#define SIG_IN_FUNC225_IDX            225
#define SIG_IN_FUNC226_IDX            226
#define SIG_IN_FUNC227_IDX            227
#define SIG_IN_FUNC228_IDX            228

#endif							/* __ARCH_XTENSA_SRC_ESP32_CHIP_ESP32_GPIO_SIGMAP_H */
