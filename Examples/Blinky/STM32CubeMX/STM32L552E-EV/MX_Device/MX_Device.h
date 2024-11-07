/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 06/11/2024 14:54:14
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated with a generator out of the
 *               STM32CubeMX project and its generated files (DO NOT EDIT!)
 ******************************************************************************/

#ifndef MX_DEVICE_H__
#define MX_DEVICE_H__

/* MX_Device.h version */
#define MX_DEVICE_VERSION                       0x01000000


/*------------------------------ FDCAN1         -----------------------------*/
#define MX_FDCAN1                               1

/* Pins */

/* FDCAN1_RX */
#define MX_FDCAN1_RX_Pin                        PB8
#define MX_FDCAN1_RX_GPIO_Pin                   GPIO_PIN_8
#define MX_FDCAN1_RX_GPIOx                      GPIOB
#define MX_FDCAN1_RX_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_FDCAN1_RX_GPIO_PuPd                  GPIO_NOPULL
#define MX_FDCAN1_RX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_FDCAN1_RX_GPIO_AF                    GPIO_AF9_FDCAN1

/* FDCAN1_TX */
#define MX_FDCAN1_TX_Pin                        PB9
#define MX_FDCAN1_TX_GPIO_Pin                   GPIO_PIN_9
#define MX_FDCAN1_TX_GPIOx                      GPIOB
#define MX_FDCAN1_TX_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_FDCAN1_TX_GPIO_PuPd                  GPIO_NOPULL
#define MX_FDCAN1_TX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_FDCAN1_TX_GPIO_AF                    GPIO_AF9_FDCAN1

/*------------------------------ I2C1           -----------------------------*/
#define MX_I2C1                                 1

/* Filter Settings */
#define MX_I2C1_ANF_ENABLE                      1
#define MX_I2C1_DNF                             0

/* Peripheral Clock Frequency */
#define MX_I2C1_PERIPH_CLOCK_FREQ               55000000

/* Pins */

/* I2C1_SCL */
#define MX_I2C1_SCL_Pin                         PG14
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_14
#define MX_I2C1_SCL_GPIOx                       GPIOG
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C1_SCL_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1

/* I2C1_SDA */
#define MX_I2C1_SDA_Pin                         PG13
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_13
#define MX_I2C1_SDA_GPIOx                       GPIOG
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C1_SDA_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1

/*------------------------------ LPUART1        -----------------------------*/
#define MX_LPUART1                              1

/* Pins */

/* LPUART1_CTS */
#define MX_LPUART1_CTS_Pin                      PB13
#define MX_LPUART1_CTS_GPIO_Pin                 GPIO_PIN_13
#define MX_LPUART1_CTS_GPIOx                    GPIOB
#define MX_LPUART1_CTS_GPIO_Mode                GPIO_MODE_AF_PP
#define MX_LPUART1_CTS_GPIO_PuPd                GPIO_NOPULL
#define MX_LPUART1_CTS_GPIO_Speed               GPIO_SPEED_FREQ_LOW
#define MX_LPUART1_CTS_GPIO_AF                  GPIO_AF8_LPUART1

/* LPUART1_RTS */
#define MX_LPUART1_RTS_Pin                      PG6
#define MX_LPUART1_RTS_GPIO_Pin                 GPIO_PIN_6
#define MX_LPUART1_RTS_GPIOx                    GPIOG
#define MX_LPUART1_RTS_GPIO_Mode                GPIO_MODE_AF_PP
#define MX_LPUART1_RTS_GPIO_PuPd                GPIO_NOPULL
#define MX_LPUART1_RTS_GPIO_Speed               GPIO_SPEED_FREQ_LOW
#define MX_LPUART1_RTS_GPIO_AF                  GPIO_AF8_LPUART1

/* LPUART1_RX */
#define MX_LPUART1_RX_Pin                       PG8
#define MX_LPUART1_RX_GPIO_Pin                  GPIO_PIN_8
#define MX_LPUART1_RX_GPIOx                     GPIOG
#define MX_LPUART1_RX_GPIO_Mode                 GPIO_MODE_AF_PP
#define MX_LPUART1_RX_GPIO_PuPd                 GPIO_NOPULL
#define MX_LPUART1_RX_GPIO_Speed                GPIO_SPEED_FREQ_LOW
#define MX_LPUART1_RX_GPIO_AF                   GPIO_AF8_LPUART1

/* LPUART1_TX */
#define MX_LPUART1_TX_Pin                       PG7
#define MX_LPUART1_TX_GPIO_Pin                  GPIO_PIN_7
#define MX_LPUART1_TX_GPIOx                     GPIOG
#define MX_LPUART1_TX_GPIO_Mode                 GPIO_MODE_AF_PP
#define MX_LPUART1_TX_GPIO_PuPd                 GPIO_NOPULL
#define MX_LPUART1_TX_GPIO_Speed                GPIO_SPEED_FREQ_LOW
#define MX_LPUART1_TX_GPIO_AF                   GPIO_AF8_LPUART1

/*------------------------------ SDMMC1         -----------------------------*/
#define MX_SDMMC1                               1

/* Mode */
#define MX_SDMMC1_MODE_SD                       1

/* Peripheral Clock Frequency */
#define MX_SDMMC1_PERIPH_CLOCK_FREQ             31428571.42857143

/* Pins */

/* SDMMC1_CK */
#define MX_SDMMC1_CK_Pin                        PC12
#define MX_SDMMC1_CK_GPIO_Pin                   GPIO_PIN_12
#define MX_SDMMC1_CK_GPIOx                      GPIOC
#define MX_SDMMC1_CK_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_CK_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_CK_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_CK_GPIO_AF                    GPIO_AF12_SDMMC1

/* SDMMC1_CMD */
#define MX_SDMMC1_CMD_Pin                       PD2
#define MX_SDMMC1_CMD_GPIO_Pin                  GPIO_PIN_2
#define MX_SDMMC1_CMD_GPIOx                     GPIOD
#define MX_SDMMC1_CMD_GPIO_Mode                 GPIO_MODE_AF_PP
#define MX_SDMMC1_CMD_GPIO_PuPd                 GPIO_NOPULL
#define MX_SDMMC1_CMD_GPIO_Speed                GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_CMD_GPIO_AF                   GPIO_AF12_SDMMC1

/* SDMMC1_D0 */
#define MX_SDMMC1_D0_Pin                        PC8
#define MX_SDMMC1_D0_GPIO_Pin                   GPIO_PIN_8
#define MX_SDMMC1_D0_GPIOx                      GPIOC
#define MX_SDMMC1_D0_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_D0_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_D0_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_D0_GPIO_AF                    GPIO_AF12_SDMMC1

/* SDMMC1_D1 */
#define MX_SDMMC1_D1_Pin                        PC9
#define MX_SDMMC1_D1_GPIO_Pin                   GPIO_PIN_9
#define MX_SDMMC1_D1_GPIOx                      GPIOC
#define MX_SDMMC1_D1_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_D1_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_D1_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_D1_GPIO_AF                    GPIO_AF12_SDMMC1

/* SDMMC1_D2 */
#define MX_SDMMC1_D2_Pin                        PC10
#define MX_SDMMC1_D2_GPIO_Pin                   GPIO_PIN_10
#define MX_SDMMC1_D2_GPIOx                      GPIOC
#define MX_SDMMC1_D2_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_D2_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_D2_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_D2_GPIO_AF                    GPIO_AF12_SDMMC1

/* SDMMC1_D3 */
#define MX_SDMMC1_D3_Pin                        PC11
#define MX_SDMMC1_D3_GPIO_Pin                   GPIO_PIN_11
#define MX_SDMMC1_D3_GPIOx                      GPIOC
#define MX_SDMMC1_D3_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_D3_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_D3_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_D3_GPIO_AF                    GPIO_AF12_SDMMC1

/*------------------------------ USART1         -----------------------------*/
#define MX_USART1                               1

/* Virtual mode */
#define MX_USART1_VM                            VM_SMARTCARD
#define MX_USART1_VM_SMARTCARD                  1

/* Pins */

/* USART1_CK */
#define MX_USART1_CK_Pin                        PA8
#define MX_USART1_CK_GPIO_Pin                   GPIO_PIN_8
#define MX_USART1_CK_GPIOx                      GPIOA
#define MX_USART1_CK_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_USART1_CK_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART1_CK_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_USART1_CK_GPIO_AF                    GPIO_AF7_USART1

/* USART1_TX */
#define MX_USART1_TX_Pin                        PA9
#define MX_USART1_TX_GPIO_Pin                   GPIO_PIN_9
#define MX_USART1_TX_GPIOx                      GPIOA
#define MX_USART1_TX_GPIO_Mode                  GPIO_MODE_AF_OD
#define MX_USART1_TX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART1_TX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_USART1_TX_GPIO_AF                    GPIO_AF7_USART1

/*------------------------------ USART3         -----------------------------*/
#define MX_USART3                               1

/* Virtual mode */
#define MX_USART3_VM                            VM_ASYNC
#define MX_USART3_VM_ASYNC                      1

/* Pins */

/* USART3_RX */
#define MX_USART3_RX_Pin                        PB11
#define MX_USART3_RX_GPIO_Pin                   GPIO_PIN_11
#define MX_USART3_RX_GPIOx                      GPIOB
#define MX_USART3_RX_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_USART3_RX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART3_RX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_USART3_RX_GPIO_AF                    GPIO_AF7_USART3

/* USART3_TX */
#define MX_USART3_TX_Pin                        PB10
#define MX_USART3_TX_GPIO_Pin                   GPIO_PIN_10
#define MX_USART3_TX_GPIOx                      GPIOB
#define MX_USART3_TX_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_USART3_TX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART3_TX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_USART3_TX_GPIO_AF                    GPIO_AF7_USART3

/*------------------------------ USB            -----------------------------*/
#define MX_USB                                  1

/* Handle */
#define MX_USB_HANDLE                           hpcd_USB_FS

/* Pins */

/* USB_DM */
#define MX_USB_DM_Pin                           PA11
#define MX_USB_DM_GPIO_Pin                      GPIO_PIN_11
#define MX_USB_DM_GPIOx                         GPIOA
#define MX_USB_DM_GPIO_Mode                     GPIO_MODE_AF_PP
#define MX_USB_DM_GPIO_PuPd                     GPIO_NOPULL
#define MX_USB_DM_GPIO_Speed                    GPIO_SPEED_FREQ_LOW
#define MX_USB_DM_GPIO_AF                       GPIO_AF10_USB

/* USB_DP */
#define MX_USB_DP_Pin                           PA12
#define MX_USB_DP_GPIO_Pin                      GPIO_PIN_12
#define MX_USB_DP_GPIOx                         GPIOA
#define MX_USB_DP_GPIO_Mode                     GPIO_MODE_AF_PP
#define MX_USB_DP_GPIO_PuPd                     GPIO_NOPULL
#define MX_USB_DP_GPIO_Speed                    GPIO_SPEED_FREQ_LOW
#define MX_USB_DP_GPIO_AF                       GPIO_AF10_USB

#endif  /* MX_DEVICE_H__ */
