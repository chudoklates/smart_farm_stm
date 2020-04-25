/**
  ******************************************************************************
  * @file 		stm32f446xx_gpio.h
  * @author  	Kamil Mezynski
  * @version 	v1.0.0
  * @date    	22-March-2020
  * @brief   	This file contains macros and functions prototypes for the GPIO
  * 			peripheral for STM32F446xx MCU
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F446XX_GPIO_H
#define __STM32F446XX_GPIO_H

/* Define to ensure compilation in C -----------------------------------------*/
#ifdef __cplusplus
	extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Enumeration types and macros ----------------------------------------------*/

/** @defgroup GPIO bit set/reset
  *	@{
  */
typedef enum {
	Bit_RESET = 0,
	Bit_SET
} Bit_Action;
/**
  *	@}
  */

/**	@defgroup GPIO ports
  *	@{
  */
#define PORTA			GPIOA
#define PORTB			GPIOB
#define PORTC			GPIOC
#define PORTD			GPIOD
#define PORTE			GPIOE
#define PORTF			GPIOF
#define PORTG			GPIOG
#define PORTH			GPIOH
/**
  * @}
  */

/**	@defgroup GPIO pins
  *	@{
  */
#define GPIO_Pin0		((uint16_t) 0x0001)		/* pin 0 selected */
#define GPIO_Pin1		((uint16_t) 0x0002)		/* pin 1 selected */
#define GPIO_Pin2		((uint16_t) 0x0004)		/* pin 2 selected */
#define GPIO_Pin3		((uint16_t) 0x0008)		/* pin 3 selected */
#define GPIO_Pin4		((uint16_t) 0x0010)		/* pin 4 selected */
#define GPIO_Pin5		((uint16_t) 0x0020)		/* pin 5 selected */
#define GPIO_Pin6		((uint16_t) 0x0040)		/* pin 6 selected */
#define GPIO_Pin7		((uint16_t) 0x0080)		/* pin 7 selected */
#define GPIO_Pin8		((uint16_t) 0x0100)		/* pin 8 selected */
#define GPIO_Pin9		((uint16_t) 0x0200)		/* pin 9 selected */
#define GPIO_Pin10		((uint16_t) 0x0400)		/* pin 10 selected */
#define GPIO_Pin11		((uint16_t) 0x0800)		/* pin 11 selected */
#define GPIO_Pin12		((uint16_t) 0x1000)		/* pin 12 selected */
#define GPIO_Pin13		((uint16_t) 0x2000)		/* pin 13 selected */
#define GPIO_Pin14		((uint16_t) 0x4000)		/* pin 14 selected */
#define GPIO_Pin15		((uint16_t) 0x8000)		/* pin 15 selected */
#define GPIO_Pin_All	((uint16_t) 0xFFFF)		/* all pins selected */
/**
  * @}
  */

/**	@defgroup Configuration mode
  * @{
  */
typedef enum {
	GPIO_Mode_IN		= ((uint8_t) 0x00),		/* GPIO input mode */
	GPIO_Mode_OUT		= ((uint8_t) 0x01),		/* GPIO output mode */
	GPIO_Mode_AF		= ((uint8_t) 0x02),		/* GPIO alternate function mode */
	GPIO_Mode_AN		= ((uint8_t) 0x03)		/* GPIO analog IN/OUT mode */
} GPIO_Mode_TypeDef;
/**
  * @}
  */

/**	@defgroup Output type
  * @{
  */
typedef enum {
	GPIO_OType_PP		= ((uint8_t) 0x00),
	GPIO_OType_OD		= ((uint8_t) 0x01)
} GPIO_OType_TypeDef;
/**
  * @}
  */

/**	@defgroup Output maximum frequency
  * @{
  */
typedef enum {
	GPIO_Speed_8MHz		= ((uint8_t) 0x00),		/* I/O low speed - 8MHz */
	GPIO_Speed_50MHz	= ((uint8_t) 0x01),		/* I/O medium speed - 50MHz */
	GPIO_Speed_100MHz 	= ((uint8_t) 0x02),		/* I/O high speed - 100MHz */
	GPIO_Speed_180MHz	= ((uint8_t) 0x03)		/* I/O super speed - 180MHz */
} GPIO_Speed_TypeDef;
/**
  * @}
  */

/**	@defgroup Configuration pull-up/pull-down
  * @{
  */
typedef enum {
	GPIO_PuPd_NOPULL	= ((uint8_t) 0x00),
	GPIO_PuPd_UP		= ((uint8_t) 0x01),
	GPIO_PuPd_DOWN		= ((uint8_t) 0x03)
} GPIO_PuPd_TypeDef;
/**
  * @}
  */

/**	@brief  GPIO initialisation structure
  * @{
  */
typedef struct {
	uint16_t GPIO_Pin;					/* Determine the GPIO pins to be configured.
											this parameter can be any value of @ref GPIO_pins_define */

	GPIO_Mode_TypeDef GPIO_Mode;		/* Determine the GPIO operating mode for the selected pins.
											This parameter can be a value of @ref GPIO_Mode_TypeDef */

	GPIO_Speed_TypeDef GPIO_Speed;		/* Determine the GPIO speed for the selected pins.
											This parameter can be a value of @ref GPIO_Speed_TypeDef */

	GPIO_OType_TypeDef GPIO_OType;		/* Determine the GPIO output type for the selected pins.
											This parameter  can be a value of @ref GPIO_OType_TypeDef */

	GPIO_PuPd_TypeDef GPIO_PuPd;		/* Determine the GPIO pull-up/pull-down for the selected pins.
											This parameter can be a value of @ref GPIO_PuPd_TypeDef */
} GPIO_Init_TypeDef;
/**
  * @}
  */

/**	@defgroup GPIO alternate function
  * @{
  */
#define GPIO_AF0		((uint8_t)0x00)
#define GPIO_AF1		((uint8_t)0x01)
#define GPIO_AF2		((uint8_t)0x02)
#define GPIO_AF3		((uint8_t)0x03)
#define GPIO_AF4		((uint8_t)0x04)
#define GPIO_AF5		((uint8_t)0x05)
#define GPIO_AF6		((uint8_t)0x06)
#define GPIO_AF7		((uint8_t)0x07)
/**
  * @}
  */

/* Functions prototypes ----------------------------------------------------- */
/* GPIO reset to the default state function ***********************************/
void GPIO_deinit(GPIO_TypeDef* GPIOx);

/* GPIO initialisation and configuration functions ****************************/
void GPIO_init(GPIO_TypeDef* GPIOx, GPIO_Init_TypeDef* GPIO_Init_Struct);
void GPIO_pin_lock_config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* GPIO read and write functions **********************************************/
uint8_t GPIO_read_input_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_read_input(GPIO_TypeDef* GPIOx);
uint8_t GPIO_read_output_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_read_output(GPIO_TypeDef* GPIOx);
void GPIO_set_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_reset_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_write_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Bit_Action Bit_Val);
void GPIO_write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_toggle_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* GPIO alternate function configuration function ****************************/
void GPIO_AF_config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_Source, uint8_t GPIO_AF);

/* GPIO interrupt configuration functions *************************************/


#ifdef __cpluslus
}
#endif

#endif /* __STM32F446XX_GPIO_H */

/* END OF FILE ****************************************************************/
