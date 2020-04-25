/**
  ******************************************************************************
  * @file 		stm32f446xx_gpio.c
  * @author  	Kamil Mezynski
  * @version 	v1.0.0
  * @date    	22-March-2020
  * @brief   	This file contains functions definitions for the GPIO
  *				peripheral for STM32F446xx MCU
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f446xx_rcc.h"
#include "stm32f446xx_gpio.h"

/* Functions declarations --------------------------------------------------- */

/**	@defgroup GPIO reset to the default state function
  *	@{
================================================================================
                  ##### GPIO Reset to the Default State #####
================================================================================
  */

/**
  * @brief	Deinitializes the GPIOx peripheral register to its default reset state
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @retval None
  */
void GPIO_deinit(GPIO_TypeDef* GPIOx) {
	if (GPIOx == GPIOA) {
		RCC_AHB_set(RCC_AHB1_GPIOA, DISABLE);
	} else if (GPIOx == GPIOB) {
		RCC_AHB_set(RCC_AHB1_GPIOB, DISABLE);
	} else if (GPIOx == GPIOC) {
		RCC_AHB_set(RCC_AHB1_GPIOC, DISABLE);
	} else if (GPIOx == GPIOD) {
		RCC_AHB_set(RCC_AHB1_GPIOD, DISABLE);
	} else if (GPIOx == GPIOE) {
		RCC_AHB_set(RCC_AHB1_GPIOE, DISABLE);
	} else if (GPIOx == GPIOF) {
		RCC_AHB_set(RCC_AHB1_GPIOF, DISABLE);
	} else if (GPIOx == GPIOG) {
		RCC_AHB_set(RCC_AHB1_GPIOG, DISABLE);
	} else if (GPIOx == GPIOH) {
		RCC_AHB_set(RCC_AHB1_GPIOH, DISABLE);
	}
}

/**
  *	@}
  */

/**	@defgroup GPIO initialization and configuration functions
  *	@{
================================================================================
                  ##### GPIO Initialization and Configuration #####
================================================================================
  */

/**
  * @brief 	Initializes the GPIOx peripheral according to the specified parameters
  *			in the GPIO_Init_Struct
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Init_Struct: pointer to a GPIO_Init_TypeDef structure that contains
  *			the configuration parameters for the specified GPIO peripheral
  * @retval	None
  */
void GPIO_init(GPIO_TypeDef* GPIOx, GPIO_Init_TypeDef* GPIO_Init_Struct) {
	uint16_t pin_pos = 0x00, pos = 0x00, current_pin = 0x00;

	/* Port pins configuration */
	for (pin_pos = 0x00; pin_pos < 0x10; ++pin_pos) {
		pos = 0x01 << pin_pos;

		/* Get the port pins position */
		current_pin = (GPIO_Init_Struct->GPIO_Pin) & pos;

		if (current_pin == pos) {
			if ((GPIO_Init_Struct->GPIO_Mode == GPIO_Mode_OUT) || (GPIO_Init_Struct->GPIO_Mode == GPIO_Mode_AF)) {
				/* Speed mode configuration */
				GPIOx->OSPEEDR |= (GPIO_Init_Struct->GPIO_Speed) << (pin_pos * 2);

				/* Output mode configuration */
				GPIOx->OTYPER |= (GPIO_Init_Struct->GPIO_PuPd) << (pin_pos);
			}
			/* Port mode configuration */
			GPIOx->MODER |= (GPIO_Init_Struct->GPIO_Mode) << (pin_pos * 2);

			/* Pull-up/pull-down resistor configuration */
			GPIOx->PUPDR |= (GPIO_Init_Struct->GPIO_PuPd) << (pin_pos * 2);
		}
	}
}

/**
  * @brief 	Locks GPIO pins configuration registers
  * @note 	The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
  *			GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH
  * @note	The configuration of the locked GPIO pins can no longer be modified
  *			until the next MCU or peripheral reset
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Pin: specifies the port bit to be written
  *			This parameter can be GPIO_Pinx where x can be (0..15)
  * @retval	None
  */
void GPIO_pin_lock_config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	__IO uint32_t tmp = 0x00010000;
	tmp |= GPIO_Pin;

	/* LCKK write sequence */
	/* Set LCKK bit */
	GPIOx->LCKR = tmp;
	/* Reset LCKK bit */
	GPIOx->LCKR = GPIO_Pin;
	/* Set LCKK bit */
	GPIOx->LCKR = tmp;
	/* Read LCKK bit */
	tmp = GPIOx->LCKR;
	/* Read LCKK bit */
	tmp = GPIOx->LCKR;
}

/**
  *	@}
  */

/**	@defgroup GPIO read and write functions
  *	@{
================================================================================
                      ##### GPIO Read and Write #####
================================================================================
  */

/**
  * @brief	Reads the specified input port pin
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Pin: specifies the port bit to be written
  *			This parameter can be GPIO_Pinx where x can be (0..15)
  * @retval	The input port pin value
  */
uint8_t GPIO_read_input_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	uint8_t bit_status = 0x00;

	if ((GPIOx->IDR & GPIO_Pin) != Bit_RESET)
		bit_status = Bit_SET;
	else
		bit_status = Bit_RESET;

	return (uint8_t)bit_status;
}

/**
  * @brief	Reads the specified input port pin
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @retval	The input port pin value
  */
uint16_t GPIO_read_input(GPIO_TypeDef* GPIOx) {
	return (uint16_t)GPIOx->IDR;
}

/**
  * @brief  Reads the specified output data port bit
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Pin: specifies the port bit to be written
  *			This parameter can be GPIO_Pinx where x can be (0..15)
  * @retval The output port pin value
  */
uint8_t GPIO_read_output_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	uint8_t bit_status = 0x00;

	if ((GPIOx->ODR & GPIO_Pin) != Bit_RESET)
		bit_status = Bit_SET;
	else
		bit_status = Bit_RESET;

	return (uint8_t)bit_status;
}

/**
  * @brief  Reads the specified GPIO output data port
  * @param  GPIOx: where x can be (A, B, C, D, E, F, G or H) to select the GPIO peripheral
  * @retval GPIO output data port value
  */
uint16_t GPIO_read_output(GPIO_TypeDef* GPIOx) {
	return (uint16_t)GPIOx->ODR;
}

/**
  * @brief  Sets the selected data port bits
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Pin: specifies the port bit to be written
  *			This parameter can be GPIO_Pinx where x can be (0..15)
  * @retval None
  */
void GPIO_set_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOx->BSRR = GPIO_Pin;
}

/**
  * @brief  Clears the selected data port bits
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Pin: specifies the port bit to be written
  *			This parameter can be GPIO_Pinx where x can be (0..15)
  * @retval None
  */
void GPIO_reset_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOx->BSRR = GPIO_Pin << 16;
}

/**
  * @brief  Sets or clears the selected data port bit
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Pin: specifies the port bit to be written
  *			This parameter can be GPIO_Pinx where x can be (0..15)
  * @param  Bit_Val: specifies the value to be written to the selected bit
  *         This parameter can be (Bit_SET, Bit_RESET)
  * @retval None
  */
void GPIO_write_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Bit_Action Bit_Val) {
	if (Bit_Val != Bit_RESET)
		GPIOx->BSRR = GPIO_Pin;
	else
		GPIOx->BSRR = GPIO_Pin << 16;
}

/**
  * @brief  Writes data to the specified GPIO data port
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Pin: specifies the port bit to be written
  *			This parameter can be GPIO_Pinx where x can be (0..15)
  * @retval None
  */
void GPIO_write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOx->ODR = GPIO_Pin;
}

/**
  * @brief  Toggles data at the specified GPIO data port
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param	GPIO_Pin: specifies the port bit to be written
  *			This parameter can be GPIO_Pinx where x can be (0..15)
  * @retval None
  */
void GPIO_toggle_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOx->ODR ^= GPIO_Pin;
}

/**
  * @}
  */

/**	@defgroup GPIO alternate function configuration function
  *	@{
================================================================================
                      ##### GPIO Alternate Function #####
================================================================================
  */

/**
  * @brief  Writes data to the specified GPIO data port
  * @param	GPIOx: specifies the port to be selected
  * 		This parameter can be GPIOx where x can be (A, B, C, D, E, F, G or H)
  * @param  GPIO_Pin_Source: specifies the pin for the Alternate Function
  *         This parameter can be GPIO_PinSourcex where x can be (0..15)
  * @param  GPIO_AF: selects the pin to used as Alternate Function
  *         This parameter can be GPIO_AFx where x can be (0..7)
  * @note   The pin should already been configured in Alternate Function mode(AF)
  * @retval None
  */
void GPIO_AF_config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_Source, uint8_t GPIO_AF) {
	uint32_t temp = 0x00;
	uint32_t temp_2 = 0x00;

	temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_Pin_Source & (uint32_t)0x07) * 4));
	GPIOx->AFR[GPIO_Pin_Source >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_Pin_Source & (uint32_t)0x07) * 4));
	temp_2 = GPIOx->AFR[GPIO_Pin_Source >> 0x03] | temp;
	GPIOx->AFR[GPIO_Pin_Source >> 0x03] = temp_2;
}

/**
  * @}
  */

/**	@defgroup GPIO interrupt configuration functions
  *	@{
================================================================================
                   ##### GPIO Interrupt Configuration #####
================================================================================
  */

/**
  * @}
  */

/* END OF FILE ****************************************************************/
