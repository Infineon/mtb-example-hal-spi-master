/******************************************************************************
* File Name: main.c
*
* Description: This example project demonstrates the basic operation of the I2C master
* resource using high level APIs. The I2C master SCB sends the
* command packets to the I2C slave SCB to control an user LED.
*
*******************************************************************************
* Copyright (2018-2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "resource_map.h"

/***************************************
*            Constants
****************************************/
#define CMD_TO_CMD_DELAY     	(1000UL)

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
*	1. SPI Master sends command packet to the slave
*	2. Slave reads the packet and executes the instructions
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    /* Set up the device based on configurator selections */
    result = cybsp_init();
	if(result != CY_RSLT_SUCCESS)
	{
		handle_error();
	}
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if(result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("**************************\r\n");
    printf("PSoC 6 MCU SPI Master\r\n");
    printf("**************************\r\n\n");

    cyhal_spi_t mSPI;
    cyhal_spi_t sSPI;

    /* Configure user LED */
    printf(">> Configure user LED \r\n");
    result = cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	if(result != CY_RSLT_SUCCESS)
	{
		handle_error();
	}

    /* Configure SPI Master */
    printf(">> Configure SPI master \r\n");
    result = cyhal_spi_init(&mSPI, mSPI_MOSI, mSPI_MISO, mSPI_SCLK, mSPI_SS, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);
    if(result != CY_SCB_SPI_SUCCESS)
    {
    	handle_error();
    }
    result = cyhal_spi_set_frequency(&mSPI, 1000000);
    if(result != CY_SCB_SPI_SUCCESS)
    {
    	handle_error();
    }

    /* Configure SPI Slave */
    printf(">> Configure SPI slave \r\n\n");
    result = cyhal_spi_init(&sSPI, sSPI_MOSI, sSPI_MISO, sSPI_SCLK, sSPI_SS, NULL, 8, CYHAL_SPI_MODE_00_MSB, true);
    if(result != CY_SCB_SPI_SUCCESS)
    {
    	handle_error();
    }
    result = cyhal_spi_set_frequency(&sSPI, 1000000);
    if(result != CY_SCB_SPI_SUCCESS)
    {
    	handle_error();
    }

    uint32_t cmd_send = true;
    uint32_t cmd_recv = false;

    /* Enable interrupts */
    __enable_irq();

    printf("User LED should start blinking \r\n");
 
    for(;;)
    {
    	/* Toggle the current LED state */
    	cmd_send = (cmd_send == true) ? false : true;

    	/* Send packet with command to the slave. */
        if (CY_RSLT_SUCCESS == cyhal_spi_send(&mSPI, cmd_send))
		{
        	/* The below code is for slave function. It is implemented in
        	   this code example so that the master function can be tested
        	   without the need of one more kit. */

			/* Read command packet at slave */
			if (CY_SCB_I2C_SUCCESS == cyhal_spi_recv(&sSPI, &cmd_recv))
			{
				/* Execute command */
				cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED, cmd_recv);
			}
			else
			{
				handle_error();
			}
		}
        else
        {
        	handle_error();
        }
		/* Give delay between commands. */
		Cy_SysLib_Delay(CMD_TO_CMD_DELAY);
    }
}
