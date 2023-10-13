/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of SPI
* resource as Master using HAL. The SPI master sends command packetsto the SPI
* slave to control an user LED.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* SPI baud rate in Hz */
#define SPI_FREQ_HZ                (1000000UL)
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY           (1000UL)
/* SPI transfer bits per frame */
#define BITS_PER_FRAME             (8)


/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* The main function.
*   1. Initializes the board, retarget-io and led
*   2. Configures the SPI Master to send command packet to the slave
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
    uint32_t cmd_send = CYBSP_LED_STATE_OFF;
    cyhal_spi_t mSPI;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize retarget-io for uart logs */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                  CY_RETARGET_IO_BAUDRATE);
    /* Retarget-io init failed. Stop program execution */
    handle_error(result);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("*************** "
           "HAL: SPI Master "
           "*************** \r\n\n");

    printf("Configuring SPI master...\r\n");
    /* Init SPI master */
    result = cyhal_spi_init(&mSPI,CYBSP_SPI_MOSI,CYBSP_SPI_MISO,CYBSP_SPI_CLK,
                            CYBSP_SPI_CS,NULL,BITS_PER_FRAME,
                            CYHAL_SPI_MODE_00_MSB,false);
    handle_error(result);

    /* Set the SPI baud rate */
    result = cyhal_spi_set_frequency(&mSPI, SPI_FREQ_HZ);
    handle_error(result);

    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
        /* Toggle the slave LED state */
        cmd_send = (cmd_send == CYBSP_LED_STATE_OFF) ?
                     CYBSP_LED_STATE_ON : CYBSP_LED_STATE_OFF;

        /* Send the command packet to the slave */
        result = cyhal_spi_send(&mSPI, cmd_send);

        handle_error(result);

        /* Give delay between commands */
        cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
    }
}


/* [] END OF FILE */
