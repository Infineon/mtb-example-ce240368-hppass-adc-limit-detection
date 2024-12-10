/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the HPPASS SAR ADC limit detection
*              example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cy_pdl.h"
#include "cybsp.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* The definition of HPPASS AC startup timeout in microseconds.
 * HPPASS startup time contains AREF startup 40us, CSG startup about 15us and 
 * SAR ADC maximum self-calibration time 9ms (HPPASS input clock is 240MHz). To be
 * on the safe side, add to 10ms.
 */
#define HPPASS_AC_STARTUP_TIMEOUT           (10000U)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t  DEBUG_UART_context;   /* Debug UART context */
static mtb_hal_uart_t DEBUG_UART_hal_obj;               /* Debug UART HAL object */

/* SAR ADC limit 0/1/2/3 interrupt flag */
volatile bool adc_limit_0_int_flag = false;
volatile bool adc_limit_1_int_flag = false;
volatile bool adc_limit_2_int_flag = false;
volatile bool adc_limit_3_int_flag = false;

/* The voltage value of ADC channel */
float32_t adc_channel_voltage = 0.0;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
/* SAR ADC limit 0/1/2/3 interrupt handler */
void adc_limit_0_intr_handler(void);
void adc_limit_1_intr_handler(void);
void adc_limit_2_intr_handler(void);
void adc_limit_3_intr_handler(void);

/* Get ADC channel voltage value */
float32_t get_adc_channel_voltage(uint8_t chanIdx);

/* Check if the user button is pressed */
bool user_button_is_pressed(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU.
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

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the debug UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable SAR ADC limit 0/1/2/3 interrupt */
    Cy_HPPASS_SAR_Limit_SetInterruptMask(CY_HPPASS_INTR_SAR_LIMIT_GROUP_0
                                         |CY_HPPASS_INTR_SAR_LIMIT_GROUP_1
                                         |CY_HPPASS_INTR_SAR_LIMIT_GROUP_2
                                         |CY_HPPASS_INTR_SAR_LIMIT_GROUP_3);

    /* Configure SAR ADC limit 0 interrupt */
    cy_stc_sysint_t sar_adc_limit_intr_config;
    sar_adc_limit_intr_config.intrPriority = 0;
    sar_adc_limit_intr_config.intrSrc = pass_interrupt_sar_range_0_IRQn;
    Cy_SysInt_Init(&sar_adc_limit_intr_config, adc_limit_0_intr_handler);
    NVIC_EnableIRQ(sar_adc_limit_intr_config.intrSrc);

    /* Configure SAR ADC limit 1 interrupt */
    sar_adc_limit_intr_config.intrSrc = pass_interrupt_sar_range_1_IRQn;
    Cy_SysInt_Init(&sar_adc_limit_intr_config, adc_limit_1_intr_handler);
    NVIC_EnableIRQ(sar_adc_limit_intr_config.intrSrc);

    /* Configure SAR ADC limit 2 interrupt */
    sar_adc_limit_intr_config.intrSrc = pass_interrupt_sar_range_2_IRQn;
    Cy_SysInt_Init(&sar_adc_limit_intr_config, adc_limit_2_intr_handler);
    NVIC_EnableIRQ(sar_adc_limit_intr_config.intrSrc);

    /* Configure SAR ADC limit 3 interrupt */
    sar_adc_limit_intr_config.intrSrc = pass_interrupt_sar_range_3_IRQn;
    Cy_SysInt_Init(&sar_adc_limit_intr_config, adc_limit_3_intr_handler);
    NVIC_EnableIRQ(sar_adc_limit_intr_config.intrSrc);

    /* Start the HPPASS autonomous controller (AC) from state 0 */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, HPPASS_AC_STARTUP_TIMEOUT))
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("HPPASS: SAR ADC limit detection example\r\n");
    printf("********************************************************************************\r\n");
    printf("Press user switch (SW2) to start SAR ADC conversion\r\n");

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Check if 'SW2' key was pressed */
        if(user_button_is_pressed() && (!Cy_HPPASS_SAR_IsBusy()))
        {
            /* Trigger SAR ADC group 0 conversion */
            Cy_HPPASS_SetFwTrigger(CY_HPPASS_TRIG_0_MSK);
            printf("\r\nStart SAR ADC conversion\r\n");
        }

        /* ADC limit 0 detection */
        if(adc_limit_0_int_flag)
        {
            adc_limit_0_int_flag = false;
            adc_channel_voltage = get_adc_channel_voltage(AN_A0_CHAN_IDX);
            printf("ADC limit 0 detected the AN_A0 input voltage (%.1fV) is below 1.0V\r\n", adc_channel_voltage);
        }

        /* ADC limit 1 detection */
        if(adc_limit_1_int_flag)
        {
            adc_limit_1_int_flag = false;
            adc_channel_voltage = get_adc_channel_voltage(AN_A1_CHAN_IDX);
            printf("ADC limit 1 detected the AN_A1 input voltage (%.1fV) is inside 1.0V and 2.0V\r\n", adc_channel_voltage);
        }

        /* ADC limit 2 detection */
        if(adc_limit_2_int_flag)
        {
            adc_limit_2_int_flag = false;
            adc_channel_voltage = get_adc_channel_voltage(AN_A2_CHAN_IDX);
            printf("ADC limit 2 detected the AN_A2 input voltage (%.1fV) is above 2.0V\r\n", adc_channel_voltage);
        }

        /* ADC limit 3 detection */
        if(adc_limit_3_int_flag)
        {
            adc_limit_3_int_flag = false;
            adc_channel_voltage = get_adc_channel_voltage(AN_A3_CHAN_IDX);
            printf("ADC limit 3 detected the AN_A3 input voltage (%.1fV) is outside 1.0V and 2.0V\r\n", adc_channel_voltage);
        }
    }
}

/*******************************************************************************
* Function Name: adc_limit_0_intr_handler
********************************************************************************
* Summary:
* This function is the SAR ADC limit 0 interrupt handler
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void adc_limit_0_intr_handler(void)
{
    /* Clear SAR ADC limit 0 interrupt */
    Cy_HPPASS_SAR_Limit_ClearInterrupt(CY_HPPASS_INTR_SAR_LIMIT_GROUP_0);
    /* Set the ADC limit 0 interrupt flag */
    adc_limit_0_int_flag = true;
}

/*******************************************************************************
* Function Name: adc_limit_1_intr_handler
********************************************************************************
* Summary:
* This function is the SAR ADC limit 1 interrupt handler
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void adc_limit_1_intr_handler(void)
{
    /* Clear SAR ADC limit 1 interrupt */
    Cy_HPPASS_SAR_Limit_ClearInterrupt(CY_HPPASS_INTR_SAR_LIMIT_GROUP_1);
    /* Set the ADC limit 1 interrupt flag */
    adc_limit_1_int_flag = true;
}

/*******************************************************************************
* Function Name: adc_limit_2_intr_handler
********************************************************************************
* Summary:
* This function is the SAR ADC limit 2 interrupt handler
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void adc_limit_2_intr_handler(void)
{
    /* Clear SAR ADC limit 2 interrupt */
    Cy_HPPASS_SAR_Limit_ClearInterrupt(CY_HPPASS_INTR_SAR_LIMIT_GROUP_2);
    /* Set the ADC limit 2 interrupt flag */
    adc_limit_2_int_flag = true;
}

/*******************************************************************************
* Function Name: adc_limit_3_intr_handler
********************************************************************************
* Summary:
* This function is the SAR ADC limit 3 interrupt handler
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void adc_limit_3_intr_handler(void)
{
    /* Clear SAR ADC limit 3 interrupt */
    Cy_HPPASS_SAR_Limit_ClearInterrupt(CY_HPPASS_INTR_SAR_LIMIT_GROUP_3);
    /* Set the ADC limit 3 interrupt flag */
    adc_limit_3_int_flag = true;
}

/*******************************************************************************
* Function Name: get_adc_channel_voltage
****************************************************************************//**
* Summary:
*  Returns the ADC channel voltage value.
*
* Parameters:
*  chanIdx - The index of the channel instance.
*
* Return:
*  float32_t - Channel voltage value.
*
*
*******************************************************************************/
float32_t get_adc_channel_voltage(uint8_t chanIdx)
{
     /* Get channel data */
    uint16_t channel_result = Cy_HPPASS_SAR_Result_ChannelRead(chanIdx);
    /* Convert the result to voltage, 3300 is the ADC reference voltage (3.3V) */
    return Cy_HPPASS_SAR_CountsTo_Volts(chanIdx, 3300, channel_result);
}

/*******************************************************************************
* Function Name: user_button_is_pressed
****************************************************************************//**
* Summary:
*  Check if the user button is pressed.
*
* Return:
*  Returns the status of user button.
*
*******************************************************************************/
bool user_button_is_pressed(void)
{
    uint32_t pressCount = 0;

    if(Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) != CYBSP_BTN_PRESSED)
    {
        return false;
    }
    /* Check if User button is pressed */
    while (Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        Cy_SysLib_Delay(10);
        pressCount++;
    }
    /* Add a delay to avoid glitches */
    Cy_SysLib_Delay(10);

    if(10 < pressCount)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* [] END OF FILE */
