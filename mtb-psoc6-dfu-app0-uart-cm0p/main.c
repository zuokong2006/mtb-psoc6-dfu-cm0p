/******************************************************************************
* File Name:   main.c
*
* Description:
* This file provides App0 example source. App0 firmware does the following:
* - Downloads App1 firmware image if Host sends it
* - Switches to App1 if App1 image has successfully downloaded and is valid
* - Switches to existing App1 if button is pressed
* - Blinks a LED
* - Halts on timeout
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2019, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
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
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cy_dfu.h"


/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Name: CopyRow
********************************************************************************
* Copies data from a "src" address to a flash row with the address "dest".
* If "src" data is the same as "dest" data then no copy is needed.
*
* Parameters:
*  dest     Destination address. Has to be an address of the start of flash row.
*  src      Source address. Has to be properly aligned.
*  rowSize  Size of flash row.
*
* Returns:
*  CY_DFU_SUCCESS if operation is successful.
*  Error code in a case of failure.
*******************************************************************************/
cy_en_dfu_status_t CopyRow(uint32_t dest, uint32_t src, uint32_t rowSize, cy_stc_dfu_params_t * params)
{
    cy_en_dfu_status_t status;

    /* Save params->dataBuffer value */
    uint8_t *buffer = params->dataBuffer;

    /* Compare "dest" and "src" content */
    params->dataBuffer = (uint8_t *)src;
    status = Cy_DFU_ReadData(dest, rowSize, CY_DFU_IOCTL_COMPARE, params);

    /* Restore params->dataBuffer */
    params->dataBuffer = buffer;

    /* If "dest" differs from "src" then copy "src" to "dest" */
    if (status != CY_DFU_SUCCESS)
    {
        (void) memcpy((void *) params->dataBuffer, (const void*)src, rowSize);
        status = Cy_DFU_WriteData(dest, rowSize, CY_DFU_IOCTL_WRITE, params);
    }
    /* Restore params->dataBuffer */
    params->dataBuffer = buffer;

    return (status);
}


/*******************************************************************************
* Function Name: HandleMetadata
********************************************************************************
* The goal of this function is to make DFU SDK metadata (MD) valid.
* The following algorithm is used (in C-like pseudocode):
* ---
* if (isValid(MD) == true)
* {   if (MDC != MD)
*         MDC = MD;
* } else
* {   if(isValid(MDC) )
*         MD = MDC;
*     else
*         MD = INITIAL_VALUE;
* }
* ---
* Here MD is metadata flash row, MDC is flash row with metadata copy,
* INITIAL_VALUE is known initial value.
*
* In this code example MDC is placed in the next flash row after the MD, and
* INITIAL_VALUE is MD with only CRC, App0 start and size initialized,
* all the other fields are not touched.
*
* Parameters:
*  params   A pointer to a DFU SDK parameters structure.
*
* Returns:
* - CY_DFU_SUCCESS when finished normally.
* - Any other status code on error.
*******************************************************************************/
cy_en_dfu_status_t HandleMetadata(cy_stc_dfu_params_t *params)
{
    const uint32_t MD     = (uint32_t)(&__cy_boot_metadata_addr   ); /* MD address  */
    const uint32_t mdSize = (uint32_t)(&__cy_boot_metadata_length ); /* MD size, assumed to be one flash row */
    const uint32_t MDC    = MD + mdSize;                             /* MDC address */

    cy_en_dfu_status_t status = CY_DFU_SUCCESS;

    status = Cy_DFU_ValidateMetadata(MD, params);
    if (status == CY_DFU_SUCCESS)
    {
        /* Checks if MDC equals to DC, if no then copies MD to MDC */
        status = CopyRow(MDC, MD, mdSize, params);
    }
    else
    {
        status = Cy_DFU_ValidateMetadata(MDC, params);
        if (status == CY_DFU_SUCCESS)
        {
            /* Copy MDC to MD */
            status = CopyRow(MD, MDC, mdSize, params);
        }
        if (status != CY_DFU_SUCCESS)
        {
            const uint32_t elfStartAddress = 0x10000000;
            const uint32_t elfAppSize      = 0x8000;
            /* Set MD to INITIAL_VALUE */
            status = Cy_DFU_SetAppMetadata(0u, elfStartAddress, elfAppSize, params);
        }
    }
    return (status);
}


/*******************************************************************************
* Function Name: counterTimeoutSeconds
********************************************************************************
* Returns number of counts that correspond to number of seconds passed as
* a parameter.
* E.g. comparing counter with 300 seconds is like this.
* ---
* uint32_t counter = 0u;
* for (;;)
* {
*     Cy_SysLib_Delay(UART_TIMEOUT);
*     ++count;
*     if (count >= counterTimeoutSeconds(seconds: 300u, timeout: UART_TIMEOUT))
*     {
*         count = 0u;
*         DoSomething();
*     }
* }
* ---
*
* Both parameters are required to be compile time constants,
* so this function gets optimized out to single constant value.
*
* Parameters:
*  seconds    Number of seconds to pass. Must be less that 4_294_967 seconds.
*  timeout    Timeout for Cy_DFU_Continue() function, in milliseconds.
*             Must be greater than zero.
*             It is recommended to be a value that produces no reminder
*             for this function to be precise.
* Return:
*  See description.
*******************************************************************************/
static uint32_t counterTimeoutSeconds(uint32_t seconds, uint32_t timeout);
static uint32_t counterTimeoutSeconds(uint32_t seconds, uint32_t timeout)
{
    return (seconds * 1000ul) / timeout;
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  Main function of the firmware application.
*  1. If application started from Non-Software reset it validates app #1
*  1.1. If app#1 is valid it switches to app#1, else goto #2.
*  2. Start DFU communication.
*  3. If updated application has been received it validates this app.
*  4. If app#1 is valid it switches to it, else wait for new application.
*  5. If 300 seconds has passed and no new application has been received
*     then validate app#1, if it is valid then switch to it, else freeze.
*
* Parameters:
*  seconds    Number of seconds to pass
*  timeout    Timeout for Cy_DFU_Continue() function, in milliseconds
*
* Return:
*  Counter value at which specified number of seconds has passed.
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* timeout for Cy_DFU_Continue(), in milliseconds */
    const uint32_t paramsTimeout = 20u;

    /* DFU params, used to configure DFU */
    cy_stc_dfu_params_t dfuParams;

    /* Status codes for DFU SDK API */
    cy_en_dfu_status_t status;

    /*
    * DFU state, one of the:
    * - CY_DFU_STATE_NONE
    * - CY_DFU_STATE_UPDATING
    * - CY_DFU_STATE_FINISHED
    * - CY_DFU_STATE_FAILED
    */
    uint32_t state;

    /*
    * Used to count seconds, to convert counts to seconds use
    * counterTimeoutSeconds(SECONDS, paramsTimeout)
    */
    uint32_t count = 0;

#if CY_DFU_OPT_CRYPTO_HW != 0
    cy_en_crypto_status_t cryptoStatus;
#endif

    /* Buffer to store DFU commands */
    CY_ALIGN(4) static uint8_t buffer[CY_DFU_SIZEOF_DATA_BUFFER];

    /* Buffer for DFU packets for Transport API */
    CY_ALIGN(4) static uint8_t packet[CY_DFU_SIZEOF_CMD_BUFFER ];

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize the User LED */
    result = cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_LED, \
              CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, \
              CYBSP_LED_STATE_OFF);

    /* gpio init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User Button */
    result = cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_BTN, \
              CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, \
              CYBSP_BTN_OFF);

    /* gpio init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

#if CY_DFU_OPT_CRYPTO_HW != 0
    /* Initialize the Crypto Client code */
    cryptoStatus = Cy_Crypto_Init(&cryptoConfig, &cryptoContext);
    if (cryptoStatus != CY_CRYPTO_SUCCESS)
    {
        /* Crypto not initialized, debug what is the problem */
        Cy_SysLib_Halt(0x00u);
    }
#endif /* CY_BOOTLOAD_OPT_CRYPTO_HW != 0 */

    /* start up M4 core, with the CM4 core start address defined in the
       DFU SDK linker script */
    Cy_SysEnableCM4( (uint32_t)(&__cy_app_core1_start_addr) );

    /* Initialize dfuParams structure and DFU SDK state */
    dfuParams.timeout          = paramsTimeout;
    dfuParams.dataBuffer       = &buffer[0];
    dfuParams.packetBuffer     = &packet[0];

    status = Cy_DFU_Init(&state, &dfuParams);

    /* Ensure DFU Metadata is valid */
    status = HandleMetadata(&dfuParams);
    if (status != CY_DFU_SUCCESS)
    {
        Cy_SysLib_Halt(0x00u);
    }

    /*
    * In the case of non-software reset check if there is a valid app image.
    * If these is - switch to it.
    */

    if (Cy_SysLib_GetResetReason() != CY_SYSLIB_RESET_SOFT)
    {
        status = Cy_DFU_ValidateApp(1u, &dfuParams);
        if (status == CY_DFU_SUCCESS)
        {
            /*
            * Clear the reset reason because Cy_DFU_ExecuteApp() performs a
            * software reset. Without clearing it, two reset reasons would be
            * present.
            */
            do
            {
                Cy_SysLib_ClearResetReason();
            }while(Cy_SysLib_GetResetReason() != 0);

            /* Never returns */
            Cy_DFU_ExecuteApp(1u);
        }
    }

    /* Initialize DFU communication */
    Cy_DFU_TransportStart();

    for(;;)
    {
        status = Cy_DFU_Continue(&state, &dfuParams);
        ++count;

        if (state == CY_DFU_STATE_FINISHED)
        {
            /* Finished loading the application image */

            /* Validate DFU application, if it is valid then switch to it */
            status = Cy_DFU_ValidateApp(1u, &dfuParams);
            if (status == CY_DFU_SUCCESS)
            {
                Cy_DFU_TransportStop();
                Cy_DFU_ExecuteApp(1u);
            }
            else if (status == CY_DFU_ERROR_VERIFY)
            {
                /*
                * Restarts loading, an alternatives are to Halt MCU here
                * or switch to the other app if it is valid.
                * Error code may be handled here, i.e. print to debug UART.
                */
                status = Cy_DFU_Init(&state, &dfuParams);
                Cy_DFU_TransportReset();
            }
        }
        else if (state == CY_DFU_STATE_FAILED)
        {
            /* An error has happened during the loading process */
            /* Handle it here */

            /* In this Code Example just restart loading process */
            status = Cy_DFU_Init(&state, &dfuParams);
            Cy_DFU_TransportReset();
        }
        else if (state == CY_DFU_STATE_UPDATING)
        {
            uint32_t passed5seconds = (count >= counterTimeoutSeconds(5u, paramsTimeout) ) ? 1u : 0u;
            /*
            * if no command has been received during 5 seconds when the loading
            * has started then restart loading.
            */
            if (status == CY_DFU_SUCCESS)
            {
                count = 0u;
            }
            else if (status == CY_DFU_ERROR_TIMEOUT)
            {
                if (passed5seconds != 0u)
                {
                    count = 0u;
                    Cy_DFU_Init(&state, &dfuParams);
                    Cy_DFU_TransportReset();
                }
            }
            else
            {
                count = 0u;
                /* Delay because Transport still may be sending error response to a host */
                Cy_SysLib_Delay(paramsTimeout);
                Cy_DFU_Init(&state, &dfuParams);
                Cy_DFU_TransportReset();
            }
        }

        /* No image has been received in 300 seconds, try to load existing image, or sleep */
        if( (count >= counterTimeoutSeconds(300u, paramsTimeout) ) && (state == CY_DFU_STATE_NONE) )
        {
            /* Stop loading communication */
            Cy_DFU_TransportStop();
            /* Check if app is valid, if it is then switch to it */
            status = Cy_DFU_ValidateApp(1u, &dfuParams);
            if (status == CY_DFU_SUCCESS)
            {
                Cy_DFU_ExecuteApp(1u);
            }
            /* 300 seconds has passed and App is invalid. Handle that */
            Cy_SysLib_Halt(0x00u);
        }

        /* Blink once per two seconds */
        if ( ( count % counterTimeoutSeconds(1u, paramsTimeout) ) == 0u)
        {
            cyhal_gpio_toggle((cyhal_gpio_t)CYBSP_USER_LED);
        }

        /* If Button clicked - Switch to App1 if it is valid */
        if (cyhal_gpio_read((cyhal_gpio_t)CYBSP_USER_BTN) == 0u)
        {
            /* 50 ms delay for button debounce on button press */
            Cy_SysLib_Delay(50u);

            if (cyhal_gpio_read((cyhal_gpio_t)CYBSP_USER_BTN) == 0u)
            {
                while (cyhal_gpio_read((cyhal_gpio_t)CYBSP_USER_BTN) == 0u)
                {   /* 50 ms delay for button debounce on button release */
                    Cy_SysLib_Delay(50u);
                }

                /* Validate and switch to App1 */
                status = Cy_DFU_ValidateApp(1u, &dfuParams);

                if (status == CY_DFU_SUCCESS)
                {
                    Cy_DFU_TransportStop();
                    Cy_DFU_ExecuteApp(1u);
                }
            }
        }
    }
}

/*******************************************************************************
* Function Name: Cy_OnResetUser
********************************************************************************
*
*  This function is called at the start of Reset_Handler().
*  DFU requires it to call Cy_DFU_OnResetApp0() in app#0.
*
*******************************************************************************/
void Cy_OnResetUser(void)
{
    Cy_DFU_OnResetApp0();
}

/* [] END OF FILE */
