/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05 
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 ADC14 - Multiple Channel Sample without Repeat
 *
 * Description: In this code example, the feature of being able to scan multiple
 * ADC channels is demonstrated by the user a the DriverLib APIs.  Conversion
 * memory registers ADC_MEM0 - ADC_MEM7 are configured to read conversion
 * results from A0-A7 respectively. Conversion is enabled and then sampling is
 * toggled using a software toggle. Repeat mode is not enabled and sampling only
 * occurs once (and it is expected that the user pauses the debugger to observe
 * the results). Once the final sample has been taken, the interrupt for
 * ADC_MEM7 is triggered and the result is stored in the resultsBuffer buffer.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P6.1  |<--- A0 (Analog Input)
 *            |            P4.0  |<--- A1 (Analog Input)
 *            |            P4.2  |<--- A2 (Analog Input)
 *            |                  |
 *            |                  |
 *
 * Author: Timothy Logan - modified by A. Morell
 ******************************************************************************/

#include "adc14_multiple_channel_no_repeat.h"
/* DriverLib Includes */
#include "driverlib.h"
/* Standard Includes */
#include <string.h>

// Includes FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"

uint16_t resultsBuffer[NUM_ADC_CHANNELS];
extern SemaphoreHandle_t xSemaphoreISR;


void init_ADC(void){


    /* Zero-filling buffer */
    memset(resultsBuffer, 0x00, NUM_ADC_CHANNELS);

    /* Setting reference voltage to 2.5  and enabling reference */
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();

    /* Initializing ADC (MCLK/1/1) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

    /* Configuring GPIOs for Analog In */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);


    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A14, A13, A11)  with no repeat)
     * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, false); //ADC_VREFPOS_AVCC_VREFNEG_VSS -- ADC_VREFPOS_INTBUF_VREFNEG_VSS


    /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
     *  is complete and enabling conversions */
    // Corresponde a ADC_MEM2
    MAP_ADC14_enableInterrupt(ADC_INT2);

    // Configura la prioridad de la interrupcion del ADC14
    MAP_Interrupt_setPriority(INT_ADC14, 0xA0);

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    //MAP_Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
}

uint16_t *ADC_read(void){
    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
    if( xSemaphoreTake( xSemaphoreISR, portMAX_DELAY ) == pdPASS );
    return resultsBuffer;
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM2. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;
    static BaseType_t xHigherPriorityTaskWoken;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if(status & ADC_INT2)
    {
        MAP_ADC14_getMultiSequenceResult(resultsBuffer);
        // El par�metro xHigherPriorityTaskWoken debe inicializarse en pdFALSE,
        // ya que se establecer� en pdTRUE dentro de la funci�n API de interrupci�n
        // segura si se requiere un cambio de contexto
        xHigherPriorityTaskWoken = pdFALSE;
        // Entrega el semaforo desde la ISR para desbloquear la tarea SenderTask
        xSemaphoreGiveFromISR( xSemaphoreISR, &xHigherPriorityTaskWoken );

        // Pasa el valor xHigherPriorityTaskWoken en portYIELD_FROM_ISR().
        // Si xHigherPriorityTaskWoken se estableci� en pdTRUE dentro de
        // xSemaphoreGiveFromISR(), entonces al llamar a portYIELD_FROM_ISR()
        // solicitar� un cambio de contexto. Si xHigherPriorityTaskWoken sigue
        // siendo pdFALSE, entonces la llamada a portYIELD_FROM_ISR() no tendr�
        // ning�n efecto
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}


