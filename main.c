//*****************************************************************************
//
// Copyright (C) 2015 - 2016 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the
//  distribution.
//
//  Neither the name of Texas Instruments Incorporated nor the names of
//  its contributors may be used to endorse or promote products derived
//  from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

/* Includes standard */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* Includes FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Includes drivers */
#include "i2c_driver.h"
#include "temp_sensor.h"
#include "opt3001.h"
#include "tmp006.h"
#include "accelerometer_driver.h"
#include "adc14_multiple_channel_no_repeat.h"

/* Includes display y graficas */
#include <driverlib.h>
#include <grlib.h>
#include "Crystalfontz128x128_ST7735.h"

/* Definicion de prioridades para las tareas */
#define SENSOR_TASK_PRIORITY        2
#define ACCEL_TASK_PRIORITY         2
#define READER_TASK_PRIORITY        1
#define HEARTBEAT_TASK_PRIORITY     1

/* Definicion de constantes de tiempo */
#define ONE_SECOND_MS               1000
#define HEART_BEAT_ON_MS            10
#define HEART_BEAT_OFF_MS           990
/* Definicion de constantes de tiempo para las tareas */
#define ACCEL_PERIOD_MS             100
#define LIGHT_PERIOD_MS             500
#define RECEIVER_PERIOD_MS          10

/* Numero de muestras a medir de las colas */
#define SENSOR_QUEUE_SIZE           2   // Representa luz y temperatura
#define ACCEL_QUEUE_SIZE            10  // Represinta acelerometro

/* Prototipos de funciones privadas */
static void prvSetupHardware(void);
static void prvSensorWriterTask(void *pvParameters);
static void prvAccelWriterTask(void *pvParameters);
static void prvReaderTask(void *pvParameters);
static void prvHeartBeatTask(void *pvParameters);
/* Funciones para dibujar en el display */
void drawTitle(void);
void drawAccelData(void);
void drawLightData(void);
void drawTempData(void);

/* Tiempo de generacion las tareas */
static TickType_t accel_delay;
static TickType_t light_delay;
static TickType_t receiver_delay;

/* Declaracion de un mutex para acceso unico a temperaturea global */
SemaphoreHandle_t xMutexTemp;
/* Declaracion de un semaforo binario para la ISR/lectura de ADC */
SemaphoreHandle_t xSemaphoreISR;

/* Declaracion de cola para la temperatura/luz y acelerometro */
QueueHandle_t xSensorQueue; // Contiene info de luz y temperatura
QueueHandle_t xAccelQueue;  // Contiene info del acelerometro

/* Declaracion de estructuras */
// Sensor, l-luz, t-temperatura
typedef enum Sensor{
    l = 1,
    t = 2
} Sensor;

// Queue element
typedef struct{
    Sensor sensor;
    float value;
} Sensor_t;

// Acelerometro
typedef struct{
    float acc_x;
    float acc_y;
    float acc_z;
} Acc_t;

// Display
typedef struct{
    float light;
    float temp;
    float diffTemp;
    Acc_t acc;
} Avg_t;
Avg_t avg; // registro display

// Registro sensor, luz y temperatura
static Sensor_t xStructuresToSend[2] =
{
 {l, 0.00},     // Used by light sensor
 {t, 0.00}      // Used by temperature sensor
};

/* Variables para gestion de luz */
static float light = 0;
static uint8_t light_counter = 0;

/* Variables para gestion de temperatura */
static float temp = 0;
static float diff_avg_temp = 0;
static float last_avg_temp = 0;
static uint8_t diff_temp_counter = 0;
static uint16_t temp_counter = 0;
static uint8_t write_temp_counter = 0;
static float temperature = 20.0;
static uint8_t first_diff_temp_counter = 2;
static uint16_t change_temp_counter = 0;

/* Variables para gestion de acelerometro */
static float avg_accX = 0, avg_accY = 0, avg_accZ = 0;
static uint8_t accel_counter = 0;
static uint16_t accel_temp_counter = 0;

/* Graphic library context */
Graphics_Context g_sContext;

int main(void)
{
    // Inicializacion de semaforo binario
    xSemaphoreISR = xSemaphoreCreateBinary();
    // Inicializacio de mutexs
    xMutexTemp= xSemaphoreCreateMutex();

    // Inicializacion de la cola
    xSensorQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(Sensor_t));
    xAccelQueue = xQueueCreate(ACCEL_QUEUE_SIZE, sizeof(Acc_t));

    light_delay = pdMS_TO_TICKS(LIGHT_PERIOD_MS);
    accel_delay = pdMS_TO_TICKS(ACCEL_PERIOD_MS);
    receiver_delay = pdMS_TO_TICKS(RECEIVER_PERIOD_MS);

    // Comprueba si semaforo y mutex se han creado bien
    if ((xSemaphoreISR != NULL) && (xMutexTemp != NULL) && (xSensorQueue != NULL) && (xAccelQueue != NULL)){
        // Inicializacion del hardware (clocks, GPIOs, IRQs)
        prvSetupHardware();

        // Creacion de tareas
        xTaskCreate(prvSensorWriterTask, "LightWriter", configMINIMAL_STACK_SIZE, (void *) (&(xStructuresToSend[0])), SENSOR_TASK_PRIORITY, NULL);
        xTaskCreate(prvSensorWriterTask, "TempWriter", configMINIMAL_STACK_SIZE, (void *) (&(xStructuresToSend[1])), SENSOR_TASK_PRIORITY, NULL);
        xTaskCreate(prvAccelWriterTask, "AccelWriterTask", configMINIMAL_STACK_SIZE, NULL, ACCEL_TASK_PRIORITY, NULL);
        xTaskCreate(prvReaderTask, "ReaderTask", configMINIMAL_STACK_SIZE, NULL, READER_TASK_PRIORITY, NULL);
        xTaskCreate(prvHeartBeatTask, "HeartBeatTask", configMINIMAL_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, NULL);

        vTaskStartScheduler();
    }
    // Solo llega aqui si no hay suficiente memoria
    // para iniciar el scheduler
    return 0;
}

// Inicializacion del hardware del sistema
static void prvSetupHardware(void)
{
    extern void FPU_enableModule(void);

    // Configuracion del pin P1.0 - LED 1 - como salida y puesta en OFF
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    // Inicializacion de pins sobrantes para reducir consumo
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, PIN_ALL8);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PB, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PC, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PD, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PE, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PB, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PC, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PD, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PE, PIN_ALL16);

    // Habilita la FPU
    MAP_FPU_enableModule();
    // Cambia el numero de "wait states" del controlador de Flash
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    // Selecciona la frecuencia central de un rango de frecuencias del DCO
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_6);
    // Configura la frecuencia del DCO
    CS_setDCOFrequency(CS_8MHZ);

    // Inicializa los clocks HSMCLK, SMCLK, MCLK y ACLK
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Selecciona el nivel de tension del core
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE0);

    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_PURPLE);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    drawTitle();

    // Inicializacion del I2C
    I2C_init();
    // Inicializacion del sensor TMP006
    TMP006_init();
    // Inicializacion del sensor opt3001
    sensorOpt3001Init();
    sensorOpt3001Enable(true);

    // Inicializa acelerometro+ADC
    init_Accel();

    // Habilita que el procesador responda a las interrupciones
    MAP_Interrupt_enableMaster();
}

/* Tarea heart beat */
static void prvHeartBeatTask (void *pvParameters)
{
    for(;;){
        // Enciende led
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        vTaskDelay(pdMS_TO_TICKS(HEART_BEAT_ON_MS));
        // Apaga led
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        vTaskDelay(pdMS_TO_TICKS(HEART_BEAT_OFF_MS));
    }
}

/* Tarea lectura de sensores luz y temperatura */
static void prvSensorWriterTask(void *pvParameters)
{
    BaseType_t xStatus;
    uint16_t rawData;
    float convertedLight, read_temp;

    for(;;){
        // Lee el valor de lectura de la luz
        sensorOpt3001Read(&rawData);
        sensorOpt3001Convert(rawData, &convertedLight);
        // Guardamos el valor en la estructura de la luz
        xStructuresToSend[0].value = convertedLight;

        // Esperamos un segundo para leer la temperatura
        write_temp_counter++;
        if(write_temp_counter >= 2){
            write_temp_counter = 0;

            // Lee el valor de lectura de la temperatura
            read_temp = TMP006_readAmbientTemperature();
            // Guardamos el valor en la estructura de la temperatura
            xStructuresToSend[1].value = read_temp;
        }

        // Enviamos los datos leidos a la cola de sensores
        xStatus = xQueueSendToBack(xSensorQueue, pvParameters, portMAX_DELAY);

        // Si no se ha podido enviar los valores, escribimos error en la pantalla
        if(xStatus != pdPASS){
            Graphics_drawStringCentered(&g_sContext, "Sensor queue error", AUTO_STRING_LENGTH, 64, 5, OPAQUE_TEXT);
        }

        vTaskDelay(light_delay);
    }
}

/* Tarea lectura de acelerometr */
static void prvAccelWriterTask(void *pvParameter)
{
    Acc_t accel_info;
    float Data[NUM_ADC_CHANNELS];
    float temp_offset;

    xSemaphoreTake(xSemaphoreISR, 0);

    // La tarea se repite en un bucle infinito
    for(;;) {
        Accel_read(Data); // Leemos los valores de X Y Z

        // Calculamos el offset de temperatura para X e Y
        temp_offset = (float)(1 + 0.0001*temperature);
        accel_info.acc_x = (float)(Data[0]/temp_offset);
        accel_info.acc_y = (float)(Data[1]/temp_offset);
        // Calculamos el offset de temperatura para Z
        temp_offset = (float)(1 + 0.0004*temperature);
        accel_info.acc_z = (float)(Data[2]/temp_offset);

        // Enviamos informacion a la cola del acelerometro
        xQueueSend(xAccelQueue, &accel_info, portMAX_DELAY);

        // Esperamos 30 segundos para poder actualizar la temperatura de offset
        accel_temp_counter++;
        if(accel_temp_counter == 30){ // cada 30s; 100*30 = 3000ms
            // Si la tarea tiene acceso al mutex de actualizacion de temperatura
            if(xSemaphoreTake(xMutexTemp, portMAX_DELAY) == pdTRUE){
                temperature = avg.temp;
                xSemaphoreGive(xMutexTemp);
                accel_temp_counter = 0;
            }
            // si no hemos podido coger el mutex, probamos en el segundo
            // ciclo de lectura de acelerometro (100ms)
            else{
                accel_temp_counter = 299;
            }
        }

        vTaskDelay(accel_delay);
    }
}

/* Tarea lectura de cola */
static void prvReaderTask (void *pvParameters)
{
    Sensor_t xSensorStructure;
    Acc_t xAccelStructure;
    float last_light = 0, last_temp = 0;

    for(;;){
        // Leemos valores sensores luz y temperatura
        xQueueReceive(xSensorQueue, &xSensorStructure, 0);
        // Si en la cola tenemos el elemento de la luz, guardamos valor
        if(xSensorStructure.sensor == l){
            light = xSensorStructure.value;
        }
        // Si en la cola tenemos el elemento de la temperatura, guardamos valor
        if(xSensorStructure.sensor == t){
            temp = xSensorStructure.value;
        }

        // Incrementamos contador de luz para llegar a 500ms
        light_counter++;
        if(light_counter >= 5){
            // Calculamos medida luz entre la ultima lectura y la anterior
            avg.light = (float)((light + last_light)/2);

            // Inicializamos contador
            light_counter = 0;
            // Escribimos por pantalla la medida de luz
            drawLightData();
        }
        // Guardamos el ultimo valor de la luz para hacer la medida en el tiempo siguiente (500ms)
        last_light = light;

        // Incrementamos contador de temperatura para llegar a 60s
        temp_counter++;
        if(temp_counter >= 600){
            // Calculamos medida temperatura entre la ultima lectura y la anterior
            avg.temp = (float)((temp + last_temp)/2);

            // Contador para el calculo de la diferencia entre temperaturas
            // Se espera dos tiempos de 60sla primera vez, despues hace la diferencia respeto a la anterior
            diff_temp_counter++;
            if(diff_temp_counter == first_diff_temp_counter){
                // Calculo de la diferencia
                diff_avg_temp = (float)(avg.temp - last_avg_temp);
                // Guardamos diferencia al registro de deisplay
                avg.diffTemp = diff_avg_temp;

                // Inicializamos datos de contador
                diff_temp_counter = 0;
                first_diff_temp_counter = 1;
            }

            /*if(xSemaphoreTake(xMutexTemp, portMAX_DELAY) == pdTRUE){
                temperature = avg.temp;
                xSemaphoreGive(xMutexTemp);
            }*/

            // Guardamos el ultimo valor de la medida de temperatura
            last_avg_temp = avg.temp;
            // Inicializamos contador
            temp_counter = 0;
            // Escribimos por pantalla la medida de temperatura
            drawTempData();
        }
        // Guardamos el ultimo valor de la luz para hacer la medida en el tiempo siguiente (60s)
        last_temp = temp;

        // Incrementamos contador para cambiar la temperatura global
        change_temp_counter++;
        if(change_temp_counter >= 600){
            change_temp_counter = 0;

            // Esperamos mutex
            if(xSemaphoreTake(xMutexTemp, portMAX_DELAY) == pdTRUE){
                temperature = avg.temp;
                xSemaphoreGive(xMutexTemp);
            }
            // Si no accedemos al mutex, intentamos en la vuleta siguiente
            else{
                change_temp_counter = 599;
            }
        }

        // enviamos los datos del acelerometro a pantalla
        if(accel_counter == 1){ // si han pasado 100ms
            // Calculamos las medidas de los tres ejes
            avg_accX = (float)(avg_accX/accel_counter);
            avg_accY = (float)(avg_accY/accel_counter);
            avg_accZ = (float)(avg_accZ/accel_counter);

            // Actualizamos el registro del display
            avg.acc.acc_x = avg_accX;
            avg.acc.acc_y = avg_accY;
            avg.acc.acc_z = avg_accZ;

            // Escribimos por pantalla
            drawAccelData();

            // Inicializamos los valores
            avg_accX = 0;
            avg_accY = 0;
            avg_accZ = 0;
            accel_counter = 0;
        }
        else{
            // Lectura de la cola
            xQueueReceive(xAccelQueue, &xAccelStructure, portMAX_DELAY);

            // Guardamos valores de los ejes
            avg_accX += xAccelStructure.acc_x;
            avg_accY += xAccelStructure.acc_y;
            avg_accZ += xAccelStructure.acc_z;

            // Incrementamos contador
            accel_counter++;
        }

        vTaskDelay(receiver_delay);
    }
}

/* Borramos la pantalla y añadimos los valores */
void drawTitle()
{
    // Inicializacion de la pantalla
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawStringCentered(&g_sContext, "Hello Emil!!! :)", AUTO_STRING_LENGTH, 64, 5, OPAQUE_TEXT);

    // Escribimos los datos de inicializacion de pantalla
    drawAccelData();    // acelerometro
    drawLightData();    // luz
    drawTempData();     // temperatura

}

/* Escribimos los valores de media de lux, temperatura (actual y diferencia con el anterior) y aceleracion */
void drawAccelData()
{
    char string[20];

    Graphics_drawStringCentered(&g_sContext, "Accelerometer:", AUTO_STRING_LENGTH, 64, 80, OPAQUE_TEXT);

    // Escribimos el valor de X del acelerometro
    sprintf(string, "X: %6.3f", avg.acc.acc_x);
    Graphics_drawStringCentered(&g_sContext, (int8_t *)string, 12, 64, 90, OPAQUE_TEXT);

    // Escribimos el valor de Y del acelerometro
    sprintf(string, "Y: %6.3f", avg.acc.acc_y);
    Graphics_drawStringCentered(&g_sContext, (int8_t *)string, 12, 64, 100, OPAQUE_TEXT);

    // Escribimos el valor de Z del acelerometro
    sprintf(string, "Z: %6.3f", avg.acc.acc_z);
    Graphics_drawStringCentered(&g_sContext, (int8_t *)string, 12, 64, 110, OPAQUE_TEXT);
}

void drawLightData()
{
    char string[20];

    // Escribimos el valor de la luz
    sprintf(string, "Light: %7.2f", avg.light);
    Graphics_drawStringCentered(&g_sContext, (int8_t *)string, 14, 64, 30, OPAQUE_TEXT);
}

void drawTempData()
{
    char string[20];

    // Escribimos el valor de la temperatura
    sprintf(string, "Temp: %6.2f", avg.temp);
    Graphics_drawStringCentered(&g_sContext, (int8_t *)string, 14, 64, 50, OPAQUE_TEXT);

    // Escribimos el valor de la diferencia entre temperaturas
    sprintf(string, "Tdif: %5.2f", avg.diffTemp);
    Graphics_drawStringCentered(&g_sContext, (int8_t *)string, 12, 64, 60, OPAQUE_TEXT);
}
