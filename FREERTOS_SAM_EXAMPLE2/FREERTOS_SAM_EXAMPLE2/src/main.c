/** 
* \file 
* 
* \brief FreeRTOS Real Time Kernel example. 
* 
* Copyright (c) 2012-2016 Atmel Corporation. All rights reserved. 
* 
* \asf_license_start 
* 
* \page License 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, 
*    this list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
*    this list of conditions and the following disclaimer in the documentation 
*    and/or other materials provided with the distribution. 
* 
* 3. The name of Atmel may not be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* 
* 4. This software may only be redistributed and used in connection with an 
*    Atmel microcontroller product. 
* 
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED 
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE 
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR 
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
* 
* \asf_license_stop 
* 
*/ 
  
/** 
* \mainpage FreeRTOS Real Time Kernel example 
* 
* \section Purpose 
* 
* The FreeRTOS example will help users how to use FreeRTOS in SAM boards. 
* This basic application shows how to create task and get information of 
* created task. 
* 
* \section Requirements 
* 
* This package can be used with SAM boards. 
* 
* \section Description 
* 
* The demonstration program create two task, one is make LED on the board 
* blink at a fixed rate, and another is monitor status of task. 
* 
* \section Usage 
* 
* -# Build the program and download it inside the evaluation board. Please 
*    refer to the 
*    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf"> 
*    SAM-BA User Guide</a>, the 
*    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf"> 
*    GNU-Based Software Development</a> 
*    application note or to the 
*    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf"> 
*    IAR EWARM User Guide</a>, 
*    depending on your chosen solution. 
* -# On the computer, open and configure a terminal application 
*    (e.g. HyperTerminal on Microsoft Windows) with these settings: 
*   - 115200 bauds 
*   - 8 bits of data 
*   - No parity 
*   - 1 stop bit 
*   - No flow control 
* -# Start the application. 
* -# LED should start blinking on the board. In the terminal window, the 
*    following text should appear (values depend on the board and chip used): 
*    \code 
    -- Freertos Example xxx -- 
    -- xxxxxx-xx 
    -- Compiled: xxx xx xxxx xx:xx:xx -- 
\endcode 
* 
*/ 
  
#include <asf.h> 
#include "stdio_serial.h" 
#include "conf_board.h" 
#include "conf_clock.h" 
  
#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE)) 
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY) 
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE)) 
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY) 
#define TASK_SENSOR_STACK_SIZE             (1024/sizeof(portSTACK_TYPE)) 
#define TASK_SENSOR_STACK_PRIORITY         (tskIDLE_PRIORITY +3) 
#define TASK_CONTROLADOR_STACK_SIZE        (2048/sizeof(portSTACK_TYPE)) 
#define TASK_CONTROLADOR_STACK_PRIORITY    (tskIDLE_PRIORITY +2) 
#define TASK_ATUADOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE)) 
#define TASK_ATUADOR_STACK_PRIORITY        (tskIDLE_PRIORITY +2) 
  
/** PWM frequency in Hz */ 
#define PWM_FREQUENCY      1000 
/** Period value of PWM output waveform */ 
#define PERIOD_VALUE       100 
/** Initial duty cycle value */ 
#define INIT_DUTY_VALUE    0 
  
/** PWM channel instance for LEDs */ 
pwm_channel_t g_pwm_channel_led; 
  
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, 
        signed char *pcTaskName); 
extern void vApplicationIdleHook(void); 
extern void vApplicationTickHook(void); 
extern void vApplicationMallocFailedHook(void); 
extern void xPortSysTickHandler(void); 
  
#if !(SAMV71 || SAME70) 
/** 
* \brief Handler for System Tick interrupt. 
*/ 
void SysTick_Handler(void) 
{ 
    xPortSysTickHandler(); 
} 
#endif 
  
/** 
* \brief Called if stack overflow during execution 
*/ 
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, 
        signed char *pcTaskName) 
{ 
    printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName); 
    /* If the parameters have been corrupted then inspect pxCurrentTCB to 
     * identify which task has overflowed its stack. 
     */ 
    for (;;) { 
    } 
} 
  
xQueueHandle xQueue1; 
//Cria handler da fila 2; 
xQueueHandle xQueue2; 
  
/** 
* \brief This function is called by FreeRTOS idle task 
*/ 
extern void vApplicationIdleHook(void) 
{ 
} 
  
/** 
* \brief This function is called by FreeRTOS each tick 
*/ 
extern void vApplicationTickHook(void) 
{ 
} 
  
extern void vApplicationMallocFailedHook(void) 
{ 
    /* Called if a call to pvPortMalloc() fails because there is insufficient 
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called 
    internally by FreeRTOS API functions that create tasks, queues, software 
    timers, and semaphores.  The size of the FreeRTOS heap is set by the 
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */ 
  
    /* Force an assert. */ 
    configASSERT( ( volatile void * ) NULL ); 
} 
  
/** 
* \brief This task, when activated, send every ten seconds on debug UART 
* the whole report of free heap and total tasks status 
*/ 
static void task_monitor(void *pvParameters) 
{ 
    static portCHAR szList[256]; 
    UNUSED(pvParameters); 
  
    for (;;) { 
        printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks()); 
        vTaskList((signed portCHAR *)szList); 
        printf(szList); 
        vTaskDelay(10000); 
    } 
} 
  
/** 
    ACIONA O SENSOR E LE OS SEUS DADOS  
*/ 
static void Dados_Sensor(void *pvParameters) 
{ 
    //static portCHAR szList[256]; 
	
    unsigned long dist= 10.70; 
    portBASE_TYPE xStatus; 
	 
  
    for (;;) { 
        /** MANDA ALTO PARA O PINO DE TRIGGER PRO SENSOR*/ 
  
        /** ARMAZENA O VALOR DO PINO ECHO DO SENSOR EM pvParameters E MANDA BAIXO PARA O TRIGGER*/ 
  
        /** MANDA ALTO PARA O PINO DE TRIGGER PRO SENSOR*/ 
        xStatus = xQueueSendToBack(xQueue1, &dist, 10 ); 
		
		printf("Esta no sensor\r\n"); 
  
        /** CHECA SE DEU ERRADO*/ 
        if( xStatus != pdPASS){ 
            printf("Não conseguiu mandar o elemento para fila\r\n"); 
        } 
		else{
			//printf("Enviou elemento pra fila = %u\r\n", (unsigned long) dist); 
		}
		
		/*if( uxQueueMessagesWaiting( xQueue1 ) != 0){
			printf("Fila deveria estar vazia \r \n");
		}*/
		
		printf("Elementos na fila: %u\r\n", (unsigned int) uxQueueMessagesWaiting( xQueue1 ));
		
        /** delay de 200 ms*/ 
        vTaskDelay (200); 
  
  
    } 
} 
  
/** 
        MALHA DE CONTROLE - CALCULA OS VALORES PARA ATUAÇÃO 
*/ 
static void Controlador(void *pvParameters) 
{ 
    static portCHAR szList[256]; 
    unsigned long Resultado, ValorLido; //Resultado é o duty-cicle 
     
    const Tempo_espera = 250 / portTICK_RATE_MS;  
  
    portBASE_TYPE xStatus; 
	
	
     
    for (;;) { 
		
			
			
		printf("Esta no controlador\r\n");
		
		xStatus = xQueueReceive (xQueue1 , &ValorLido, Tempo_espera);
		
        /** CHECA FILA */
        //printf("Elementos na fila: %u\r\n", (unsigned int) uxQueueMessagesWaiting( xQueue1 ));
         
        /** ARMAZENA VALOR DA FILA EM ValorLido E PRINTA ELE NA TELA*/ 
         
  
        if(xStatus == pdPASS){ 
           // printf("Número recebido = %u\r\n", (unsigned long) ValorLido); 
        } 
        else{ 
            //printf("Falhou\n"); 
        } 
         
        Resultado = ValorLido * 8; 
         
        /* MANDA RESULTADO PRA FILA 2 */
        xStatus = xQueueSend(xQueue2, &Resultado, 0); 
  
        /** CHECA SE DEU ERRADO*/
        if( xStatus != pdPASS){
	        printf("Não conseguiu mandar o elemento para fila\r\n");
        }
        else{
	       // printf("Enviou elemento pra fila = %u\r\n", (unsigned long) Resultado);
        } 
  
        /** Faz o cáculo com o pvParameters e armazena em resultado*/
  
		
    } 
} 
  
static void Atuador(void *pvParameters) 
{ 
    static portCHAR szList[256]; 
    unsigned long DutyCicle; 
     
    const Tempo_espera = 250 / portTICK_RATE_MS; 
  
    portBASE_TYPE xStatus; 
	
     
    for (;;) { 
		
		printf("Esta no atuador\r\n"); 
        /** CHECA FILA*/ 
        /*if( uxQueueMessagesWaiting( xQueue2 ) != 0){ 
            printf("Fila deveria estar vazia \r \n"); 
        } */
         
        /** ARMAZENA VALOR DA FILA EM ValorLido E PRINTA ELE NA TELA*/ 
        xStatus = xQueueReceive (xQueue2 , &DutyCicle, Tempo_espera); 
  
        /** CHECA SE DEU ERRADO*/
        if( xStatus != pdPASS){
	        printf("Não conseguiu receber o elemento da fila\r\n");
        }
        else{
	        printf("Recebeu elemento da fila = %u\r\n", (unsigned long) DutyCicle);
        }
         
        g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL; 
        pwm_channel_update_duty(PWM, &g_pwm_channel_led, DutyCicle); 
        g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL; 
        pwm_channel_update_duty(PWM, &g_pwm_channel_led, DutyCicle); 
  
  
    } 
} 
  
  
  
/** 
* \brief This task, when activated, make LED blink at a fixed rate 
*/ 
static void task_led(void *pvParameters) 
{ 
    UNUSED(pvParameters); 
    for (;;) { 
    /*#if SAM4CM 
        LED_Toggle(LED4); 
    #else 
        LED_Toggle(LED0); 
    #endif */
        vTaskDelay(1000); 
    } 
} 
  
/** 
* \brief Configure the console UART. 
*/ 
static void configure_console(void) 
{ 
    const usart_serial_options_t uart_serial_options = { 
        .baudrate = CONF_UART_BAUDRATE, 
#if (defined CONF_UART_CHAR_LENGTH) 
        .charlength = CONF_UART_CHAR_LENGTH, 
#endif 
        .paritytype = CONF_UART_PARITY, 
#if (defined CONF_UART_STOP_BITS) 
        .stopbits = CONF_UART_STOP_BITS, 
#endif 
    }; 
  
    /* Configure console UART. */ 
    stdio_serial_init(CONF_UART, &uart_serial_options); 
  
    /* Specify that stdout should not be buffered. */ 
#if defined(__GNUC__) 
    setbuf(stdout, NULL); 
#else 
    /* Already the case in IAR's Normal DLIB default configuration: printf() 
     * emits one character at a time. 
     */ 
#endif 
} 
  
/** 
*  \brief FreeRTOS Real Time Kernel example entry point. 
* 
*  \return Unused (ANSI-C compatibility). 
*/ 
int main(void) 
{ 
    /* Initialize the SAM system */ 
    sysclk_init(); 
    board_init(); 
  
    /* Initialize the console uart */ 
    configure_console(); 
     
    /* Configura pinos de LEDs para PWM */ 
    pwm_channel_disable(PWM, PIN_PWM_LED0_CHANNEL); 
    pwm_channel_disable(PWM, PIN_PWM_LED1_CHANNEL); 
     
    /* Inicializa canais de PWM dos LEDs */ 
     
        /* LED0 */ 
        /* Period is left-aligned */ 
        g_pwm_channel_led.alignment = PWM_ALIGN_LEFT; 
        /* Output waveform starts at a low level */ 
        g_pwm_channel_led.polarity = PWM_LOW; 
        /* Use PWM clock A as source clock */ 
        g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA; 
        /* Period value of output waveform */ 
        g_pwm_channel_led.ul_period = PERIOD_VALUE; 
        /* Duty cycle value of output waveform */ 
        g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE; 
        g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL; 
        pwm_channel_init(PWM, &g_pwm_channel_led); 
  
        /* LED1 */ 
        /* Period is center-aligned */ 
        g_pwm_channel_led.alignment = PWM_ALIGN_CENTER; 
        /* Output waveform starts at a high level */ 
        g_pwm_channel_led.polarity = PWM_HIGH; 
        /* Use PWM clock A as source clock */ 
        g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA; 
        /* Period value of output waveform */ 
        g_pwm_channel_led.ul_period = PERIOD_VALUE; 
        /* Duty cycle value of output waveform */ 
        g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE; 
        g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL; 
        pwm_channel_init(PWM, &g_pwm_channel_led); 
  
    /* CRIA A FILA COM 5 ELEMENTOS DO TIPO FLOAT */ 
    xQueue1 = xQueueCreate (5, sizeof(long) );
    //xQueue2 = xQueueCreate (5, sizeof(float) ); 
	xQueue2 = xQueueCreate (5, sizeof(long) );
  
    /* Output demo information. */ 
    printf("-- Freertos Example --\n\r"); 
    printf("-- %s\n\r", BOARD_NAME); 
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__); 
  
  
    /* Create task to monitor processor activity */ 
    if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL, 
            TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) { 
        printf("Failed to create Monitor task\r\n"); 
    } 
  
    // Create task to make led blink 
    if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL, 
            TASK_LED_STACK_PRIORITY, NULL) != pdPASS) { 
        printf("Failed to create test led task\r\n"); 
    } 
  
    /* CRIA A TASK QUE ATIVA O SENSOR */ 
    if (xTaskCreate(Dados_Sensor, "Sensor", TASK_SENSOR_STACK_SIZE, NULL, 
    TASK_SENSOR_STACK_PRIORITY, NULL) != pdPASS) { 
        printf("Failed to create test led task\r\n"); 
    } 
  
    /* CRIA A TASK DO CONTROLADOR */ 
    if (xTaskCreate(Controlador, "Controlador", TASK_CONTROLADOR_STACK_SIZE, NULL, 
    TASK_CONTROLADOR_STACK_PRIORITY, NULL) != pdPASS) { 
        printf("Failed to create test led task\r\n"); 
    } 
     
    if (xTaskCreate(Atuador, "Atuador", TASK_ATUADOR_STACK_SIZE, NULL, 
    TASK_ATUADOR_STACK_PRIORITY, NULL) != pdPASS) { 
        printf("Failed to create test led task\r\n"); 
    } 
  
    /* Start the scheduler. */ 
    vTaskStartScheduler(); 
  
    /* Will only get here if there was insufficient memory to create the idle task. */ 
    return 0; 
} 