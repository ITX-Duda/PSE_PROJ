/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Grupo: Arthur Furuta, Erik Lima e Maria Eduarda Brito
  * Projeto de PSE - Prof. João Ranhel
  *
  * notas: J Ranhel - rev 2026.07
  * PA5 =1 na entrada do callBack do RxCpltCallback(), =0 na saída
  * PB7 =1 na entrada do PeriodElapsedCallback(), =0 ma saída
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// --- veja includes, macro, e constantes no main.h ---
// #include "funcoes_SPI_display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ---- ver vários #defines (kts, delays, etc) no main.h ----
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//  --- ver macros definidas no main.h ---
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fn_checaBotao */
osThreadId_t fn_checaBotaoHandle;
const osThreadAttr_t fn_checaBotao_attributes = {
  .name = "fn_checaBotao",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for fn_mostraDispla */
osThreadId_t fn_mostraDisplaHandle;
const osThreadAttr_t fn_mostraDispla_attributes = {
  .name = "fn_mostraDispla",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for fn_UART_TX */
osThreadId_t fn_UART_TXHandle;
const osThreadAttr_t fn_UART_TX_attributes = {
  .name = "fn_UART_TX",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for fn_UART_RX */
osThreadId_t fn_UART_RXHandle;
const osThreadAttr_t fn_UART_RX_attributes = {
  .name = "fn_UART_RX",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for fila_uart_eventos */
osMessageQueueId_t fila_uart_eventosHandle;
const osMessageQueueAttr_t fila_uart_eventos_attributes = {
  .name = "fila_uart_eventos"
};
/* Definitions for fila_uart_tx */
osMessageQueueId_t fila_uart_txHandle;
const osMessageQueueAttr_t fila_uart_tx_attributes = {
  .name = "fila_uart_tx"
};
/* Definitions for estadoMutex */
osMutexId_t estadoMutexHandle;
const osMutexAttr_t estadoMutex_attributes = {
  .name = "estadoMutex"
};
/* Definitions for bufOutMutex */
osMutexId_t bufOutMutexHandle;
const osMutexAttr_t bufOutMutex_attributes = {
  .name = "bufOutMutex"
};
/* USER CODE BEGIN PV */
// variáveis que todos vamos usar: buffers de entrada/saída na comunicação
// buffers para entrada e saida de dados via USART
uint8_t BufOUT[] = {'0','0','0','0','0'};  // inicia buffer OUT com  "0"
uint8_t BufIN[]  = {'0','0','0','0','0'};  // inicia buffer IN com "0"
int8_t DspHex[]  = {0x10,0x10,0x10,0x10};  // vetor val display (se=16 => off)
size_t sizeBuffs = sizeof(BufOUT);     // tamanho dos buffers - usa geral
// qual dig liga pto? (ex: 0xA=>1010=> 1000=MSD + 0010=DG2)
uint8_t ptoDec = 0;
uint8_t oQueEnv = 0;
volatile uint8_t ping_timeout = 0;
volatile uint8_t link_ok = 0;
int timerBotao = 0;

// os vetores abaixo tem idx[0] = digito menos significativo no display
volatile int8_t estado = ST_TESTE;
// rev 2026.08: a antiga flag única 'estaServ' foi separada em duas,
// pois são conceitos independentes no enunciado (itens d, e, g, g.2):
uint8_t atendendoColega = 0;   // =1 se RECEBI rqsrv e estou servindo o colega
uint8_t a1Pressed = 0;         // =1 depois que A1 foi apertado ao menos 1x
uint8_t ncon_ping = 0;         // nCon alterna quando o link cai e ainda não voltou
int8_t modoBotao = 0;
int8_t testeDisplay[] = {8,8,8,8};
/* 0x11, 0x12 e 0x13 são os glifos minúsculos o, n e r, implementados no driver
 * de 7 segmentos. Os vetores são LSB -> MSB, como Crono e ValAdc. */
int8_t nCon[] = {0x12,0x11,0x0C,0x12};  // "nCon"
int8_t nSer[] = {0x13,0x0E,5,0x12};     // "n5Er"
int8_t Crono[] = {0,0,0,0};            // vetor com vals decimais do cronometro
int8_t ValAdc[] = {0,0,0,0};           // vetor com vals decimais do ADC
int8_t ExCrono[] = {0,0,0,0};          // vetor com vals decimais do cronometro
int8_t ExValAdc[] = {0,0,0,0};         // vetor com vals decimais do ADC
size_t sizeVals = sizeof(Crono);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void checaBotao(void *argument);
void mostraDisplay(void *argument);
void UART_TX(void *argument);
void UART_RX(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of estadoMutex */
  estadoMutexHandle = osMutexNew(&estadoMutex_attributes);

  /* creation of bufOutMutex */
  bufOutMutexHandle = osMutexNew(&bufOutMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of fila_uart_eventos */
  fila_uart_eventosHandle = osMessageQueueNew(16, sizeof(uart_frame_t), &fila_uart_eventos_attributes);
  fila_uart_txHandle = osMessageQueueNew(16, sizeof(uint8_t), &fila_uart_tx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of fn_checaBotao */
  fn_checaBotaoHandle = osThreadNew(checaBotao, NULL, &fn_checaBotao_attributes);

  /* creation of fn_mostraDispla */
  fn_mostraDisplaHandle = osThreadNew(mostraDisplay, NULL, &fn_mostraDispla_attributes);

  /* creation of fn_UART_TX */
  fn_UART_TXHandle = osThreadNew(UART_TX, NULL, &fn_UART_TX_attributes);

  /* creation of fn_UART_RX */
  fn_UART_RXHandle = osThreadNew(UART_RX, NULL, &fn_UART_RX_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /* rev 2026.08: buzzer no PB5 - inicia desligado (mesmo nível ativo-baixo
     dos LEDs B12..B15; se o buzzer soar invertido, troque SET<->RESET aqui
     e em mostraDisplay()) */
  HAL_GPIO_WritePin(BUZZER_GPIO, BUZZER_PIN, GPIO_PIN_SET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB10 PB12 PB13 PB14
                           PB15 PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// callback do ADC: lê o valor bruto e já monta os 4 dígitos do display
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t val_adc = 0;                // guarda o valor lido do ADC
  if(hadc->Instance == ADC1) {         // só entra se for o ADC1
    val_adc = HAL_ADC_GetValue(&hadc1);// pega o valor atual do conversor
    int miliVolt = val_adc*3300/4095;
    int uniADC = miliVolt/1000;
    int decADC = (miliVolt-(uniADC*1000))/100;
    int cnsADC = (miliVolt-(uniADC*1000)-(decADC*100))/10;
    int mlsADC = miliVolt-(uniADC*1000)-(decADC*100)-(cnsADC*10);
    ValAdc[3] = uniADC;                // dígito mais significativo
    ValAdc[2] = decADC;
    ValAdc[1] = cnsADC;
    ValAdc[0] = mlsADC;                // dígito menos significativo
  }
}

// ISR só joga o frame na fila e reativa o DMA pra receber o próximo pacote
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_frame_t frame;
        memcpy(frame.bytes, BufIN, sizeof(frame.bytes));
        (void)osMessageQueuePut(fila_uart_eventosHandle, &frame, 0, 0);
    }

    // recomeça a recepção pra não perder o fluxo
    HAL_UART_Receive_DMA(&huart1, BufIN, sizeBuffs);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  // essa é a task default - vai colocar algo nela? nope
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_checaBotao */
/**
* @brief Function implementing the fn_checaBotao thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_checaBotao */
void checaBotao(void *argument) {
    // guarda o estado anterior pra detectar borda de aperto com bounce
    uint8_t ultimo_a1 = 1, ultimo_a2 = 1, ultimo_a3 = 1;
    uint32_t delay_antirrebote = 0;

    for(;;) {
        uint8_t a1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
        uint8_t a2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
        uint8_t a3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

        if(delay_antirrebote == 0) {
            // A1 é local e deve funcionar mesmo sem comunicação com a outra placa.
            if(a1 == GPIO_PIN_RESET && ultimo_a1 == GPIO_PIN_SET) {
                if(osMutexAcquire(estadoMutexHandle, 100) == osOK) {
                    if(atendendoColega) {
                        // se eu tava servindo, avisa que não vou mais
                        uint8_t mensagem = sndMSGNSV;
                        (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0);
                        atendendoColega = 0;
                    }
                    a1Pressed = 1;
                    estado = ST_LOCAL_CRN;
                    osMutexRelease(estadoMutexHandle);
                }
                delay_antirrebote = DT_DEBOUNCING;
            }

            // A2 pede serviço pro colega
            if(a2 == GPIO_PIN_RESET && ultimo_a2 == GPIO_PIN_SET) {
                uint8_t mensagem = sndREQSRV;
                (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0);
                delay_antirrebote = DT_DEBOUNCING;
            }

            // A3 cancela o pedido de serviço
            if(a3 == GPIO_PIN_RESET && ultimo_a3 == GPIO_PIN_SET) {
                uint8_t mensagem = sndREQOFF;
                (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0);
                delay_antirrebote = DT_DEBOUNCING;
            }
        } else {
            delay_antirrebote--;
        }

        ultimo_a1 = a1;
        ultimo_a2 = a2;
        ultimo_a3 = a3;

        osDelay(10);
    }
}

/* USER CODE BEGIN Header_mostraDisplay */
/**
* @brief Function implementing the fn_mostraDispla thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mostraDisplay */
/* USER CODE BEGIN Header_mostraDisplay */
/**
* @brief Function implementing the fn_mostraDispla thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mostraDisplay */
void mostraDisplay(void *argument) {
    int8_t estado_local;
    int8_t ex_crono_local[4];
    int8_t ex_adc_local[4];

    uint32_t led_timer = 0;
    uint8_t led_estado = 0;

    static uint32_t timer_5er = 0;
    static uint8_t mostrando_5er = 0;

    // Novo timer global para o overlay do nCon piscando em segundo plano
    uint32_t timer_ncon_global = 0;

    for(;;) {
        if(osMutexAcquire(estadoMutexHandle, 100) == osOK) {
            estado_local = estado;
            memcpy(ex_crono_local, ExCrono, sizeof(ex_crono_local));
            memcpy(ex_adc_local, ExValAdc, sizeof(ex_adc_local));
            osMutexRelease(estadoMutexHandle);
        } else {
            estado_local = 0;
            memset(ex_crono_local, 0, sizeof(ex_crono_local));
            memset(ex_adc_local, 0, sizeof(ex_adc_local));
        }

        // Alterna o estado global do pisca (usado pelos LEDs normais e de erro)
        led_timer += DT_MUX_DISP;
        if(led_timer >= (DT_LEDS + 1)) {
            led_timer = 0;
            led_estado = !led_estado;
        }

        // --- LÓGICA DE OVERLAY DO NCON EM SEGUNDO PLANO ---
        if(!link_ok && estado_local != ST_TESTE) {
            timer_ncon_global += DT_MUX_DISP;
            if(timer_ncon_global >= 2000) { // Reseta a cada 2 segundos
                timer_ncon_global = 0;
            }
        } else {
            // Se restabeleceu o ping (link_ok == 1), zera o timer do erro
            timer_ncon_global = 0;
        }

        // Flag ativada apenas nos primeiros 500ms do ciclo de 2s
        uint8_t overlay_ncon = (!link_ok && (timer_ncon_global < 500));

        // Erro de serviço crítico: mostra 5Er e deixa o buzzer gritando (mantido inalterado)
        if(estado_local == ST_ERRO_SERVICO) {
            if(!mostrando_5er) {
                mostrando_5er = 1;
                timer_5er = 0;
            }
            timer_5er++;
            mostrar_no_display(nSer, 0);
            HAL_GPIO_WritePin(BUZZER_GPIO, BUZZER_PIN, led_estado ? GPIO_PIN_RESET : GPIO_PIN_SET);

            if(timer_5er >= (DT_BUZZER_MS / DT_MUX_DISP)) {
                mostrando_5er = 0;
                HAL_GPIO_WritePin(BUZZER_GPIO, BUZZER_PIN, GPIO_PIN_SET);
                if(osMutexAcquire(estadoMutexHandle, 100) == osOK) {
                    estado = a1Pressed ? ST_LOCAL_CRN : ST_IDLE;
                    osMutexRelease(estadoMutexHandle);
                }
            }
            osDelay(DT_MUX_DISP);
            continue;
        }

        // --- EXIBIÇÃO NO DISPLAY ---
        if (overlay_ncon) {
            // Se estiver na janela de erro de ping, ignora o estado local e mostra nCon
            mostrar_no_display(nCon, 0);

            // Pisca TODOS os LEDs independentemente do estado
            if (led_estado) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_SET);
            }
        }
        else {
            // Se o ping está OK ou estamos na janela de 1.5s normais, roda a máquina de estados
            switch(estado_local) {
                case ST_TESTE:
                    mostrar_no_display(testeDisplay, 15);
                    break;
                case ST_IDLE:
                case ST_AGUARDA_PING:
                case ST_ERRO_CONEXAO:
                    mostrar_no_display(testeDisplay, 15); // Mostra 8888
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_SET);
                    break;
                case ST_LOCAL_CRN:
                    mostrar_no_display(Crono, 10);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, led_estado ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_SET);
                    break;
                case ST_LOCAL_ADC:
                    mostrar_no_display(ValAdc, 8);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, led_estado ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_SET);
                    break;
                case ST_SERV_CRN:
                    mostrar_no_display(a1Pressed ? Crono : testeDisplay, a1Pressed ? 10 : 15);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, led_estado ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_SET);
                    break;
                case ST_SERV_ADC:
                    mostrar_no_display(a1Pressed ? ValAdc : testeDisplay, a1Pressed ? 8 : 15);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, led_estado ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_SET);
                    break;
                case ST_SERV_EXCRN:
                    mostrar_no_display(ex_crono_local, 10);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, led_estado ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_12, GPIO_PIN_SET);
                    break;
                case ST_SERV_EXADC:
                    mostrar_no_display(ex_adc_local, 8);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, led_estado ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13, GPIO_PIN_SET);
                    break;
            }
        }

        osDelay(DT_MUX_DISP);
    }
}

/* USER CODE BEGIN Header_UART_TX */
/**
* @brief Function implementing the fn_UART_TX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_TX */
void UART_TX(void *argument) {
    uint8_t mensagem;

    for(;;) {
        if(osMessageQueueGet(fila_uart_txHandle, &mensagem, NULL, osWaitForever) == osOK) {
            // espera a UART ficar livre pra não sobrescrever a tx
            while(huart1.gState != HAL_UART_STATE_READY) {
                osDelay(1);
            }
            if(osMutexAcquire(bufOutMutexHandle, osWaitForever) == osOK) {
                // monta o frame certo conforme a mensagem pedida
                if(mensagem == sndPNGOK) {
                    STR_BUFF(PNGRSP);
                } else if(mensagem == sndPING) {
                    STR_BUFF(PNGPRG);
                } else if(mensagem == sndCRN) {
                    BufOUT[0] = 'c';
                    BufOUT[1] = Crono[0] + '0';
                    BufOUT[2] = Crono[1] + '0';
                    BufOUT[3] = Crono[2] + '0';
                    BufOUT[4] = Crono[3] + '0';
                } else if(mensagem == sndADC) {
                    BufOUT[0] = 'a';
                    BufOUT[1] = ValAdc[0] + '0';
                    BufOUT[2] = ValAdc[1] + '0';
                    BufOUT[3] = ValAdc[2] + '0';
                    BufOUT[4] = ValAdc[3] + '0';
                } else if(mensagem == sndREQCRN) {
                    STR_BUFF(REQCRN);
                } else if(mensagem == sndREQADC) {
                    STR_BUFF(REQADC);
                } else if(mensagem == sndREQSRV) {
                    STR_BUFF(REQSRV);
                } else if(mensagem == sndREQOFF) {
                    STR_BUFF(REQOFF);
                } else if(mensagem == sndMSGNSV) {
                    STR_BUFF(MSGNSV);
                }
                (void)HAL_UART_Transmit_DMA(&huart1, BufOUT, sizeBuffs);
                osMutexRelease(bufOutMutexHandle);
            }
        }
    }
}
  /* USER CODE END UART_TX */

/* USER CODE BEGIN Header_UART_RX */
/**
* @brief Function implementing the fn_UART_RX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_RX */
void UART_RX(void *argument) {
    uart_frame_t frame;

    // liga a DMA da UART pra receber sem ficar lendo byte a byte
    HAL_UART_Receive_DMA(&huart1, BufIN, sizeBuffs);

    for(;;) {
        if(osMessageQueueGet(fila_uart_eventosHandle, &frame, NULL, osWaitForever) == osOK) {
            uint8_t evento = rcvNADA;
            uint8_t mensagem = sndNADA;
            if(memcmp(frame.bytes, PNGPRG, 5) == 0) {
                mensagem = sndPNGOK;
            } else if(memcmp(frame.bytes, PNGRSP, 5) == 0) {
                ping_timeout = 0;
                link_ok = 1;
                ncon_ping = 0;
                if(osMutexAcquire(estadoMutexHandle, 100) == osOK) {
                    if(estado == ST_AGUARDA_PING) {
                        estado = ST_IDLE;
                    }
                    osMutexRelease(estadoMutexHandle);
                }
            } else if(memcmp(frame.bytes, REQCRN, 5) == 0) {
                mensagem = sndCRN;
            } else if(memcmp(frame.bytes, REQADC, 5) == 0) {
                mensagem = sndADC;
            } else if(memcmp(frame.bytes, REQSRV, 5) == 0) {
                evento = rcvREQSRV;
            } else if(memcmp(frame.bytes, REQOFF, 5) == 0) {
                evento = rcvREQOFF;
            } else if(memcmp(frame.bytes, MSGNSV, 5) == 0) {
                evento = rcvMSGNSV;
            } else if(frame.bytes[0] == 'c' && frame.bytes[1] >= '0' && frame.bytes[1] <= '9'
                      && frame.bytes[2] >= '0' && frame.bytes[2] <= '9'
                      && frame.bytes[3] >= '0' && frame.bytes[3] <= '9'
                      && frame.bytes[4] >= '0' && frame.bytes[4] <= '9') {
                // CONVERSÃO DE ASCII: frame do cronômetro remoto
                if(osMutexAcquire(estadoMutexHandle, 100) == osOK) {
                    ExCrono[0] = frame.bytes[1] - '0'; ExCrono[1] = frame.bytes[2] - '0';
                    ExCrono[2] = frame.bytes[3] - '0'; ExCrono[3] = frame.bytes[4] - '0';
                    osMutexRelease(estadoMutexHandle);
                }
            } else if(frame.bytes[0] == 'a' && frame.bytes[1] >= '0' && frame.bytes[1] <= '9'
                      && frame.bytes[2] >= '0' && frame.bytes[2] <= '9'
                      && frame.bytes[3] >= '0' && frame.bytes[3] <= '9'
                      && frame.bytes[4] >= '0' && frame.bytes[4] <= '9') {
                // CONVERSÃO DE ASCII: frame do ADC remoto
                if(osMutexAcquire(estadoMutexHandle, 100) == osOK) {
                    ExValAdc[0] = frame.bytes[1] - '0'; ExValAdc[1] = frame.bytes[2] - '0';
                    ExValAdc[2] = frame.bytes[3] - '0'; ExValAdc[3] = frame.bytes[4] - '0';
                    osMutexRelease(estadoMutexHandle);
                }
            }
            if(mensagem != sndNADA) {
                (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0);
            }
            if(evento != rcvNADA) {
            if(osMutexAcquire(estadoMutexHandle, 100) == osOK) {
                switch(evento) {
                    case rcvREQSRV:
                        // colega pediu serviço; eu entro no modo de atendimento
                        atendendoColega = 1;
                        estado = ST_SERV_CRN;
                        break;
                    case rcvREQOFF:
                        // colega desistiu; volto pro meu estado local
                        if(estado >= ST_SERV_CRN && estado <= ST_SERV_EXADC) {
                            estado = a1Pressed ? ST_LOCAL_CRN : ST_IDLE;
                        }
                        atendendoColega = 0;
                        break;
                    case rcvMSGNSV:
                        // colega parou de me atender; toca erro de serviço
                        estado = ST_ERRO_SERVICO;
                        break;
                }
                osMutexRelease(estadoMutexHandle);
            }
            }
        }
    }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // contadores de tempo: cronômetro, ADC, ping e troca de slot
    static uint16_t contaModo = 0;
    static uint16_t contaCRN = 0;
    static uint16_t contaADC = 0;
    static uint16_t ping_timer = 0;
    static uint16_t contaReqRemota = 0;

    if (htim->Instance == TIM4) {
        HAL_IncTick();

        // cronômetro local vai contando em cima do timer
        if (contaCRN >= DT_CRONO) {
            contaCRN = 0;
            if(MD_CRONO == 0) {
                ++Crono[0];
                if (Crono[0] > 9) {
                    Crono[0] = 0;
                    ++Crono[1];
                    if (Crono[1] > 9) {
                        Crono[1] = 0;
                        ++Crono[2];
                        if (Crono[2] > 5) {
                            Crono[2] = 0;
                            ++Crono[3];
                            if (Crono[3] > 9) {
                                Crono[3] = 0;
                            }
                        }
                    }
                }
            }
        } else {
            ++contaCRN;
        }

        // ADC dispara uma leitura na frequência certa
        if (contaADC >= DT_ADC) {
            contaADC = 0;
            HAL_ADC_Start_IT(&hadc1);
        } else {
            ++contaADC;
        }

        // ping periódico: manda msg pra placa e, se não responder, apenas sinaliza
        // a perda da conexão. A tarefa do display alterna nCon com o modo atual.
        if(estado != ST_TESTE) {
                    if(++ping_timer >= DT_PING) {
                        ping_timer = 0;
                        uint8_t mensagem = sndPING;
                        (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0);

                        if(++ping_timeout >= 5) {
                            link_ok = 0;

                            // --- CORREÇÃO: Destravar e realinhar o DMA ---
                            // Aborta a escuta atual (limpa erros de Overrun/Framing do ruído do cabo)
                            HAL_UART_AbortReceive(&huart1);
                            // Força o DMA a escutar do zero, realinhando o índice 0 do buffer
                            HAL_UART_Receive_DMA(&huart1, BufIN, sizeBuffs);
                            // ---------------------------------------------

                            ping_timeout = 0;
                            if(estado == ST_AGUARDA_PING) {
                                estado = ST_IDLE;
                            }
                        }
                    }
                }

        // máquina de estados do display: troca o slot no tempo certo
        ++contaModo;
        switch(estado) {
            case ST_TESTE:
                if(contaModo >= DT_Inicial) {
                    contaModo = 0;
                    estado = ST_AGUARDA_PING;
                    { uint8_t mensagem = sndPING;
                      (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0); }
                }
                break;
            case ST_LOCAL_CRN:
                if(contaModo >= DT_DISPLAY_MD1) {
                    estado = ST_LOCAL_ADC;
                    contaModo = 0;
                }
                break;
            case ST_LOCAL_ADC:
                if(contaModo >= DT_DISPLAY_MD1) {
                    estado = ST_LOCAL_CRN;
                    contaModo = 0;
                }
                break;
            case ST_SERV_CRN:
                if(contaModo >= DT_DISPLAY_MD2) {
                    estado = ST_SERV_ADC;
                    contaModo = 0;
                }
                break;
            case ST_SERV_ADC:
                if(contaModo >= DT_DISPLAY_MD2) {
                    estado = ST_SERV_EXCRN;
                    contaModo = 0;
                    contaReqRemota = 0;
                    { uint8_t mensagem = sndREQCRN;
                      (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0); }
                }
                break;
            case ST_SERV_EXCRN:
                // atualiza o cronômetro remoto enquanto esse slot tá ativo
                if(++contaReqRemota >= DT_NEWREQ) {
                    uint8_t mensagem = sndREQCRN;
                    contaReqRemota = 0;
                    (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0);
                }
                if(contaModo >= DT_DISPLAY_MD2) {
                    estado = ST_SERV_EXADC;
                    contaModo = 0;
                    contaReqRemota = 0;
                    { uint8_t mensagem = sndREQADC;
                      (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0); }
                }
                break;
            case ST_SERV_EXADC:
                // atualiza o ADC remoto enquanto esse slot tá ativo
                if(++contaReqRemota >= DT_NEWREQ) {
                    uint8_t mensagem = sndREQADC;
                    contaReqRemota = 0;
                    (void)osMessageQueuePut(fila_uart_txHandle, &mensagem, 0, 0);
                }
                if(contaModo >= DT_DISPLAY_MD2) {
                    estado = ST_SERV_CRN;
                    contaModo = 0;
                    contaReqRemota = 0;
                }
                break;
            case ST_IDLE:
            case ST_AGUARDA_PING:
            case ST_ERRO_CONEXAO:
            case ST_ERRO_SERVICO:
                contaReqRemota = 0;
                // nesses estados o evento decide o que vai acontecer
                break;
        }
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
