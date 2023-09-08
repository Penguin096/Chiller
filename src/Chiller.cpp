#include <Wire.h>
#include "LiquidCrystal_I2C.h"
#include "EncButton2.h"
#include "microDS18B20.h"
#include "GyverStepper.h"
#include "GyverPID.h"

#ifdef __AVR_ATmega328PB__

#include <Arduino.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#define ADC_REF 1.094
#define Button 2
#define Compressor 5
#define FAN 3
#define Valve_1_Hot A1
#define Valve_2_Cold 4
#define WL 10
#define FS 9
#define RS485_REDE 13
#define PUMP 6
// пины для подключения контактов STEP, DIR
#define PIN_STEP 7
#define PIN_DIR 8
// #define CansiderTemp A2
// #define FanTemp A3
// #define Pressure A0
#define DS_PIN A3             // пин для термометров
#define USART_BAUDRATE 250000 // Desired Baud Rate
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define ASYNCHRONOUS (0 << UMSEL00) // USART Mode Selection
#define DISABLED (0 << UPM00)
#define EVEN_PARITY (2 << UPM00)
#define ODD_PARITY (3 << UPM00)
#define PARITY_MODE DISABLED // USART Parity Bit Selection
#define ONE_BIT (0 << USBS0)
#define TWO_BIT (1 << USBS0)
#define STOP_BIT ONE_BIT // USART Stop Bit Selection
#define FIVE_BIT (0 << UCSZ00)
#define SIX_BIT (1 << UCSZ00)
#define SEVEN_BIT (2 << UCSZ00)
#define EIGHT_BIT (3 << UCSZ00)
#define DATA_BIT EIGHT_BIT // USART Data Bit Selection
#define RX_COMPLETE_INTERRUPT (1 << RXCIE0)

#endif
#ifdef STM32F10X_MD

#include <Arduino.h>

#include "../CubeMX/Inc/main.h"
#include "../CubeMX/Inc/usb_device.h"
// #include "stm32f1xx_hal.h"
// #include "stm32f1xx_hal_adc.h"
// #include "stm32f1xx_hal_iwdg.h"

#define ADC_REF 1.208

#define Button PC13
#define Compressor PB3
#define FAN PB6
#define Valve_1_Hot PB4
#define Valve_2_Cold PB5
#define WL PB13
#define FS PB12
#define RS485_REDE PA15
#define PUMP PB7
#define PIN_STEP PB10
#define PIN_DIR PB11
#define DS_PIN PA2 // пин для термометров

// #define RELAY_Pin GPIO_PIN_3
// #define RELAY_GPIO_Port GPIOB
// #define FAN_Pin GPIO_PIN_6
// #define FAN_GPIO_Port GPIOB
// #define VALVE_1_Pin GPIO_PIN_4
// #define VALVE_1_GPIO_Port GPIOB
// #define VALVE_2_Pin GPIO_PIN_5
// #define VALVE_2_GPIO_Port GPIOB
// #define RS_DIR_Pin GPIO_PIN_15
// #define RS_DIR_GPIO_Port GPIOA
// #define PUMP_Pin GPIO_PIN_7
// #define PUMP_GPIO_Port GPIOB
// // пины для подключения контактов STEP, DIR
// #define STEP_Pin GPIO_PIN_10
// #define STEP_GPIO_Port GPIOB
// #define DIR_Pin GPIO_PIN_11
// #define DIR_GPIO_Port GPIOB
// #define EN_4988_Pin GPIO_PIN_1
// #define EN_4988_GPIO_Port GPIOB

// #define BUTTON_Pin GPIO_PIN_13
// #define BUTTON_GPIO_Port GPIOC
// #define LEVEL_SENS_Pin GPIO_PIN_13
// #define LEVEL_SENS_GPIO_Port GPIOB
// #define FLOW_SENS_Pin GPIO_PIN_12
// #define FLOW_SENS_GPIO_Port GPIOB
// #define FAN_1_Pin GPIO_PIN_14
// #define FAN_1_GPIO_Port GPIOB
// #define FAN_2_Pin GPIO_PIN_15
// #define FAN_2_GPIO_Port GPIOB
// #define FAN_3_Pin GPIO_PIN_8
// #define FAN_3_GPIO_Port GPIOA

// #define P_SENSE_Pin GPIO_PIN_0
// #define P_SENSE_GPIO_Port GPIOA
// #define T_SENSE_Pin GPIO_PIN_1
// #define T_SENSE_GPIO_Port GPIOA
// #define C_SENSE_Pin GPIO_PIN_2
// #define C_SENSE_GPIO_Port GPIOA
// #define R_SENSE_Pin GPIO_PIN_3
// #define R_SENSE_GPIO_Port GPIOA

// #define DS_PIN_PIN GPIO_PIN_15 // пин для термометров
// #define DS_PIN_GPIO_Port GPIOA

#endif

// #define FanProtec
#define Pump_Off 60 // Задержка выключения помпы в сек.
// #define Pressure (11.0*14.504 - (-0.5*14.504))    //PAA-21Y 81556.11
#define Pressure (12.8 * 14.504 - (-1.0 * 14.504)) // BC-TP-013N
#define Fan_PWM_Low 128
// скорость двигателя
#define SPEED 10
#define BUS_RET_COMMAND_HEAD 0x72
#define default_ID_COOLING 0x51
#define tail 0xFF
// command id:
#define WATER_ON 0x04
#define WATER_OFF 0x05
#define CL_SET_TEMP 0x15
#define CL_GET_SET_TEMP 0x16
#define CL_GET_STATUS 0x17
#define CL_PUMP_START 0x18 // start lamp
#define CL_PUMP_STOP 0x19  // stop  lamp
// Fault code:
#define CL_WATER_LEVEL_ERR 0x01 << 0
#define CL_WATER_OVERHEAT 0x01 << 1
#define CL_LAB_OVERHEAT 0x01 << 2
#define CL_FLOW_LOW 0x01 << 3
#define CL_WATER_HEATING 0x01 << 4
#define CL_WATER_OFF 0x01 << 5
#define COOLING_COMM_FAULT 0x01 << 6

// LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
EncButton2<EB_BTN> enc(INPUT_PULLUP, Button);
GStepper<STEPPER2WIRE> stepper(500, PIN_STEP, PIN_DIR);
GyverPID regulator(9.0, 1.0, 0.01); // можно П, И, Д, без dt, dt будет по умолч. 100 мс

byte fanThermometer[] = {0x28, 0x99, 0x1D, 0xFB, 0x0C, 0x00, 0x00, 0xF7};
byte DS18B20_3[] = {0x28, 0xEB, 0xDD, 0x57, 0x04, 0xE1, 0x3C, 0x11};

MicroDS18B20<DS_PIN, fanThermometer> sensor1; // Создаем термометр с адресацией
MicroDS18B20<DS_PIN, DS18B20_3> sensor2;      // Создаем термометр с адресацией

volatile uint8_t Cansider_Sp; // Уставка
uint8_t Cansider_Gb = 10;     // Гистерезис
uint16_t Temp_Low_Power = 260;
uint16_t Temp_High_Power = 300;

uint8_t Sp_Low_Press_Pup = 110;
uint8_t Sp_Critical_Press_Pup = 50;
uint8_t Sp_Low_Press = 20;
uint8_t Sp_Critical_Press = 10;

volatile bool Chiler_On;
bool Chiller_Switch;
bool LowPressure;
bool CriticalPressure;
bool Freez_Temp;
bool Fan;
bool TestStart;

uint16_t PressureTransducer;
uint16_t Cansider_Temp;
int16_t Fan_Ctrl_Temp;
int16_t Test_Temp;
int16_t Press_Temp;

uint32_t time_05;
uint32_t time_01;
uint32_t time_1;
uint32_t time_20;

#ifdef FanProtec
uint8_t Fan1_Off;
uint8_t Fan2_Off;
uint8_t Fan3_Off;
#endif

volatile uint8_t PumpDelay_Off;

// шаги изменения температуры уставки 0.1
int8_t step_a = 1;

volatile uint8_t reserved[4];

volatile uint16_t ticks;

volatile float Power_Laser;

volatile uint32_t Comm_timeout = micros();
volatile uint32_t varTime = millis();

uint32_t valveTime = millis();

int pos = 0;

int16_t simEEPdata[] = {
    -250, // starting temperature = -20.0 deg °C
    50,   // increment by 5.0 deg  °c
    11,   // 10 entries in the lookup table
    0,    // average step of ADC reading
    218,  // 360,  //-25.0 deg °C
    296,  // 435,  //-20.0
    384,  // 524,  //-15
    487,  // 625,  //-10
    603,  // 740,  //-5
    734,  // 870,  // 0
    882,  // 1018, // 5
    1046, // 1184, // 10
    1230, // 1368, // 15
    1434, // 1572, // 20
    1662, // 1800, // 25.0 deg °C
};

uint16_t adc;

#ifdef __AVR_ATmega328PB__

void USART1_Init()
{
  // Set Baud Rate
  UBRR1H = BAUD_PRESCALER >> 8;
  UBRR1L = BAUD_PRESCALER;

  // Set Frame Format
  UCSR1C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;

  // Enable Receiver and Transmitter
  UCSR1B = (1 << RXEN0) | (1 << TXEN0);

  UCSR1B |= RX_COMPLETE_INTERRUPT; // enable interrupt

  // Enable Global Interrupts
  sei();
}
#endif
#ifdef STM32F10X_MD
void SysTick_Handler(void)
{
  HAL_IncTick();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 2500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */
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
  huart1.Init.BaudRate = 250000;
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

// /**
//   * Init USB device Library, add supported class and start the library
//   * @retval None
//   */
// void MX_USB_DEVICE_Init(void)
// {
//   /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

//   /* USER CODE END USB_DEVICE_Init_PreTreatment */

//   /* Init Device Library, add supported class and start the library. */
//   if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
//   {
//     Error_Handler();
//   }
//   if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
//   {
//     Error_Handler();
//   }
//   if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
//   {
//     Error_Handler();
//   }
//   if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
//   {
//     Error_Handler();
//   }

//   /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

//   /* USER CODE END USB_DEVICE_Init_PostTreatment */
// }

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_4988_Pin | RELAY_Pin | VALVE_1_Pin | VALVE_2_Pin | FAN_Pin | PUMP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEP_Pin | DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : P_SENSE_Pin */
  GPIO_InitStruct.Pin = P_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(P_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_4988_Pin STEP_Pin DIR_Pin RELAY_Pin
                           VALVE_1_Pin VALVE_2_Pin FAN_Pin PUMP_Pin */
  GPIO_InitStruct.Pin = EN_4988_Pin | STEP_Pin | DIR_Pin | RELAY_Pin | VALVE_1_Pin | VALVE_2_Pin | FAN_Pin | PUMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FLOW_SENS_Pin LEVEL_SENS_Pin FAN_1_Pin FAN_2_Pin */
  GPIO_InitStruct.Pin = FLOW_SENS_Pin | LEVEL_SENS_Pin | FAN_1_Pin | FAN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_3_Pin */
  GPIO_InitStruct.Pin = FAN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAN_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS_DIR_Pin */
  GPIO_InitStruct.Pin = RS_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS_DIR_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}
#endif

void setup()
{
#ifdef __AVR_ATmega328PB__
  wdt_reset();
  wdt_enable(WDTO_2S);

  TCCR2B = (TCCR2B & B11111000) | B00000101; // делитель 128 для (245 Гц)
                                             //  TCCR2B = (TCCR2B & B11111000) | B00000110; //делитель 256 для

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Compressor, OUTPUT);  // PWM //Relay
  pinMode(FAN, OUTPUT);         // PWM //FAN
  pinMode(Valve_1_Hot, OUTPUT); //  //Valve
  pinMode(Valve_2_Cold, OUTPUT);
  pinMode(RS485_REDE, OUTPUT);
  pinMode(PUMP, OUTPUT);
  digitalWrite(RS485_REDE, LOW);
  pinMode(11, OUTPUT); // TXD1
  // PORTB &= ~(1 << PB5);

  pinMode(Button, INPUT_PULLUP); // кнопка INT0
  // pinMode(7, INPUT_PULLUP);      // FAN1
  // pinMode(8, INPUT_PULLUP);      // FAN2
  // pinMode(9, INPUT_PULLUP);  // FAN3
  // pinMode(PIN_STEP, OUTPUT); // STEP
  // pinMode(PIN_DIR, OUTPUT);  // DIR
  // digitalWrite(PIN_STEP, LOW);
  // digitalWrite(PIN_DIR, LOW);
  pinMode(WL, INPUT_PULLUP); // WL
  pinMode(FS, INPUT_PULLUP); // FS
#endif
#ifdef STM32F10X_MD
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
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  // MX_USB_PCD_Init();

  /* Run the ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

#endif

#ifdef __AVR_ATmega328PB__
  //  PCICR |= (1 << PCIE0);     // прерывания для FS
  PCMSK0 |= (1 << (FS - 8)); // прерывания для FS

  ADCSRA |= (1 << ADEN); // set adc enable
  // ADMUX &= ~(1 << REFS1);                 // ADC_VCC
  // ADMUX |= (1 << REFS0);
  ADMUX |= ((1 << REFS1) | (1 << REFS0)); // ADC_1V1

  USART1_Init();
#endif

  Serial.begin(9600);

#ifdef __AVR_ATmega328PB__
  lcd.init(); // initialize the lcd

  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("OrchiChiller v3");
#endif

#ifdef __AVR_ATmega328PB__
  // Cansider_Sp = EEPROM.read(0) ? EEPROM.read(0) : 100;
  Cansider_Sp = EEPROM.read(0);
#endif
  if (Cansider_Sp < 35 || Cansider_Sp > 250)
    Cansider_Sp = 100;
#ifdef __AVR_ATmega328PB__
  wdt_reset();
#endif
#ifdef STM32F10X_MD
  HAL_IWDG_Refresh(&hiwdg);
#endif

  int N = simEEPdata[2];
  simEEPdata[3] = (simEEPdata[3 + N] - simEEPdata[4]) / N; // average step of ADC reading

  // установка макс. скорости в шагах/сек
  stepper.setMaxSpeed(50);
  // режим следования к целевй позиции
  stepper.setRunMode(FOLLOW_POS);
  // можно установить позицию
  stepper.setTarget(-500); // в шагах
  while (stepper.tick())
  {
#ifdef __AVR_ATmega328PB__
    wdt_reset();
#endif
#ifdef STM32F10X_MD
    HAL_IWDG_Refresh(&hiwdg);
#endif
  }
  stepper.setCurrent(0);
  stepper.setTarget(50); // в шагах

  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 450);    // пределы. ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = 670;       // сообщаем регулятору, которую он должен поддерживать

#ifdef __AVR_ATmega328PB__
  wdt_reset();
#endif
#ifdef STM32F10X_MD
  HAL_IWDG_Refresh(&hiwdg);
#endif
#ifdef __AVR_ATmega328PB__
  lcd.clear();
#endif
}

#ifdef __AVR_ATmega328PB__

ISR(USART1_RX_vect) // Обрабатываем прерывание по поступлению байта
{
  static uint8_t CountArr;     // счетчик принятых байтов
  static uint8_t IncomArr[14]; // входящий массив
  uint8_t SendArr[14];         // исходящий массив
  bool ReadOk;
  bool send;

  IncomArr[CountArr] = UDR1; // принимаем байт в массив
  if (IncomArr[0] == BUS_RET_COMMAND_HEAD && ReadOk == false)
  {
    CountArr++;
    if (CountArr == sizeof(IncomArr))
    { // если приняли все байты
      CountArr = 0;
      if ((IncomArr[1] == default_ID_COOLING) && (IncomArr[11] == tail) && (IncomArr[12] == tail) && (IncomArr[13] == tail))
        ReadOk = true;

      if (ReadOk)
      {
        Comm_timeout = millis();

        SendArr[0] = BUS_RET_COMMAND_HEAD;
        SendArr[1] = default_ID_COOLING;
        SendArr[2] = IncomArr[2];
        switch (IncomArr[2])
        {
        case WATER_ON:
          SendArr[5] = Chiler_On;
          send = true;
          if (!Chiler_On)
          {
            Chiler_On = true;
            PCICR |= (1 << PCIE0); // прерывания для FS
            varTime = millis();    // Сбрасываем счётчик и сохраняем время расчёта
            LowPressure = false;
          }
          break;
        case WATER_OFF:
          Chiler_On = false;
          PumpDelay_Off = Pump_Off;
          break;
        case CL_SET_TEMP:
          // Cansider_Sp = (IncomArr[4] << 8) | IncomArr[3];
          break;
        case CL_GET_SET_TEMP:
          SendArr[5] = Cansider_Sp & 0xff;
          SendArr[6] = Cansider_Sp >> 8;
          send = true;
          break;
        case CL_GET_STATUS:
          SendArr[3] = Fan_Ctrl_Temp & 0xff;
          SendArr[4] = Fan_Ctrl_Temp >> 8;
          SendArr[5] = Cansider_Temp & 0xff;
          SendArr[6] = Cansider_Temp >> 8;
          SendArr[7] = reserved[0];
          SendArr[8] = (reserved[1] > 30) ? reserved[1] : 30;
          SendArr[9] = reserved[2];
          SendArr[10] = reserved[3];
          send = true;
          break;
        case CL_PUMP_START:
          Power_Laser = pow((short)((IncomArr[4] << 8) | IncomArr[3]), 3) * ((short)((IncomArr[6] << 8) | IncomArr[5])) * 0.000001 * IncomArr[7] / pow(40, 2); // приблизительный расчет мощности лазера
          break;
        case CL_PUMP_STOP:
          send = true;
          break;
        default:
          return;
        }
        SendArr[11] = tail;
        SendArr[12] = tail;
        SendArr[13] = tail;

        if (send)
        {
          PORTB |= (1 << PB5);
          for (uint8_t i = 0; i < sizeof(SendArr); i++)
          {
            while (!(UCSR1A & (1 << UDRE1)))
              ;                // ждем опустошения буфера
            UDR1 = SendArr[i]; // отправляем байт
            // SendArr[i] = 0;    // сразу же чистим переменную
          }
          while (!(UCSR1A & (1 << UDRE1)))
            ; // ждем опустошения буфера
          for (int i = 0; i < 1000; i++)
          {
            asm("NOP");
          }
          PORTB &= ~(1 << PB5);
        }
      }
    }
  }
}

ISR(PCINT0_vect)
{
  if (bitRead(PINB, FS - 8))
  {
    ticks++;

    if ((varTime + 1000) < millis() || varTime > millis())
    { // Если c момента последнего расчёта прошла 1 секунда, или произошло переполнение millis то ...
      reserved[1] = ticks;
      ticks = 0;
      varTime = millis(); // Сбрасываем счётчик и сохраняем время расчёта
    }
  }
}

#endif

// управление через плоттер
void parsing()
{
  if (Serial.available() > 1)
  {
    char incoming = Serial.read();
    float value = Serial.parseFloat();
    switch (incoming)
    {
    case 'p':
      regulator.Kp = value;
      break;
    case 'i':
      regulator.Ki = value;
      break;
    case 'd':
      regulator.Kd = value;
      break;
    case 's':
      regulator.setpoint = value;
      break;
    }
  }
}

int16_t Lookup()
{

  int16_t dm_902;
  if (int(PressureTransducer) < simEEPdata[4])
  {                                                                                     // less than the first entry of Lookup table (LUT)
    dm_902 = (int(PressureTransducer) - simEEPdata[4]) * simEEPdata[1] / simEEPdata[3]; // extrapolate from first LUT data
    dm_902 = dm_902 + simEEPdata[0];                                                    // compute the temperature
    return dm_902;
  }
  else if (int(PressureTransducer) > simEEPdata[3 + simEEPdata[2]])
  {                                                                                                     // more than the last entry of Lookup table (LUT)
    dm_902 = (int(PressureTransducer) - simEEPdata[3 + simEEPdata[2]]) * simEEPdata[1] / simEEPdata[3]; // extrapolate from last LUT data
    dm_902 = simEEPdata[0] + simEEPdata[1] * (simEEPdata[2] - 1) + dm_902;                              // compute the temperature
    return dm_902;
  }

  int I = int(PressureTransducer) / simEEPdata[3] - 5; // find approximate location to lookup
  // simEEPdata[2] is the average ADC increment per table entry
  if (I < 1 || I > simEEPdata[2])
    I = 1; // Out of range. Then start from first table entry

  while (I < simEEPdata[2])
  { // up to the end of lookup table
    if (int(PressureTransducer) < simEEPdata[3 + I])
      break;
    I = I + 1;
  }

  dm_902 = (int(PressureTransducer) - simEEPdata[3 + I - 1]) * simEEPdata[1] / (simEEPdata[3 + I] - simEEPdata[3 + I - 1]);
  dm_902 = simEEPdata[0] + simEEPdata[1] * (I - 2) + dm_902; // compute actual value
  return dm_902;
}

// бегущее среднее
float expRunningAverage(float newVal)
{
  static float filVal = 0;
  filVal += (newVal - filVal) * 0.25; // коэфф. фильтрации 0.5 Чем он меньше, тем плавнее фильтр
  return filVal;
}

// бегущее среднее 2
float expRunningAverage2(float newVal)
{
  static float filVal2 = 0;
  filVal2 += (newVal - filVal2) * 0.5; // коэфф. фильтрации 0.5 Чем он меньше, тем плавнее фильтр
  return filVal2;
}

void Check_Pressure()
{
  static uint8_t dm_90;
  static uint8_t dm_91;
  static uint8_t dm_92;
  static uint8_t dm_93;

  PressureTransducer = 0;

#ifdef __AVR_ATmega328PB__
  //  + 2разряда
  for (uint8_t i = 0; i < (1ul << (2 << 1)); i++)
  {
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); // ADC_A0
    ADCSRA |= (1 << ADSC);                                             // ручной старт преобразования
    while (ADCSRA & (1 << ADSC))
      ;                        // пока преобразование не готово - ждем
    PressureTransducer += ADC; // Pressure
  }
  PressureTransducer >>= 2;
  //

  PressureTransducer = ((Pressure / ((50.8 / (ADC_REF / 4096.0 / 0.020)) - (50.8 / (ADC_REF / 4096.0 / 0.004)))) * (PressureTransducer - (50.8 / (ADC_REF / 4096.0 / 0.004))) + (-1.0 * 14.504)) * 10.0; // 3808ацп-20мА //752-4мА
#endif
#ifdef STM32F10X_MD
  ADC_ChannelConfTypeDef sConfig;

  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);   // ожидаем окончания преобразования
  adc = (uint32_t)HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
  HAL_ADC_Stop(&hadc1);                     // останавливаем АЦП (не обязательно)

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);                  // ожидаем окончания преобразования
  PressureTransducer = (uint32_t)HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
  HAL_ADC_Stop(&hadc1);                                    // останавливаем АЦП (не обязательно)

  PressureTransducer = ((Pressure / ((62.0 / ((ADC_REF / adc * 4095.0) / 4095.0 / 0.020)) - (62.0 / ((ADC_REF / adc * 4095.0) / 4095.0 / 0.004)))) * (PressureTransducer - (62.0 / ((ADC_REF / adc * 4095.0) / 4095.0 / 0.004))) + (-1.0 * 14.504)) * 10.0; // 3808ацп-20мА //752-4мА
#endif

  PressureTransducer = expRunningAverage2(PressureTransducer);

  if (Chiler_On == 0)
  {
    dm_92 = 0;
    dm_93 = 0;

    //---- Low Pressure Chiler Off ----
    if ((PressureTransducer / 10) < Sp_Low_Press_Pup)
    {
      dm_90++;
      if (dm_90 > 30)
      {
        LowPressure = true; // после 30 проверок
      }
    }
    else
    {
      dm_90 = 0;
      LowPressure = false;
    }

    //---- Critical Pressure ChilerOff ----
    if ((PressureTransducer / 10) < Sp_Critical_Press_Pup)
    {
      dm_91++;
      if (dm_91 > 30)
      {
        CriticalPressure = true; // после 30 проверок
      }
    }
    else
    {
      dm_91 = 0;
      CriticalPressure = false;
    }
  }
  else
  {
    dm_90 = 0;
    dm_91 = 0;

    //---- Low Pressure Chiler On ----
    if ((PressureTransducer / 10) < Sp_Low_Press)
    {
      dm_92++;
      if (dm_92 > 30)
      {
        LowPressure = true; // после 30 проверок
      }
    }
    else
    {
      dm_92 = 0;
      LowPressure = false;
    }

    //---- Critical Pressure Chiler On ----
    if ((PressureTransducer / 10) < Sp_Critical_Press)
    {
      dm_93++;
      if (dm_93 > 100)
      {
        CriticalPressure = true; // после 100 проверок
      }
    }
    else
    {
      dm_93 = 0;
      CriticalPressure = false;
    }
  }
}

void Chiller_Protec()
{
  static uint8_t dm_45;
  static uint8_t dm_46;
  static uint8_t dm_47;
  static uint8_t dm_48;

  if ((reserved[1] < 35 || (millis() - varTime) > 1000) && !reserved[0])
  {
    dm_48++;
    if (dm_48 > 10)
    {
      reserved[0] |= CL_FLOW_LOW;
    }
  }
  else
  {
    dm_48 = 0;
    reserved[0] &= ~(CL_FLOW_LOW);
  }

  if (Cansider_Temp <= 350)
  {
    dm_45 = 0;
    reserved[0] &= ~(CL_WATER_OVERHEAT);
  }
  else if (Cansider_Temp > 350)
  {
    dm_45++;
    if (dm_45 > 100)
    {
      Chiler_On = false;
      reserved[0] |= CL_WATER_OVERHEAT; // water temp max
    }
  }

  if (Cansider_Temp >= 35)
  { // 80
    dm_46 = 0;
    Freez_Temp = false;
  }
  else if (Cansider_Temp < 35)
  { // 80
    dm_46++;
    if (dm_46 > 100)
    {
      Chiler_On = false;
      Freez_Temp = true;
    }
  }

  if (Fan_Ctrl_Temp <= 750)
  {
    dm_47 = 0;
    reserved[0] &= ~(CL_LAB_OVERHEAT);
  }
  else if (Fan_Ctrl_Temp > 750)
  {
    dm_47++;
    if (dm_47 > 100)
    {
      Chiler_On = false;
      reserved[0] |= CL_LAB_OVERHEAT; // air temp max
    }
  }
}

void Control_Fan()
{
  unsigned int Fan_PWM;
  if (Chiler_On)
  {
    if (Fan_Ctrl_Temp <= Temp_Low_Power)
    {
      Fan_PWM = Fan_PWM_Low;
    }
    else if (Fan_Ctrl_Temp >= Temp_High_Power)
    {
      Fan_PWM = 255;
    }
    else
    {
      Fan_PWM = Fan_PWM_Low + ((Fan_Ctrl_Temp - Temp_Low_Power) * ((255 - Fan_PWM_Low) / (Temp_High_Power - Temp_Low_Power)));

      if (Fan_PWM > 255)
      {
        Fan_PWM = 255;
      }
      else if (Fan_PWM < 0)
      {
        Fan_PWM = 0;
      }
    }
  }
  else
  {
    Fan_PWM = 0;
  }
  analogWrite(FAN, Fan_PWM); // в оригинале 200Гц
  reserved[2] = 100 / 255 * Fan_PWM;
}

void Control_Values()
{
  if (Chiler_On)
  {
    if (Cansider_Temp > Cansider_Sp)
    {
      digitalWrite(Valve_2_Cold, HIGH);
      digitalWrite(Valve_1_Hot, LOW);
    }
    else if (Cansider_Temp < uint16_t(Cansider_Sp - Cansider_Gb))
    {
      digitalWrite(Valve_2_Cold, LOW);
      digitalWrite(Valve_1_Hot, HIGH);
    }
  }
  else
  {
    digitalWrite(Valve_1_Hot, HIGH);
    digitalWrite(Valve_2_Cold, HIGH);
    TestStart = false;
  }
}

void readTemp()
{
  Cansider_Temp = 0;

#ifdef __AVR_ATmega328PB__
  for (uint8_t i = 0; i < (1ul << (2 << 1)); i++)
  {
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX0)); // ADC_A2
    ADMUX |= (1 << MUX1);
    ADCSRA |= (1 << ADSC); // ручной старт преобразования
    while (ADCSRA & (1 << ADSC))
      ;                   // пока преобразование не готово - ждем
    Cansider_Temp += ADC; // FanTemp
  }
  Cansider_Temp = Cansider_Temp >> 2;

  Cansider_Temp = (Cansider_Temp / 4096.0 * ADC_REF * 1000.0);
#endif
#ifdef STM32F10X_MD
  ADC_ChannelConfTypeDef sConfig;

  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);   // ожидаем окончания преобразования
  adc = (uint32_t)HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
  HAL_ADC_Stop(&hadc1);                     // останавливаем АЦП (не обязательно)

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);             // ожидаем окончания преобразования
  Cansider_Temp = (uint32_t)HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
  HAL_ADC_Stop(&hadc1);                               // останавливаем АЦП (не обязательно)

  Cansider_Temp = (Cansider_Temp / 4095.0 * (ADC_REF / adc * 4095.0) * 1000.0);
#endif

  Cansider_Temp = expRunningAverage(Cansider_Temp);

  if (sensor1.readTemp())
    Fan_Ctrl_Temp = int(sensor1.getTemp() * 10.0);
  else
    // Fan_Ctrl_Temp = 999;
    ;

  if (sensor2.readTemp())
    Test_Temp = int(sensor2.getTemp() * 10.0);
  else
    // Test_Temp = 999;
    ;

  sensor1.requestTemp(); // Запрашиваем преобразование температуры, но не ждем.
  sensor2.requestTemp();
}

void loop()
{
#ifdef STM32F10X_MD
  if (Serial)
  { // USB  подключен
  }
  else
  {
  }
#endif
#ifdef __AVR_ATmega328PB__
  wdt_reset();
#endif
#ifdef STM32F10X_MD
  HAL_IWDG_Refresh(&hiwdg);
#endif
  stepper.tick();
  parsing();
  pos = int(regulator.getResultTimer());

  if ((millis() - time_20) > 2000)
  {
    time_20 = millis();
    if (Chiler_On)
    {
      if (Cansider_Temp > Cansider_Sp)
      {
        // if (regulator.setpoint > 100)
        //   regulator.setpoint = regulator.setpoint - 10;
        if (regulator.setpoint > 670)
          regulator.setpoint = regulator.setpoint - 10;
      }
      else if (Cansider_Temp < Cansider_Sp)
      {
        // if (regulator.setpoint < 250)
        //   regulator.setpoint = regulator.setpoint + 10;
        if (regulator.setpoint < 900)
          regulator.setpoint = regulator.setpoint + 10;
      }
    }
  }

  if ((millis() - time_1) > 1000)
  {
    time_1 = millis();

    if (PumpDelay_Off)
      PumpDelay_Off--;

    if (digitalRead(WL))
    {
      reserved[0] |= CL_WATER_LEVEL_ERR;
    }
    else
    {
      reserved[0] &= ~(CL_WATER_LEVEL_ERR);
    }

#ifdef __AVR_ATmega328PB__
    Wire.beginTransmission(0x27);
    if (Wire.endTransmission())
      lcd.init();
#endif

#ifdef FanProtec
    if (Fan1_Off >= 20 || Fan2_Off >= 20 || Fan3_Off >= 20)
    {
      Fan = true;
      Chiler_On = false;
    }
    else
    {
      Fan = false;
    }
#endif

    if (((millis() - Comm_timeout) > 3000) && (TestStart == false))
    {
      reserved[0] |= COOLING_COMM_FAULT;
      Chiler_On = false;
    }
    else
      reserved[0] &= ~(COOLING_COMM_FAULT);

    if ((reserved[0] || LowPressure || CriticalPressure || Freez_Temp || Fan) && Chiller_Switch)
    {
#ifdef __AVR_ATmega328PB__
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error:");
      lcd.setCursor(0, 1);
#endif
      String Error;
      if (Freez_Temp)
        Error = "Fan";
      else if (Freez_Temp)
        Error = "Freez Temp";
      else if (CriticalPressure)
        Error = "Critical Pressure";
      else if (reserved[0] & CL_WATER_LEVEL_ERR)
        Error = "Water level";
      else if (reserved[0] & CL_WATER_OVERHEAT)
        Error = "Water overheat";
      else if (reserved[0] & CL_LAB_OVERHEAT)
        Error = "Air overheat";
      else if (reserved[0] & CL_FLOW_LOW)
        Error = "Flow low";
      else if (LowPressure)
        Error = "Low Pressure";
      else if (reserved[0] & COOLING_COMM_FAULT)
        Error = "Comm fault";
      else if (reserved[0] & CL_WATER_HEATING)
        Error = "ON";
      else if (reserved[0] & CL_WATER_OFF)
        Error = "OFF";
      else
        Error = "None";
#ifdef __AVR_ATmega328PB__
      lcd.print(Error);
#endif
    }
    else
    {
#ifdef __AVR_ATmega328PB__
      lcd.clear();
      lcd.setCursor(0, 0);
      // lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.print("Set:");
      lcd.print(Cansider_Sp / 10.0, 1);
      lcd.setCursor(10, 0);
      lcd.print(PressureTransducer / 10);
      lcd.print("psi");
      lcd.setCursor(0, 1);
      // lcd.print("                ");
      lcd.setCursor(0, 1);
      // lcd.print("T1:");
      // lcd.print(Cansider_Temp / 10.0, 1);
      // lcd.print("F:");
      // lcd.print(reserved[1]);
      lcd.print((Test_Temp - Press_Temp) / 10.0, 1);
      // lcd.print(" T2:");
      // lcd.print(Fan_Ctrl_Temp / 10.0, 1);
      // lcd.setCursor(5, 1);
      // lcd.print(Power_Laser, 0);
      // lcd.print(" W");
      // lcd.print(Test_Temp / 10.0, 1);
      lcd.setCursor(11, 1);
      lcd.print(100.0 / 450.0 * stepper.getCurrent(), 0);
      lcd.print("%");
#endif
    }
  }

  if ((millis() - time_05) > 500)
  {
    time_05 = millis();

    // Serial.print(regulator.setpoint);
    // Serial.print(',');
    // Serial.print(Test_Temp - Press_Temp);
    // Serial.print(',');
    // Serial.print(Test_Temp);
    // Serial.print(',');
    // Serial.print(Press_Temp);
    // Serial.print(',');
    // Serial.println();
    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);   // ожидаем окончания преобразования
    adc = (uint32_t)HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
    HAL_ADC_Stop(&hadc1);                     // останавливаем АЦП (не обязательно)
    Serial.println(3.290 / 4095.0 * adc, 3);
    Serial.println(ADC_REF / adc * 4095.0, 3);

    if (Chiler_On)
    {
      digitalWrite(PUMP, HIGH);
      if (reserved[1] > 35)
      {
        Chiller_Switch = true;
        digitalWrite(Compressor, HIGH);
        reserved[0] &= ~(CL_WATER_OFF);
      }
    }
    else
    {
#ifdef __AVR_ATmega328PB__
      PCICR &= ~(1 << PCIE0); // прерывания выкл для FS
#endif
      reserved[1] = 0;
      digitalWrite(Compressor, LOW);
      if (PumpDelay_Off == 0)
        digitalWrite(PUMP, LOW);
      reserved[0] |= CL_WATER_OFF;
    }

    readTemp();
    Control_Values();
    Control_Fan();

    if (CriticalPressure)
    {
      Chiler_On = false;
    }

    if (reserved[0] & CL_FLOW_LOW)
    {
      Chiler_On = false;
    }
  }

  if ((millis() - time_01) > 100)
  {
    time_01 = millis();

    Check_Pressure();
    Press_Temp = Lookup();

    regulator.input = float(PressureTransducer);

    if (digitalRead(Valve_1_Hot))
    {
      stepper.setTarget(200); // в шагах
    }
    else
    {
      stepper.setTarget(pos);
    }

    if (Chiler_On)
    {
      Chiller_Protec();

      if (Cansider_Temp < (Cansider_Sp - 10))
      {
        digitalWrite(Valve_1_Hot, HIGH);
      }
#ifdef __AVR_ATmega328PB__
#ifdef FanProtec
      if ((1 << PD3) & PIND)
      {
        if ((1 << PD7) & PIND)
        {
          Fan1_Off++;
        }
        else
        {
          Fan1_Off = 0;
        }
        if ((1 << PB0) & PINB)
        {
          Fan2_Off++;
        }
        else
        {
          Fan2_Off = 0;
        }
        if ((1 << PB1) & PINB)
        {
          Fan3_Off++;
        }
        else
        {
          Fan3_Off = 0;
        }
      }
#endif
#endif
    }
  }

  enc.tick(); // опрос происходит здесь

  if (enc.click())
  {
    Chiller_Switch = false;
#ifdef __AVR_ATmega328PB__
    lcd.clear();
#endif
  }

  if (enc.step(1))
  {
    Cansider_Sp += step_a;
    if (Cansider_Sp < 50 || Cansider_Sp > 350)
      Cansider_Sp -= step_a;
#ifdef __AVR_ATmega328PB__
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("Set:");
    lcd.print(Cansider_Sp / 10.0, 1);
#endif
  }
  // разворачиваем шаг для изменения в обратную сторону
  // передаём количество предварительных кликов
  if (enc.releaseStep(1))
  {
    step_a = -step_a;
#ifdef __AVR_ATmega328PB__
    if (EEPROM.read(0) != Cansider_Sp)
      EEPROM.write(0, Cansider_Sp);
#endif
  }

  if (enc.held(0))
  {
    if (Chiler_On)
    {
      Chiler_On = false;
      PumpDelay_Off = Pump_Off;
    }
    else
    {
      TestStart = true;
      Chiler_On = true;
#ifdef __AVR_ATmega328PB__
      PCICR |= (1 << PCIE0); // прерывания для FS
#endif
      varTime = millis(); // Сбрасываем счётчик и сохраняем время расчёта
      LowPressure = false;
    }
  }
}