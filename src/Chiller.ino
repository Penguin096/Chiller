// OUTPUT: D3...D6
// INPUT:  D7...D11
// ANALOG: A0, A2

// #define test

#define Button 2
#define Compressor 5
#define FAN 3
// #define Valve_1_Hot 6
#define Valve_2_Cold 4
#define WL 10
#define FS 11
#define RS485_REDE 12
#define PUMP 6

// #define CansiderTemp A3
#define DS_PIN A3 // пин для термометров
// #define FanTemp A2
// #define Pressure A0

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

#define _TACHO_TICKS_AMOUNT 10 // количество тиков для счёта времени
#define _TACHO_TIMEOUT 1000000 // таймаут прерываний (мкс), после которого считаем что вращение прекратилось

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

#include <Arduino.h>
#include <avr/wdt.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

#include <EncButton2.h>
EncButton2<EB_BTN> enc(INPUT_PULLUP, Button);

#include <EEPROM.h>

#include <microDS18B20.h>

byte cansiderThermometer[] = {0x28, 0x99, 0x1D, 0xFB, 0x0C, 0x00, 0x00, 0xF7};
byte fanThermometer[] = {0x28, 0x2F, 0x50, 0xFB, 0x0C, 0x00, 0x00, 0x47};

MicroDS18B20<DS_PIN, fanThermometer> sensor1;      // Создаем термометр с адресацией
MicroDS18B20<DS_PIN, cansiderThermometer> sensor2; // Создаем термометр с адресацией

#ifdef test
#include "GyverRelay.h"
// установка, гистерезис, направление регулирования
GyverRelay regulator(NORMAL);
#endif

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

uint16_t PressureTransducer;
uint16_t Cansider_Temp;
uint16_t Fan_Ctrl_Temp;

uint32_t time_05;
uint32_t time_01;
uint32_t time_10;

uint8_t Fan1_Off;
uint8_t Fan2_Off;
uint8_t Fan3_Off;

// шаги изменения температуры уставки 0.1
int8_t step_a = 1;

volatile uint8_t reserved[4];

volatile uint16_t ticks;

volatile float Power_Laser;

volatile uint32_t Comm_timeout = micros();
volatile uint32_t varTime = millis();

void USART_Init()
{
  // Set Baud Rate
  UBRR0H = BAUD_PRESCALER >> 8;
  UBRR0L = BAUD_PRESCALER;

  // Set Frame Format
  UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;

  // Enable Receiver and Transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  UCSR0B |= RX_COMPLETE_INTERRUPT; // enable interrupt

  // Enable Global Interrupts
  sei();
}

void setup()
{
  wdt_reset();
  wdt_enable(WDTO_2S);

  TCCR2B = (TCCR2B & B11111000) | B00000101; // делитель 128 для (245 Гц)
  //  TCCR2B = (TCCR2B & B11111000) | B00000110; //делитель 256 для

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Compressor, OUTPUT); // PWM //Relay
  pinMode(FAN, OUTPUT);        // PWM //FAN
  // pinMode(Valve_1_Hot, OUTPUT); // PWM //Valve
  pinMode(Valve_2_Cold, OUTPUT);
  pinMode(RS485_REDE, OUTPUT);
  pinMode(PUMP, OUTPUT);
  digitalWrite(RS485_REDE, LOW);

  pinMode(Button, INPUT_PULLUP); // кнопка INT0
  pinMode(7, INPUT_PULLUP);      // FAN1
  pinMode(8, INPUT_PULLUP);      // FAN2
  pinMode(9, INPUT_PULLUP);      // FAN3
  pinMode(WL, INPUT_PULLUP);     // WL
  pinMode(FS, INPUT_PULLUP);     // FS

  //  PCICR |= (1 << PCIE0);     // прерывания для FS
  PCMSK0 |= (1 << (FS - 8)); // прерывания для FS

  ADCSRA |= (1 << ADEN); // set adc enable
  // ADMUX &= ~(1 << REFS1);                 // ADC_VCC
  // ADMUX |= (1 << REFS0);
  ADMUX |= ((1 << REFS1) | (1 << REFS0)); // ADC_1V1

  USART_Init();

  lcd.init(); // initialize the lcd

  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("OrchiChiller v2");

  Cansider_Sp = EEPROM.read(0) ? EEPROM.read(0) : 100;

#ifdef test
  regulator.setpoint = Cansider_Sp; // установка (ставим на 10 градусов)
  regulator.hysteresis = 10;        // ширина гистерезиса 1 градус
  regulator.k = 0.5;                // коэффициент обратной связи (подбирается по факту)
#endif
  wdt_reset();
  delay(1000);
  lcd.clear();
}

ISR(USART_RX_vect) // Обрабатываем прерывание по поступлению байта
{
  static uint8_t CountArr;     // счетчик принятых байтов
  static uint8_t IncomArr[14]; // входящий массив
  uint8_t SendArr[14];         // исходящий массив
  bool ReadOk;
  bool send;

  IncomArr[CountArr] = UDR0; // принимаем байт в массив
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
          Chiler_On = true;
          break;
        case WATER_OFF:
          Chiler_On = false;
          break;
        case CL_SET_TEMP:
          Cansider_Sp = (IncomArr[4] << 8) | IncomArr[3];
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
          Power_Laser = ((short)((IncomArr[4] << 8) | IncomArr[3]) ^ 3) * (short)((IncomArr[6] << 8) | IncomArr[5]) * IncomArr[7] / (40 ^ 2); // приблизительный расчет мощности лазера
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
          PORTB |= (1 << PB4);
          for (uint8_t i = 0; i < sizeof(SendArr); i++)
          {
            while (!(UCSR0A & (1 << UDRE0)))
              ;                // ждем опустошения буфера
            UDR0 = SendArr[i]; // отправляем байт
            // SendArr[i] = 0;    // сразу же чистим переменную
          }
          while (!(UCSR0A & (1 << UDRE0)))
            ; // ждем опустошения буфера
          for (int i = 0; i < 1000; i++)
          {
            asm("NOP");
          }
          PORTB &= ~(1 << PB4);
        }

        // ReadOk = false;
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

void loop()
{
  wdt_reset();

  if ((millis() - time_10) > 1000)
  {
    time_10 = millis();

    if (digitalRead(WL))
    {
      reserved[0] |= CL_WATER_LEVEL_ERR;
    }
    else
    {
      reserved[0] &= ~(CL_WATER_LEVEL_ERR);
    }

    Wire.beginTransmission(0x27);
    if (Wire.endTransmission())
      lcd.init();

    if (Fan1_Off >= 20 || Fan2_Off >= 20 || Fan3_Off >= 20)
    {
      Fan = true;
      Chiler_On = false;
    }
    else
    {
      Fan = false;
    }

    if ((millis() - Comm_timeout) > 3000)
    {
      reserved[0] |= COOLING_COMM_FAULT;
      Chiler_On = false;
    }
    else
      reserved[0] &= ~(COOLING_COMM_FAULT);

    if ((reserved[0] || LowPressure || CriticalPressure || Freez_Temp || Fan) && Chiller_Switch)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error:");
      lcd.setCursor(0, 1);
      String Error;
      if (Freez_Temp)
        Error = "Fan";
      else if (Freez_Temp)
        Error = "Freez Temp";
      else if (LowPressure)
        Error = "Low Pressure";
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
      else if (reserved[0] & CL_WATER_HEATING)
        Error = "Pump ON";
      else if (reserved[0] & CL_WATER_OFF)
        Error = "Pump OFF";
      else if (reserved[0] & COOLING_COMM_FAULT)
        Error = "Comm fault";
      else
        Error = "None";
      lcd.print(Error);
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.print("Set:");
      lcd.print(Cansider_Sp / 10.0, 1);
      lcd.setCursor(10, 0);
      lcd.print(PressureTransducer);
      lcd.print("psi");
      lcd.setCursor(0, 1);
      lcd.print("T1:");
      lcd.print(Cansider_Temp / 10.0, 1);
      // lcd.print(" T2:");
      // lcd.print(Fan_Ctrl_Temp / 10.0, 1);
      lcd.setCursor(7, 1);
      lcd.print("W:");
      lcd.print(Power_Laser);
    }
  }

  if ((millis() - time_05) > 500)
  {
    time_05 = millis();
    if (Chiler_On)
    {
      Chiller_Switch = true;
      digitalWrite(PUMP, HIGH);
      digitalWrite(Compressor, HIGH);
      reserved[0] &= ~(CL_WATER_OFF);
      PCICR |= (1 << PCIE0); // прерывания для FS
    }
    else
    {
      PCICR &= ~(1 << PCIE0); // прерывания выкл для FS
      digitalWrite(Compressor, LOW);
      digitalWrite(PUMP, LOW);
      reserved[0] &= ~(CL_FLOW_LOW);
      reserved[0] |= CL_WATER_OFF;
      LowPressure = false;
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
    if (Chiler_On)
    {
      Chiller_Protec();
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
    }
  }

  enc.tick(); // опрос происходит здесь

  if (enc.click())
  {
    Chiller_Switch = false;
    lcd.clear();
  }

  if (enc.step(1))
  {
    Cansider_Sp += step_a;
    if (Cansider_Sp < 50 || Cansider_Sp > 350)
      Cansider_Sp -= step_a;
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("Set:");
    lcd.print(Cansider_Sp / 10.0, 1);
#ifdef test
    regulator.setpoint = Cansider_Sp;
#endif
  }
  // разворачиваем шаг для изменения в обратную сторону
  // передаём количество предварительных кликов
  if (enc.releaseStep(1))
  {
    step_a = -step_a;
    if (EEPROM.read(0) != Cansider_Sp)
      EEPROM.write(0, Cansider_Sp);
  }

  if (enc.held(0))
  {
    Chiler_On = Chiler_On ? false : true;
  }
}

// бегущее среднее
float expRunningAverage(float newVal)
{
  static float filVal = 0;
  filVal += (newVal - filVal) * 0.5; // коэфф. фильтрации 0.5
  return filVal;
}

// бегущее среднее 2
float expRunningAverage2(float newVal)
{
  static float filVal2 = 0;
  filVal2 += (newVal - filVal2) * 0.5; // коэфф. фильтрации 0.5
  return filVal2;
}

void Check_Pressure()
{
  static uint8_t dm_90;
  static uint8_t dm_91;
  static uint8_t dm_92;
  static uint8_t dm_93;

  PressureTransducer = 0;

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

  PressureTransducer = expRunningAverage2(PressureTransducer);
  PressureTransducer = (PressureTransducer - 815.0) * (159.542 - (-7.252)) / (4076.0 - 815.0); // 4076ацп-20мА //815ацп-4мА

  if (Chiler_On == 0)
  {
    dm_92 = 0;
    dm_93 = 0;

    //---- Low Pressure Chiler Off ----
    if (PressureTransducer < Sp_Low_Press_Pup)
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
    if (PressureTransducer < Sp_Critical_Press_Pup)
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
    if (PressureTransducer < Sp_Low_Press)
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
    if (PressureTransducer < Sp_Critical_Press)
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

  if ((reserved[1] < 40 || (millis() - varTime) > 1000) && !reserved[0])
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

  if (Cansider_Temp >= 50)
  { // 80
    dm_46 = 0;
    Freez_Temp = false;
  }
  else if (Cansider_Temp < 50)
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
      Fan_PWM = 128;
    }
    else if (Fan_Ctrl_Temp >= Temp_High_Power)
    {
      Fan_PWM = 255;
    }
    else
    {
      Fan_PWM = 128 + ((Fan_Ctrl_Temp - Temp_Low_Power) * ((255 - 128) / (Temp_High_Power - Temp_Low_Power)));

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
#ifndef test
    if (Cansider_Temp > Cansider_Sp)
    {
      digitalWrite(Valve_2_Cold, HIGH);
      // digitalWrite(Valve_1_Hot, LOW);
    }
    else if (Cansider_Temp < uint16_t(Cansider_Sp - Cansider_Gb))
    {
      digitalWrite(Valve_2_Cold, LOW);
      // digitalWrite(Valve_1_Hot, HIGH);
    }
#else
    regulator.input = Cansider_Temp; // сообщаем регулятору текущую температуру
    if (regulator.getResult())
    {
      digitalWrite(Valve_2_Cold, HIGH);
      // digitalWrite(Valve_1_Hot, LOW);
    }
    else
    {
      digitalWrite(Valve_2_Cold, LOW);
      // digitalWrite(Valve_1_Hot, HIGH);
    }
#endif
  }
  else
  {
    // digitalWrite(Valve_1_Hot, HIGH);
    digitalWrite(Valve_2_Cold, HIGH);
  }
}

void readTemp()
{
  Cansider_Temp = 0;

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

  Cansider_Temp = expRunningAverage(Cansider_Temp);
  Cansider_Temp = (Cansider_Temp / 4096.0 * 1.025 * 1000.0 + 10.0);

  if (sensor1.readTemp())
    Fan_Ctrl_Temp = sensor1.getTemp() * 10.0;
  else if (sensor2.readTemp())
    Fan_Ctrl_Temp = sensor2.getTemp() * 10.0;
  else
    ;

  sensor1.requestTemp(); // Запрашиваем преобразование температуры, но не ждем.
  sensor2.requestTemp();

  if (Cansider_Temp < 0 || Cansider_Temp > 999)
  {
    Cansider_Temp = 999;
  }
  if (Fan_Ctrl_Temp < 0 || Fan_Ctrl_Temp > 999)
  {
    Fan_Ctrl_Temp = 999;
  }
}
