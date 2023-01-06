#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <Arduino.h>
#include <SX127x.h>

//#define CRC16_ENABLE
#define LORA_ENABLE
//#define UART_ENABLE

#define LORA_BW 125000
#define LORA_CODE_RATE 5
#define LORA_CRC_ENABLE true
#define LORA_FREQ 869525000
#define LORA_HDR_TYPE SX127X_HEADER_IMPLICIT
#define LORA_PREAMBLE_LEN 8
#define LORA_SPREAD_FACTOR 7
#define LORA_SYNC_WORD 0x4C
#define LORA_TX_PA_PIN SX127X_TX_POWER_PA_BOOST
#define LORA_TX_POWER 17

#define HDR_SIZE 2
#define PULSE_SIZE 4
#define MISC_SIZE 3
#if defined(CRC16_ENABLE)
  #define CRC_SIZE 2
#else
  #define CRC_SIZE 0
#endif /* CRC16_ENABLE */

#define DATA_SIZE (HDR_SIZE + PULSE_SIZE + MISC_SIZE)
#define PKT_SIZE (DATA_SIZE + CRC_SIZE)

#define ADC_MASK 0x3FF
#define VCC_SHIFT 0
#define TEMP_SHIFT 10
#define PULSE_BIT (1UL << 20)

#if defined(__AVR_ATtiny84__)
  #define ADC_PIN A0 /* PIN_PA0 */
  #define LORA_RST_PIN PIN_PA3
  #define LORA_SS_PIN PIN_PA7
  #define PULSE_ACTIVE LOW
  #define PULSE_MODE CHANGE
  #define PULSE_PIN PIN_PB2
  #if (F_CPU >= 12000000)
    #define UART_SPEED 57600
  #elif (F_CPU >= 8000000)
    #define UART_SPEED 38400
  #elif (F_CPU >= 6000000)
    #define UART_SPEED 28800
  #else
    #define UART_SPEED 4800
  #endif /* F_CPU -> UART_SPEED */
#endif /* __AVR_ATtiny84__ */

#if defined(__AVR_ATmega328P__)
  #define ADC_PIN A0
  #define LORA_RST_PIN 9
  #define LORA_SS_PIN 10
  #define PULSE_ACTIVE LOW
  #define PULSE_MODE CHANGE
  #define PULSE_PIN 2
  #define UART_SPEED 9600
#endif /* __AVR_ATmega328P__ */

#if defined(__AVR_ATmega2560__)
  #define ADC_PIN A0
  #define LORA_RST_PIN 49
  #define LORA_SS_PIN 53
  #define PULSE_ACTIVE LOW
  #define PULSE_MODE CHANGE
  #define PULSE_PIN 20
  #define UART_SPEED 9600
#endif /* __AVR_ATmega2560__ */

#define PC_ICR_REG *digitalPinToPCICR(PULSE_PIN)
#define PC_ICR_BIT digitalPinToPCICRbit(PULSE_PIN)
#define PC_MSK_REG *digitalPinToPCMSK(PULSE_PIN)
#define PC_MSK_BIT digitalPinToPCMSKbit(PULSE_PIN)

#if defined(__AVR_ATtiny84__)
  #define PCINT_FORCE

  #if (PC_ICR_BIT == PCIE0)
    #define PULSE_PCINT_VECT PCINT0_vect
  #elif (PC_ICR_BIT == PCIE1)
    #define PULSE_PCINT_VECT PCINT1_vect
  #endif /* PC_ICR_BIT -> PCINTx_vect */
#endif /* __AVR_ATtiny84__ */

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__)
  #if (PC_ICR_BIT == 0)
    #define PULSE_PCINT_VECT PCINT0_vect
  #elif (PC_ICR_BIT == 1)
    #define PULSE_PCINT_VECT PCINT1_vect
  #elif (PC_ICR_BIT == 2)
    #define PULSE_PCINT_VECT PCINT2_vect
  #endif /* PC_ICR_BIT -> PCINTx_vect */
#endif /* __AVR_ATmega328P__ || __AVR_ATmega2560__ */

#if defined(PCINT_FORCE) || (digitalPinToInterrupt(PULSE_PIN) == NOT_AN_INTERRUPT)
  #define PULSE_PCINT PULSE_PCINT_VECT
#endif /* PCINT vs INT */

volatile uint32_t pulse_cnt = 0;
volatile uint8_t pulse_status = LOW;
#if defined(PULSE_PCINT)
  volatile uint8_t last_pulse_status = LOW;
#endif /* PULSE_PCINT */

SX127x LoRa;

#if defined(CRC16_ENABLE)
  uint16_t crc16(const uint8_t *data, uint8_t len)
  {
    uint16_t crc = 0xFFFF;
    uint8_t i;

    while (len--)
    {
      crc ^= *data++;

      for (i = 0; i < 8; i++)
      {
        if ((crc & 0x01) != 0)
        {
          crc >>= 1;
          crc ^= 0xA001;
        }
        else
        {
          crc >>= 1;
        }
      }
    }

    return crc;
  }
#endif /* CRC16_ENABLE */

static inline uint16_t adc_read(void)
{
  #if defined(ADC_PIN)
    analogReference(DEFAULT);
    return (uint16_t) analogRead(ADC_PIN);
  #else
    return 0;
  #endif /* ADC_PIN */
}

static inline uint16_t temp_read(void)
{
  #if defined(ADC_TEMPERATURE)
    analogReference(INTERNAL);
    return (uint16_t) analogRead(ADC_TEMPERATURE);
  #else
    return 0;
  #endif /* ADC_TEMPERATURE */
}

void calc_data(uint8_t pkt[PKT_SIZE])
{
  #if defined(CRC16_ENABLE)
    uint16_t crc;
  #endif /* CRC16_ENABLE */
  uint32_t misc;

  misc = (adc_read() & ADC_MASK) << VCC_SHIFT;
  misc |= (temp_read() & ADC_MASK) << TEMP_SHIFT;
  if (pulse_status == PULSE_ACTIVE)
  {
    misc |= PULSE_BIT;
  }

  /* Header */
  pkt[0] = 'L';
  pkt[1] = 'A';

  /* Pulse Counter */
  pkt[2] = ((pulse_cnt >> 24) & 0xFF);
  pkt[3] = ((pulse_cnt >> 16) & 0xFF);
  pkt[4] = ((pulse_cnt >> 8) & 0xFF);
  pkt[5] = (pulse_cnt & 0xFF);

  /* Misc */
  pkt[6] = ((misc >> 16) & 0xFF);
  pkt[7] = ((misc >> 8) & 0xFF);
  pkt[8] = (misc & 0xFF);

  /* CRC16 */
  #if defined(CRC16_ENABLE)
    crc = crc16(pkt, DATA_SIZE);
    pkt[9] = ((crc >> 8) & 0xFF);
    pkt[10] = (crc & 0xFF);
  #endif /* CRC16_ENABLE */
}

void send_data(void)
{
  uint8_t pkt[PKT_SIZE];

  calc_data(pkt);

  #if defined(UART_ENABLE)
    for (uint8_t i = 0; i < PKT_SIZE; i++)
    {
      char buf[3];
      sprintf(buf, "%02X", pkt[i]);
      Serial.print(buf);
    }
    Serial.println();
    Serial.flush();
  #endif /* UART_ENABLE */

  #if defined(LORA_ENABLE)
    if (LoRa.begin(LORA_SS_PIN, LORA_RST_PIN, -1, -1, -1))
    {
      LoRa.setFrequency(LORA_FREQ);
      LoRa.setTxPower(LORA_TX_POWER, LORA_TX_PA_PIN);

      LoRa.setSpreadingFactor(LORA_SPREAD_FACTOR);
      LoRa.setBandwidth(LORA_BW);
      LoRa.setCodeRate(LORA_CODE_RATE);

      LoRa.setHeaderType(LORA_HDR_TYPE);
      LoRa.setPreambleLength(LORA_PREAMBLE_LEN);
      LoRa.setPayloadLength(PKT_SIZE);
      LoRa.setCrcEnable(LORA_CRC_ENABLE);

      LoRa.setSyncWord(LORA_SYNC_WORD);

      LoRa.beginPacket();
      LoRa.write(pkt, PKT_SIZE);
      LoRa.endPacket();
      LoRa.wait();

      #if defined(UART_ENABLE)
        Serial.print("LoRa TX Time: ");
        Serial.print(LoRa.transmitTime());
        Serial.println(" ms");
        Serial.flush();
      #endif /* UART_ENABLE */

      LoRa.end();
    }
    else
    {
      #if defined(UART_ENABLE)
        Serial.println("LoRa error");
        Serial.flush();
      #endif /* UART_ENABLE */
    }
  #endif /* LORA_ENABLE */
}

#if defined(PULSE_PCINT)
  ISR(PULSE_PCINT)
  {
    pulse_status = digitalRead(PULSE_PIN);

  #if (PULSE_MODE == RISING)
    if ((pulse_status != last_pulse_status) && (pulse_status == HIGH))
  #elif (PULSE_MODE == FALLING)
    if ((pulse_status != last_pulse_status) && (pulse_status == LOW))
  #else /* CHANGE */
    if (pulse_status != last_pulse_status)
  #endif /* PULSE_MODE */
    {
      if (pulse_status == PULSE_ACTIVE)
      {
        pulse_cnt++;
      }
    }

    last_pulse_status = pulse_status;
  }
#else
  void pulse_ISR(void)
  {
    pulse_status = digitalRead(PULSE_PIN);
    if (pulse_status == PULSE_ACTIVE)
    {
      pulse_cnt++;
    }
  }
#endif /* PCINT vs INT */

void setup(void)
{
  #if defined(__AVR_ATtiny84__)
    pinMode(PIN_PA0, INPUT_PULLUP);
    pinMode(PIN_PA1, INPUT_PULLUP);
    pinMode(PIN_PA2, INPUT_PULLUP);
    pinMode(PIN_PA3, INPUT_PULLUP);
    pinMode(PIN_PA4, INPUT_PULLUP);
    pinMode(PIN_PA5, INPUT_PULLUP);
    pinMode(PIN_PA6, INPUT_PULLUP);
    pinMode(PIN_PA7, INPUT_PULLUP);
    pinMode(PIN_PB0, INPUT_PULLUP);
    pinMode(PIN_PB1, INPUT_PULLUP);
    pinMode(PIN_PB2, INPUT_PULLUP);
    pinMode(PIN_PB3, INPUT_PULLUP);
  #endif /* __AVR_ATtiny84__ */

  pinMode(PULSE_PIN, INPUT_PULLUP);
  pulse_status = digitalRead(PULSE_PIN);
  #if defined(PULSE_PCINT)
    last_pulse_status = pulse_status;
  #endif /* PULSE_PCINT */

  #if defined(ADC_PIN)
    pinMode(ADC_PIN, INPUT);
  #endif /* ADC_PIN */

  #if defined(UART_ENABLE)
    Serial.begin(UART_SPEED);
    Serial.println("Booting...");
    Serial.flush();
  #endif /* UART_ENABLE */

  #if defined(PULSE_PCINT)
    PC_MSK_REG |= (1 << PC_MSK_BIT);
    PC_ICR_REG |= (1 << PC_ICR_BIT);
  #else
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), pulse_ISR, PULSE_MODE);
  #endif /* PCINT vs INT */

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  power_all_disable();
}

void loop(void)
{
  /* Ready */
  power_adc_enable();

  #if defined(LORA_ENABLE)
    #if defined(__AVR_ATmega328P__)
      power_timer0_enable();
      power_spi_enable();
    #endif /* __AVR_ATmega328P__ */

     #if defined(__AVR_ATmega2560__)
      power_timer0_enable();
      power_spi_enable();
    #endif /* __AVR_ATmega2560__ */

    #if defined(__AVR_ATtiny84__)
      power_timer0_enable();
      power_usi_enable();
    #endif /* __AVR_ATtiny84__ */
  #endif /* LORA_ENABLE */

  #if defined(UART_ENABLE)
    #if defined(__AVR_ATmega328P__)
      power_usart0_enable();
    #endif /* __AVR_ATmega328P__ */

     #if defined(__AVR_ATmega2560__)
      power_usart0_enable();
    #endif /* __AVR_ATmega2560__ */
  #endif /* UART_ENABLE */

  /* Send */
  send_data();

  /* Sleep */
  power_all_disable();
  sleep_mode();
}
