
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h> // https://github.com/adafruit/Adafruit-PN532
#include <LowPower.h> // https://github.com/rocketscream/Low-Power
#include <avr/wdt.h>

#include <EEPROMex.h> // https://github.com/thijse/Arduino-EEPROMEx
#include <EEPROMVar.h>


// Interrupt pin https://www.arduino.cc/en/Reference/attachInterrupt
#define PIN_BUTTON 2

#define PIN_NPN_NFC 4
#define PIN_NPN_VIBRO 3

#define PIN_LED_GREEN 5
#define PIN_LED_RED 6
#define PIN_LED_BLUE 11
#define PIN_LED_YELLOW 9

// Connection from NFC module V3 I2C
//A4 - SDA
//A5 - SCL
#define PIN_PN532_IRQ   12
#define PIN_PN532_RESET 10

// -----------------------

#define PN532_PACKBUFFSIZ 64
byte _pn532_packetbuffer[PN532_PACKBUFFSIZ];
Adafruit_PN532 nfc(PIN_PN532_IRQ, PIN_PN532_RESET);

// -----------------------

#define VIBRO_LEVEL 200

#define MILISEC_NFC 500
#define MILISEC_LED_AFTER_NFC 3000
#define MILISEC_LED_BLINK 250
#define MILISEC_VIBRO 300
#define MILISEC_VIBRO_DELAY 300

#define MILISEC_DEEP_SLEEP 30000
#define MILISEC_LONG_PRESS 3000

#define COUNT_LED 4
#define COUNT_NFC_BITS 4

#define IS_DEBUG_SERIAL 0
#define IS_DEBUG_NFC 0
#define IS_DEBUG_LED 0

#define SERIAL_BAUD 9600

//Statistics
#define RESET_EEPROM_STATS 0
EEPROMVar<int> _ee_count_short_press(0);
EEPROMVar<int> _ee_count_long_press(0);
EEPROMVar<int> _ee_count_led_green(0);
EEPROMVar<int> _ee_count_led_red(0);
EEPROMVar<int> _ee_count_led_blue(0);
EEPROMVar<int> _ee_count_led_yellow(0);

int _milisec_since_button_down = 0;
bool _is_button_down = false;
bool _is_button_reset = false;

int* _arr_milisec_since_nfc = new int[COUNT_LED] {
  MILISEC_LED_AFTER_NFC,
  MILISEC_LED_AFTER_NFC,
  MILISEC_LED_AFTER_NFC,
  MILISEC_LED_AFTER_NFC };

int _led_blink_id = -1;
int _led_blink_timeout = 0;
bool _led_blink_status = true;

int* _arr_leds = new int[COUNT_LED] {
  PIN_LED_GREEN,
  PIN_LED_RED,
  PIN_LED_BLUE,
  PIN_LED_YELLOW
};

bool* _arr_status_led = new bool[COUNT_LED] {
  false,
  false,
  false,
  false
};

uint8_t _arr_uid[COUNT_LED][COUNT_NFC_BITS];

void sleep(int milisec) {
  period_t period_sleep = SLEEP_1S;

  if(milisec <= 120)
  {
    period_sleep = SLEEP_120MS;
  }
  else if(milisec <= 250)
  {
    period_sleep = SLEEP_250MS;
  }
  else if(milisec <= 500)
  {
    period_sleep = SLEEP_500MS;
  }
  LowPower.powerStandby(period_sleep, ADC_OFF, BOD_OFF);
}

void setup(void)
{
    for(int i = 0; i < COUNT_LED; i++)
    {
      uint8_t* id;
      switch (i) {
          case 0: //PIN_LED_GREEN
            id = new uint8_t[COUNT_NFC_BITS] { 0x0, 0xEE, 0x39, 0x83 };
            break;
          case 1: //PIN_LED_RED
            id = new uint8_t[COUNT_NFC_BITS] { 0xD0, 0xDB, 0x33, 0x83 };
            break;
          case 2: //PIN_LED_BLUE
            id = new uint8_t[COUNT_NFC_BITS] { 0x70, 0x24, 0x31, 0x83 };
            break;
          case 3: //PIN_LED_YELLOW
            id = new uint8_t[COUNT_NFC_BITS] { 0xF0, 0xFE, 0x39, 0x52 };
            break;
      }
      for(int j = 0; j < COUNT_NFC_BITS; j++)
      {
        _arr_uid[i][j] = id[j];
      }
    }

    pinMode(PIN_BUTTON, INPUT_PULLUP);

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);

    pinMode(PIN_NPN_NFC, OUTPUT);
    pinMode(PIN_NPN_VIBRO, OUTPUT);

    EEPROM.setMemPool(0, EEPROMSizeATmega328);
    EEPROM.setMaxAllowedWrites(1e6);

    _ee_count_short_press.restore();
    _ee_count_long_press.restore();
    _ee_count_led_green.restore();
    _ee_count_led_red.restore();
    _ee_count_led_blue.restore();
    _ee_count_led_yellow.restore();

    if(RESET_EEPROM_STATS) {
      _ee_count_short_press = 0;
      _ee_count_long_press = 0;
      _ee_count_led_green = 0;
      _ee_count_led_red = 0;
      _ee_count_led_blue = 0;
      _ee_count_led_yellow = 0;
      save_eeprom();
    }

    if(IS_DEBUG_SERIAL)
    {
      Serial.begin(SERIAL_BAUD);
      Serial.println("started");

      Serial.println("stats: ");
      Serial.println(_ee_count_short_press);
      Serial.println(_ee_count_long_press);
      Serial.println(_ee_count_led_green);
      Serial.println(_ee_count_led_red);
      Serial.println(_ee_count_led_blue);
      Serial.println(_ee_count_led_yellow);
      Serial.print("---");

      Serial.end();
    }

    digitalWrite(PIN_LED_RED, HIGH);
    delay(100);
    digitalWrite(PIN_LED_RED, LOW);
}

int read_nfc() {
  int result = 0;

  digitalWrite(PIN_NPN_NFC, HIGH);
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata)
  {
    if(IS_DEBUG_NFC)
    {
      Serial.print("Didn't find PN53x board");
    }
  }
  else
  {
      if(IS_DEBUG_NFC)
      {
        Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
        Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
        Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
      }

      // Set the max number of retry attempts to read from a card
      // This prevents us from waiting forever for a card, which is
      // the default behaviour of the PN532.
      nfc.setPassiveActivationRetries(0xFF);

      // configure board to read RFID tags
      nfc.SAMConfig();

      if(IS_DEBUG_NFC)
      {
        Serial.println("Waiting for an ISO14443A card");
      }

      //Inquiry card
      _pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
      _pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
      _pn532_packetbuffer[2] = PN532_MIFARE_ISO14443A;
      if (nfc.sendCommandCheckAck(_pn532_packetbuffer, 3, 10))
      {
          delay(10);
          //Check if existing
          if(digitalRead(PIN_PN532_IRQ) == LOW)
          {
              // read data packet
              nfc.readdata(_pn532_packetbuffer, 20);

              uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
              uint8_t uidLength = _pn532_packetbuffer[12];       // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
              for (uint8_t i = 0; i < uidLength; i++)
              {
                  uid[i] = _pn532_packetbuffer[13+i];
              }

              // timeout 50ms * 20 = 1 second
              //success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength, 100);

              if(IS_DEBUG_NFC)
              {
                  Serial.println("Found a card!");
                  Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
                  Serial.print("UID Value: ");
                  for (uint8_t i=0; i < uidLength; i++)
                  {
                    Serial.print(" 0x");Serial.print(uid[i], HEX);
                    if(i < uidLength -1)
                    {
                      Serial.print(",");
                    }
                  }
                  Serial.println("");
              }

              //Check ID
              for(int i = 0; i < COUNT_LED; i++)
              {
                  bool is_found = true;
                  for(int j = 0; j < COUNT_NFC_BITS; j++)
                  {
                      if(_arr_uid[i][j] != uid[j])
                      {
                        is_found = false;
                        break;
                      }
                  }

                  if(is_found)
                  {
                      if(i == 0) {
                        _ee_count_led_green++;
                      } else if(i == 1) {
                        _ee_count_led_red++;
                      } else if(i == 2) {
                        _ee_count_led_blue++;
                      } else if(i == 3) {
                        _ee_count_led_yellow++;
                      }

                      result = i + 1;

                      if(IS_DEBUG_NFC)
                      {
                        Serial.print("Found: ");
                        Serial.println(result);
                      }

                      break;
                  }
              }

          }
          else
          {
            if(IS_DEBUG_NFC)
            {
              Serial.println("No card present");
            }
          }
      }
      else
      {
          if(IS_DEBUG_NFC)
          {
            Serial.println("Error: PN532_COMMAND_INLISTPASSIVETARGET");
          }
      }
  }

  digitalWrite(PIN_NPN_NFC, LOW);

  return result;
}

void vibrate(int count)
{
  if(IS_DEBUG_SERIAL)
  {
    Serial.print("vibrate: ");
    Serial.println(count);
  }

  for(int i = 0; i < count; i++)
  {
    if(IS_DEBUG_LED)
      digitalWrite(PIN_LED_GREEN, HIGH);

    analogWrite(PIN_NPN_VIBRO, VIBRO_LEVEL);
    //digitalWrite(PIN_NPN_VIBRO, HIGH);
    //LowPower.powerSave(SLEEP_120MS, ADC_OFF, BOD_OFF, TIMER2_OFF);
    delay(125);
    analogWrite(PIN_NPN_VIBRO, LOW);

    if(IS_DEBUG_LED)
      digitalWrite(PIN_LED_GREEN, LOW);

    if(i < count -1)
    {
      delay(125);
      //LowPower.powerSave(SLEEP_120MS, ADC_OFF, BOD_OFF, TIMER2_OFF);
    }
  }
}

bool update_leds(int milisec_sleep)
{
  bool is_led_on = false;
  for(int i = 0; i < COUNT_LED; i++)
  {
    if(_arr_milisec_since_nfc[i] < MILISEC_LED_AFTER_NFC)
    {
      _arr_milisec_since_nfc[i] += milisec_sleep;

      is_led_on = true;
      if(IS_DEBUG_SERIAL)
      {
        Serial.print(i);
        Serial.println(" led_on");
      }
    }
    int ledIntensity = 5;

    switch (_arr_leds[i]) {
      case PIN_LED_GREEN:
        ledIntensity = 50;
        break;
      case PIN_LED_RED:
          ledIntensity = 10;
          break;
      case PIN_LED_YELLOW:
          ledIntensity = 5;
          break;
      case PIN_LED_BLUE:
          ledIntensity = 50;
          break;
    }

    if(_led_blink_id == _arr_leds[i])
    {
      if(!_led_blink_status)
      {
        ledIntensity = LOW;
      }
    }

    analogWrite(_arr_leds[i], (_arr_milisec_since_nfc[i] < MILISEC_LED_AFTER_NFC) ? ledIntensity : LOW);
  }
  return is_led_on;
}

void show_status_leds() {
  for(int i = 0; i < COUNT_LED; i++)
  {
    if(_arr_status_led[i])
    {
      _arr_milisec_since_nfc[i] = 0;
    }
  }
}

void check_button()
{
  if(digitalRead(PIN_BUTTON) == LOW)
  {
    if(!_is_button_down)
    {
      if(IS_DEBUG_SERIAL)
        Serial.println("button_down");

      _is_button_down = true;
      _is_button_reset = false;
      _milisec_since_button_down = 0;
    }
    else
    {
      if(_milisec_since_button_down >= MILISEC_LONG_PRESS)
      {
        //Reset leds
        for(int i = 0; i < COUNT_LED; i++)
        {
          _arr_status_led[i] = false;
        }

        vibrate(2);
        _is_button_reset = true;
        _ee_count_long_press++;

        _milisec_since_button_down = 0;
      }
    }
  }
  else
  {
    if(_is_button_down)
    {
      _is_button_down = false;
      if(!_is_button_reset)
      {
        if(IS_DEBUG_SERIAL)
        {
          Serial.print("button_up: ");
          Serial.println(_milisec_since_button_down);
        }

        vibrate(1);
        show_status_leds();
        _ee_count_short_press++;

      }

      _is_button_reset = false;
    }
  }
}

/*
MAIN LOOP
*/
void loop() {

  if(IS_DEBUG_SERIAL)
  {
    Serial.begin(SERIAL_BAUD);
  }

  check_button();

  int led_nfc = 0;
  int period_sleep = 250;
  int milisec_sleep = period_sleep;

  bool is_led_on = update_leds(milisec_sleep);
  if(!is_led_on)
  {
      _led_blink_id = -1;
      led_nfc = read_nfc();

      milisec_sleep = 500 + MILISEC_NFC;
  }

  if(led_nfc)
  {
    _arr_status_led[led_nfc-1] = true;

    _led_blink_id = _arr_leds[led_nfc-1];
    _led_blink_status = true;
    _led_blink_timeout = 0;

    show_status_leds();
  }

  check_button();

  _milisec_since_button_down += milisec_sleep;

  if(_led_blink_id >= 0)
  {
      _led_blink_timeout += milisec_sleep;
      if(_led_blink_timeout >= MILISEC_LED_BLINK) {
        _led_blink_timeout = 0;
        _led_blink_status = !_led_blink_status;
      }
  }

  if(led_nfc)
  {
    vibrate(1);
  }

  check_button();

  if(IS_DEBUG_SERIAL)
    Serial.end();

  //INT0 = PIN 2
  attachInterrupt(0, wakeUpVector, LOW);
  if(!is_led_on && _milisec_since_button_down >= MILISEC_DEEP_SLEEP)
  {
      save_eeprom();

      // Deep sleep
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
  else
  {
      //Analog to Digital Converter (ADC)
      //Brown out detection (BOD)
      if(!is_led_on)
      {
          LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
      }
      else
      {
        delay(period_sleep);
      }
      //LowPower.idle(period_sleep, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
  }
  detachInterrupt(0);

  //Restore state
  digitalWrite(PIN_NPN_VIBRO, LOW);
  digitalWrite(PIN_NPN_NFC, LOW);
  update_leds(0);
}

void save_eeprom() {
  _ee_count_short_press.save();
  _ee_count_long_press.save();
  _ee_count_led_green.save();
  _ee_count_led_red.save();
  _ee_count_led_blue.save();
  _ee_count_led_yellow.save();
}

void  wakeUpVector() {
    check_button();
}
