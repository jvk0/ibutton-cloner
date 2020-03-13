#include <Wire.h>

#include <OneWire.h>

#define BATRON_FIX
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Clcd.h>

// Pins
const int RANDOM_SEED_PIN = 14;
const int OW_DATA_PIN     = 10;
const int POR_PIN         = 12;
const int LED_PIN         = 8;
const int BTN1_PIN        = 2;
const int BTN2_PIN        = 3;

// Serial configuration
const int BAUDRATE = 9600;

// OneWire configuration
const int OW_TREC = 20;

OneWire rw1990(OW_DATA_PIN, OW_TREC);

// Display configuration
#define LCD_INTRO

const int I2C_ADDR = 0x3B;
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

hd44780_I2Clcd lcd(I2C_ADDR);

// Global variables
static volatile int btn1_state = 0;
static volatile int btn2_state = 0;
static byte id[8]              = {0x00};

// Function prototypes
void btn1_isr();
void btn2_isr();
inline void clear_btn_state();
inline void led_on(int pin);
inline void led_off(int pin);
void clear_cols(hd44780_I2Clcd &lcd, int start, int ncols);
void set_random_id(byte* id);
inline void clear_id(byte* id);
int cmp_id(const byte* id1, const byte* id2);
void write_byte_rw1990(byte data);
void ibutton_read();
void ibutton_write();
void ibutton_random();

// *** Setup and main loop *************************************
void setup()
{
  // Serial setup
  Serial.begin(BAUDRATE);
  Serial.println(F("*** iButton Cloner v7.11 ***"));

  // Display setup
  Serial.println(F("Display setup..."));
  pinMode(POR_PIN, OUTPUT);
  digitalWrite(POR_PIN, HIGH);
  delay(5);
  digitalWrite(POR_PIN, LOW);
  delay(250);
  lcd.begin(LCD_COLS, LCD_ROWS);
  clear_cols(lcd, 0, LCD_COLS * LCD_ROWS);

  //Button pin setup
  Serial.println(F("Button pin setup..."));
  pinMode(BTN1_PIN, INPUT);
  pinMode(BTN2_PIN, INPUT);

  // Button ISR setup
  Serial.println(F("Button ISR setup..."));
  attachInterrupt(digitalPinToInterrupt(BTN1_PIN), btn1_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN2_PIN), btn2_isr, FALLING);

  //Random seed pin setup
  Serial.println(F("Random seed pin setup..."));
  pinMode(RANDOM_SEED_PIN, INPUT);

  Serial.println(F("Setup finished!"));

  #ifdef LCD_INTRO
    lcd.print(F("iButton Cloner"));
    lcd.setCursor(LCD_COLS * LCD_ROWS - 5, 0);
    lcd.print(F("v7.11"));
    delay(2000);
    clear_cols(lcd, 0, LCD_COLS * LCD_ROWS);
  #endif
}

void loop()
{
   if (btn1_state)
    ibutton_write();
  else if (btn2_state)
    ibutton_random();
  else
    ibutton_read();
}

// *** Functions ***********************************************

void btn1_isr()
{
  if (id[0])
    btn1_state = 1;
}

void btn2_isr()
{
  btn2_state = 1;
}

inline void clear_btn_state()
{
  btn1_state = btn2_state = 0;
}

inline void led_on(int pin)
{
  digitalWrite(pin, HIGH);
}

inline void led_off(int pin)
{
  digitalWrite(pin, LOW);
}

void clear_cols(hd44780_I2Clcd &lcd, int start, int ncols)
{
    // Requires line wrap! (Default: ON)
    lcd.setCursor(start, 0);
    for (int c = 0; c < ncols; c++) {
      lcd.print(' ');
    }
  // Return to start position
  lcd.setCursor(start, 0);
}

void set_random_id(byte* id)
{
  randomSeed(analogRead(RANDOM_SEED_PIN) ^ millis());

  id[0] = 1; // Family Code
  for (int i = 1; i < 7; i++)
    id[i] = random(0xFF + 1);

  id[7] = rw1990.crc8(id, 7); // New CRC
}

inline void clear_id(byte* id)
{
  for (int i = 0; i < 8; i++)
    id[i] = 0;
}

int cmp_id(const byte* id1, const byte* id2)
{
  for (int i = 0; i < 8; i++) {
    if (id1[i] != id2[i])
      return 0;
  }
  return 1;
}

void write_byte_rw1990(byte data)
{
  for (int i = 0; i < 8; i++) {
    rw1990.write_bit_rw1990(data);
    data = data >> 1;
  }
}

void ibutton_read()
{
  clear_cols(lcd, 0, LCD_COLS * LCD_ROWS);
  lcd.print(F("ID:"));

  // Print last ID if not empty
  if (id[0]) {

    // CRC indicator position
    lcd.setCursor(LCD_COLS - 2, 0);

    // Bad CRC warning
    if (id[7] == rw1990.crc8(id, 7)) {
      lcd.print(F(":)"));
      Serial.print(F("[CRC: OK] "));
    } else {
      lcd.print(F(":("));
      Serial.print(F("[CRC: ERROR] "));
    }

    // Output ID bytes
    Serial.print(F("ID: "));
    for (int i = 7; i >= 0; i--) { // MSB first (CRC code)
      Serial.print(id[i], HEX);
      Serial.print(' ');
      lcd.print(id[i], HEX);
    }
    // Serial newline
    Serial.print('\n');
  }

  // Will block
  while (!rw1990.search(id)){
    rw1990.reset_search();

    // Change mode
    if (btn1_state || btn2_state)
      return;

    delay(200);
  }
}

void ibutton_write()
{
    byte id_after[8];

    Serial.println(F("Write new ID"));
    clear_cols(lcd, 0, LCD_COLS * LCD_ROWS);
    lcd.print(F("Write new ID"));

    // Check CRC
    #ifndef ALLOW_BAD_CRC
      if (id[7] != rw1990.crc8(id, 7)) {
        Serial.println(F("[CRC: ERROR] Bad CRC! Write aborted!"));
        lcd.setCursor(LCD_COLS, 0); // Set cursor to the second line
        lcd.print(F("Bad CRC!"));
        delay(3000);
        clear_id(id);
        clear_btn_state();
        return;
      }
    #endif

    Serial.println(F("Waiting for iButton..."));
    lcd.setCursor(LCD_COLS, 0); // Set cursor to the second line
    lcd.print(F("Waiting..."));

    clear_btn_state();
    // Wait for iButton
    while (!rw1990.reset()) {
      // Abort
      if (btn1_state || btn2_state) {
        Serial.println(F("Write aborted!"));
        led_off(LED_PIN);
        clear_btn_state();
        return;
      } // if
      led_on(LED_PIN);
      delay(500);
      led_off(LED_PIN);
      delay(500);
    } // while

    Serial.println(F("Writing new ID..."));
    clear_cols(lcd, LCD_COLS, LCD_COLS); // Clear the second line
    lcd.print(F("Writing..."));
    led_on(LED_PIN); // Writing in progress

    do {
      Serial.println(F("Trying..."));

      // Start ID write
      while(!rw1990.reset()) // In case device is not ready
        delay(100);
      rw1990.write(0xD1);
      rw1990.write_bit_rw1990(1);

      // Write ID
      rw1990.reset();
      rw1990.write(0xD5);
      for (int i = 0; i < 8; i++)
        write_byte_rw1990(id[i]);

      // End ID write
      rw1990.reset();
      rw1990.write(0xD1);
      rw1990.write_bit_rw1990(0);

      // Bus reset for address search
      rw1990.reset();
      // Read new ID
      while (!rw1990.search(id_after))
        delay(100);

    } while (!cmp_id(id, id_after)); // Repeat until successful

    led_off(LED_PIN); // Reset LED

    Serial.println(F("Done!"));
    clear_cols(lcd, LCD_COLS, LCD_COLS); // Clear the second line
    lcd.print(F("Done!"));

    delay(3000);
    clear_id(id);
    clear_btn_state();
}

void ibutton_random()
{
  set_random_id(id);
  Serial.println(F("Random ID set!"));
  clear_btn_state();
}
