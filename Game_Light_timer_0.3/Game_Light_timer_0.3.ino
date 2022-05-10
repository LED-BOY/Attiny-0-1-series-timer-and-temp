/*LedBoyAdventures with Clock for LEDBOY and any Attiny series 1 compatible using neopixels
  Flash CPU Speed 5MHz.
  this code is released under GPL v3, you are free to use and modify.
  released on 2022.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    To contact us: ledboy.net
    ledboyconsole@gmail.com
*/
#include "tinyOLED.h"
#include "sprites.h"
#include "digits.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>

#if defined(MILLIS_USE_TIMERA0) || defined(MILLIS_USE_TIMERB0)
#error "This sketch does't use TCA0 nor TCB0 to save flash, select Disabled."
#endif

#define LEDON PORTA.OUT &= ~PIN3_bm;// led ON, also resets the oled screen.
#define LEDOFF PORTA.OUT |= PIN3_bm;// led OFF
#define BUTTONLOW !(PORTA.IN & PIN6_bm)// button press low
#define BUTTONHIGH (PORTA.IN & PIN6_bm)// button not pressed high
#define wdt_reset() __asm__ __volatile__ ("wdr"::) // watchdog reset macro

uint8_t seconds = 0;
uint8_t minutes = 0;
uint8_t hours = 0;
uint8_t lastMinute = 0;
uint8_t timerAlarm = 0;
uint8_t sleepTimer = 0;
uint16_t timer = 0;
uint16_t timer2 = 0;
uint16_t buttonTimer = 0;
bool ledState = true;
volatile uint16_t interruptTimer = 0;


void setup() {
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_4KCLK_gc);// enable watchdog 1 sec. aprox.
  PORTA.DIR = 0b10000110; // setup ports in and out
  PORTA.PIN6CTRL = PORT_PULLUPEN_bm;// button pullup

  TCB0.INTCTRL = TCB_CAPT_bm; // Setup Timer B as compare capture mode to trigger interrupt
  TCB0_CCMP = 5000;// CLK
  TCB0_CTRLA = (1 << 1) | TCB_ENABLE_bm;

  // We will be outputting PWM on PA3 on an 8-pin part PA3 - TCA0 WO0, pin 4 on 8-pin parts
  PORTMUX.CTRLC     = PORTMUX_TCA00_ALTERNATE_gc; // turn off PORTMUX, returning WO0 PORTMUX_TCA00_DEFAULT_gc for PA3. PORTMUX_TCA00_ALTERNATE_gc; for PA7
  takeOverTCA0();                               // this replaces disabling and resettng the timer, required previously.
  TCA0.SINGLE.CTRLB = (TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc); // Single slope PWM mode, PWM on WO0
  TCA0.SINGLE.PER   = 255;                    // Count all the way up to (255) - 8-bit PWM. At 5MHz, this gives ~19.607kHz PWM
  TCA0.SINGLE.CMP0  = 115; // 45% duty cycle
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm; // enable the timer with no prescaler

  sei(); // enable interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);// configure sleep mode

  if (readSupplyVoltage() > 3800 && BUTTONHIGH) { // If voltage is above 3.8v the led is on to increase Amp consumption and lower batt.
    PORTA.DIR = (1 << 3);// pin3 (LED) as output
  }

  if (BUTTONHIGH) goToSleep(); // if button is high it is assumed that the watchdog bite and there is no intention to wake up.

  PORTA.DIR = (1 << 3); // pin3 (LED) as output

  for (uint8_t x = 4; x < 40; x += 34) { // led PWM and led blinking
    buttonDebounce();
    while (BUTTONHIGH) {

      if (interruptTimer % x == 0) {

        if (readSupplyVoltage() < 2300) goToSleep(); // To preserve the LIC to stay above 2.2v no operation is allowed under 2.4v
        LEDON
      } else {
        LEDOFF
      }
    }
    LEDOFF
  }
  oled.begin();// start oled screen
  buttonDebounce();
  sleepTimer = 0;
}

void loop() {
  displayInfo();

  while (BUTTONLOW) { // reset time, timer select, and turn off
    if ((interruptTimer - timer2) > 200) {
      timer2 = interruptTimer;
      sleepTimer++;
      seconds = 0;
      minutes = 0;
      hours = 0;
      timerAlarm += 5;

      if (timerAlarm > 30) timerAlarm = 0;

      if (sleepTimer >= 8) {
        clearScreen();
        oled.ssd1306_send_command(0xAE);
        goToSleep();
      }
    }
  }
  sleepTimer = 0; // if button is high sleep timer resets
}

void displayInfo(void) {
  if ((interruptTimer - timer) > 200) {

    if (readSupplyVoltage() < 2300) goToSleep(); // To preserve the LIC to stay above 2.2v no operation is allowed under 2.4v

    if (timerAlarm == minutes && timerAlarm != 0) { // if the alarm is in on state the led is turn on, if the button is pressed it goes to sleep.
      while (BUTTONHIGH) {

        if (interruptTimer % 4 == 0) {
          LEDON
        } else {
          LEDOFF
        }
      }
      LEDOFF
      goToSleep();
    }

    int8_t voltageReading = map(readSupplyVoltage(), 2600, 3800, 0, 117);

    if (voltageReading < 0) voltageReading = 0;
    oled.drawLine(10, 0, voltageReading, 0b00011110); // draws voltage meter
    drawSprite(0, 0, battIcon, false);
    timer = interruptTimer;
    drawSprite(18, 1, tempSimbol, false);

    if (lastMinute != minutes) {
      lastMinute = minutes;
      drawDecimal (0, readTemp());
    }
    drawDecimal (32, hours);
    oled.drawLine (52, 2, 1, 0b00010001);
    drawDecimal (56, minutes);
    oled.drawLine (76, 2, 1, 0b00010001);
    drawDecimal (82, seconds);
    drawDecimal (110 , timerAlarm);
    timer = interruptTimer;
  }
}

void drawDecimal (uint8_t firstPixel, uint8_t value) {// this function takes the digit and value from characters.h and draws it without the 0 to the left in hours
  uint8_t valueUnits = value;                      // always draws 2 digits.

  if (value < 10) {
    drawSprite(firstPixel, 1, numbers[0], true);
    drawSprite((firstPixel + 8), 1, numbers[value], true);
  } else {
    value /= 10; // some math to substract the 0 from the left in hours digits
    drawSprite(firstPixel, 1, numbers[value], true);
    value *= 10;

    for (uint8_t x = 0; x < value ; x++) {
      valueUnits--;
    }
    drawSprite((firstPixel + 8), 1, numbers[valueUnits], true);
  }
}

void clearScreen (void) {
  for ( uint8_t x = 0; x < 4; x++) {
    oled.drawLine(0, x, 128, 0x00);
  }
}


void drawSprite (uint8_t column, uint8_t page, uint8_t sprite[], bool digit) {
  oled.setCursor(column, page);// position cursor column - page
  oled.ssd1306_send_data_start();

  for (uint8_t x = 0; x < 8; x++) {
    oled.ssd1306_send_data_byte(sprite[x]);
  }
  oled.ssd1306_send_data_stop();

  if (digit) {
    oled.setCursor(column, (page + 1));
    oled.ssd1306_send_data_start();

    for (uint8_t x = 8; x < 16; x++) {
      oled.ssd1306_send_data_byte(sprite[x]);
    }
    oled.ssd1306_send_data_stop();
  }
}

void buttonDebounce(void) {
  buttonTimer = interruptTimer;
  while (BUTTONLOW || (interruptTimer - buttonTimer) < 200); // super simple button debounce
}

void goToSleep (void) {
  PORTA.PIN6CTRL  |= PORT_ISC_BOTHEDGES_gc; //attach interrupt to portA pin 3 keeps pull up enabled
  //_PROTECTED_WRITE(WDT.CTRLA, 0);
  TCA0.SPLIT.CTRLA = 0; //disable TCA0 and set divider to 1
  TCA0.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03; //set CMD to RESET to do a hard reset of TCA0.
  PORTA.OUT |= PIN7_bm;// P CHANNEL mosfet High to deactivate
  sleep_enable();
  sleep_cpu();// go to sleep
}

uint16_t readSupplyVoltage() { //returns value in millivolts  taken from megaTinyCore example
  analogReference(VDD);
  VREF.CTRLA = VREF_ADC0REFSEL_1V5_gc;
  // there is a settling time between when reference is turned on, and when it becomes valid.
  // since the reference is normally turned on only when it is requested, this virtually guarantees
  // that the first reading will be garbage; subsequent readings taken immediately after will be fine.
  // VREF.CTRLB|=VREF_ADC0REFEN_bm;
  // delay(10);
  uint16_t reading = analogRead(ADC_INTREF);
  reading = analogRead(ADC_INTREF);
  uint32_t intermediate = 1023UL * 1500;
  reading = intermediate / reading;
  return reading;
}

uint16_t readTemp() {
  // based on the datasheet, in section 30.3.2.5 Temperature Measurement
  int8_t sigrow_offset = SIGROW.TEMPSENSE1; // Read signed value from signature row
  uint8_t sigrow_gain = SIGROW.TEMPSENSE0; // Read unsigned value from signature row
  analogReference(INTERNAL1V1);
  ADC0.SAMPCTRL = 0x1F; // maximum length sampling
  ADC0.CTRLD &= ~(ADC_INITDLY_gm);
  ADC0.CTRLD |= ADC_INITDLY_DLY32_gc; // wait 32 ADC clocks before reading new reference
  uint16_t adc_reading = analogRead(ADC_TEMPERATURE); // ADC conversion result with 1.1 V internal reference
  analogReference(VDD);
  ADC0.SAMPCTRL = 0x0E; // 14, what we now set it to automatically on startup so we can run the ADC while keeping the same sampling time
  ADC0.CTRLD &= ~(ADC_INITDLY_gm);
  ADC0.CTRLD |= ADC_INITDLY_DLY16_gc;
  uint32_t temp = adc_reading - sigrow_offset;
  temp *= sigrow_gain; // Result might overflow 16 bit variable (10bit+8bit)
  temp += 0x80; // Add 1/2 to get correct rounding on division below
  temp >>= 8; // Divide result to get Kelvin
  temp -= 286; //0°C = 273.15K
  return temp;
}

ISR(TCB0_INT_vect) {// timmer
  wdt_reset(); // reset watchdog
  interruptTimer++;

  if (interruptTimer++, interruptTimer >= 1000) { // millis counter
    interruptTimer = 0;

    if (seconds++, seconds > 59) {// acutal time keeping
      seconds = 0;

      if (minutes++, minutes > 59) {
        minutes = 0;

        if (hours++, hours > 23) {
          hours = 0;
        }
      }
    }
  }
  TCB0_INTFLAGS = 1; // clear flag
}

ISR(PORTA_PORT_vect) {
  sleep_disable();
  _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
}
