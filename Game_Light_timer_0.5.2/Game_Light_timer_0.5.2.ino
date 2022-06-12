/*Game&Light Timer with Clock for LEDBOY and any Attiny series 1 compatible using neopixels
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
#include <avr/eeprom.h>

#if defined(MILLIS_USE_TIMERA0) || defined(MILLIS_USE_TIMERB0)
#error "This sketch does't use TCA0 nor TCB0 to save flash, select Disabled."
#endif

#define LEDON PORTA.OUT &= ~PIN3_bm;// led ON, also resets the oled screen.
#define LEDOFF PORTA.OUT |= PIN3_bm;// led OFF
#define BUTTONLOW !(PORTA.IN & PIN6_bm)// button press low
#define BUTTONHIGH (PORTA.IN & PIN6_bm)// button not pressed high
#define wdt_reset() __asm__ __volatile__ ("wdr"::) // watchdog reset macro
#define MAXVOLTAGE 3750 // max voltage allowed to the battery

uint8_t seconds = 0;
uint8_t minutes = 0;
uint8_t hours = 0;
uint8_t lastSecond = 0;
uint8_t lastMinute = 60;
uint8_t timerAlarm = 0;
uint16_t totalSeconds = 0;
uint16_t timer = 0;
uint16_t buttonTimer = 0;
volatile uint16_t interruptTimer = 0;
bool buttonReleased = false;
bool useCelsius = true;// true for °C false for °F
bool ledsOn = true;

void setup() {
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_4KCLK_gc);// enable watchdog 1 sec. aprox.
  PORTA.DIR = 0b10000110; // setup ports in and out
  PORTA.PIN6CTRL = PORT_PULLUPEN_bm;// button pullup

  TCB0.INTCTRL = TCB_CAPT_bm; // Setup Timer B as compare capture mode to trigger interrupt
  TCB0_CCMP = 5000;// CLK
  TCB0_CTRLA = (1 << 1) | TCB_ENABLE_bm;

  RTC.CTRLA |= (1 << RTC_RTCEN_bp);
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; /* 32KHz Internal Ultra Low Power Oscillator (OSCULP32K) */
  RTC.PITINTCTRL = RTC_PI_bm; /* Periodic Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 32768 */
                 | RTC_PITEN_bm; /* Enable: enabled */

  // We will be outputting PWM on PA3 on an 8-pin part PA3 - TCA0 WO0, pin 4 on 8-pin parts
  PORTMUX.CTRLC     = PORTMUX_TCA00_ALTERNATE_gc; // turn off PORTMUX, returning WO0 PORTMUX_TCA00_DEFAULT_gc for PA3. PORTMUX_TCA00_ALTERNATE_gc; for PA7
  takeOverTCA0();                               // this replaces disabling and resettng the timer, required previously.
  TCA0.SINGLE.CTRLB = (TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc); // Single slope PWM mode, PWM on WO0
  TCA0.SINGLE.PER   = 255;                    // Count all the way up to (255) - 8-bit PWM. At 5MHz, this gives ~19.607kHz PWM
  TCA0.SINGLE.CMP0  = 115; // 45% duty cycle
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm; // enable the timer with no prescaler

  sei(); // enable interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);// configure sleep mode

  if (readSupplyVoltage() > MAXVOLTAGE && BUTTONHIGH) { // If voltage is above 3.8v the led is on to increase Amp consumption and lower batt.
    PORTA.DIR = (1 << 3);// pin3 (LED) as output
  }

  if (BUTTONHIGH) goToSleep(); // if button is high it is assumed that the watchdog bite and there is no intention to wake up.

  PORTA.DIR = (1 << 3); // pin3 (LED) as output

  ledsOn = eeprom_read_byte((uint8_t*)127);

  if (ledsOn) {
    for (uint8_t x = 4; x < 40; x += 34) { // led PWM and led blinking
      buttonDebounce();
      ledPWM (x);
    }
  }
  LEDOFF
  oled.begin();// start oled screen
  buttonDebounce();
  useCelsius = eeprom_read_byte((uint8_t*)126); //read stored color
  drawSpritesOnScreen();
}

void loop() {

  if (seconds != lastSecond) { // if seconds are even
    lastSecond = seconds;
    displayInfo();
  }

  while (BUTTONLOW) { // reset time, timer select, and turn off
    buttonReleased = true;

    if ((interruptTimer - timer) > 4000) {
      timer = interruptTimer;
      options(); // go to options
    }
  }

  if (buttonReleased) {    
    timer = interruptTimer;
    buttonReleased = false;
    totalSeconds = 0;
    seconds = 0;
    minutes = 0;
    hours = 0;
    timerAlarm += 5;

    if (timerAlarm > 30)  timerAlarm = 0;
    drawDecimal (110 , timerAlarm);
  }
}

void drawSpritesOnScreen (void) {
  drawSprite(0, 0, battIcon, false);
  drawSprite(0, 3, sandClock, false);
  if (useCelsius)  drawSprite(0, 1, tempSimbolC, false);
  if (!useCelsius)  drawSprite(0, 1, tempSimbolF, false);
}

void displayInfo(void) {
  int8_t voltageReading = map(readSupplyVoltage(), 2500, MAXVOLTAGE, 0, 117);

  oled.drawLine(8, 3, 119, 0);// clear minutes countdown bar

  if (timerAlarm != 0) {
    totalSeconds++;
    uint8_t minutesCountdown = map(totalSeconds, 0 , (timerAlarm * 60), 119, 2);
    oled.drawLine(8, 3, minutesCountdown, 0b00111000); // draws voltage meter
  }

  if ((timerAlarm == minutes && timerAlarm != 0) || voltageReading < 1 || hours > 23) { // if the alarm is in on state the led is turn on, if the button is pressed it goes to sleep.
    clearScreen();
    oled.ssd1306_send_command(0xAE);
    ledPWM (4);
    LEDOFF
    goToSleep();
  }

  if (minutes != lastMinute) { // voltage and temp reading
    lastMinute = minutes;
    int8_t actualTemp = 0;

    if (voltageReading < 0) voltageReading = 0;
    oled.drawLine(10, 0, voltageReading, 0b00011110); // draws voltage meter

    if (useCelsius)  actualTemp = (readTemp() - 283);
    if (!useCelsius) actualTemp = (readTemp() - 265);
    drawDecimal (10, actualTemp); //0°C = 273.15K - 0°F = 255.37
    if (actualTemp < 0) oled.drawLine (4, 2, 5, 0b00000100);
  }

  drawDecimal (34, hours);
  oled.drawLine (54, 2, 1, 0b00010001); // draws screen colons
  drawDecimal (58, minutes);
  oled.drawLine (78, 2, 1, 0b00010001);// draws screen colons
  drawDecimal (84, seconds);
}

void options (void) { // options menu
  int8_t option = 0;
  bool setOptions = true;

  clearScreen(); // drwa icon on the screen
  drawSprite(40, 1, sleepIcon, false);
  if (useCelsius) drawSprite(60, 1, tempSimbolC, false);
  if (!useCelsius) drawSprite(60, 1, tempSimbolF, false);
  drawSprite(80, 1, ledSprite, false);
  buttonDebounce();

  while (setOptions) {

    if (BUTTONLOW) {
      oled.drawLine(0, 2, 127, 0b00000000);
      buttonDebounce();
      if (option++, option > 3) option = 0;
    }

    switch (option) { // option select menu.
      case 0:
        oled.drawLine(40, 2, 8, 0b00011000); // sleep

        if ((interruptTimer - timer) > 3000) {
          timer = interruptTimer;
          clearScreen();
          oled.ssd1306_send_command(0xAE);
          goToSleep();
        }
        break;
      case 1:
        oled.drawLine(60, 2, 8, 0b00011000); // change C to F degrees

        if ((interruptTimer - timer) > 4000) {
          timer = interruptTimer;
          useCelsius = !useCelsius;
          eeprom_write_byte((uint8_t*)126, useCelsius);
          setOptions = false;
        }
        break;
      case 2:
        oled.drawLine(80, 2, 8, 0b00011000); // invert or not screen colors.

        if ((interruptTimer - timer) > 4000) {
          timer = interruptTimer;
          ledsOn = !ledsOn;
          eeprom_write_byte((uint8_t*)127, ledsOn);
          setOptions = false;
        }
        break;
      case 3:
        setOptions = false;
        break;
    }
  }
  clearScreen();
  drawSpritesOnScreen();
  lastMinute = 60;
}

void ledPWM (uint8_t pwm) { // led blinking function.
  while (BUTTONHIGH) {

    if (interruptTimer % pwm == 0) {

      if (readSupplyVoltage() < 2300) goToSleep(); // To preserve the LIC to stay above 2.2v no operation is allowed under 2.4v
      LEDON
    } else {
      LEDOFF
    }
  }
}

void drawDecimal (uint8_t firstPixel, int8_t value) {// this function takes the digit and value from characters.h and draws it without the 0 to the left in hours
  if (value < 0) value = (~value) + 1; // xor value and add one to make it posite
  int8_t valueUnits = value;                      // always draws 2 digits.

  if (value < 10) {
    drawSprite(firstPixel, 1, numbers[0], true);
    drawSprite((firstPixel + 8), 1, numbers[value], true);
  } else {
    value /= 10; // some math to substract the 0 from the left in hours digits
    drawSprite(firstPixel, 1, numbers[value], true);
    value *= 10;

    for (int8_t x = 0; x < value ; x++) {
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

void buttonDebounce(void) { // simple debounce
  buttonTimer = interruptTimer;
  while (BUTTONLOW || (interruptTimer - buttonTimer) < 200); // super simple button debounce
}

void goToSleep (void) {
  PORTA.PIN6CTRL  |= PORT_ISC_BOTHEDGES_gc; //attach interrupt to portA pin 3 keeps pull up enabled
  _PROTECTED_WRITE(WDT.CTRLA, 0);
  TCA0.SPLIT.CTRLA = 0; //disable TCA0 and set divider to 1
  RTC.PITINTCTRL &= ~ RTC_PI_bm; // rtc periodic interrupt disabled
  //TCA0.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03; //set CMD to RESET to do a hard reset of TCA0.
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
  int32_t temp = adc_reading - sigrow_offset;
  temp *= sigrow_gain; // Result might overflow 16 bit variable (10bit+8bit)
  temp += 0x80; // Add 1/2 to get correct rounding on division below
  temp >>= 8; // Divide result to get Kelvin
  return temp;
}

ISR(TCB0_INT_vect) {// timmer
  interruptTimer += 2;
  TCB0_INTFLAGS = 1; // clear flag
}

ISR(PORTA_PORT_vect) {
  _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
}

ISR(RTC_PIT_vect) {// rtc interupt
  wdt_reset(); // reset watchdog

  if (seconds++, seconds > 59) {// acutal time keeping
    seconds = 0;

    if (minutes++, minutes > 59) {
      minutes = 0;

      if (hours++, hours > 23) {
        hours = 0;
      }
    }
  }
  /* Clear flag by writing '1': */
  RTC.PITINTFLAGS = RTC_PI_bm;
}
