#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define VOLTAGE_PROTECT   28.5    // voltage at which pedallers are disconnected
#define VOLTAGE_UNPROTECT 26.5    // voltage at which pedallers are re-connected
#define VOLT_PIN        A0
#define AMPS_IN_PIN     A3      // labeled PLUSRAIL/PLUSOUT IC2
#define AMPS_OUT_PIN    A2      // labeled MINUSRAIL/MINUSOUT IC3
#define INVERTER_AMPS1_PIN     A4      // one of two current sensors for inverter
#define INVERTER_AMPS2_PIN     A5      // two of two current sensors for inverter
#define MATRIX01_PIN    11
#define MATRIX02_PIN    12
#define PEDALOMETER_PIN     13
#define BUTTONLEFT      6
#define SWITCHMODE      7
#define BUTTONRIGHT     8
#define RELAY_OVERPEDAL  2 // this relay disconnects pedal power input when activated
#define RELAY_INVERTERON 3 // this relay turns on the inverter by its own power switch
#define RELAY_DROPSTOP   4 // this relay connects this arbduino to the battery power

#define VOLTCOEFF       13.13   // convert ADC value to voltage
#define AMPS_IN_COEFF   11.94   // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
float amps_in_offset = 124.5;   // when current sensor is at 0 amps this is the ADC value
#define AMPS_OUT_COEFF  11.97   // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
#define AMPS_OUT_OFFSET 122.0   // when current sensor is at 0 amps this is the ADC value
#define INVERTER_AMPS1_COEFF   12.30   // one of two current sensors for inverter
#define INVERTER_AMPS1_OFFSET  119.5
#define INVERTER_AMPS2_COEFF   12.63  // two of two current sensors for inverter
#define INVERTER_AMPS2_OFFSET  120.5

#define INTERVAL_PRINT  1000    // time between printInfo() events
#define BRIGHTNESS      20
#define MATRIX_HEIGHT   8       // matrix height
#define MATRIX_WIDTH    28      // matrix width
#define STRIP_COUNT     60      // how many LEDs

Adafruit_NeoMatrix matrix02 =  Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX02_PIN,  NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG, NEO_GRB + NEO_KHZ800);
Adafruit_NeoMatrix matrix01 =  Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX01_PIN,  NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pedalometer(STRIP_COUNT, PEDALOMETER_PIN, NEO_GRB + NEO_KHZ800);

#define LED_BLACK		      0
#define LED_RED_HIGH 		  (31 << 11)
#define LED_GREEN_HIGH 		(63 << 5)  
#define LED_BLUE_HIGH 		31
#define LED_WHITE_HIGH		(LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)

uint32_t lastPrintInfo = 0;  // last time printInfo() happened
uint32_t inverter_amps1 = 0; // accumulator for inverter current sensor 1
uint32_t inverter_amps2 = 0; // accumulator for inverter current sensor 2
float voltage = 0;              // system DC voltage
float current_pedal = 0;        // pedal input current
float current_inverter = 0;     // inverter output DC current
float watts_pedal = 0;
float watts_inverter = 0;
float energy_pedal = 0;         // energy accumulators
float energy_inverter = 0;      // energy accumulators

#define AVG_CYCLES 30 // how many times to average analog readings over

void setup() {
  pinMode(RELAY_DROPSTOP, OUTPUT);
  digitalWrite(RELAY_DROPSTOP, HIGH); // turn on relay so we stay on until we decide otherwise
  pinMode(RELAY_INVERTERON, OUTPUT);
  pinMode(RELAY_OVERPEDAL, OUTPUT);
  Serial.begin(115200);
  digitalWrite(BUTTONLEFT,HIGH);  // enable internal pull-up resistor
  digitalWrite(SWITCHMODE,HIGH);  // enable internal pull-up resistor
  digitalWrite(BUTTONRIGHT,HIGH); // enable internal pull-up resistor
  matrix01.begin();
  matrix01.setTextWrap(false);
  matrix01.setBrightness(BRIGHTNESS);
  matrix01.clear();
  matrix02.begin();
  matrix02.setTextWrap(false);
  matrix02.setBrightness(BRIGHTNESS);
  matrix02.clear();
  pedalometer.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  pedalometer.show();            // Turn OFF all pixels ASAP
  pedalometer.setBrightness(BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void loop() {
  getAnalogs();
  doProtectionRelay(); // disconnect pedallers if necessary, shutoff inverter if necessary
  if (switchInUtilityMode()) {
    utilityModeLoop();
  } else {
    energyBankingModeLoop();
  }
  if (millis() - lastPrintInfo > INTERVAL_PRINT) {
    lastPrintInfo = millis();
    printInfo();
  }
}

void utilityModeLoop() {
  if (! digitalRead(BUTTONLEFT)) {
    disNeostring(&matrix01,"LEFT", LED_WHITE_HIGH);
    disNeostring(&matrix02,"LEFT", LED_WHITE_HIGH);
    delay(1000);
    if (! digitalRead(BUTTONLEFT)) attemptShutdown();
  } else if (! digitalRead(BUTTONRIGHT)) {
    disNeostring(&matrix01,"RIGHT", LED_WHITE_HIGH);
    disNeostring(&matrix02,"RIGHT", LED_WHITE_HIGH);
  } else {
    disNeostring(&matrix01,intAlignRigiht(voltage*100), LED_WHITE_HIGH);
    disNeostring(&matrix02,intAlignRigiht(watts_pedal*100), LED_WHITE_HIGH);
  }
  disNeowipePedalometer(Wheel(80), voltage*4);
}

void energyBankingModeLoop() {
}

boolean switchInUtilityMode() { // LEFT is LOW/FALSE, RIGHT is HIGH/TRUE
  return digitalRead(SWITCHMODE);
}

void printInfo() {
  Serial.println(String(analogRead(VOLT_PIN))+"	voltage:"+String(voltage)+
      "	"+String(analogRead(AMPS_IN_PIN))+" amps_in:"+String(current_pedal)+
      "	current_inverter: "+String(current_inverter));
}

void getAnalogs() {
  voltage = average(analogRead(VOLT_PIN) / VOLTCOEFF, voltage);
  int amps_in_pin_reading = analogRead(AMPS_IN_PIN);
  if (amps_in_pin_reading < amps_in_offset) amps_in_offset = amps_in_pin_reading; // update adc offset if negative
  if (amps_in_pin_reading - amps_in_offset < 6) amps_in_pin_reading = amps_in_offset; // zero current if near zero
  current_pedal = average(( amps_in_pin_reading - amps_in_offset ) / AMPS_IN_COEFF , current_pedal);
  watts_pedal = voltage * current_pedal;

  float inverter_amps1_calc = ( analogRead(INVERTER_AMPS1_PIN) - INVERTER_AMPS1_OFFSET ) / INVERTER_AMPS1_COEFF;
  float inverter_amps2_calc = ( analogRead(INVERTER_AMPS2_PIN) - INVERTER_AMPS2_OFFSET ) / INVERTER_AMPS2_COEFF;
  current_inverter = average(inverter_amps1_calc + inverter_amps2_calc, current_inverter);
  watts_inverter = voltage * current_inverter;
}

float average(float val, float avg){
  if (avg == 0) avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

String intAlignRigiht(int num) {
  int cursorPos = 0;
  int    space  = 0;
  String spaces = "";

  if      (num < 10.00)                       {space=2;    cursorPos=2;} //3
  else if (num >= 10.00 && num < 100.00)      {space=1;    cursorPos=2;}
  else if (num >= 100.00 && num < 1000.00)    {space=0;    cursorPos=2;}
  else if (num >= 1000.00 && num < 10000.00)  {space=0;    cursorPos=2;}
  
  for (uint8_t s=0; s<space; s++) spaces += F(" ");
  
  return spaces+String(num);
}

void disNeostring(Adafruit_NeoMatrix* matrix, String nval, uint32_t col) { // https://forums.adafruit.com/viewtopic.php?t=101790
  //uint8_t dval = bval; //uint8_t dval = map(constrain(V_Value, 0, 1200), 0, 1200, 0, 255);
  matrix->clear();
  matrix->setCursor(4, 0); // 3 allows 3 character, greater moves pixels to the right and allows fewer characters
  matrix->setTextColor(col);
  matrix->print(nval);
  //matrix->drawLine(24, 7, map(dval, 0, 255, 24, 0), 7, matrix01->Color(map(dval, 0, 255, 255, 150),dval,0));
  matrix->show();
}

void disNeowipePedalometer(uint32_t color, int pixlevel) {
  if (pixlevel >= 60) {pixlevel = 59;}
  for(int i=0; i<pedalometer.numPixels(); i++) { // For each pixel in strip...
    if (i <= pixlevel) {pedalometer.setPixelColor(i, color);}
    else {pedalometer.setPixelColor(i, 0);}         //  Set pixel's color (in RAM)
    //strip.show();                          //  Update strip to match
    //delay(wait);                           //  Pause for a moment
  }
  pedalometer.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return pedalometer.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return pedalometer.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return pedalometer.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void doProtectionRelay() {
  if (voltage > VOLTAGE_PROTECT) {
    digitalWrite(RELAY_OVERPEDAL, HIGH); // disconnect pedallers
  }
  if (voltage < VOLTAGE_UNPROTECT) {
    digitalWrite(RELAY_OVERPEDAL, LOW); // don't disconnect pedallers
  }
}

void attemptShutdown() {
  digitalWrite(RELAY_DROPSTOP, LOW); // turn off
  delay(2000);  // power will disappear by now
  digitalWrite(RELAY_DROPSTOP, HIGH); // if we're still on might as well own it
}
