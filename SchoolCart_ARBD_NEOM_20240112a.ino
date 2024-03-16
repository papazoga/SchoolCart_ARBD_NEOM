#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#define VOLTAGE_PROTECT   29.2 // if voltage > 29.2 open relay
#define VOLTAGE_UNPROTECT 27.2 // if relay has been opened and  voltage is <=27.2, close relay
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
float amps_out_offset = 122.0;   // when current sensor is at 0 amps this is the ADC value
#define INVERTER_AMPS1_COEFF   12.30   // one of two current sensors for inverter
float inverter_amps1_offset = 119.5;
#define INVERTER_AMPS2_COEFF   12.63  // two of two current sensors for inverter
float inverter_amps2_offset = 120.5;

#define INTERVAL_PRINT  1000    // time between printInfo() events
#define INTERVAL_NEOPIXELS 250  // time between neopixel update events WHICH CORRUPTS millis()
#define BRIGHTNESS      20
#define MATRIX_HEIGHT   8       // matrix height
#define MATRIX_WIDTH    28      // matrix width
#define STRIP_COUNT     60      // how many LEDs
#define WATTHOURS_EEPROM_ADDRESS 20

Adafruit_NeoMatrix matrix02 =  Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX02_PIN,  NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG, NEO_GRB + NEO_KHZ800);
Adafruit_NeoMatrix matrix01 =  Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX01_PIN,  NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pedalometer(STRIP_COUNT, PEDALOMETER_PIN, NEO_GRB + NEO_KHZ800);

#define LED_BLACK		      0
#define LED_RED_HIGH 		  (31 << 11)
#define LED_GREEN_HIGH 		(63 << 5)  
#define LED_BLUE_HIGH 		31
#define LED_WHITE_HIGH		(LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)

uint32_t lastPrintInfo = 0;  // last time printInfo() happened
uint32_t lastNeopixels = 0;  // last time NeoPixels updated
uint32_t lastGetAnalogs = 0;  // last time getAnalogs() happened
uint32_t inverter_amps1 = 0; // accumulator for inverter current sensor 1
uint32_t inverter_amps2 = 0; // accumulator for inverter current sensor 2
float voltage = 0;              // system DC voltage
float current_pedal = 0;        // pedal input current
float current_inverter = 0;     // inverter output DC current
float watts_pedal = 0;
float watts_inverter = 0;
uint32_t energy_pedal = 0;         // energy accumulators ALL IN MILLIJOULES
uint32_t energy_inverter = 0;      // energy accumulators
uint32_t energy_balance;           // energy banking account value, loaded from EEPROM

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
  load_energy_balance(); // load energy_balance from EEPROM
  lastGetAnalogs = millis(); // initialize integrator timer before first getAnalogs() call
}

void loop() {
  getAnalogs(); // read voltages and currents, integrate energy counts, calculate energy_balance
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
  if (millis() - lastNeopixels > INTERVAL_NEOPIXELS) { // update neopixels at a reasonable rate
    lastNeopixels = millis(); // reset interval
    if (! digitalRead(BUTTONLEFT)) {
      disNeostring(&matrix01,"LEFT", LED_WHITE_HIGH);
      disNeostring(&matrix02,"LEFT", LED_WHITE_HIGH);
      delay(500);
      if (! digitalRead(BUTTONLEFT)) attemptShutdown(); // if button is still being held down, try to shut down
    } else if (! digitalRead(BUTTONRIGHT)) {
      disNeostring(&matrix01,"RIGHT", LED_WHITE_HIGH);
      disNeostring(&matrix02,"RIGHT", LED_WHITE_HIGH);
    } else {
      disNeostring(&matrix01,intAlignRigiht(voltage*100), LED_WHITE_HIGH);
      disNeostring(&matrix02,intAlignRigiht(watts_pedal), LED_WHITE_HIGH);
    }
    int soc = estimateStateOfCharge();
    disNeowipePedalometer(Wheel(80), (soc*60)/100); // 60 is max pedalometer
    if (soc > 10) {
      digitalWrite(RELAY_INVERTERON, HIGH); // turn on inverter
    } else if (soc < 5) {
      digitalWrite(RELAY_INVERTERON, LOW); // shut inverter OFF
    }
  }
}

void energyBankingModeLoop() {
}

boolean switchInUtilityMode() { // LEFT is LOW/FALSE, RIGHT is HIGH/TRUE
  return digitalRead(SWITCHMODE);
}

int estimateStateOfCharge() { 
  int soc = 0;
  if (voltage >= 20.0)  soc =   0 +  (voltage - 20.0 ) * (10 / 4.0);  //  0,   20.0
  if (voltage >= 24.0)  soc =  10 +  (voltage - 24.0 ) * (10 / 1.6);  //  10,  24.0
  if (voltage >= 25.6)  soc =  20 +  (voltage - 25.6 ) * (10 / 0.2);  //  20,  25.6
  if (voltage >= 25.8)  soc =  30 +  (voltage - 25.8 ) * (10 / 0.2);  //  30,  25.8
  if (voltage >= 26.0)  soc =  40 +  (voltage - 26.0 ) * (10 / 0.2);  //  40,  26.0
  if (voltage >= 26.2)  soc =  50 +  (voltage - 26.2 ) * (10 / 0.1);  //  50,  26.2
  if (voltage >= 26.3)  soc =  60 +  (voltage - 26.3 ) * (10 / 0.05); //  60,  26.3
  if (voltage >= 26.35) soc =  66 +  (voltage - 26.35) * (10 / 0.05); //  66,  26.35  
  if (voltage >= 26.4)  soc =  70 +  (voltage - 26.4 ) * (10 / 0.2);  //  70,  26.4
  if (voltage >= 26.6)  soc =  80 +  (voltage - 26.6 ) * (10 / 0.2);  //  80,  26.6
  if (voltage >= 26.8)  soc =  90 +  (voltage - 26.8 ) * (10 / 0.4);  //  90,  26.8
  if (voltage >= 27.2)  soc =  100;                                   //  100, 27.2 
  return soc;
}

void printInfo() {
  // voltage = (millis() % 7200) / 1000.0 + 20.0; // TODO: take out this debugging feature
  Serial.println(String(millis()/1000)+"	voltage:"+String(voltage)+
      "	SOC:"+String(estimateStateOfCharge())+
      "	"+String(analogRead(AMPS_IN_PIN))+" amps_in:"+String(current_pedal)+
      "	current_inverter: "+String(current_inverter)+
      " energy_pedal:"+String(energy_pedal/1000)+
      " energy_inverter:"+String(energy_inverter/1000)+
      " energy_balance:"+String(energy_balance/1000));
}

void getAnalogs() {
  uint32_t integrationTime = millis() - lastGetAnalogs; // time since last integration
  lastGetAnalogs = millis();

  voltage = average(analogRead(VOLT_PIN) / VOLTCOEFF, voltage);

  int amps_in_pin_reading = analogRead(AMPS_IN_PIN);
  current_pedal = average(( amps_in_pin_reading - amps_in_offset ) / AMPS_IN_COEFF , current_pedal);
  if (amps_in_pin_reading - amps_in_offset < 6) current_pedal = 0;
  if (current_pedal < 0) current_pedal = 0;
  watts_pedal = voltage * current_pedal;

  int inverter_amps1_pin_reading = analogRead(INVERTER_AMPS1_PIN);
  float inverter_amps1_calc = ( inverter_amps1_pin_reading - inverter_amps1_offset ) / INVERTER_AMPS1_COEFF;
  if (inverter_amps1_pin_reading - inverter_amps1_offset < 6) inverter_amps1_calc = 0;
  if (inverter_amps1_calc < 0)     inverter_amps1_calc = 0;

  int inverter_amps2_pin_reading = analogRead(INVERTER_AMPS2_PIN);
  float inverter_amps2_calc = ( inverter_amps2_pin_reading - inverter_amps2_offset ) / INVERTER_AMPS2_COEFF;
  if (inverter_amps2_pin_reading - inverter_amps2_offset < 6) inverter_amps2_calc = 0;
  if (inverter_amps2_calc < 0)     inverter_amps2_calc = 0;

  current_inverter = average(inverter_amps1_calc + inverter_amps2_calc, current_inverter);
  watts_inverter = voltage * current_inverter;

  energy_pedal    += watts_pedal    * integrationTime;
  energy_inverter += watts_inverter * integrationTime;

  energy_balance  += watts_pedal    * integrationTime; // adjust energy_balance
  energy_balance  -= watts_inverter * integrationTime; // adjust energy_balance
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
  if (switchInUtilityMode() == false) store_energy_balance(); // save our present energy bank account
  digitalWrite(RELAY_INVERTERON, LOW); // shut inverter OFF
  digitalWrite(RELAY_DROPSTOP, LOW); // turn off
  while(digitalRead(BUTTONRIGHT)); // wait until unless right button is pressed (TODO)
  delay(2000);  // power will disappear by now
  digitalWrite(RELAY_DROPSTOP, HIGH); // if we're still on might as well own it
}

union float_and_byte { // https://www.tutorialspoint.com/cprogramming/c_unions
  float f; // accessed as fab.f
  unsigned char bs[sizeof(float)]; // accessed as fab.bs
} fab; // a union is multiple vars taking up the same memory location

void store_energy_balance() {
  fab.f = energy_balance;
  for( int i=0; i<sizeof(float); i++ )
    EEPROM.write( WATTHOURS_EEPROM_ADDRESS+i, fab.bs[i] );
}

void load_energy_balance() {
  Serial.print( "Loading watthours bytes 0x" );
  bool blank = true;
  for( int i=0; i<sizeof(float); i++ ) {
    fab.bs[i] = EEPROM.read( WATTHOURS_EEPROM_ADDRESS+i );
    Serial.print( fab.bs[i], HEX );
    if( blank && fab.bs[i] != 0xff )  blank = false;
  }
  energy_balance = blank ? 0 : fab.f;
  Serial.println( ", so energy_balance is "+String(energy_balance));
}

void reset_energy_balance() {
  Serial.println("Zeroing energy_balance and storing to EEPROM");
  energy_balance = 0;
  store_energy_balance();
  delay(1000); // otherwise it resets a million times each press
}
