// I2C slave Arduino RPM sensor code

#include <Wire.h>
#include <EnableInterrupt.h>

// Memory saving definitions for EnableInterrupt library;
// Disable external interrupts and unused pin change interrupt port
#define EI_NOTEXTERNAL
#define EI_NOTPORTC

// Define to selectively compile either Serial console or I2C communication
#define SERIAL_TEST_SCRIPT

// Define global constants
#ifdef SERIAL_TEST_SCRIPT
// Define Serial parameters
static constexpr uint16_t SERIAL_BAUD_RATE    = 9600;
static constexpr uint16_t SERIAL_PRINT_PERIOD = 2000;     // Serial output print time in ms
#else
// Define I2C parameters
static constexpr uint8_t  SLAVE_ADDR  = 0x6F;             // Arbitrarily chosen I2C address
static constexpr uint32_t I2C_CLOCK   = 400000;           // I2C communication speed in Hz
#endif

// Define pin change interrupt pins for sensors
//  All on digital pins
static constexpr uint8_t PIN_PROP1 = 2;
static constexpr uint8_t PIN_PROP2 = 3;
static constexpr uint8_t PIN_PROP3 = 4;
static constexpr uint8_t PIN_PROP4 = 5;
static constexpr uint8_t PIN_PROP5 = 6;
static constexpr uint8_t PIN_PROP6 = 7;
static constexpr uint8_t PIN_PROP7 = 8;
static constexpr uint8_t PIN_PROP8 = 9;

// Define number of propellers on aircraft and size of moving average for propellers
static constexpr uint8_t N_PROPS      = 8;
static constexpr uint8_t TIME_COUNTS  = 5;       // N_PROPS * TIME_COUNTS < 255 for N_PROPS = 8; TIME_COUNTS < 30
static constexpr unsigned long AVG_MICRO_TO_RPM = TIME_COUNTS * 60000000;

// Define RPM union so RPM data stored as uint can be sent as individual bytes during Wire.write()
union rpmUnion {
  unsigned int rpmUInt[N_PROPS];
  byte rpmByte[sizeof(int) * N_PROPS];
};

// Define global variables
unsigned long timeLastProp[N_PROPS];
unsigned long timeCountsProp[TIME_COUNTS * N_PROPS];
unsigned long timeCountsSum[N_PROPS];
unsigned long *indProp[N_PROPS];
unsigned long *indPropEnd[N_PROPS];
int8_t pinIdxFlag = -1;
rpmUnion propRPM;

#ifdef SERIAL_TEST_SCRIPT
// Define serial timer variable
unsigned long lastMillis;
#endif

void setup() {
  // Initialise sensor pins with INPUT_PULLUP resistors and enable pin change interrupts on FALLING edge of sensor pins
  pinMode(PIN_PROP1, INPUT_PULLUP);
  enableInterrupt(PIN_PROP1,ISR_Prop1,FALLING);
  pinMode(PIN_PROP2, INPUT_PULLUP);
  enableInterrupt(PIN_PROP2,ISR_Prop2,FALLING);
  pinMode(PIN_PROP3, INPUT_PULLUP);
  enableInterrupt(PIN_PROP3,ISR_Prop3,FALLING);
  pinMode(PIN_PROP4, INPUT_PULLUP);
  enableInterrupt(PIN_PROP4,ISR_Prop4,FALLING);
  pinMode(PIN_PROP5, INPUT_PULLUP);
  enableInterrupt(PIN_PROP5,ISR_Prop5,FALLING);
  pinMode(PIN_PROP6, INPUT_PULLUP);
  enableInterrupt(PIN_PROP6,ISR_Prop6,FALLING);
  pinMode(PIN_PROP7, INPUT_PULLUP);
  enableInterrupt(PIN_PROP7,ISR_Prop7,FALLING);
  pinMode(PIN_PROP8, INPUT_PULLUP);
  enableInterrupt(PIN_PROP8,ISR_Prop8,FALLING);

#ifdef SERIAL_TEST_SCRIPT
  // Initialise Serial communication
  Serial.begin(SERIAL_BAUD_RATE);
#else
  // Initialise I2C communication as slave
  Wire.begin(SLAVE_ADDR);
  Wire.setClock(I2C_CLOCK);
  Wire.onRequest(I2C_Request);
#endif

  // Initialise index pointers for timeCountsProp array
  for (int i = 0; i < N_PROPS; i++) {
    indProp[i] = &timeCountsProp[i * TIME_COUNTS];
    indPropEnd[i] = &timeCountsProp[i * TIME_COUNTS + TIME_COUNTS - 1];
    propRPM.rpmUInt[i] = 0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (pinIdxFlag != -1) {
    // Disable interrupts
    noInterrupts();

    // Store current time in micros
    unsigned long timeNow = micros();

    // Remove oldest recorded time for this propeller from sum
    timeCountsSum[pinIdxFlag] -= *indProp[pinIdxFlag];

    // Overwrite oldest time step for this propeller
    *indProp[pinIdxFlag] = timeNow - timeLastProp[pinIdxFlag];
    timeLastProp[pinIdxFlag] = timeNow;

    // Add new time to sum
    timeCountsSum[pinIdxFlag] += *indProp[pinIdxFlag];

    // Move index pointer to new oldest time for this propeller
    if (indProp[pinIdxFlag] == indPropEnd[pinIdxFlag]) {
      indProp[pinIdxFlag] -= TIME_COUNTS - 1;        
    } else {
      indProp[pinIdxFlag]++;
    }
    
    // Calculate RPM
    propRPM.rpmUInt[pinIdxFlag] = AVG_MICRO_TO_RPM / timeCountsSum[pinIdxFlag];

    // Reset pin flag
    pinIdxFlag = -1;

    // Enable interrupts
    interrupts();
  }

#ifdef SERIAL_TEST_SCRIPT
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= SERIAL_PRINT_PERIOD) {
    Serial.println(propRPM.rpmUInt[0]);
    lastMillis = currentMillis;
  }
#endif

}

#ifndef SERIAL_TEST_SCRIPT
// Return RPM data on I2C request
void I2C_Request() {
  Wire.write(propRPM.rpmByte, sizeof(propRPM));
}
#endif

// Propeller pin ISRs
void ISR_Prop1() {
  pinIdxFlag = 0;
}

void ISR_Prop2() {
  pinIdxFlag = 1;
}

void ISR_Prop3() {
  pinIdxFlag = 2;
}

void ISR_Prop4() {
  pinIdxFlag = 3;
}

void ISR_Prop5() {
  pinIdxFlag = 4;
}

void ISR_Prop6() {
  pinIdxFlag = 5;
}

void ISR_Prop7() {
  pinIdxFlag = 6;
}

void ISR_Prop8() {
  pinIdxFlag = 7;
}