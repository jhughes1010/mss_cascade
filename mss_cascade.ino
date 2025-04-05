//MSS Cascade Controller
//James Hughes 03/31/2025
#define VERSION "2025.4.4"

//09-28-2024  0.3 Refactor code with new loop
//03-07-2025  0.4 New main loop and signals go dark after 60s of green
//in layout edits to get everything functioning


#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_RGBLCDShield.h>
//I2C port expander outputs
#define A_GREEN 7
#define A_YELLOW 6
#define A_RED 5
#define B_GREEN 4
#define B_YELLOW 3
#define B_RED 2

//I2C port expander inputs
#define A_LOCAL_OCCUPIED 0x02
#define B_LOCAL_OCCUPIED 0x01
#define A_MCU_OCCUPIED_OUT 14
#define B_MCU_OCCUPIED_OUT 15

#define A_FACING 0
#define B_FACING 1
Adafruit_MCP23X17 mcp;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup() {

  Serial.begin(9600);
  //while (!Serial);
  Serial.println("MSS Simple Cascade Controller");
  Serial.println("James Hughes - 2025");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //jh delay(2000);
  // put your setup code here, to run once:
  // uncomment appropriate mcp.begin
  if (!mcp.begin_I2C(0x27)) {
    //if (!mcp.begin_SPI(CS_PIN)) {
    Serial.println("Port Expander not responding");
    while (1) {}
  }
  //setPIN direction
  int pin;
  for (pin = 0; pin < 16; pin++) {
    if (pin <= 7 && pin >= 2 || pin >= 14) {
      mcp.pinMode(pin, OUTPUT);
      Serial.print("Output pin:");
      Serial.println(pin);
    } else {
      mcp.pinMode(pin, INPUT);
      Serial.print("Input pin:");
      Serial.println(pin);
    }
  }
  //turn off signal lights (active low)
  mcp.writeGPIOA(0xff);
  //release MCU_A and MCU_B out (active high)
  mcp.writeGPIOB(0x00);
  //mcp.readGPIOAB()
  //builtin set low means setup completed
  digitalWrite(LED_BUILTIN, LOW);

  //set interrupts
  //mcp.setupInterruptPin(1, LOW);
  //mcp.setupInterruptPin(0, LOW);

  //Let the show begin
  dark();
  delay(1000);
  set(A_GREEN);
  set(B_GREEN);
  delay(1000);
  dark();
  set(A_YELLOW);
  set(B_YELLOW);
  delay(1000);
  dark();
  set(A_RED);
  set(B_RED);
  delay(1000);
  dark();
  set(A_GREEN);
  set(B_GREEN);
  delay(1000);
  dark();

  //LCD
  lcd.begin(16, 2);
  lcd.setBacklight(0x01);
  lcd.print("Simple Cascade");
}

void loop() {
  static long int activeTime = 0, occupiedTime = 0;
  long int currentTime;
  int GPIORegister = 0, localOccupancy = 0;
  bool optical, aOccupied = false, bOccupied = false;
  bool darkFlag = false;
  static bool wakeTrigger = false, opticalTrigger = false;
  int AStatus=0, BStatus=0;
  long int darkTime = 120000;
  int heartbeat = (millis() / 1000) % 2;



  //heartbeat on internal LED
  digitalWrite(LED_BUILTIN, heartbeat);

  //get time
  currentTime = millis();
  //check to see if dark flag is active dark flag
  if ((currentTime - activeTime) > darkTime) {
    darkFlag = true;
  }

  //sample inputs, but not constantly
  //if (currentTime % 500 < 100) {
  GPIORegister = (0xffff - mcp.readGPIOAB()) & 0x3f03;
  //jhSerial.println(GPIORegister,HEX);
  localOccupancy = GPIORegister & 0x0003;
  //get optical status
  optical = readOptical(A0);
  //optical=false;
  //jhSerial.println(optical);
  delay(20);
  //}

  if (localOccupancy) {
    opticalTrigger = true;
    occupiedTime = millis();
  }
  if (optical && opticalTrigger) {
    aOccupied = true;
    bOccupied = true;
  }
  if (occupiedTime + 30000 < millis()) {
    opticalTrigger = false;
  }

  //Check local occupancy detectors
  if (localOccupancy & A_LOCAL_OCCUPIED) {
    aOccupied = true;
  }
  if (localOccupancy & B_LOCAL_OCCUPIED) {
    bOccupied = true;
  }

  LocalOccupiedA(aOccupied);
  LocalOccupiedB(bOccupied);


  //determine singal level
  //3 = occupied (current or optical), 2 = approach, 1 = advanced approach, 0 = clear

  AStatus = readA(GPIORegister, optical);
  BStatus = readB(GPIORegister, optical);


  //any non-zero status resets activeTime
  if (AStatus || BStatus) {
    activeTime = currentTime;
    //Serial.println("Setting activeTime");
  }

  lcdPrintStatus(AStatus, BStatus);
  printStatus(AStatus, BStatus);

  //set local occupied if needed
  //set signal mast state


  if (darkFlag) {
    dark();
    wakeTrigger = true;
  } else {
    if (currentTime % 1000 < 100) {
      setMastA(AStatus, wakeTrigger);
      setMastB(BStatus, wakeTrigger);
      wakeTrigger = false;
    }
  }
}

//Simple print-on-change
void printStatus(int A, int B) {
  static int APrior = 0;
  static int BPrior = 0;
  if (APrior != A || BPrior != B) {
    Serial.print("Status ");
    Serial.print(A);
    Serial.print(":");
    Serial.println(B);
  }

  APrior = A;
  BPrior = B;
}

//Simple print-on-change
void lcdPrintStatus(int A, int B) {
  static int APrior = 0;
  static int BPrior = 0;
  if (APrior != A || BPrior != B) {
    lcd.clear();
    lcd.home();
    lcd.print("Status ");
    lcd.print(A);
    lcd.print(":");
    lcd.print(B);
  }

  APrior = A;
  BPrior = B;
}
