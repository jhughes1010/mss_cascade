//MSS Cascade Controller
//James Hughes 08/20/2024
//debug 9/27/24
//Version 0.2

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
#define A_MCU_OCCUPIED_OUT 15
#define B_MCU_OCCUPIED_OUT 14

#define A_FACING 0
#define B_FACING 1
Adafruit_MCP23X17 mcp;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup() {

  Serial.begin(9600);
  //while (!Serial);
  Serial.println("MSS Simple Cascade Controller-2");
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

  //LCD
  lcd.begin(16, 2);
  lcd.setBacklight(0x01);
  lcd.print("Simple Cascade");
}

void loop() {
  char main[3];
  char mainApr[3];
  char diverge[3];
  char tmp[16];
  int GPIORegister;
  int localOccupied;
  int A_occupied, B_occupied;
  int A_approach, B_approach;
  int A_advance_approach, B_advance_approach;
  static long int currentTime = 0, transitionTime = 0;
  bool optical = false, A_active = false, B_active = false;
  static bool stable = false;

  currentTime = millis();
  //Serial.println(((float)currentTime / 1000));
  Serial.println("\n\nTop of loop");
  // read optical
  optical = readOptical(A0);
  //read GPIOs
  GPIORegister = (0xffff - mcp.readGPIOAB()) & 0x3f03;
  localOccupied = GPIORegister & 0x03;
  //----------
  //SET OUTPUT OCCUPANCY PULLDOWNS
  //----------
  if (optical || (localOccupied & A_LOCAL_OCCUPIED)) {
    mcp.digitalWrite(A_MCU_OCCUPIED_OUT, HIGH);
    Serial.println("A local occupied");
    //stable = false;
  } else {
    mcp.digitalWrite(A_MCU_OCCUPIED_OUT, LOW);
  }
  if (optical || (localOccupied & B_LOCAL_OCCUPIED)) {
    mcp.digitalWrite(B_MCU_OCCUPIED_OUT, HIGH);
    Serial.println("B local occupied");
    //stable = false;
  } else {
    mcp.digitalWrite(B_MCU_OCCUPIED_OUT, LOW);
  }

  if (GPIORegister || A_active || B_active || optical) {
    stable = false;
  }

  if (GPIORegister & 0x0700) {
    A_active = true;
    transitionTime = currentTime;
  } else if (currentTime - transitionTime > 1000) {
    A_active = false;
  }



  //Set active flag to reduce flicker
  if (GPIORegister & 0x3800) {
    B_active = true;
    transitionTime = currentTime;
  } else if (currentTime - transitionTime > 1000) {
    B_active = false;
  }



  Serial.print("GPIO: ");
  Serial.println(GPIORegister, 16);
  Serial.print("Local Occupied: ");
  Serial.println(localOccupied, 16);


  A_occupied = GPIORegister & 0x100;
  B_occupied = GPIORegister & 0x800;
  A_approach = GPIORegister & 0x200;
  B_approach = GPIORegister & 0x1000;
  A_advance_approach = GPIORegister & 0x400;
  B_advance_approach = GPIORegister & 0x2000;




  //----------
  //SET OUTPUT LAMPS
  //----------

  if (!stable || A_active || B_active) {
    if (optical) {
      dark();
      set(A_RED);
      set(B_RED);
    }

    if (A_occupied) {
      dark(B_FACING);
      set(B_RED);
      Serial.println("B RED");
    }
    if (B_occupied) {
      dark(A_FACING);
      set(A_RED);
      Serial.println("A RED");
    }


    if (A_approach) {
      dark(1);
      set(B_YELLOW);
    }
    if (B_approach) {
      dark(0);
      set(A_YELLOW);
    }

    if (A_advance_approach) {
      dark(1);
      if ((currentTime / 1000) % 2) {
        set(B_YELLOW);
      }
    }
    if (B_advance_approach) {
      dark(0);
      if ((currentTime / 1000) % 2) {
        set(A_YELLOW);
      }
    }

    if (!A_occupied && !A_approach && !A_advance_approach && !optical) {
      dark(1);
      set(B_GREEN);
      stable = true;
    }

    if (!B_occupied && !B_approach && !B_advance_approach && !optical) {
      dark(0);
      set(A_GREEN);
      stable = true;
    }
  }
  Serial.println(stable);
  //delay(1000);
}
