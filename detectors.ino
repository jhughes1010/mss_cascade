//optical delay for hiding flicker
long int opticalDelay = 3000;
long int currentDelay = 1000;


bool readOptical(uint8_t sensorPin) {
  static bool status = false;
  static long int event = 0;
  int pinValue;
  int sensorThreshold = 13;

  pinValue = analogRead(sensorPin);
  //Serial.print("Optical Sensor: ");
  //Serial.println(pinValue);
  if (pinValue >= sensorThreshold) {
    event = millis();
    status = true;
    //Serial.println("Optical TRUE");
  } else {
    if (event + opticalDelay < millis()) {
      status = false;
    }
  }
  //digitalWrite(LED_BUILTIN, status);
  return status;
}

int readA(int GPIO, bool optical) {
  int level = 0;
  int maskOccupied = 0x0800;
  int maskApproach = 0x1000;
  int maskAdvanceApproach = 0x2000;

  if (GPIO & maskAdvanceApproach) {
    level = 1;
  }
  if (GPIO & maskApproach) {
    level = 2;
  }
  if (GPIO & maskOccupied) {
    level = 3;
  }
  return level;
}

int readB(int GPIO, bool optical) {
  int level = 0;
  int maskOccupied = 0x0100;
  int maskApproach = 0x0200;
  int maskAdvanceApproach = 0x0400;

  if (GPIO & maskAdvanceApproach) {
    level = 1;
  }
  if (GPIO & maskApproach) {
    level = 2;
  }
  if (GPIO & maskOccupied) {
    level = 3;
  }
  return level;
}

void LocalOccupied(int status) {
  mcp.digitalWrite(A_MCU_OCCUPIED_OUT, status);
  mcp.digitalWrite(B_MCU_OCCUPIED_OUT, status);
}

void LocalOccupiedA(int status) {
  mcp.digitalWrite(A_MCU_OCCUPIED_OUT, status);
  //mcp.digitalWrite(B_MCU_OCCUPIED_OUT, status);
}

void LocalOccupiedB(int status) {
  //mcp.digitalWrite(A_MCU_OCCUPIED_OUT, status);
  mcp.digitalWrite(B_MCU_OCCUPIED_OUT, status);
}
