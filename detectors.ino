//optical delay for hiding flicker
long int opticalDelay = 1000;
long int currentDelay = 1000;


bool readOptical(uint8_t sensorPin) {
  static bool status = false;
  static long int event = 0;
  int pinValue;
  int sensorThreshold = 13;

  pinValue = analogRead(sensorPin);
  Serial.print("Optical Sensor: ");
  Serial.println(pinValue);
  if (pinValue >= sensorThreshold) {
    event = millis();
    status = true;
    Serial.println("Optical TRUE");
  } else {
    if (event + opticalDelay < millis()) {
      status = false;
    }
  }
  digitalWrite(LED_BUILTIN, status);
  return status;
}
