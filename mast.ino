//mast::mast

void set(int pin) {
  mcp.digitalWrite(pin, LOW);
}

void dark(void) {
  //Serial.println("dark");
  mcp.digitalWrite(A_GREEN, HIGH);
  mcp.digitalWrite(B_GREEN, HIGH);
  mcp.digitalWrite(A_YELLOW, HIGH);
  mcp.digitalWrite(B_YELLOW, HIGH);
  mcp.digitalWrite(A_RED, HIGH);
  mcp.digitalWrite(B_RED, HIGH);
}
void dark(int side) {
  //Serial.print("dark setting side ");
  //Serial.println(side);
  if (side == 0) {
    mcp.digitalWrite(A_GREEN, HIGH);
    mcp.digitalWrite(A_YELLOW, HIGH);
    mcp.digitalWrite(A_RED, HIGH);
  }
  if (side == 1) {
    mcp.digitalWrite(B_GREEN, HIGH);
    mcp.digitalWrite(B_YELLOW, HIGH);
    mcp.digitalWrite(B_RED, HIGH);
  }
}

void blink(int pin, int time_delay) {
  mcp.digitalWrite(pin, LOW);
  delay(time_delay);
  mcp.digitalWrite(pin, HIGH);
}

void blinkInv(int pin, int time_delay) {
  digitalWrite(pin, HIGH);
  delay(time_delay);
  digitalWrite(pin, LOW);
}


void setMastA(int status, bool wakeFromDark) {
  //static long int changeTime = 0;
  long int seconds = millis() / 1000;
  static int priorStatus = -1;
  if ((status != priorStatus) || status == 1 || wakeFromDark) {
    dark(0);
    //delay(500);
    if (status == 3)
      mcp.digitalWrite(A_RED, LOW);
    if (status == 2)
      mcp.digitalWrite(A_YELLOW, LOW);
    if (status == 1) {
      if (seconds % 2 == 0)
        mcp.digitalWrite(A_YELLOW, LOW);
      else {
        mcp.digitalWrite(A_YELLOW, HIGH);
      }
      if (status == 0)
        mcp.digitalWrite(A_GREEN, LOW);
    }

    //changeTime = millis();
  }
}

void setMastB(int status, bool wakeFromDark) {
  long int seconds = millis() / 1000;
  static int priorStatus = -1;

  if ((status != priorStatus) || status == 1 || wakeFromDark) {
    dark(1);
    if (status == 3)
      mcp.digitalWrite(B_RED, LOW);
    if (status == 2)
      mcp.digitalWrite(B_YELLOW, LOW);
    if (status == 1) {
      if (seconds % 2 == 0)
        mcp.digitalWrite(B_YELLOW, LOW);
      else {
        mcp.digitalWrite(B_YELLOW, HIGH);
      }
      if (status == 0)
        mcp.digitalWrite(B_GREEN, LOW);
    }
  }
  priorStatus = status;
}