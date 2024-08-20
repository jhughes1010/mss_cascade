//mast::mast

void set(int pin) {
  mcp.digitalWrite(pin, LOW);
}

void dark(void) {
  Serial.println("dark");
  mcp.digitalWrite(A_GREEN, HIGH);
  mcp.digitalWrite(B_GREEN, HIGH);
  mcp.digitalWrite(A_YELLOW, HIGH);
  mcp.digitalWrite(B_YELLOW, HIGH);
  mcp.digitalWrite(A_RED, HIGH);
  mcp.digitalWrite(B_RED, HIGH);
}
void dark(int side) {
  Serial.println("dark overload");
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