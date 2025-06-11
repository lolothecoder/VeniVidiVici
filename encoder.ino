const int encoderPinA = A0;
unsigned long switchCount = 0;
bool lastA = LOW;
unsigned long lastReport = 0;

void setup() {
  Serial.begin(115200);
  pinMode(encoderPinA, INPUT_PULLUP);
  lastA = digitalRead(encoderPinA);
}

void loop() {
  // detect change
  bool nowA = digitalRead(encoderPinA);
  if (nowA != lastA) {
    switchCount++;
    lastA = nowA;
  }
  // report every second
  if (millis() - lastReport >= 1000) {
    Serial.print("Edges in last 1 s: ");
    Serial.println(switchCount);
    switchCount = 0;
    lastReport += 1000;
  }
}
