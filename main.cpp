#include <arduino.h>
// --- MOTOR PINS ---
const int dirPin = 6;
const int pwmPin = 7;
const int phaseA = 2;
const int phaseB = 3;

// --- ENCODER & RPM ---
volatile long pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;
const int ppr = 64;

// --- PID CONSTANTS (Tuning needed!) ---
float Kp = 0.18;  
float Ki = 0.55;  
float Kd = 0.0;  

// --- PID VARIABLES ---
float targetRPM = 0;
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;
int finalPWM = 127; // Antiphase stop

// --- DIRECTIONAL ISRs ---
void encoderA_ISR() {
  if (digitalRead(phaseA) != digitalRead(phaseB)) pulseCount--;
  else pulseCount++;
}
void encoderB_ISR() {
  if (digitalRead(phaseA) == digitalRead(phaseB)) pulseCount--;
  else pulseCount++;
}

void setup(){
  Serial.setTimeout(10);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(phaseA, INPUT_PULLUP);
  pinMode(phaseB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(phaseA), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(phaseB), encoderB_ISR, CHANGE);

  digitalWrite(pwmPin, LOW); 
  Serial.begin(115200);
  Serial.println("PID Mode: Enter target RPM (e.g., 500 or -500)");
}

void loop() {
  unsigned long currentTime = millis();

  // --- PID CALCULATION (Every 20ms) ---
  if (currentTime - lastTime >= 50) {
    noInterrupts();
    long countCopy = pulseCount;
    pulseCount = 0;
    interrupts();

    float instantRpm = (countCopy / (float)ppr) * 1200.0;
    // Low-pass filter (Smooths out the 47 RPM jumps)
    // 0.7 = how much to trust the NEW reading. Adjust 0.1 to 0.9.
    rpm = (rpm * 0.7) + (instantRpm * 0.3);

    // 2. PID Logic
    error = targetRPM - rpm;
    if (abs(pidOutput) <120){
      integral += error*0.05;
    }


    integral = constrain(integral, -100, 100); 
    derivative = (error - lastError) / 0.05;

    pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

    finalPWM = 127 + (int)pidOutput;
    finalPWM = constrain(finalPWM, 0, 255);

    analogWrite(dirPin, finalPWM);

    // 4. Debugging
    // --- CORRECT TELEPLOT FORMAT ---
    Serial.print(">Target:"); 
    Serial.print(targetRPM); 
    Serial.println("|g"); // Must be println to end the command

    Serial.print(">Actual:"); 
    Serial.print(rpm); 
    Serial.println("|g"); // Must be println to end the command

    lastError = error;
    lastTime = currentTime;
  }

  // --- SERIAL INPUT ---
  if (Serial.available() > 0) {
    char c = Serial.peek();
    if (c == 's' || c == 'S') {
      Serial.read();
      targetRPM = 0;
      integral = 0;
      digitalWrite(pwmPin, LOW); // Kill power
      Serial.println("STOP");
    } 
    else if (isDigit(c) || c == '-') {
      digitalWrite(pwmPin, HIGH); // Wake up driver
      targetRPM = Serial.parseFloat();
    } 
    else {
      Serial.read(); 
    }
  }
}