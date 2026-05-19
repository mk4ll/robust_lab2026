// =====================================================
//  Frequency Response Sweep — DC Motor (YL-86 / L9110)
//  -----------------------------------------------------
//  YL-86 wiring (Convention B):
//    IA=LOW,  IB=PWM   -> forward (modulated speed)
//    IA=PWM,  IB=LOW   -> reverse
//    IA=LOW,  IB=LOW   -> coast
//    IA=HIGH, IB=HIGH  -> brake (avoid)
//
//  Operating point V1* = 200 PWM (as per TA).
//  Model G(s) is in deviation variables around (V1*, O*).
//  Input u(t) = 0.3*sin(omega*t) is the SCALED DEVIATION input.
//
//  V1(t)  = V1* + PWM_RANGE * u_pu(t)
//         = 200 + 50 * 0.3 * sin(omega*t)
//         = 200 + 15 * sin(omega*t)   in [185, 215]
// =====================================================

// ---- Pins (per the YL-86 wiring photo) ----
const int motorIN_A = 5;   // IA2 on YL-86
const int motorIN_B = 4;   // IB2 on YL-86
const int encA      = 18;  // Encoder channel A (interrupt)
const int encB      = 19;  // Encoder channel B (direction)

// ---- Encoder ----
volatile long encoderPos = 0; // 'volatile' is required for variables changed in an ISR
const float PPR = 2150.0;

// ---- Sampling / RPM ----
long previousPos = 0;
unsigned long previousTime = 0;

// ---- Frequency sweep (rad/s) ----
// 20 ms sampling -> aliasing-safe ~30 rad/s
// ω_c ≈ 4–5 rad/s -> σαρώνουμε από 0.1 έως 10 για να πιάσουμε roll-off
float omegas[] = {0.1, 0.3, 0.7, 1.2, 2.0, 3.0, 4.0, 5.0, 6.5, 8.0, 10.0};
int numOmegas = 11;
int currentOmegaIndex = 0;

unsigned long stateStartTime = 0;
bool finished = false;
bool isNewFreq = true;
unsigned long timeToWait = 0;

// ---- Operating point ----
const float PWM_BIAS  = 200.0;
const float PWM_RANGE = 50.0;
const float AMP_PU    = 0.3;

void setup() {
  // CRITICAL FIX: 115200 Baud to prevent the 20ms loop from stuttering/blocking
  Serial.begin(115200);

  pinMode(motorIN_A, OUTPUT);
  pinMode(motorIN_B, OUTPUT);

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encA), readEncoder, RISING);

  // For forward-only operation: IA stays LOW for the whole experiment.
  // IB gets the PWM. Set IA once here and never touch it again.
  analogWrite(motorIN_A, 0);
  analogWrite(motorIN_B, 0);

  // Give the user 2 seconds to clear the serial monitor before starting
  delay(2000);
}

void loop() {
  if (finished) return;

  unsigned long currentMillis = millis();

  // --- 1. Setup a New Frequency Block ---
  if (isNewFreq) {
    Serial.println();
    Serial.print("Omega = ");
    Serial.println(omegas[currentOmegaIndex]);
    Serial.println("Time_s,u_pwm,u_pu,RPM");

    float period = (2.0 * PI) / omegas[currentOmegaIndex];
    
    // Calculate wait time (3 periods)
    timeToWait = (unsigned long)(3.0 * period * 1000.0);
    
    // CRITICAL FIX: Enforce minimum wait time (4 seconds) so fast frequencies have time to settle!
    if (timeToWait < 4000) { timeToWait = 4000; } 
    // Cap at 6 minutes just in case
    if (timeToWait > 360000) { timeToWait = 360000; }  

    stateStartTime = currentMillis;
    previousTime   = currentMillis;
    
    // CRITICAL FIX: Atomic read to safely copy the 32-bit volatile variable
    noInterrupts();
    previousPos = encoderPos;
    interrupts();
    
    isNewFreq = false;
  }

  // --- 2. Check if it's time to move to the next frequency ---
  if (currentMillis - stateStartTime >= timeToWait) {
    currentOmegaIndex++;
    
    // If we've run out of frequencies, shut down
    if (currentOmegaIndex >= numOmegas) {
      finished = true;
      analogWrite(motorIN_A, 0);
      analogWrite(motorIN_B, 0);
      Serial.println();
      Serial.println("DONE! ALL FREQUENCIES COMPLETED.");
      return;
    }
    
    // Otherwise, flag that we need to start the next frequency
    isNewFreq = true;
    return;
  }

  // --- 3. The 20ms Control and Sampling Loop ---
  if (currentMillis - previousTime >= 20) {
    unsigned long dt = currentMillis - previousTime;
    float t = (currentMillis - stateStartTime) / 1000.0;

    // Calculate inputs
    float u_pu  = AMP_PU * sin(omegas[currentOmegaIndex] * t);
    float u_pwm = PWM_BIAS + PWM_RANGE * u_pu;   // [185, 215], ALWAYS POSITIVE

    // Apply motor command (Convention B forward drive: IA=LOW, IB=PWM)
    int pwmValue = constrain((int)u_pwm, 0, 255);
    analogWrite(motorIN_B, pwmValue);

    // CRITICAL FIX: Atomic encoder read to prevent corrupted RPM spikes
    noInterrupts();
    long currentPos = encoderPos;
    interrupts();
    
    // Calculate Speed
    long dPos = currentPos - previousPos;
    float rpm = ((float)dPos / PPR) * (60000.0 / (float)dt);

    // Send CSV data over Serial
    Serial.print(t, 3);
    Serial.print(",");
    Serial.print(u_pwm, 1);
    Serial.print(",");
    Serial.print(u_pu, 4);
    Serial.print(",");
    Serial.println(rpm, 3);

    // Update state for the next 20ms cycle
    previousTime = currentMillis;
    previousPos  = currentPos;
  }
}

// =====================================================
//  Encoder Interrupt Service Routine (ISR)
// =====================================================
void readEncoder() {
  // Sign of count follows direction via channel B
  if (digitalRead(encB) == HIGH) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}