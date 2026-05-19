// ==========================================
// SMART DC MOTOR CONTROLLER SCRIPT (DYNAMIC MODE SWITCHING)
// ==========================================

const int motorPWM = 4;
const int motorDir = 5;
const int encA = 18; 
const int encB = 19;

volatile long encoderPos = 0; 
long previousPos = 0;
unsigned long previousTime = 0;
const float PPR = 2150.0; 

const float Ts = 0.1;       
const unsigned long Ts_ms = 20;

// --- ACTIVE CONTROLLER PARAMETERS ---
float Kp = 0.0; 
float Ti = 1.0; 
float Tt = 1.0; 
float filterTau = 1.0; 
bool usePrefilter = false;
bool useHinf = false;

// --- H-INFINITY MATRICES ---
float A[3][3] = { {9.996e-01, 0.0, 0.0}, {0.0, 5.0e-01, 2.552e-01}, {5.328e+08, -3.001e+08, -2.172e+07} }; 
float B[3] = { 0.02, 0.0, 0.0 }; 
float C[3] = { 2.664e+10, -1.500e+10, -1.086e+09 }; 
float D = 0.0; 

// --- STATE VARIABLES ---
float I_old = 0.0;          
float u_k = 0.0;            
float u_sat = 0.0;          
float x[3] = {0.0, 0.0, 0.0};      
float x_new[3] = {0.0, 0.0, 0.0};  

// --- REFERENCE VARIABLES ---
float targetRPM_raw = 20.0; 
float targetRPM_filtered = 20.0; 
float currentRPM_filtered = 0.0;
const float rpmFilterTau = 0.20; 

void setup() {
  Serial.begin(9600);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA), readEncoder, RISING);
  
  setControllerMode(3); // Default to 2-DoF at startup
}

void loop() {
  // 1. DYNAMIC SERIAL COMMAND PARSER
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    
    if (msg.startsWith("M")) {
      int mode = msg.substring(1).toInt();
      setControllerMode(mode);
    } 
    else if (msg.startsWith("T")) {
      float newTarget = msg.substring(1).toFloat();
      if (newTarget > 0) targetRPM_raw = newTarget;
    }
  }

  unsigned long currentMillis = millis();

  // 2. CONTROL LOOP
  if (currentMillis - previousTime >= Ts_ms) {
    
    if (usePrefilter && !useHinf) {
      float alpha = Ts / (Ts + filterTau); 
      targetRPM_filtered = (1.0 - alpha) * targetRPM_filtered + alpha * targetRPM_raw;
    } else {
      targetRPM_filtered = targetRPM_raw;
    }

    long currentPos = encoderPos;
    long dPos = currentPos - previousPos;
    float currentRPM = ((float)dPos / PPR) * (60000.0 / Ts_ms);

    float alpha_rpm = Ts / (Ts + rpmFilterTau);
    currentRPM_filtered = (1.0 - alpha_rpm) * currentRPM_filtered + alpha_rpm * currentRPM;

    float e_k = targetRPM_filtered - currentRPM;

    if (useHinf) {
      u_k = (C[0]*x[0] + C[1]*x[1] + C[2]*x[2]) + D * e_k;
    } else {
      float P_k = Kp * e_k;
      u_k = P_k + I_old;
    }

    if (u_k > 255.0) u_sat = 255.0;
    else if (u_k < -255.0) u_sat = -255.0;
    else u_sat = u_k;

    if (u_sat >= 0) {
      digitalWrite(motorDir, LOW);  
      analogWrite(motorPWM, (int)u_sat);
    } else {
      digitalWrite(motorDir, HIGH); 
      analogWrite(motorPWM, (int)(-u_sat)); 
    }

    if (useHinf) {
      x_new[0] = (A[0][0]*x[0] + A[0][1]*x[1] + A[0][2]*x[2]) + B[0] * e_k;
      x_new[1] = (A[1][0]*x[0] + A[1][1]*x[1] + A[1][2]*x[2]) + B[1] * e_k;
      x_new[2] = (A[2][0]*x[0] + A[2][1]*x[1] + A[2][2]*x[2]) + B[2] * e_k;
      x[0] = x_new[0]; x[1] = x_new[1]; x[2] = x_new[2];
    } else {
      float I_new = I_old + (Kp * Ts / Ti) * e_k + (Ts / Tt) * (u_sat - u_k);
      I_old = I_new;
    }

    Serial.print("Target:"); Serial.print(targetRPM_filtered); Serial.print(",");
    Serial.print("Filtered_RPM:"); Serial.print(currentRPM_filtered); Serial.print(",");
    Serial.print("PWM:"); Serial.println(u_sat);

    previousTime = currentMillis;
    previousPos = currentPos;
  }
}

// 3. FUNCTION TO SWITCH CONTROLLERS DYNAMICALLY
void setControllerMode(int mode) {
  // Reset integrators and states to avoid jumps
  I_old = 0.0;
  for(int i=0; i<3; i++) x[i] = 0.0;
  
  useHinf = false;
  usePrefilter = false;

  switch(mode) {
    case 1: // INVERSE BASED
      Kp = 7.84; Ti = 0.500; Tt = 0.500;
      break;
    case 2: // LOOP SHAPING
      Kp = 8.00; Ti = 0.228; Tt = 0.228;
      break;
    case 3: // 2-DoF
      Kp = 8.00; Ti = 0.228; Tt = 0.228;
      usePrefilter = true; filterTau = 0.228;
      break;
    case 4: // H-INFINITY
      useHinf = true;
      break;
    default:
      break;
  }
}

void readEncoder() {
  if (digitalRead(encB) == HIGH) encoderPos++; 
  else encoderPos--; 
}