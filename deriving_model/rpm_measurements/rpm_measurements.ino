// define pins and variables
const int motorPWM = 4;
const int motorDir = 5;
const int encA = 18; 
const int encB = 19;

volatile long encoderPos = 0; 

// vars for RPM calculation
long previousPos = 0;
unsigned long previousTime = 0;

const float PPR = 2150.0;                  // PPR TOU MOTER

// --- NEW FEATURES: Μεταβλητές για τον αυτοματισμό του PWM ---
int currentPWM = 150;
int printCount = 0;
bool isNewPWM = true;
bool finished = false;

void setup() {
  Serial.begin(115200);
  
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encA), readEncoder, RISING);

  // Ξεκινάμε με το μοτέρ να πηγαίνει μπροστά
  digitalWrite(motorDir, LOW);
  
  // Δίνουμε 2 δευτερόλεπτα κενό στην αρχή για να ανοίξεις το Serial Monitor με την ησυχία σου
  delay(2000); 
}

void loop() {
  // Αν τελειώσαμε με το 250, μην κάνεις τίποτα άλλο
  if (finished) return;

  // Αν μόλις αλλάξαμε PWM, τύπωσε την επικεφαλίδα
  if (isNewPWM) {
    analogWrite(motorPWM, currentPWM);
    
    Serial.println();
    Serial.print("PWM = ");
    Serial.println(currentPWM);
    Serial.println("------------");
    
    isNewPWM = false;
    
    // Μηδενίζουμε τους χρόνους για να είναι ακριβές το πρώτο δείγμα του νέου PWM
    previousTime = millis();
    previousPos = encoderPos;
    return; // Περιμένουμε την επόμενη λούπα για να ξεκινήσει η μέτρηση
  }

  unsigned long currentTime = millis(); // read current time
  unsigned long timeDiff = currentTime - previousTime; // calc diff

  // Υπολογισμός κάθε 100 ms
  if (timeDiff >= 100) {
    
    long currentPos = encoderPos;            // read current pos
    long posDiff = currentPos - previousPos; // calc "clicks" during that dt

    // CALC RPM
    float rpm = ((float)posDiff / PPR) * (60000.0 / timeDiff);

    // Τυπώνουμε ΜΟΝΟ το RPM όπως ζήτησες
    Serial.println(abs(rpm)); 

    printCount++; // Αυξάνουμε τον μετρητή των εκτυπώσεων

    // Αν τυπώσαμε 30 τιμές, πάμε στο επόμενο PWM
    if (printCount >= 30) {
      currentPWM += 10;
      printCount = 0;
      isNewPWM = true;

      // Αν ξεπεράσαμε το 250, τελειώσαμε
      if (currentPWM > 250) {
        finished = true;
        analogWrite(motorPWM, 0); // Σταματάμε το μοτέρ
        Serial.println();
        Serial.println("DONE! ALL MEASUREMENTS COMPLETED.");
      }
    }

    // update time and pos variables
    previousPos = currentPos;
    previousTime = currentTime;
  }
}

// interrupt function
void readEncoder() {
  if (digitalRead(encB) == HIGH) {
    encoderPos++; 
  } else {
    encoderPos--; 
  }
}