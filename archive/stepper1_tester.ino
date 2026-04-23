#include <AccelStepper.h>
#include <Encoder.h>

// Define stepper objects (pins A,B)
AccelStepper stepper1(AccelStepper::DRIVER, 22, 28); //Varus-Valgus (-10,10)
AccelStepper stepper2(AccelStepper::DRIVER, 23, 29); //Flexion-Extension (-20,160)
AccelStepper stepper3(AccelStepper::DRIVER, 24, 30); //Internal-External (-65,65) (never disable)
AccelStepper stepper4(AccelStepper::DRIVER, 25, 31); // Anterior-Posterior 1 (-20,20)
AccelStepper stepper5(AccelStepper::DRIVER, 26, 32); // Anterior-Posterior 2 (-20,20)
AccelStepper stepper6(AccelStepper::DRIVER, 27, 33); // Medial-Lateral (-20,20)

// Define encoder objects (pins A,B)
Encoder encoder_FE(3,51);
Encoder encoder_IE(2,50);
Encoder encoder_VV(18,52);
// Initialize encoder reading values
long FE_position = 0;
long IE_position = 0;
long VV_position = 0;

// Define useful conversions
const int MICROSTEPPING = 8; // Driver Setting, 1600 steps/rev 
const float STEPS_PER_DEG = 200.0f * MICROSTEPPING / 360.0f; // For rotation
const float STEPS_PER_MM = 200.0f * MICROSTEPPING / 5.0f; // For translation
const float ENCODER_COUNTS_PER_DEG = (5000.0f *4) / 360.0f;

// Define encoder index pin variables
volatile bool triggeredFE = false;
volatile bool triggeredIE = false;
volatile bool triggeredVV = false;
bool step2 = false;

// Define Serial parsing variables
char input[64];
float values[5];
char buffer[20];

void setup() {
  // Initialize Serial communication port
  Serial.begin(9600);

  // Define encoder index interrupt pins
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), ISR_IE, RISING);
  attachInterrupt(digitalPinToInterrupt(21), ISR_FE, RISING);
  attachInterrupt(digitalPinToInterrupt(20), ISR_VV, RISING);

  // Set speed and accelerations of all stepper objects
  stepper1.setMaxSpeed(100); // In Steps/sec, this gives us about 16 seconds/rotation
  stepper1.setAcceleration(100);
  stepper2.setMaxSpeed(100);
  stepper2.setAcceleration(100);
  stepper3.setMaxSpeed(100); 
  stepper3.setAcceleration(100);
  // stepper4.setMaxSpeed(100);
  // stepper4.setAcceleration(100.0);
  // stepper5.setMaxSpeed(100);
  // stepper5.setAcceleration(100.0);
  // stepper6.setMaxSpeed(100);
  // stepper6.setAcceleration(100.0);

  // Set enable pins for steppers
  stepper1.setEnablePin(34);
  stepper1.enableOutputs();
  stepper2.setEnablePin(35);
  stepper2.enableOutputs();
  stepper3.setEnablePin(36);
  stepper3.enableOutputs();
  // stepper4.setEnablePin(37);
  // stepper4.disableOutputs();
  // stepper5.setEnablePin(38);
  // stepper5.disableOutputs();
  // stepper6.setEnablePin(38);
  // stepper6.disableOutputs();
  
  // Go to home position before waiting for next location
  home(0,0,0);
}

void loop() {

  // Checks if a command has been given
  if (Serial.available()) {
    // reads serial into input buffer and delimits it with \0
    int len = Serial.readBytesUntil('\n', input, sizeof(input) - 1);
    input[len] = '\0';  // null terminate

    // delimits input by commas
    char *token = strtok(input, ",");
    int i = 0;

    // puts delimited input into values array
    while (token != NULL && i < 5) {
      values[i++] = atof(token);  //handles floats
      token = strtok(NULL, ",");
    }
      
    // Converts from deg or mm float to long steps
    long FE = values[0] * STEPS_PER_DEG;
    long IE = values[1] * STEPS_PER_DEG;
    long VV = values[2] * STEPS_PER_DEG;
    long AP = values[3] * STEPS_PER_MM;
    long ML = values[4] * STEPS_PER_MM;
    
    // stepper4.enableOutputs();
    // stepper5.enableOutputs();
    // stepper4.moveTo(AP);
    // stepper5.moveTo(AP);

    // while (stepper4.distanceToGo() != 0 ||
    //        stepper5.distanceToGo() != 0){
    //   stepper4.run();
    //   stepper5.run();
    // }
    // stepper4.disableOutputs();
    // stepper5.disableOutputs();

    // stepper6.moveTo(ML);
    // while (stepper6.distanceToGo() != 0){
    //   stepper6.run();
    // }
    // stepper6.disableOutputs();

    //home();
    //stepper1.enableOutputs();
    //stepper2.enableOutputs();
    stepper1.moveTo(FE);
    stepper2.moveTo(IE);
    stepper3.moveTo(VV);

    while (stepper1.distanceToGo() != 0 ||
           stepper2.distanceToGo() != 0 ||
           stepper3.distanceToGo() != 0 ){
      stepper1.run();
      stepper2.run();
      stepper3.run();
    }

    // clears desired location array just in case
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;
    values[3] = 0;
    values[4] = 0;

    // Read encoders after move is finished   
    FE_position = encoder_FE.read();
    IE_position = encoder_IE.read();
    VV_position = encoder_VV.read();

    // Converts encoder steps to degrees 
    float a = FE_position/ENCODER_COUNTS_PER_DEG;
    float b = IE_position/ENCODER_COUNTS_PER_DEG;
    float c = VV_position/ENCODER_COUNTS_PER_DEG;

    // turns a,b,c degrees into character arrays strA, strB, strC with 2 decimal places
    char strA[10], strB[10], strC[10];
    dtostrf(a, 7, 2, strA);
    dtostrf(b, 7, 2, strB);
    dtostrf(c, 7, 2, strC);

    // prints degree character arrays delimited by commas and terminated by \n
    snprintf(buffer, sizeof(buffer), "%s,%s,%s", strA, strB, strC);
    Serial.write(buffer, 23);
    Serial.write('\n');  // total size = 24 bytes
  }
}

// homing sequence for all motors
void home(long FE, long IE, long VV) {
  // set inital guess movements
  //stepper2.moveTo(-45*STEPS_PER_DEG);
  stepper3.moveTo(-180*STEPS_PER_DEG);
  stepper1.moveTo(365*STEPS_PER_DEG);

  // Runs motors while all not homed
  while (triggeredFE == false || triggeredVV == false){
    stepper2.run();
    stepper3.run();    
    stepper1.run();
    
    // // Stops IE motor if homed
    // if (triggeredIE == true){
    //   stepper2.moveTo(0*STEPS_PER_DEG);
    //   stepper2.setCurrentPosition(0*STEPS_PER_DEG);
    //   encoder_IE.write(0*ENCODER_COUNTS_PER_DEG);
    // }

    // Stops FE motor if homed
    if (triggeredFE == true){
      stepper3.moveTo(-90*STEPS_PER_DEG);  
      stepper3.setCurrentPosition(-90*STEPS_PER_DEG);
    }

    // Stops VV motor if homed
    else if (triggeredVV == true){
      stepper1.moveTo(90*STEPS_PER_DEG);  
      stepper1.setCurrentPosition(90*STEPS_PER_DEG);
    }
  }
  // triggeredIE = false;
  triggeredFE = false;
  triggeredVV = false;
  
  // Moves to desired location
  stepper1.moveTo(VV);
  stepper2.moveTo(IE);
  stepper3.moveTo(FE);

  while (stepper1.distanceToGo() != 0 ||
          stepper2.distanceToGo() != 0 ||
          stepper3.distanceToGo() != 0 ){

    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  encoder_FE.write(0*ENCODER_COUNTS_PER_DEG);
  encoder_IE.write(0*ENCODER_COUNTS_PER_DEG);
  encoder_VV.write(0*ENCODER_COUNTS_PER_DEG);

}

// Interrupts for encoder index pins
void ISR_FE() {
  triggeredFE = true;}
void ISR_IE() {
  triggeredIE = true;}
void ISR_VV() {
  triggeredVV = true;}
