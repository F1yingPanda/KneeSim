 #include <AccelStepper.h>
 #include <Encoder.h>
 
// Define some steppers and the pins the will use (DRIVER, PUL-, DIR-)
AccelStepper stepper1(AccelStepper::DRIVER, 22, 28); // Flexion-Extension
AccelStepper stepper2(AccelStepper::DRIVER, 23, 29); // Internal-External
AccelStepper stepper3(AccelStepper::DRIVER, 24, 30); // Varus-Valgrus
AccelStepper stepper4(AccelStepper::DRIVER, 25, 31); // Anterior-Posterior 1
AccelStepper stepper5(AccelStepper::DRIVER, 26, 32); // Anterior-Posterior 2
AccelStepper stepper6(AccelStepper::DRIVER, 27, 33); // Medial-Lateral

// Define Encoder pins (pinA, pinB)
Encoder encoder_FE(18,51);
Encoder encoder_IE(19,52);
Encoder encoder_VV(20,53);
const int pinZ_FE = 48;
const int pinZ_IE = 49;
const int pinZ_VV = 50;

// Define useful conversions
const int MICROSTEPPING = 16; // Driver Setting
const int STEPS_PER_DEG = 200.0f * MICROSTEPPING / 360.0f; // For rotation
const int STEPS_PER_MM = 200.0f * MICROSTEPPING / 5.0f; // For translation

void setup()
{  
    // Initialize serial communicationat 115200 bits/sec
    Serial.begin(115200);

    // Initialize max speeds & accelerations
    stepper1.setMaxSpeed(200);
    stepper1.setAcceleration(100.0);
    stepper2.setMaxSpeed(200);
    stepper2.setAcceleration(100.0);
    stepper3.setMaxSpeed(200);
    stepper3.setAcceleration(100.0);
    stepper4.setMaxSpeed(200);
    stepper4.setAcceleration(100.0);
    stepper5.setMaxSpeed(200);
    stepper5.setAcceleration(100.0);
    stepper6.setMaxSpeed(200);
    stepper6.setAcceleration(100.0);

    // Design homing sequence
}
 
void loop()
{
    if (Serial.available()) {
    
    // Reads in degrees and mm  
    float FE_deg = Serial.parseFloat();
    float IE_deg = Serial.parseFloat();
    float VV_deg = Serial.parseFloat();
    float AP_mm = Serial.parseFloat();
    float ML_mm = Serial.parseFloat();
    
    // Converts from float to long
    long FE = FE_deg * STEPS_PER_DEG;
    long IE = IE_deg * STEPS_PER_DEG;
    long VV = VV_deg * STEPS_PER_DEG;
    long AP = AP_mm * STEPS_PER_MM;
    long ML = ML_mm * STEPS_PER_MM;

    // Moves to new position until finished
    stepper1.runToNewPosition(FE);
    stepper2.runToNewPosition(IE);
    stepper3.runToNewPosition(VV);
    stepper4.runToNewPosition(AP);
    stepper5.runToNewPosition(AP);
    stepper6.runToNewPosition(ML);
    }
}