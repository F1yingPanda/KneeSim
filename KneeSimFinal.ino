#include <AccelStepper.h>
#include <Encoder.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

// Define stepper objects (pins A,B)
AccelStepper stepperVV(AccelStepper::DRIVER, 32, 33); //Varus-Valgus (-10,10)
AccelStepper stepperIE(AccelStepper::DRIVER, 28, 29); //Internal-External (-65,65) 
AccelStepper stepperFE(AccelStepper::DRIVER, 30, 31); //Flexion-Extension (-20,160)
AccelStepper stepperAP1(AccelStepper::DRIVER, 22, 23); // Anterior-Posterior 1 (-20,20)
AccelStepper stepperAP2(AccelStepper::DRIVER, 24, 25); // Anterior-Posterior 2 (-20,20)
AccelStepper stepperML(AccelStepper::DRIVER, 26, 27); // Medial-Lateral (-20,20)

// Define encoder objects (pins A,B)
Encoder encoderFE(50,51);
Encoder encoderIE(48,49);
Encoder encoderVV(52,53);

// Define TOF sensor objects
VL53L4CD sensorAP(&Wire,0x59); // 0x59 is default address
VL53L4CD sensorML(&Wire,0x59);

// Initialize encoder reading values
long positionFE = 0;
long positionIE = 0;
long positionVV = 0;
long positionAP = 0;
long positionML = 0;

// Define useful conversions
const float MICROSTEPPING = 8; // Driver Setting, 1600 steps/rev 
const float STEPS_PER_DEG = 200.0f * MICROSTEPPING / 360.0f; // For rotation
const float STEPS_PER_MM = 200.0f * MICROSTEPPING / 5.0f; // For translation
const float ENCODER_COUNTS_PER_DEG = (5000.0f * 4) / 360.0f;

// Define encoder index pin variables
volatile bool triggeredFE = false;
volatile bool triggeredIE = false;
volatile bool triggeredVV = false;

// Define Serial parsing variables
char input[64];
float values[5];
char buffer[42];

void setup() {
  // Initialize Serial and I2C communication port
  Serial.begin(9600);
  DEV_I2C.begin();

  // TOF Sensor XShut pins
  pinMode(4, OUTPUT);
  pinMode(5,OUTPUT);
  delay(10);

  // Sensor 1 rewrite address
  digitalWrite(4, HIGH); // XSHUT pin for AP
  delay(10);
  sensorAP.begin();
  sensorAP.InitSensor();
  sensorAP.VL53L4CD_SetI2CAddress(0x54);
  
  //Sensor 2 rewrite address  
  digitalWrite(5, HIGH); // XSHUT pin for ML
  delay(10);
  sensorML.begin();
  sensorML.InitSensor();
  sensorML.VL53L4CD_SetI2CAddress(0x56); 

  // Start TOF sensor reading
  sensorAP.VL53L4CD_StartRanging();
  sensorML.VL53L4CD_StartRanging();

  // Define encoder index interrupt pins
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), ISR_IE, RISING);
  attachInterrupt(digitalPinToInterrupt(3), ISR_FE, RISING);
  attachInterrupt(digitalPinToInterrupt(19), ISR_VV, RISING);

  // Set speed and accelerations of all stepper objects
  stepperVV.setMaxSpeed(100); // In Steps/sec, this gives us about 16 seconds/rotation
  stepperVV.setAcceleration(100);
  stepperFE.setMaxSpeed(100);
  stepperFE.setAcceleration(100);
  stepperIE.setMaxSpeed(100); 
  stepperIE.setAcceleration(100);
  stepperAP1.setMaxSpeed(500);
  stepperAP1.setAcceleration(300.0);
  stepperAP2.setMaxSpeed(500);
  stepperAP2.setAcceleration(300.0);
  stepperML.setMaxSpeed(500);
  stepperML.setAcceleration(300.0);
  
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

    // Makes sure input does not exceed boundaries, but allows rest of motors to continue
    if (values[0]<-20 || values[0]>160){
      FE = 0;
    }
    if (values[1]<-65 || values[1] > 65){
      IE = 0;
    }
    if (values[2]<-10 || values[2]>10){
      VV = 0;
    }
    if (values[3]<-20 || values[3]>20){
      AP = 0;
    }
    if (values[4]<-20 || values[4]>20){
      ML = 0;
    }

    // Sets destinations for each motor
    stepperVV.moveTo(VV);
    stepperFE.moveTo(-FE);
    stepperIE.moveTo(IE);
    stepperAP1.moveTo(AP);
    stepperAP2.moveTo(AP);
    stepperML.moveTo(ML);

    // Moves motors and encoders
    while (stepperVV.distanceToGo() != 0 ||
           stepperIE.distanceToGo() != 0 ||
           stepperFE.distanceToGo() != 0 ||
           stepperAP1.distanceToGo() != 0 ||
           stepperAP2.distanceToGo() != 0 ||
           stepperML.distanceToGo() != 0 ){
      stepperVV.run();
      stepperIE.run();
      stepperFE.run();
      stepperAP1.run();
      stepperAP2.run();
      stepperML.run();
      encoderVV.read();
      encoderIE.read();
      encoderFE.read();
    }

    // clears desired location array just in case
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;
    values[3] = 0;
    values[4] = 0;

    // Read encoders after move is finished   
    positionFE = encoderFE.read();
    positionIE = encoderIE.read();
    positionVV = encoderVV.read();
    positionAP = readSensor(sensorAP);
    positionML = readSensor(sensorML);

    // Converts encoder steps to degrees 
    float a = positionFE/ENCODER_COUNTS_PER_DEG;
    float b = positionIE/ENCODER_COUNTS_PER_DEG;
    float c = positionVV/ENCODER_COUNTS_PER_DEG;
    float d = positionAP;
    float e = positionML;

    // turns a,b,c degrees into character arrays strA, strB, strC with 2 decimal places
    char strA[10], strB[10], strC[10], strD[10], strE[10];
    dtostrf(a, 7, 2, strA);
    dtostrf(b, 7, 2, strB);
    dtostrf(c, 7, 2, strC);
    dtostrf(d, 7, 2, strD);
    dtostrf(e, 7, 2, strE);

    // prints degree character arrays delimited by commas and terminated by \n
    snprintf(buffer, sizeof(buffer), "%s,%s,%s,%s,%s", strA, strB, strC, strD, strE);
    Serial.write(buffer, 39);
    Serial.write('\n');  // total size = 42 bytes
  }
}

// homing sequence for all motors
void home(long FE, long IE, long VV) {
  triggeredFE = false;
  triggeredIE = false;
  triggeredVV = false;
  
  // set inital guess movements
  stepperFE.move(-120*STEPS_PER_DEG);
  stepperIE.move(120*STEPS_PER_DEG);
  stepperVV.move(365*STEPS_PER_DEG);

  // Runs motors while all not homed
  while (triggeredFE == false || triggeredVV == false){
    
    // IE Encoder does not work
    // // Stops IE motor if homed
    // if (!triggeredIE){
    //   stepperIE.run();
    // }

    // Stops FE motor if homed
    if (!triggeredFE){
      stepperFE.run();  
    }

    // Stops VV motor if homed
    if (!triggeredVV){
      stepperVV.run();  
    }
  }
   
  // Sets offset for home position
  stepperFE.setCurrentPosition(-90*STEPS_PER_DEG);
  stepperVV.setCurrentPosition(-90*STEPS_PER_DEG);
  stepperIE.setCurrentPosition(0);

  // Moves to desired location
  stepperVV.moveTo(VV);
  stepperIE.moveTo(IE);
  stepperFE.moveTo(FE);

  while (stepperVV.distanceToGo() != 0 ||
          stepperIE.distanceToGo() != 0 ||
          stepperFE.distanceToGo() != 0 ){

    stepperVV.run();
    stepperIE.run();
    stepperFE.run();
  }

  // Zeros encoder readings
  encoderFE.write(0*ENCODER_COUNTS_PER_DEG);
  encoderIE.write(0*ENCODER_COUNTS_PER_DEG);
  encoderVV.write(0*ENCODER_COUNTS_PER_DEG);
  triggeredFE = false;
  triggeredIE = false;
  triggeredVV = false;
  return;
}

// Read TOF sensors
int readSensor(VL53L4CD &sensor){
  uint8_t ready = 0;
  int offset = 0; // find out real offset in mm
  
  // Checks if new data is available
  do{
    sensor.VL53L4CD_CheckForDataReady(&ready);
  } while (!ready);

  // gets result from sensor
  VL53L4CD_Result_t result;
  sensor.VL53L4CD_GetResult(&result);
  sensor.VL53L4CD_ClearInterrupt();

  return (result.distance_mm-offset);
}

// Interrupts for encoder index pins
void ISR_FE() {
  triggeredFE = true;
}
void ISR_IE() {
  triggeredIE = true;
}
void ISR_VV() {
  triggeredVV = true;
}
