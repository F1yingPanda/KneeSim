 #include <AccelStepper.h>
 #include <Encoder.h>

bool home_position();
// We need to make an interrupt function for the home-defining limit switches

// Steppers/Power Screws move 1.8 deg/step, 0.225 with 8 microstepping
// Power Screws move 5mm/rotation

// Define some steppers and the pins the will use (DRIVER, PUL-, DIR-)
AccelStepper stepper1(AccelStepper::DRIVER, 22, 28); // Flexion-Extension
AccelStepper stepper2(AccelStepper::DRIVER, 23, 29); // Internal-External
AccelStepper stepper3(AccelStepper::DRIVER, 24, 30); // Varus-Valgrus
AccelStepper stepper4(AccelStepper::DRIVER, 25, 31); // Anterior-Posterior 1
AccelStepper stepper5(AccelStepper::DRIVER, 26, 32); // Anterior-Posterior 2
AccelStepper stepper6(AccelStepper::DRIVER, 27, 33); // Medial-Lateral

// Define Encoder pins (pinA, pinB) and create encoder the encoders will write to
Encoder encoder_FE(18,51);
Encoder encoder_IE(19,52);
Encoder encoder_VV(20,53);
const int pinZ_FE = 48;
const int pinZ_IE = 49;
const int pinZ_VV = 50;
long FE_position = 0;
long IE_position = 0;
long VV_position = 0;

// Define useful conversions
const int MICROSTEPPING = 8; // Driver Setting, 1600 steps/rev 
const int STEPS_PER_DEG = 200.0f * MICROSTEPPING / 360.0f; // For rotation
const int STEPS_PER_MM = 200.0f * MICROSTEPPING / 5.0f; // For translation

// define read in buffer and read out buffer
char input[64];
float values[5];
char buffer[20];


void setup()
{  
    // Initialize serial communicationat 115200 bits/sec
    Serial.begin(115200);
    while(!Serial){};

    // Initialize max speeds & accelerations
    stepper1.setMaxSpeed(100); //In Steps/sec, this gives us about 16 seconds/rotation
    stepper1.setAcceleration(100.0);
    stepper2.setMaxSpeed(100);
    stepper2.setAcceleration(100.0);
    stepper3.setMaxSpeed(100);
    stepper3.setAcceleration(100.0);
    stepper4.setMaxSpeed(100);
    stepper4.setAcceleration(100.0);
    stepper5.setMaxSpeed(100);
    stepper5.setAcceleration(100.0);
    stepper6.setMaxSpeed(100);
    stepper6.setAcceleration(100.0);

    // Design homing sequence

    //Tessa: Working under the assumption that the limit switches dictating home position will be at a minima or maxima for the machine
    // ( because otherwise they'd get in the way while in operation ), we can design a loop which will iteratively check
    // for if the switches are active, and then move the motors towards that minima if they aren't. 
    // Primary concern I have with this logic is, we don't want the motors to move *too* far between checks
 
    // solution: design an interrupt for the system that will detect when the switches are pressed? need to relearn
    // how interrupts work and how to create one :/ ( notes for myself, not a to-do list for others )

    // For now, can simply have the loop move our motors one step at a time between checks - not fast, but it can do the job. 
    int numsteps_x = 0; 
    int numsteps_y = 0; 
    int numsteps_FE = 0;
    int numsteps_IE = 0;
    int numsteps_VV = 0;
    
    while(home_position == false) {
    if ( x_home == false ) { // Working under the assumption that the limit switch will return a value of either 0 or 1
    //we could also use the stepper.stepForward() or stepBackward function
    numsteps_x = numsteps_x - 1; // Signed negative for reverse motion - may need to be positive depending on position of limit switches
    stepper4.runToNewPosition(MICROSTEPPING*numsteps_x); 
    stepper5.runToNewPosition(MICROSTEPPING*numsteps_x); 
    }
    stepper4.setCurrentPosition(); //Sets position as 0
    stepper5.setCurrentPosition();

    if (y_home == false ) {
    numsteps_y = numsteps_y - 1; 
    stepper6.runToNewPosition(MICROSTEPPING*numsteps_y); 
    }
    stepper6.setCurrentPosition();

    if ( FE_home == false ) {
    numsteps_FE = numsteps_FE - 1; 
    stepper1.runToNewPosition(MICROSTEPPING*numsteps_FE); 
    } 
    stepper1.setCurrentPosition();
    encoder_FE.write(0);
     
    if ( IE_home == false ) {
    numsteps_IE = numsteps_IE - 1;
    stepper2.runToNewPosition(MICROSTEPPING*numsteps_IE); 
    }
    stepper2.setCurrentPosition();
    encoder_IE.write(0);

    if ( VV_home == false ) {
    numsteps_VV = numsteps_VV - 1; 
    stepper3.runToNewPosition(MICROSTEPPING*numsteps_VV); 
    }
    stepper3.setCurrentPosition();
    encoder_VV.write(0);

    numsteps_x = 0; 
    numsteps_y = 0; 
    numsteps_FE = 0;
    numsteps_IE = 0;
    numsteps_VV = 0;
}
 
void loop()
{
    // Allows motors to run
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    stepper5.run();
    stepper6.run();

    if (Serial.available()) {
     // reads serial into input buffer and delimits it with \0
      int len = Serial.readBytesUntil('\n', input, sizeof(input) - 1);
      input[len] = '\0';  // null terminate

      // delimits input by commas
      char *token = strtok(input, ",");
      int i = 0;

     // puts delimited input into values array
      while (token != NULL && i < 5) {
        values[i++] = atof(token);  // <-- handles floats
        token = strtok(NULL, ",");
      }
        
        // Converts from deg/mm float to long steps
        long FE = values[0] * STEPS_PER_DEG;
        long IE = values[1] * STEPS_PER_DEG;
        long VV = values[2] * STEPS_PER_DEG;
        long AP = values[3] * STEPS_PER_MM;
        long ML = values[4] * STEPS_PER_MM;

        // Moves to new position until finished
        stepper1.moveTo(FE);
        stepper2.moveTo(IE);
        stepper3.moveTo(VV);
        stepper4.moveTo(AP);
        stepper5.moveTo(AP);
        stepper6.moveTo(ML);

        // Read encoders after move is finished
        FE_position = encoder_FE.read();
        IE_position = encoder_IE.read();
        VV_position = encoder_IE.read();;

        // Converts steps to degrees 
        long a = FE_position/STEPS_PER_DEG;
        long b = IE_position/STEPS_PER_DEG;
        long c = VV_position/STEPS_PER_DEG;

        // Converts degrees into a char array, terminated with newline
        snprintf(buffer, sizeof(buffer), "%+04ld,%+04ld,%+04ld", a, b, c);
        Serial.write(buffer, 14);
        Serial.write('\n');  // total size = 15 bytes
    }
}

bool home_position() {
    if (x_home == 0 || y_home == 0 || FE_home == 0 || IE_home == 0 || VV_home == 0 ) return false;
    else return true; 
}
