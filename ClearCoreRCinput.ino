// Code to read 3 servo inputs from an RC controller, and then drive output velocity based on the pwm signal values

//TODO: implement pre-programmed mode
//TODO: implement limit switches
//TODO: Consider using "inhibit" lines for disabling pitch axis, to prevent it from dropping
#include "ClearCore.h"

#define modePin A11
#define interruptPin A12

#define yawPin DI6
#define pitchPin DI7
#define rollPin DI8

#define yawMotor ConnectorM0
#define pitchMotor ConnectorM1
#define rollMotor ConnectorM2

//Parameters for parsing PWM inputs
const int noiseFloor = 200; //microseconds. Below this, the readings are not considered servo pulses.
const int minWidth = 1000; //microseconds
const int maxWidth = 2000; //microseconds
const int midPoint = 1500; //microseconds
const int deadZone = 20; //microseconds, dead zone around midPoint
const int timeout = 5000; //microseconds, so we dont spend to long waiting for a PWM pulse

bool useRCinputs; //This will track whether we are using RC or pre-programmed running modes.

// Select the baud rate to match the target serial device
#define baudRate 9600

int getPulseWidth(int pinNum){
  int rawValue = pulseIn(pinNum, HIGH, timeout); //TODO this might need to be inverted, if the numbers are way off
  
  //Return immediately if signal is below noise floor. This usually means there is no signal,
  //usually because the transmitter is disconnected or another fault has occured.
  if (rawValue < noiseFloor){
    //Serial.print(pinNum);
    //Serial.println(" Pulse width below noise floor");
    return midPoint;
  }
  
  //truncate rawValue to be between minWidth and maxWidth
  //this prevents us from ever getting wildly miscalibrated,
  //or getting weird wrap around values on the later mapping.
  if(rawValue < minWidth){
    Serial.print(pinNum);
    Serial.println(rawValue);
    rawValue = minWidth;
  } else if(rawValue > maxWidth){
    Serial.print(pinNum);
    Serial.println(rawValue);
    rawValue = maxWidth;
  }
  return rawValue;
}

/*
 * CommandVelocity
 *    Command the motor to move using a velocity of commandedVelocity
 *    Prints the move status to the USB serial port
 * Parameters:
 *    int pulseWidth  - The width of the input servo pulse
 *    MotorDriver motor - the motor to drive
 */
void CommandVelocity(int pulseWidth, ClearCore::MotorDriver motor) { 
  int dutyCycle = 128;
  //dont move if in the deadZone
  //Serial.print(motor.InputAConnector());
  if(abs(pulseWidth-midPoint)<=deadZone){
    //leave duty cycle at the midpoint of 128
  }
  else if (pulseWidth < midPoint){ //move backwards
    dutyCycle = map(pulseWidth, minWidth, midPoint-deadZone, 1, 128);
    Serial.println(dutyCycle);
  } else { //move forwards
    dutyCycle = map(pulseWidth, midPoint+deadZone, maxWidth, 128, 254);
    Serial.println(dutyCycle);
  }
  motor.MotorInBDuty(dutyCycle);
}

//interrupt routine
void MyCallback() {
  disableMotors();
}

void enableMotors(){
  //make sure that any interrupts happen after, so we arent partially enabled
  noInterrupts();
  //check if the estop switch is disabled
  //true means grounded means e stop is not pressed
  if(digitalRead(interruptPin)){
    // Enables the motors
    yawMotor.EnableRequest(true);
    pitchMotor.EnableRequest(true);
    rollMotor.EnableRequest(true);
    //turn on indicator LED
    digitalWrite(LED_BUILTIN, true);
    if(Serial){
      Serial.println("Motors Enabled");
    }
  }
  interrupts();
}
void disableMotors(){
  // Enables the motor
  yawMotor.EnableRequest(false);
  pitchMotor.EnableRequest(false);
  rollMotor.EnableRequest(false);
  digitalWrite(LED_BUILTIN, false);
  if(Serial){
    Serial.println("Motors disabled");
  }
}

void setup() {
  pinMode(yawPin, INPUT);  //Set connector as an Input
  pinMode(pitchPin, INPUT);  //Set connector as an Input
  pinMode(rollPin, INPUT);  //Set connector as an Input

  // Set up the interrupt pin in digital input mode.
  pinMode(interruptPin, INPUT);
  pinMode(modePin, INPUT);

  //Set RC vs autonomous mode. Only checks once on bootup, so changing requires rebooting the device
  useRCinputs = !digitalRead(modePin); //if modePin is floating, use RC. If grounded, use pre-programmed

  // Set an ISR to be called when the interrupt pin is no longer grounded (aka switch is opened)
  //note that "Falling" means the signal "fell" to false, not that the voltage level fell.
  //I think this is dumb
  attachInterrupt(digitalPinToInterrupt(interruptPin), MyCallback, FALLING);

  // Sets all motor connectors to the correct mode for Follow Digital Velocity, Bipolar PWM mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_PWM);

  // Set up serial communication
  Serial.begin(baudRate);
}

void loop() {
  enableMotors();
  if(useRCinputs){
    int yawWidth = getPulseWidth(yawPin);
    CommandVelocity(yawWidth, yawMotor);
    int pitchWidth = getPulseWidth(pitchPin);
    CommandVelocity(pitchWidth, pitchMotor);
    int rollWidth = getPulseWidth(rollPin);
    CommandVelocity(rollWidth, rollMotor);
  } else {
    //TODO: implement the preprogrammed mode!
    Serial.println("Turret is in preprogrammed mode, which does not exist");
  }
}