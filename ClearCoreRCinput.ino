// Code to read 3 channels from an RC controller, and then drive motor output velocity based on the signal values

//TODO: implement pre-programmed mode
//TODO: implement limit switches

//https://www.teknic.com/files/downloads/manual_install_instructions_arduino.pdf
#include "ClearCore.h"

//https://github.com/bolderflight/sbus
#include "sbus.h"

//toggle some of the print statements
#define DEBUG true

//the pin which switches between RC and preprogrammed modes
#define modePin A11

//the pin which handles the estop
#define interruptPin A12

//when using PWM inputs, which pins are the inputs connected to?
#define yawPin DI6
#define pitchPin DI7
#define rollPin DI8

//which connectors are the motors on?
#define yawMotor ConnectorM0
#define pitchMotor ConnectorM1
#define rollMotor ConnectorM2

//Parameters for parsing PWM inputs
const int noiseFloor = 200; //microseconds. Below this, the readings are not considered servo pulses.
const int minWidth = 1000; //microseconds
const int maxWidth = 2000; //microseconds
const int midPoint = 1500; //microseconds
const int deadZone = 20; //microseconds, dead zone around midPoint
const int timeout = 20000; //microseconds, so we dont spend to long waiting for a PWM pulse

      bool useRCinputs; //This will track whether we are using RC or pre-programmed running modes.

// on the clearcore we need to use an sbus RC receiver, it doesnt have a fast enough clock
// to read normal PWM signals using pulseIn.
const bool useSBUS = true; //toggle whether we are using pulseIn or decoding sbus to get signal from the receiver

//initialize the input values to the midpoint, aka stopped.
int yawWidth = midPoint; //the width (in microseconds) of the last signal received
int pitchWidth = midPoint;
int rollWidth = midPoint;

// Select the baud rates to match the target serial devices
#define baudRate 115200

//sbus config
#define sbusBaudRate 100000
bfs::SbusRx sbus_rx(&Serial0);
bfs::SbusData data;
const int sbusMin = 192;
const int sbusMax = 1792;

int pulseWidthToDutyCycle(int rawValue){
  //Return immediately if signal is below noise floor. This usually means there is no signal,
  //usually because the transmitter is disconnected or another fault has occured.
  if (rawValue < noiseFloor){
    //Serial.print(pinNum);
    return midPoint;
  }
  
  //truncate rawValue to be between minWidth and maxWidth
  //this prevents us from ever getting wildly miscalibrated,
  //or getting weird wrap around values on the later mapping.
  if(rawValue < minWidth){
    rawValue = minWidth;
  } else if(rawValue > maxWidth){
    rawValue = maxWidth;
  }
  //Serial.println(rawValue);

  int dutyCycle;
  //parse this rawValue into a single byte for PWM output
  if(abs(rawValue-midPoint)<=deadZone){
    //leave duty cycle at the midpoint of 128 if we're within the deadzone
    dutyCycle = 128;
  } else {
    if (rawValue < midPoint){ //move backwards
      dutyCycle = map(rawValue, minWidth, midPoint-deadZone, 1, 128);
    } else if (rawValue > midPoint){ //move forwards
      dutyCycle = map(rawValue, midPoint+deadZone, maxWidth, 128, 254);
    } else {
      dutyCycle = 128;
    }
  }
  return dutyCycle;
}

/*
 * CommandVelocity
 *    Command the motor to move using a velocity of commandedVelocity
 * Parameters:
 *    int pulseWidth  - The width of the input servo pulse
 *    MotorDriver motor - the motor to drive
 */
void CommandVelocity(int dutyCycle, ClearCore::MotorDriver motor) {
  //make sure values are in [2, 253]
  dutyCycle = min(dutyCycle, 253);
  dutyCycle = max(2, dutyCycle);
  motor.MotorInBDuty(dutyCycle);
}

//interrupt routine
void MyCallback() {
  disableMotors();
}

void enableMotors(){
  //check if the estop switch is disabled
  //true means grounded means e stop is not pressed
  if(digitalRead(interruptPin)){
    //make sure that any interrupts happen after, so we arent partially enabled
    noInterrupts();
    yawMotor.EnableRequest(true);
    pitchMotor.EnableRequest(true); //enable the enable pins. We need this to do the first enable.
    pitchMotor.MotorInAState(false); //remove the inhibit signal
    rollMotor.EnableRequest(true);
    //turn on indicator LED
    digitalWrite(LED_BUILTIN, true);
    interrupts(); //allow interrupts again
    if(Serial){
      Serial.println("Motors Enabled");
    }
  }
}
void disableMotors(){
  // Enables the motor
  yawMotor.EnableRequest(false);
  //pitchMotor.EnableRequest(false);
  pitchMotor.MotorInAState(true); //use inhibit instead of enable, so it holds position.
  rollMotor.EnableRequest(false);
  digitalWrite(LED_BUILTIN, false);
  delay(1); //not exactly sure why this is needed, but it did not trigger reliably without it
  if(Serial){
    Serial.println("Motors Disabled");
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

  // Set up serial communication over usb
  Serial.begin(baudRate);

  //set up the sbus read
  Serial0.begin(sbusBaudRate);
  sbus_rx.Begin();
}

void loop() {
  //if(Serial){Serial.println("The code for this lives at github.com/alvarezb/ClearCoreRCinput");}
  enableMotors();
  if(useRCinputs){
    //you need to use sbus on the clearcore, but you can use normal PWM instead on an arduino
    if(useSBUS){
      if (sbus_rx.Read()) {
        data = sbus_rx.data();
        if(data.failsafe){
          //the failsafe triggers when the receiver disconnects.
          //Set all the values to their mid (aka stopped) point
          yawWidth = midPoint;
          pitchWidth = midPoint;
          rollWidth = midPoint;
        } else {
          //read the data from the SBUS. We're only using channels 1-3 for this turret.
          yawWidth = map(data.ch[0], sbusMin, sbusMax, minWidth, maxWidth); //[0] means channel 1
          pitchWidth = map(data.ch[1], sbusMin, sbusMax, minWidth, maxWidth); //[1] -> ch 2
          rollWidth = map(data.ch[2], sbusMin, sbusMax, minWidth, maxWidth);
        }
      }
    } else {
      //This version will not work on the clearcore - its pulseIn is too slow
      //the clearcore can only read in 200us increments
      yawWidth = pulseIn(yawPin, LOW, timeout);
      pitchWidth = pulseIn(pitchPin, LOW, timeout);
      rollWidth = pulseIn(rollPin, LOW, timeout);
    }
    //convert the pulseWidth (roughtly 1000-2000 microSeconds) into
    //duty cycle (0-255).
    int yawDuty = pulseWidthToDutyCycle(yawWidth);
    int pitchDuty = pulseWidthToDutyCycle(pitchWidth);
    int rollDuty = pulseWidthToDutyCycle(rollWidth);
    /*
    Serial.print("yaw: ");
    Serial.print(yawDuty);
    Serial.print(" pitch: ");
    Serial.print(pitchDuty);
    Serial.print(" roll ");
    Serial.println(rollDuty);
    */

    //send these values to the motors
    CommandVelocity(yawDuty, yawMotor);
    CommandVelocity(pitchDuty, pitchMotor);
    CommandVelocity(rollDuty, rollMotor);
  } else {
    //we want to run in pre-programmed mode
    Serial.println("Turret is in preprogrammed mode, which has not been implemented yet");
  }
}