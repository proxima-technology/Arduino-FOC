/**
 * Comprehensive BLDC motor control example using encoder and the DRV8302 board
 *
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values
 *
 * check the https://docs.simplefoc.com for full list of motor commands
 *
 */
#include "SimpleFOC.h"
#include "src/git-version.h"

// DRV8302 pins connections
// don't forget to connect the common ground pin
#define INH_A 9
#define INH_B 10
#define INH_C 11

#define EN_GATE 8
#define M_PWM 6
#define M_OC 5
#define OC_ADJ 7
// #define OC_GAIN PB5

#define IOUTA A3
#define IOUTB A4
#define IOUTC A5

#define SUPPLY_VOLTAGE A2

// Motor instance
BLDCMotor motor = BLDCMotor(10);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// DRV8302 board has 0.005Ohm shunt resistors and the gain of 12.22 V/V
LowsideCurrentSense cs = LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

// encoder instance
Encoder encoder = Encoder(2, 3, 1024);

// Interrupt routine intialisation
// channel A and B callbacks
void doA() {
  encoder.handleA();
}
void doB() {
  encoder.handleB();
}

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}

void setup() {
  Serial.begin(921600);
  char *git_version = GIT_VERSION;
  SimpleFOCDebug::enable();

  pinMode(SUPPLY_VOLTAGE, INPUT);

  //Waiting vlotage supply
  Serial.println("Waiting!");
  while(1){
    delay(10);
    if(analogRead(SUPPLY_VOLTAGE) > 300){
      break;
    }
  }

  //"Check communication"
  Serial.println("Check communication");
  while(1){
    delay(10);
    if (Serial.available()>0){
      char com = Serial.read();
      if(com == 'V'){
        Serial.println(GIT_VERSION);
      }else if(com == 'R'){
        break;
      }
    }
  }

  //Recheck the supply voltage
  Serial.println("Recheck the supply");
  while(1){
    if(analogRead(SUPPLY_VOLTAGE) > 300){
      break;
    }
  }

  Serial.println("START calibration");
  
  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor multi-character character constant
  motor.linkSensor(&encoder);

  // DRV8302 specific code
  // M_OC  - enable overcurrent protection
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);
  // OD_ADJ - set the maximum overcurrent limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH);
  // pinMode(OC_GAIN,OUTPUT);
  // digitalWrite(OC_GAIN,LOW);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 18000;  // suggested under 18khz
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  cs.linkDriver(&driver);

  // align voltage
  motor.voltage_sensor_align = 0.5;

  // control loop type and torque mode
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.motion_downsample = 0.0;
  
  // velocity loop PID
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 5.0;
  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.02;
  // angle loop PID
  motor.P_angle.P = 20.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID
  motor.PID_current_q.P = 100.0;
  motor.PID_current_q.I = 10;
  // Low pass filtering time constant
  motor.LPF_current_q.Tf = 0.02;
  // current d loop PID
  motor.PID_current_d.P = 1.0;
  motor.PID_current_d.I = 0.1;
  // Low pass filtering time constant
  motor.LPF_current_d.Tf = 0.02;

  // Limits
  motor.velocity_limit = 30.0;  // 100 rad/s velocity limit
  motor.voltage_limit = 24.0;   // 12 Volt limit
  motor.current_limit = 5.0;    // 2 Amp current limit

  // comment out if not needed, only use for debug
  // motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D;  // monitor the two currents d and q
  motor.monitor_downsample = 0;

  // initialise motor
  motor.init();

  cs.init();
  // driver 8302 has inverted gains on all channels
  cs.gain_a *= -1;
  cs.gain_b *= -1;
  cs.gain_c *= -1;
  motor.linkCurrentSense(&cs);

  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 0;
  motor.reset_flag = false;

  // define the motor id
  command.add('M', onMotor, "right leg motor");

  // Serial.println(F("Full control example: "));
  // Serial.println(F("Run user commands to configure and the motor (find the full command list in docs.simplefoc.com) \n "));
  // Serial.println(F("Initial motion control loop is voltage loop."));
  // Serial.println(F("Initial target voltage 2V."));
  
  _delay(1000);

  Serial.println("STANDBY");
}

void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move();

  // monitoring the state variables
  motor.monitor();

  // user communication
  command.run();

  if (motor.reset_flag) {
    Serial.println("Reset");
    delay(1000);
    NVIC_SystemReset();
  }

  if(analogRead(SUPPLY_VOLTAGE) < 200){
      Serial.println("LOW power supply");
      Serial.println("RESET");
      delay(1000);
      NVIC_SystemReset();
  }
}