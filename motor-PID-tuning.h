#include <SimpleFOC.h>
#include <Wire.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// COMMANDER INSTANCE
Commander command = Commander(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // 1. HARDWARE INIT
  Wire.setSCL(PB8);
  Wire.setSDA(PB7);
  Wire.begin();
  Wire.setClock(400000); // Fast I2C is mandatory

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12.0;
  driver.init();
  motor.linkDriver(&driver);

  // 2. CONTROL CONFIG
  motor.controller = MotionControlType::angle;
  
  // Start with conservative limits, but capable
  motor.voltage_limit = 6.0;      
  motor.velocity_limit = 20.0;    

  // Start with ZERO/LOW GAINS for manual tuning
  motor.P_angle.P = 0;   // We will tune this last
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;

  motor.PID_velocity.P = 0; // We will tune this first
  motor.PID_velocity.I = 0; 
  motor.PID_velocity.D = 0;
  
  motor.LPF_velocity.Tf = 0.01; // Fast filtering

  // 3. INIT
  motor.init();
  motor.initFOC();

  // 4. ADD COMMANDER
  command.add('M', onMotor, "Motor Interface");

  Serial.println("--- TUNING MODE ---");
  Serial.println("Type 'M' to see status.");
}

void loop() {
  motor.loopFOC();
  motor.move();
  command.run(); // Reads Serial input
}