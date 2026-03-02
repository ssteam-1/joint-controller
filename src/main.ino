#include <SimpleFOC.h>
#include <Wire.h>

// ================================
// Motor & Driver Configuration
// ================================

// 7 pole pairs (confirmed)
BLDCMotor motor = BLDCMotor(7);

// B-G431B-ESC1 pinout (TIM1)
BLDCDriver6PWM driver = BLDCDriver6PWM(
  PA8,    // UH - TIM1_CH1
  PC13,   // UL - TIM1_CH1N
  PA9,    // VH - TIM1_CH2
  PA12,   // VL - TIM1_CH2N
  PA10,   // WH - TIM1_CH3
  PB15    // WL - TIM1_CH3N
);

// AS5600 I2C sensor
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ================================
// Setup
// ================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n--- B-G431B-ESC1 SimpleFOC (NO CURRENT SENSE) ---");

  // ----------------
  // I2C (AS5600)
  // ----------------
  Wire.setSCL(PB8);
  Wire.setSDA(PB7);
  Wire.begin();

  sensor.init();
  motor.linkSensor(&sensor);

  // ----------------
  // Driver
  // ----------------
  driver.voltage_power_supply = 12.0;   // Battery voltage
  driver.init();
  motor.linkDriver(&driver);

  // ----------------
  // Control mode
  // ----------------
  motor.controller = MotionControlType::angle;

  // ----------------
  // Safety limits (bare motor)
  // ----------------
  motor.voltage_limit = 6.0;      // Safe for no-load
  motor.current_limit = 2.0;      // Used only as torque limit

  // ----------------
  // Alignment
  // ----------------
  motor.voltage_sensor_align = 4.0;
  motor.velocity_limit = 50.0;    // rad/s

  // ----------------
  // PID tuning (stable + quiet)
  // ----------------
  motor.P_angle.P = 10.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  motor.P_angle.output_ramp = 230.0;

  motor.PID_velocity.P = 0.07;
  motor.PID_velocity.I = 6.0;
  motor.PID_velocity.D = 0.0;

  motor.LPF_velocity.Tf = 0.015;

  // ----------------
  // Init
  // ----------------
  motor.init();

  Serial.println("Starting FOC calibration...");
  if (motor.initFOC()) {
    Serial.println("FOC OK — motor holding position");
    Serial.println("Enter target angle in radians (e.g. 3.14 = 180°)");
  } else {
    Serial.println("FOC FAILED");
  }
}

// ================================
// Loop
// ================================
void loop() {
  motor.loopFOC();
  motor.move();

  // Serial angle input
  if (Serial.available()) {
    float target = Serial.parseFloat();
    while (Serial.available()) Serial.read();
    motor.target = target*51;

    Serial.print("Target angle: ");
    Serial.println(target);
  }
}
