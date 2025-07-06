#include "Mecanum_Base.h"

// Constructor: Assign motor pins and set them to OUTPUT
Mecanum_Base::Mecanum_Base(int fl_pwm, int fl_dir,
                         int fr_pwm, int fr_dir,
                         int bl_pwm, int bl_dir,
                         int br_pwm, int br_dir) {
  // Store pin numbers
  FL_PWM = fl_pwm; FL_DIR = fl_dir;
  FR_PWM = fr_pwm; FR_DIR = fr_dir;
  BL_PWM = bl_pwm; BL_DIR = bl_dir;
  BR_PWM = br_pwm; BR_DIR = br_dir;

  // Set all pins as outputs
  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);

  // Default speed limit
  speed_limit = 255;
}

// Set length and width of the robot base (in cm)
void Mecanum_Base::setDimensions(float length_cm, float width_cm) {
  L = length_cm;
  W = width_cm;
}

// Set the max PWM value for all motors (0–255)
void Mecanum_Base::setSpeed(int max_pwm) {
  speed_limit = constrain(max_pwm, 0, 255);
}

// Set the desired translation direction and rotation rate
void Mecanum_Base::driveAngle(float angle_deg, float rotation_rate) {
  // Convert angle from degrees to radians
  float angle_rad = angle_deg * 3.14159 / 180.0;

  // Compute directional velocities from angle
  float Vx = cos(angle_rad);  // Sideways velocity
  float Vy = sin(angle_rad);  // Forward velocity
  float omega = rotation_rate;  // Rotation rate (positive = CCW)

  // Distance term used for rotational effect
  float LpW = L + W;

  // Calculate each wheel's velocity based on mecanum kinematics
  v_FL = Vy + Vx - omega * LpW;
  v_FR = Vy - Vx + omega * LpW;
  v_BL = Vy - Vx - omega * LpW;
  v_BR = Vy + Vx + omega * LpW;

  // Normalize velocities to ensure they're between -1 and 1
  float max_v = max(max(abs(v_FL), abs(v_FR)), max(abs(v_BL), abs(v_BR)));
  if (max_v > 1.0) {
    v_FL /= max_v;
    v_FR /= max_v;
    v_BL /= max_v;
    v_BR /= max_v;
  }
}

// Apply the calculated velocities to each motor
void Mecanum_Base::update() {
  setMotor(FL_PWM, FL_DIR, v_FL);
  setMotor(FR_PWM, FR_DIR, v_FR);
  setMotor(BL_PWM, BL_DIR, v_BL);
  setMotor(BR_PWM, BR_DIR, v_BR);
}

// Helper function to apply a velocity to one motor
void Mecanum_Base::setMotor(int pwmPin, int dirPin, float velocity) {
  int pwm = int(abs(velocity) * speed_limit);      // Scale velocity to PWM
  digitalWrite(dirPin, velocity >= 0 ? HIGH : LOW); // Set direction
  analogWrite(pwmPin, pwm);                         // Set speed
}
