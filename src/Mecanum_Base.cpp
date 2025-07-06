#ifndef MECANUM_BASE_H
#define MECANUM_BASE_H

#include <Arduino.h>  // Required to use Arduino types and functions

// Define the MecanumBase class
class Mecanum_Base {
  public:
    // Constructor: assign motor pins
    Mecanum_Base(int fl_pwm, int fl_dir,
                int fr_pwm, int fr_dir,
                int bl_pwm, int bl_dir,
                int br_pwm, int br_dir);

    // Set the robot's physical dimensions (length and width)
    void setDimensions(float length_cm, float width_cm);

    // Set the maximum motor speed (PWM limit)
    void setSpeed(int max_pwm);  // PWM from 0 to 255

    // Set the desired translation angle and rotation rate
    void driveAngle(float angle_deg, float rotation_rate);

    // Compute and apply motor outputs
    void update();

  private:
    // Motor pin assignments
    int FL_PWM, FL_DIR;
    int FR_PWM, FR_DIR;
    int BL_PWM, BL_DIR;
    int BR_PWM, BR_DIR;

    // Robot dimensions
    float L, W;

    // Speed limit for PWM output
    int speed_limit;

    // Individual motor velocity values (normalized)
    float v_FL, v_FR, v_BL, v_BR;

    // Internal helper to send velocity to motor pins
    void setMotor(int pwmPin, int dirPin, float velocity);
};

#endif  // End of header guard
