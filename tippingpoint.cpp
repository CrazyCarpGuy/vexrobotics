#include "main.h"
using namespace pros;

//Define motor ports.
#define BACK_LEFT_DRIVE_MOTOR_PORT 3
#define MIDDLE_LEFT_DRIVE_MOTOR_PORT 2
#define FRONT_LEFT_DRIVE_MOTOR_PORT 1
#define BACK_RIGHT_DRIVE_MOTOR_PORT 13
#define MIDDLE_RIGHT_DRIVE_MOTOR_PORT 12
#define FRONT_RIGHT_DRIVE_MOTOR_PORT 11
#define RING_MECH_MOTOR_PORT 4
#define FOURBAR_MOTOR_PORT 20

//Define pneumatic ports.
#define FRONT_CLAMP_PNEUMATIIC_PORT 'A'
#define PIVOT_PNEUMATIC_PORT 'B'
#define BACK_CLAMP_PNEUMATIC_PORT 'C'

//Define sensor ports.
#define LEFT_DRIVE_ROTATION_SENSOR_PORT 6
#define RIGHT_DRIVE_ROTATION_SENSOR_PORT 16
#define LEFT_OPTICAL_SENSOR_PORT 5
#define RIGHT_OPTICAL_SENSOR_PORT 17
#define INERTIAL_SENSOR_PORT 19

//Define constants.
#define PI 3.141592653589793238
#define INCHES_POSITION_CONSTANT 3700.0 //Number of rotation sensor units per inch of wheel travel.
#define MOVE_P 0.045
#define MOVE_D 0.04
#define MOVE_ERROR_TOLERANCE 0.5
#define TURN_P 0.0051
#define TURN_D 0.004
#define TURN_ERROR_TOLERANCE 1
#define DOWN 0 //Pneumatic value for down.
#define UP 1 //Pneumatic value for up.

//Declare variables.
bool brakeDrive, ringMechIn, ringMechOut, frontPneumaticUp, backPneumaticUp;
float leftDrivePosition, rightDrivePosition, lastLeftDrivePosition, lastRightDrivePosition, globalDriveX, globalDriveY, initialDriveX, initialDriveY, deltaDriveX, deltaDriveY, globalDriveAngle, initialDriveAngle;

//Declare objects
Controller controller (E_CONTROLLER_MASTER);
Motor backLeftDriveMotor(BACK_LEFT_DRIVE_MOTOR_PORT, false), middleLeftDriveMotor(MIDDLE_LEFT_DRIVE_MOTOR_PORT, true), frontLeftDriveMotor(FRONT_LEFT_DRIVE_MOTOR_PORT, false),
      backRightDriveMotor(BACK_RIGHT_DRIVE_MOTOR_PORT, true), middleRightDriveMotor(MIDDLE_RIGHT_DRIVE_MOTOR_PORT, false), frontRightDriveMotor(FRONT_RIGHT_DRIVE_MOTOR_PORT, true),
      fourbarMotor(FOURBAR_MOTOR_PORT, false), ringMechMotor(RING_MECH_MOTOR_PORT, true);
ADIDigitalOut frontClampPneumatic(FRONT_CLAMP_PNEUMATIIC_PORT), pivotPneumatic(PIVOT_PNEUMATIC_PORT), backClampPneumatic(BACK_CLAMP_PNEUMATIC_PORT);
Rotation leftDriveRotationSensor(LEFT_DRIVE_ROTATION_SENSOR_PORT), rightDriveRotationSensor(RIGHT_DRIVE_ROTATION_SENSOR_PORT);
Optical leftOpticalSensor(LEFT_OPTICAL_SENSOR_PORT), rightOpticalSensor(RIGHT_OPTICAL_SENSOR_PORT);
Imu inertialSensor(INERTIAL_SENSOR_PORT);

/* Track the absolute position and orientation of the robot relative to the field.
 * globalDriveX represents the number of inches to the right of the center of the field.
 * globalDriveY represents the number of inches up from the center of the field.
 * Viewing from the bottom of the field with the red alliance on the left,
 *    |       +y      |
 *    |       |       |
 * RED|  -x --+-- +x  |BLUE
 *    |       |       |
 *    |      -y       |
 * globalDriveAngle represents the heading angle of the robot, in degrees clockwise from upward.
 * Viewing from the bottom of the field with the red alliance on the left,
 *    |       0       |
 *    |       |       |
 * RED| 270 --+-- 90  |BLUE
 *    |       |       |
 *    |      180      |
*/

void odometry() {
  pros::Task odom{[=] {
            std::uint32_t currentTime;
            while (true) {
              //Determine the current time.
              currentTime = millis();
              //Determine the current absolute orientation of the robot. Sensor values had to be scaled due to inaccurate readings.
              globalDriveAngle = initialDriveAngle + 1.01 * inertialSensor.get_rotation() + 1.24;
              //Determine the change in left and right drive positions since the last cycle.
              leftDrivePosition = -leftDriveRotationSensor.get_position() / INCHES_POSITION_CONSTANT;
              rightDrivePosition = -rightDriveRotationSensor.get_position() / INCHES_POSITION_CONSTANT;
              //Determine the magnitude of the robot's displacement on the x-axis and y-axis since the last cycle, and use this information to calculate the current absolute position.
              deltaDriveX += ((leftDrivePosition - lastLeftDrivePosition + rightDrivePosition - lastRightDrivePosition) / 2.0) * sin(globalDriveAngle * PI / 180.0);
              deltaDriveY += ((leftDrivePosition - lastLeftDrivePosition + rightDrivePosition - lastRightDrivePosition) / 2.0) * cos(globalDriveAngle * PI / 180.0);
              globalDriveX = initialDriveX + deltaDriveX;
              globalDriveY = initialDriveY + deltaDriveY;
              //Update last position values.
              lastLeftDrivePosition = leftDrivePosition;
              lastRightDrivePosition = rightDrivePosition;
              //Wait 100ms between cycles, accounting for processing time.
              pros::Task::delay_until(&currentTime, 100);
            }
  }};
}

/* Establish the starting absolute position and orientation of the robot.
 * x represents the number of inches to the right of the center of the field.
 * y represents the number of inches up from the center of the field.
 * Viewing from the bottom of the field with the red alliance on the left,
 *    |       +y      |
 *    |       |       |
 * RED|  -x --+-- +x  |BLUE
 *    |       |       |
 *    |      -y       |
 * angle represents the heading angle of the robot, in degrees clockwise from upward.
 * Viewing from the bottom of the field with the red alliance on the left,
 *    |       0       |
 *    |       |       |
 * RED| 270 --+-- 90  |BLUE
 *    |       |       |
 *    |      180      |
*/
void setReferenceFrame(float x, float y, float angle) {
  leftDriveRotationSensor.set_position(0);
  rightDriveRotationSensor.set_position(0);
  lastLeftDrivePosition = 0;
  lastRightDrivePosition = 0;
  initialDriveX = x;
  initialDriveY = y;
  initialDriveAngle = angle;
  odometry();
  delay(100);
}

//Move the left and right drive motors at the corresponding proportions of maximum speed, where positive values are forward.
void setDriveSpeed(float leftProportion, float rightProportion) {
  frontLeftDriveMotor.move_voltage(leftProportion * 12000);
  middleLeftDriveMotor.move_voltage(leftProportion * 12000);
  backLeftDriveMotor.move_voltage(leftProportion * 12000);
  frontRightDriveMotor.move_voltage(rightProportion * 12000);
  middleRightDriveMotor.move_voltage(rightProportion * 12000);
  backRightDriveMotor.move_voltage(rightProportion * 12000);
}

//Switch the brake mode of all drive motors.
void setDriveBrakes(bool brake) {
  backLeftDriveMotor.set_brake_mode(brake ? E_MOTOR_BRAKE_BRAKE : E_MOTOR_BRAKE_COAST);
  middleLeftDriveMotor.set_brake_mode(brake ? E_MOTOR_BRAKE_BRAKE : E_MOTOR_BRAKE_COAST);
  frontLeftDriveMotor.set_brake_mode(brake ? E_MOTOR_BRAKE_BRAKE : E_MOTOR_BRAKE_COAST);
  backRightDriveMotor.set_brake_mode(brake ? E_MOTOR_BRAKE_BRAKE : E_MOTOR_BRAKE_COAST);
  middleRightDriveMotor.set_brake_mode(brake ? E_MOTOR_BRAKE_BRAKE : E_MOTOR_BRAKE_COAST);
  frontRightDriveMotor.set_brake_mode(brake ? E_MOTOR_BRAKE_BRAKE : E_MOTOR_BRAKE_COAST);
}

//Move the fourbar motor at the given proportion of maximum speed, where positive values are upward.
void setFourbarSpeed(float proportion) {
  fourbarMotor.move_voltage(proportion * 12000);
}

//Move the ring mech motor at the given proportion of maximum speed, where positive values pull rings inward.
void setRingMechSpeed(float proportion) {
  ringMechMotor.move_voltage(proportion * 12000);
}

//Set the state of the front pneumatic, after a given delay in milliseconds.
void setFrontPneumatic(int state, int ms) {
  pros::Task fPneumatic{[=] {delay(ms);
    frontPneumaticUp = (bool)state;
    frontClampPneumatic.set_value(state);
  }};
}

//Set the state of the front pneumatic.
void setFrontPneumatic(int state) {
    setFrontPneumatic(state, 0);
}

//Set the state of the back pneumatics, after a given delay in milliseconds.
void setBackPneumatic(int state, int ms) {
  pros::Task bPneumatic{[=] {delay(ms);
    backPneumaticUp = (bool)(state);
    backClampPneumatic.set_value(state);
    pivotPneumatic.set_value(state);
  }};
}

//Set the state of the back pneumatics.
void setBackPneumatic(int state) {
    setBackPneumatic(state, 0);
}

/* Move the robot linearly a given amount, where positive values are forward.
 * maxSpeed represents the maximum proportion of full speed the drive can go.
 * example: moveSlow(40, 0.5) moves the drive 40 inches forward
 * with a "speed limit" of 300RPM (half of 600RPM).
 */
void moveLimited(float inches, float maxSpeed) {
  std::uint32_t currentTime;
  float error = inches, lastError = error, currentDrivePosition = (leftDrivePosition + rightDrivePosition) / 2.0, targetDrivePosition = currentDrivePosition + error, motorPower;
  bool stuck = false;
  while(error * fabs(inches) / inches > MOVE_ERROR_TOLERANCE && !stuck) {
    //Determine the current time.
    currentTime = millis();
    //Determine the current position of the drive.
    currentDrivePosition = (leftDrivePosition + rightDrivePosition) / 2.0;
    //Determine remaining distance to travel.
    error = targetDrivePosition - currentDrivePosition;
    //Calculate motor power using PD
    motorPower = (MOVE_P * error + MOVE_D * (error - lastError));
    if (motorPower > maxSpeed) {
      motorPower = maxSpeed;
    } else if (motorPower < -maxSpeed) {
      motorPower = -maxSpeed;
    }
    //If the robot travelled less than 1/2" since the last cycle  and it's at least half way to the target, the robot is stuck.
    if (fabs(error - lastError) < 0.5 && fabs(error) < fabs(inches / 2)) {
      stuck = true;
    }
    //Move the robot at calculated motor power values.
    setDriveSpeed(motorPower, motorPower);
    //Update last error values.
    lastError = error;
    //Wait 100ms between cycles, accounting for processing time.
    pros::Task::delay_until(&currentTime, 100);
  }
  //Stop the drive.
  setDriveSpeed(0, 0);
}

//Move the robot linearly a given amount, where positive values are forward.
void move(float inches) {
  moveLimited(inches, 1);
}

//Turn the robot a given amount, where positive values represent clockwise.
void turn(float angle) {
  std::uint32_t currentTime;
  float error = (0.782 * fabs(angle) + 21.5) * fabs(angle) / angle, lastError = error, targetDriveAngle = globalDriveAngle + error, motorPower;
  bool stuck = false;
  while(error * fabs(angle) / angle > TURN_ERROR_TOLERANCE && !stuck) {
    //Determine the current time.
    currentTime = millis();
    //Determine the remaining angle to travel.
    error = targetDriveAngle - globalDriveAngle;
    //Calculate motor power using PD
    motorPower = TURN_P * error + TURN_D * (error - lastError);
    //If the robot turned less than 0.1 degrees since the last cycle and it's at least half way to the target, the robot is stuck.
    if (fabs(error - lastError) < 0.1 && fabs(error) < fabs(angle / 2)) {
      stuck = true;
    }
    //Move the robot at calculated motor power values.
    setDriveSpeed(motorPower, -motorPower);
    //Update last error values.
    lastError = error;
    //Wait 100ms between cycles, accounting for processing time.
    pros::Task::delay_until(&currentTime, 100);
  }
  //Stop the drive.
  setDriveSpeed(0, 0);
}

/* Turn the robot to face the specified angle, in degrees clockwise from upward.
 * Viewing from the bottom of the field with the red alliance on the left,
 *    |       0       |
 *    |       |       |
 * RED| 270 --+-- 90  |BLUE
 *    |       |       |
 *    |      180      |
 */
void setAngle(float angle) {
  float turnAmount = angle - globalDriveAngle;
  //Find the nearest coterminal turn angle with a magnitude less than 180 degrees.
  while (turnAmount > 180) {
    turnAmount -= 360;
  }
  while (turnAmount < -180) {
    turnAmount += 360;
  }
  //Do not turn if the robot is within 5 degrees of the target angle.
  if (fabs(turnAmount) > 5) {
    turn(turnAmount);
  }
}

/* Turn the robot to face toward the point (x, y).
 * x represents the number of inches to the right of the center of the field.
 * y represents the number of inches up from the center of the field.
 * Viewing from the bottom of the field with the red alliance on the left,
 *    |       +y      |
 *    |       |       |
 * RED|  -x --+-- +x  |BLUE
 *    |       |       |
 *    |      -y       |
 */
void pointTo(float x, float y) {
  float angle, dX = x - globalDriveX, dY = y - globalDriveY;
  if (dX == 0) {
    if (dY > 0) {
      angle = 0;
    } else if (dY < 0) {
      angle = -90;
    } else {
      angle = 90 - globalDriveAngle;
    }
  } else {
      angle = 90 - atan(dY / dX) * 180 / PI;
    if (dX < 0) {
      angle += 180;
    }
  }
  setAngle(angle);
}

/* Turn the robot to face away from the point (x, y).
 * x represents the number of inches to the right of the center of the field.
 * y represents the number of inches up from the center of the field.
 * Viewing from the bottom of the field with the red alliance on the left,
 *    |       +y      |
 *    |       |       |
 * RED|  -x --+-- +x  |BLUE
 *    |       |       |
 *    |      -y       |
 */
void pointAway(float x, float y) {
  float angle, dX = x - globalDriveX, dY = y - globalDriveY;
  if (dX == 0) {
    if (dY > 0) {
      angle = 0;
    } else if (dY < 0) {
      angle = -90;
    } else {
      angle = 90 - globalDriveAngle;
    }
  } else {
      angle = 90 - atan(dY / dX) * 180 / PI;
    if (dX < 0) {
      angle += 180;
    }
  }
  setAngle(angle + 180);
}

/* Turn the robot to face toward the point (x, y), and then move the remaining distance.
 * x represents the number of inches to the right of the center of the field.
 * y represents the number of inches up from the center of the field.
 * Viewing from the bottom of the field with the red alliance on the left,
 *    |       +y      |
 *    |       |       |
 * RED|  -x --+-- +x  |BLUE
 *    |       |       |
 *    |      -y       |
 */
void moveTo(float x, float y) {
  pointTo(x, y);
  move(sqrtf(powf(x - globalDriveX, 2) + powf(y - globalDriveY, 2)));
}

void autonomous(){
  //Call desired movement functions here.
}

//Runs during the driver control period.
void opcontrol() {
  // setReferenceFrame(0, 0, 0);
  while (true) {
    //Drive motor controls.
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
      brakeDrive = !brakeDrive;
      setDriveBrakes(brakeDrive);
    }
    setDriveSpeed(controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0);
    //Fourbar motor controls.
    if (controller.get_digital(E_CONTROLLER_DIGITAL_R2)) {
      setFourbarSpeed(-1);
    } else if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
      setFourbarSpeed(1);
    } else if (!frontPneumaticUp && (leftOpticalSensor.get_proximity() > 30 || rightOpticalSensor.get_proximity() > 30)) {
      //Hold the fourbar in place by applying a slight upward voltage.
      setFourbarSpeed(0.2);
    } else {
      setFourbarSpeed(-0.2);
    }
    //Ring mech motor controls.
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
      ringMechIn = !ringMechIn;
      ringMechOut = false;
    }
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
      ringMechOut = !ringMechOut;
      ringMechIn = false;
    }
    setRingMechSpeed((int)ringMechIn - (int)ringMechOut);
    //Front pneumatic controls
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
      setFrontPneumatic((int)!frontPneumaticUp);
    }
		//Back pneumatic controls
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
      setBackPneumatic((int)!backPneumaticUp);
    }
    delay(10);
  }
}
