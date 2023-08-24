#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
#include "main.h"

class MotionController {
public:
  MotionController() {
    // Initialize odometry task
    pros::Task odometryTask(odometryLoop);
  }

  void chainDrive(std::vector<Pose> points, std::vector<bool> reverses, double exitErrorPerPoint) {
    Point initial = {x, y}; // Use odometry variables

    double dist = calc::distance(initial, {points.back().x, points.back().y});
    double angle = math::wrap180(calc::angleDifference(initial, {points[0].x, points[0].y}) - heading); // Use odometry variable

    m_linear->setTarget(dist);
    m_angular->setTarget(angle);

    double m_lastTargetAngle = calc::angleDifference(initial, {points.back().x, points.back().y}) - heading; // Use odometry variable

    int iter = 0;

    for (Pose p : points) {
      if (points.back().x == p.x && points.back().y == p.y) {
        drive({p.x, p.y}, p.theta.value_or(127));
        break;
      } else {
        double linErrorToNextPoint = calc::distance({x, y}, {p.x, p.y}); // Use odometry variables

        while (linErrorToNextPoint > exitErrorPerPoint) {
          Point current = {x, y}; // Use odometry variables

          linErrorToNextPoint = calc::distance(current, {p.x, p.y});
          dist = calc::distance(current, {points.back().x, points.back().y});
          angle = math::wrap180(calc::angleDifference(current, {p.x, p.y}) - heading); // Use odometry variable

          dist *= cos(math::degToRad(angle));

          if (reverses.size() >= iter && reverses[iter]) {
            angle = math::wrap180(angle + 180);
          }

          double distOutput = m_linear->calculate(dist);
          double angOutput = m_angular->calculate(angle);

          bool closeToTarget = (calc::distance(current, {p.x, p.y}) < 5);
          if (closeToTarget) {
            angOutput = 0;
          }

          double maxSpeed = p.theta.value_or(127);

          distOutput = std::clamp(distOutput, -maxSpeed, maxSpeed);

          double lSpeed = distOutput + angOutput;
          double rSpeed = distOutput - angOutput;

          double speedRatio = std::max(std::abs(lSpeed), std::abs(rSpeed)) / maxSpeed;
          if (speedRatio > 1) {
            lSpeed /= speedRatio;
            rSpeed /= speedRatio;
          }

          m_chassis->setVoltage(lSpeed, rSpeed);

          pros::delay(MOTION_TIMESTEP);
          m_processTimer += MOTION_TIMESTEP;
        }
      }
    }

    m_processTimer = 0;
  }

private:
  static double x, y, heading;
  static double leftLast, rightLast;
  static constexpr double WHEELBASE = 10.0;

  // Your PID implementation
  PIDController* m_linear = new PIDController(/* Your PID parameters here */);
  PIDController* m_angular = new PIDController(/* Your PID parameters here */);
  Chassis* m_chassis = new Chassis(/* Your chassis parameters here */);

  // Odometry loop
  static void odometryLoop(void*) {
    while (true) {
      double left = leftRotationSensor.get_value();
      double right = rightRotationSensor.get_value();
      double dLeft = left - leftLast;
      double dRight = right - rightLast;
      double dHeading = (dLeft - dRight) / WHEELBASE;

      heading += dHeading;
      double avgDelta = (dLeft + dRight) / 2.0;
      x += avgDelta * cos(heading);
      y += avgDelta * sin(heading);

      printf("X: %f, Y: %f, Heading: %f\n", x, y, heading);

      leftLast = left;
      rightLast = right;
      pros::delay(10);
    }
  }
};

// Define static member variables
double MotionController::x = 0;
double MotionController::y = 0;
double MotionController::heading = 0;
double MotionController::leftLast = 0;
double MotionController::rightLast = 0;

pros::ADIEncoder leftRotationSensor(1, 2, false);
pros::ADIEncoder rightRotationSensor(3, 4, false);

void initializemotion() {
  MotionController motionController;
}






void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	intializemotion();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;

		pros::delay(20);
	}
}
