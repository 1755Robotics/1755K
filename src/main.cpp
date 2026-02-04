#include "main.h"
#include "lemlib/api.hpp"

pros::adi::DigitalOut wing('H', false);        
pros::adi::DigitalOut lil_krith('G', false);

pros::Controller master(pros::E_CONTROLLER_MASTER);
// motor groups
pros::MotorGroup leftMotors({-10, -14, -15}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({9, 11, 12}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 13
pros::Imu imu(13);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-6);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -1.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical1(&rightMotors, lemlib::Omniwheel::NEW_325, 5.625, 450);
lemlib::TrackingWheel vertical2(&leftMotors, lemlib::Omniwheel::NEW_325, -5.625, 450);
// drivetrain settings

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.25, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);
Intake intake({7, -8, -20}, {20});

// lateral motion controller
lemlib::ControllerSettings linearController(  11.2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController( 3.6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              26.593, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical1, // vertical tracking wheel
                            &vertical2, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    chassis.setPose(0, 0, 0); 
	intake.init();
    startAutonSelector();
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
void autonomous() {
    runSelectedAuton();
}


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
  	intake.set_state_and_move(Intake::State::NONE);
	int forwardOut = 0;
  	while (true) {
    	// get joystick positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		constexpr int FORWARD_SLEW = 18;
		forwardOut = lemlib::slew(leftY, forwardOut, FORWARD_SLEW);

        // move the chassis with curvature drive
        chassis.arcade(forwardOut, rightX);
    	intake.opcontrol(master);
    	
		if (!pros::competition::is_connected()) {
			if (master.get_digital_new_press(DIGITAL_DOWN) && master.get_digital(DIGITAL_B)) {
				autonomous();
				pros::delay(1900);
				startAutonSelector();
			}
		}

    	// Wing: L1 extends, L2 retracts
    	if (master.get_digital(DIGITAL_L1)) {
      	wing.set_value(true);   // Extend
    	} else if (master.get_digital(DIGITAL_L2)) {
      	wing.set_value(false);  // Retract
    	}

    	static bool lil_krith_state = false;

        // Toggle when button is pressed
        if (master.get_digital_new_press(DIGITAL_Y)) {
            lil_krith_state = !lil_krith_state;
            lil_krith.set_value(lil_krith_state);
        }
    
    	pros::delay(25);
  }
}