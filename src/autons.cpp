#include "autons.hpp"
#include "main.h"

void distanceReset(double range, double x, double y, double theta){
    double rangeMM = range * 25.4;
    double distance = distance_sensor.get();
    double confidence = distance_sensor.get_confidence();
    if (distance <= rangeMM && confidence > 20) {
        chassis.setPose(x, y, theta);
    }
}
// Global selected auton
AutonRoutine selectedAuton = AutonRoutine::None;
//15.5 x 13.6
// ------------------------
// Autonomous routines
// ------------------------
void leftFourBall()   { 
    // chassis.setPose(48, -15.5, 0, false);
    // pros::lcd::print(0, "Running 4 Ball Wing");
    // pros::delay(300);
    // wing.set_value(true);
    // lil_krith.set_value(true);
    // intake.set_state_and_move(Intake::State::INTAKING);
    // chassis.moveToPoint(48, -48, 2000, {.forwards = false, .maxSpeed = 100,});
    // chassis.turnToHeading(90, 700);
    // chassis.moveToPoint(68, -48, 1000, {.maxSpeed = 70});
    // pros::delay(200);
    // chassis.moveToPoint(27, -48, 1500, {.forwards = false, .maxSpeed = 80},false);
    // wing.set_value(false);
    // lil_krith.set_value(false);
    // pros::delay(1200);
    // intake.set_state_and_move(Intake::State::NONE);


    chassis.setPose(48, -15.5, 0, false);
    
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(300);
    wing.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPoint(48, -35.3, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 80});
    chassis.turnToHeading(90, 700);
    chassis.moveToPoint(75, -48, 800, {.maxSpeed = 70});
    distanceReset(15, 57, -48.5, 90);
    chassis.moveToPose(25.5, -49.3, 90, 1100, {.forwards = false, .maxSpeed = 80},false);
    intake.set_state_and_move(Intake::State::OUTTAKING);
    pros::delay(100);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(1000);
    distanceReset(40, 26, -49.3, 90);
    intake.set_state_and_move(Intake::State::NONE);

    chassis.turnToHeading(30, 600);
    chassis.moveToPoint(36, -41, 2000, {.maxSpeed = 80, .minSpeed = 70});
    chassis.turnToHeading(90, 400);
    chassis.moveToPoint(8, -42, 5000, {.forwards = false, .maxSpeed = 40});
    chassis.turnToHeading(90,200);

}

void leftSeven() {
    chassis.setPose(48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(300);
    wing.set_value(true);
    lil_krith.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPoint(48, -48, 2000, {.forwards = false, .maxSpeed = 100,});
    chassis.turnToHeading(90, 700);
    chassis.moveToPoint(71, -48, 1000, {.maxSpeed = 60});
    pros::delay(200);
    chassis.moveToPoint(27, -48.5, 1500, {.forwards = false, .maxSpeed = 70},false);
    wing.set_value(false);
    lil_krith.set_value(false);
    pros::delay(1000);
    wing.set_value(true);

    chassis.turnToHeading(0, 700);
    chassis.moveToPoint(24, -24, 2000, {.maxSpeed = 70});
    pros::Task([] {
        pros::delay(400);
        lil_krith.set_value(true);
        pros::delay(200);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(135, 1000);
    chassis.moveToPoint(8.7, -8.7, 2000,{.forwards = false, .maxSpeed = 60});
    pros::delay(700);
    intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
    pros::delay(100);
    intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    pros::delay(1000);
    intake.set_state_and_move(Intake::State::NONE);

    
    chassis.moveToPoint(36, -40, 2000, {.maxSpeed = 70}); 
    chassis.turnToHeading(90, 700);
    wing.set_value(false);
    chassis.moveToPoint(10, -40, 5000, {.forwards = false, .maxSpeed = 40});

}

void leftAWP() {
    chassis.setPose(48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(300);
    wing.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPoint(48, -48, 2000, {.forwards = false, .maxSpeed = 100,});
    chassis.turnToHeading(90, 700);
    chassis.moveToPoint(73, -48, 1000, {.maxSpeed = 70});
    pros::delay(200);
    chassis.moveToPoint(27, -49, 1500, {.forwards = false, .maxSpeed = 70},false);
    wing.set_value(false);
    lil_krith.set_value(false);
    pros::delay(1000);
    wing.set_value(true);

    chassis.turnToHeading(0, 700);
    chassis.moveToPoint(24, -24, 2000, {.maxSpeed = 70});
    pros::Task([] {
        pros::delay(400);
        lil_krith.set_value(true);
        pros::delay(200);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(135, 700);
    chassis.moveToPoint(8.7, -8.1, 2000,{.forwards = false, .maxSpeed = 60});
    pros::delay(700);
    intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
    pros::delay(100);
    intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    pros::delay(1000);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPoint(24, -24,1500, {.maxSpeed = 80}); 
    chassis.turnToHeading(0, 700);
    chassis.moveToPoint(24, 24, 2000, {.maxSpeed = 80}); 
    pros::Task([] {
        pros::delay(1200);
        lil_krith.set_value(true);
        pros::delay(500);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(-135, 500);
    chassis.moveToPoint(8.5, 8.7, 2000,{ .maxSpeed = 70});
    intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
    
}
void rightFourBall()  { 
    chassis.setPose(-48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(300);
    wing.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPoint(-48, -33.8, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 80});
    chassis.turnToHeading(-90, 700);
    chassis.moveToPoint(-60, -48, 1000, {.maxSpeed = 65});
    pros::delay(800);
    distanceReset(15, -57, -48, -90);


    chassis.moveToPoint(-24, -48.5, 1100, {.forwards = false, .maxSpeed = 70},false);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(1000);
    distanceReset(40, -26, -48.5, -90);
    intake.set_state_and_move(Intake::State::NONE);

    chassis.turnToHeading(-150, 600);
    chassis.moveToPoint(-36, -57.5, 2000, {.maxSpeed = 80, .minSpeed = 70});
    chassis.turnToHeading(-90, 700);

    chassis.moveToPoint(-8, -57, 5000, {.forwards = false, .maxSpeed = 40});
    chassis.turnToHeading(-90,200);
   
};
void rightSeven() {
    

}

void rightAWP() {

}

void skills() { 
    chassis.setPose(48, 15.5, 0, false);
    pros::lcd::print(0, "Running Skills");
    pros::delay(300);
    wing.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPoint(48, 48, 2000);
    chassis.turnToHeading(90, 700);
    chassis.moveToPoint(72, 48, 1000, {.maxSpeed = 70});
    pros::delay(2000);
    chassis.moveToPoint(73, 49, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(73, 49, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(73, 49, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(72, 49, 200, {.maxSpeed = 70});
    chassis.moveToPose(45, 63, 90, 2000, {.forwards = false});
    chassis.moveToPose(-30, 63, 90,2000, {.forwards = false});
    lil_krith.set_value(false);
    pros::delay(500);
    chassis.moveToPose(-44, 50, 90, 2000, {.forwards = false});
    pros::delay(500);
    chassis.turnToHeading(-90, 700);
    pros::delay(500);
    chassis.moveToPose(-5, 50, -90, 2500, {.forwards = false});
    pros::delay(900);
    intake.set_state_and_move(Intake::State::OUTTAKING);
    wing.set_value(false);
    pros::delay(300);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(true);
    pros::delay(1800);
    chassis.turnToHeading(-90, 700);
    pros::delay(500);
// match loader far right side
    chassis.moveToPose(-81, 48,-90, 1400, {.maxSpeed = 60});
    wing.set_value(true);
    pros::delay(2000);
    chassis.moveToPoint(-81, 48, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-81, 48, 200, {.maxSpeed = 70});
    pros::delay(500);
    chassis.moveToPoint(-81, 48, 200, {.maxSpeed = 70});
    pros::delay(500);
    chassis.moveToPoint(-81, 48, 200, {.maxSpeed = 70});
    pros::delay(500);
    chassis.moveToPoint(-81, 48, 200, {.maxSpeed = 70});
    pros::delay(500);
    chassis.moveToPoint(-81, 48, 200, {.maxSpeed = 70});
    pros::delay(500);
    chassis.moveToPose(0, 50, -90, 3000, {.forwards = false, .maxSpeed = 90});
    pros::delay(1000);
    wing.set_value(false);
    pros::delay(1800);
    intake.set_state_and_move(Intake::State::OUTTAKING);
    pros::delay(300);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::delay(3000);
 

    //work on whole route after, just park for now
    chassis.turnToHeading(-180,900);
    chassis.turnToHeading(-270,700);
    chassis.moveToPose(53.5, 33, -270, 2000);
    lil_krith.set_value(false);
    chassis.moveToPoint(70, 20, 2000, {.maxSpeed = 70});
    pros::delay(300);
    chassis.turnToHeading(-180, 700);
    pros::delay(300);
    lil_krith.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(70, -40, 1000, {.minSpeed = 120});
    pros::delay(300);
    lil_krith.set_value(false);


   //aligning to far left side 
    /*wing.set_value(true);
    chassis.turnToHeading(-180, 700);
    pros::delay(300);
    chassis.moveToPose(-34,-54, -180, 2000, {.maxSpeed =100});
    chassis.turnToHeading(-90,700);
    chassis.moveToPoint(0, -54, 1200, {.forwards = false});
// match loader far left side
    chassis.moveToPose(-50, -54,-90, 1000, {.maxSpeed = 70});
    pros::delay(1300);
    chassis.moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-40, -54, 700, {.forwards = false, .maxSpeed = 70});
    chassis.turnToHeading(-90,600);
// getting to close left side
    lil_krith.set_value(false);
    chassis.moveToPose(-24, -67, -90, 2000, {.forwards = false});
    chassis.moveToPose(48, -64 , -90, 2000, {.forwards = false});
    /*chassis.moveToPose(57, -51.5, -90, 2000, {.forwards = false});
    chassis.turnToHeading(92, 700);
    pros::delay(1000);
    chassis.moveToPoint(10, -51.5, 3500, {.forwards = false, .maxSpeed = 60});
    pros::delay(1000);
    wing.set_value(false);
    lil_krith.set_value(true);
    pros::delay(1800);
    wing.set_value(true);
    chassis.turnToHeading(90, 300);
    pros::delay(300);
// match loader close left side
    /*chassis.moveToPose(81, -51, 90, 1000, {.maxSpeed = 70});
    pros::delay(2000);
    chassis.moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(82, -51, 200, {.maxSpeed = 70});
    chassis.moveToPose(0, -50, 92, 2000, {.forwards = false});
    pros::delay(1000);
    wing.set_value(false);
    pros::delay(1800);
    intake.set_state_and_move(Intake::State::OUTTAKING);
    pros::delay(300);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::delay(1600);
    wing.set_value(true);
    //Park
    lil_krith.set_value(false);
    chassis.turnToHeading(45, 700);
    lemlib::Pose tPose = chassis.getPose();
    chassis.moveToPose(tPose.x+90, tPose.y+20, 90, 2000);
    chassis.turnToHeading(180, 700, {.maxSpeed = 40});
    pros::delay(500);
    tPose = chassis.getPose();
    chassis.moveToPose(tPose.x+6, 16, 180, 7000, {.forwards = false, .minSpeed = 110});
    */

}
void lin_pid_tuning_test() {
    chassis.setPose(0, 0, 0); 
    pros::delay(100);
    chassis.moveToPoint(0, 48, 3000);
}
void ang_pid_tuning_test() {
    chassis.setPose(0, 0, 0); 
    pros::delay(100);
    chassis.turnToHeading(90, 2000);
}

// ------------------------
// Auton registry
// ------------------------
struct AutonEntry {
    const char* name;
    AutonRoutine id;
    void (*fn)();
};

static AutonEntry autons[] = {
    {"Left 4 Ball Wing",  AutonRoutine::LeftFourBall,  leftFourBall},
    {"Right 4 Ball Wing", AutonRoutine::RightFourBall, rightFourBall},
    {"Left 4 wing + 3",  AutonRoutine::LeftSeven,  leftSeven},
    {"Right 4 wing + 3", AutonRoutine::RightSeven, rightSeven},
    {"Left Solo AWP",  AutonRoutine::LeftAWP,  leftAWP},
    {"Right Solo AWP", AutonRoutine::RightAWP, rightAWP},
    {"Skills",     AutonRoutine::Skills,    skills},
    {"Linear PID", AutonRoutine::LinPID, lin_pid_tuning_test},
    {"Angular PID", AutonRoutine::AngPID, ang_pid_tuning_test},
};

static constexpr int AUTON_COUNT =
    sizeof(autons) / sizeof(autons[0]);

// ------------------------
// Autonomous selector task
// ------------------------
void autonSelectorTask(void*) {
    int index = 0;  // -1 = None
    uint8_t lastButtons = pros::lcd::read_buttons();

    while (true) {
        pros::lcd::clear_line(0);
        pros::lcd::clear_line(1);
        pros::lcd::clear_line(2);

        if (index == -1) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            selectedAuton = AutonRoutine::None;
        } else {
            pros::lcd::print(0, "Select Auton: %s", autons[index].name);
            selectedAuton = autons[index].id;
        }

        uint8_t buttons = pros::lcd::read_buttons();

        if ((buttons & LCD_BTN_LEFT) && !(lastButtons & LCD_BTN_LEFT)) {
            index--;
            if (index < -1) index = AUTON_COUNT - 1;
        }

        if ((buttons & LCD_BTN_RIGHT) && !(lastButtons & LCD_BTN_RIGHT)) {
            index++;
            if (index >= AUTON_COUNT) index = -1;
        }

        lastButtons = buttons;
        pros::delay(50);
    }
}

// ------------------------
// Start selector
// ------------------------
void startAutonSelector() {
    static pros::Task selector(autonSelectorTask);
}

// ------------------------
// Run selected auton
// ------------------------
void runSelectedAuton() {
    if (selectedAuton == AutonRoutine::None) {
        pros::lcd::print(1, "No auton selected");
        return;
    }

    for (int i = 0; i < AUTON_COUNT; i++) {
        if (autons[i].id == selectedAuton) {
            autons[i].fn();
            return;
        }
    }
}