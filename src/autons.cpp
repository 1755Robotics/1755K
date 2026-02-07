#include "autons.hpp"
#include "main.h"


// Global selected auton
AutonRoutine selectedAuton = AutonRoutine::None;
//15.5 x 13.6
// ------------------------
// Autonomous routines
// ------------------------
void leftQual()   { 
    chassis.setPose(60, -14.2, -90, false);
    pros::lcd::print(1, "Running Left Qual"); 
    pros::delay(200);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::Task([] {
        pros::delay(1300);
        lil_krith.set_value(true);
    });
    chassis.moveToPoint(44, -14.2, 1000, {.minSpeed = 35, .earlyExitRange = 2});
    chassis.moveToPose(19, -27, -125, 2000, {.forwards = true}, false);

    //Move to high middle goal
    chassis.turnToHeading(-225, 700);
    pros::Task([] {
        pros::delay(300);
        intake.set_state_and_move(Intake::State::NONE);
    });
    chassis.moveToPose(2, -5, -225, 2000, {.forwards = false,.minSpeed = 30}, false);
    intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    pros::delay(500);
    //Move to left feeder
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPose(40.5, -48, -225, 2000, {.forwards = true}, false);
    chassis.turnToHeading(-270, 700);
    chassis.moveToPoint(67, -48, 1000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(225);
    chassis.moveToPoint(20, -48, 1000, {.forwards = false}, false);
    wing.set_value(false);
    pros::delay(1500);
    chassis.moveToPoint(33, -50, 1000, {.forwards = true}, false);
    intake.set_state_and_move(Intake::State::NONE);
    pros::delay(200);
    chassis.turnToHeading(0, 700, {}, false);
    lil_krith.set_value(false);
    chassis.moveToPoint(35,-38, 700);
    chassis.turnToHeading(90 ,700);
    chassis.moveToPoint(18,-40,700, {.forwards = false});
}


void rightQual()  { 
    chassis.setPose(-60, -14.2, 90, false);
    pros::lcd::print(1, "Running Right Qual");  
    pros::delay(200);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::Task([] {
        pros::delay(1300);
        lil_krith.set_value(true);
        pros::delay(400);
        lil_krith.set_value(false);
    });
    chassis.moveToPoint(-44, -14.2, 1000, {.minSpeed = 35, .earlyExitRange = 2});
    chassis.moveToPose(-19, -27, 125, 2000, {.forwards = true}, false);
    pros::Task([] {
        pros::delay(1000);
        lil_krith.set_value(true);
    });
    chassis.moveToPoint(-8.5, -40, 2000, {}, false);
    chassis.turnToHeading(180, 300);
    chassis.moveToPoint(-8.5, -42, 500);
    pros::delay(200);

    chassis.moveToPoint(-10.5, -24, 2000, {.forwards= false});
    chassis.turnToHeading(270, 700);
    chassis.moveToPose(-45, -45, 225, 2000, {.forwards = true, .minSpeed = 50}, false);
    chassis.turnToHeading(270, 700);
    chassis.moveToPoint(-69, -45, 1000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(300);
    chassis.moveToPoint(-20, -45, 1000, {.forwards = false}, false);
    wing.set_value(false);
    pros::delay(1800);
    intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
    pros::delay(200);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::delay(2000);
    wing.set_value(true);
    chassis.moveToPoint(-32, -45, 1000, {.forwards = true, .minSpeed = 70}, false);
    pros::delay(200);
    chassis.moveToPoint(-26, -45, 2000, {.forwards = true, .minSpeed = 80}, false);
    /*
    //Move to left feeder
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPose(40.5, -48, -225, 2000, {.forwards = true}, false);
    chassis.turnToHeading(-270, 700);
    chassis.moveToPoint(67, -48, 1000, {.forwards = true, .maxSpeed = 80}, false);
    pros::delay(225);
    chassis.moveToPoint(20, -48, 1000, {.forwards = false}, false);
    wing.set_value(false);
    pros::delay(1500);
    chassis.moveToPoint(33, -50, 1000, {.forwards = true}, false);
    intake.set_state_and_move(Intake::State::NONE);
    chassis.turnToHeading(0, 700, {}, false);
    lil_krith.set_value(false);
    chassis.moveToPoint(31,-40, 700);
    chassis.turnToHeading(-275,700);
    chassis.moveToPoint(14,-40,700, {.forwards = false});
    */
};


void autonSkills() { 
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
    chassis.moveToPose(10, 60, 90, 2000, {.forwards = false});
    chassis.moveToPoint(-30, 60, 2000, {.forwards = false});
    lil_krith.set_value(false);
    pros::delay(500);
    chassis.moveToPose(-44, 38, 90, 2000, {.forwards = false});
    pros::delay(500);
    chassis.turnToHeading(-90, 700);
    pros::delay(500);
    chassis.moveToPoint(-5, 38, 1500, {.forwards = false});
    pros::delay(600);
    wing.set_value(false);
    lil_krith.set_value(true);
    pros::delay(1800);
    chassis.turnToHeading(-90, 700);
    pros::delay(500);
// match loader far right side
    chassis.moveToPose(-78, 36,-90, 1000, {.maxSpeed = 70});
    wing.set_value(true);
    pros::delay(2000);
    chassis.moveToPoint(-79, 36, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, 36, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, 36, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, 36, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, 36, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, 36, 200, {.maxSpeed = 70});
    chassis.moveToPose(0, 38,-90, 2000, {.forwards = false});
    pros::delay(1000);
    wing.set_value(false);
    pros::delay(1800);
    intake.set_state_and_move(Intake::State::OUTTAKING);
    pros::delay(300);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::delay(1800);
// aligning to far left side 
    chassis.moveToPoint(-36, 34,700);
    wing.set_value(true);
    chassis.turnToHeading(-180, 700);
    pros::delay(300);
    chassis.moveToPoint(-36,-54, 2000, {.maxSpeed =100});
    chassis.turnToHeading(-90,700);
    chassis.moveToPoint(0, -54, 1200, {.forwards = false});
// match loader far left side
    chassis.moveToPose(-78, -50,-90, 1000, {.maxSpeed = 70});
    pros::delay(1300);
    chassis.moveToPoint(-79, -50, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, -50, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, -50, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, -50, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, -50, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, -50, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-79, -50, 200, {.maxSpeed = 70});
    pros::delay(300);
    chassis.moveToPoint(-40, -50, 700, {.forwards = false, .maxSpeed = 70});
// getting to close left side
    lil_krith.set_value(false);
    chassis.moveToPose(0, -65, -90, 2000, {.forwards = false});
    chassis.moveToPoint(48, -65 , 2000, {.forwards = false});
    chassis.moveToPose(57, -50, -90, 2000, {.forwards = false});
    chassis.turnToHeading(92, 700);
    pros::delay(1000);
    chassis.moveToPoint(-10, -50, 3500, {.forwards = false, .maxSpeed = 60});
    pros::delay(1000);
    wing.set_value(false);
    lil_krith.set_value(true);
    pros::delay(1800);
    wing.set_value(true);
    chassis.turnToHeading(90, 300);
    pros::delay(300);
// match loader close left side
    chassis.moveToPose(81, -51, 90, 1000, {.maxSpeed = 70});
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
/*    lil_krith.set_value(false);
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
    chassis.moveToPoint(0, 24, 3000);
}
void ang_pid_tuning_test() {
    chassis.setPose(0, 0, 0); 
    pros::delay(100);
    chassis.turnToHeading(90, 3000);
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
    {"Left Qual",  AutonRoutine::LeftQual,  leftQual},
    {"Right Qual", AutonRoutine::RightQual, rightQual},
    {"Skills",     AutonRoutine::Skills,    autonSkills},
    {"Linear PID", AutonRoutine::LinPID, lin_pid_tuning_test},
    {"Angular PID", AutonRoutine::AngPID, ang_pid_tuning_test},
};

static constexpr int AUTON_COUNT =
    sizeof(autons) / sizeof(autons[0]);

// ------------------------
// Autonomous selector task
// ------------------------
void autonSelectorTask(void*) {
    int index = 2;  // -1 = None
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