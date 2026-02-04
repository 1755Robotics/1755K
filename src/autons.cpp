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
}


void rightQual()  { 
    chassis.setPose(-60, -14.2, 90, false);
    pros::lcd::print(1, "Running Right Qual");  
    pros::delay(200);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPoint(-44, -14.2, 1000, {.minSpeed = 35, .earlyExitRange = 2});
    chassis.moveToPose(-19, -27, 125, 2000, {.forwards = true}, false);
    pros::Task([] {
        pros::delay(300);
        lil_krith.set_value(true);
    });
    chassis.moveToPoint(-9, -40, 2000, {}, false);
    chassis.turnToHeading(180, 300);
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
}
void leftElim()   { 
    chassis.setPose(60, -14.2, -90, false);
    pros::lcd::print(0, "Running Left Elim"); 
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPose(20, -28, -120, 2000, {.minSpeed=72, .earlyExitRange=8});
    chassis.moveToPose(8, -42, -140, 2000);
    lil_krith.set_value(true);
}
void rightElim()  { 
    chassis.setPose(-60, -14.2, 90, false);
    pros::lcd::print(0, "Running Right Elim"); 
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    chassis.moveToPose(-20, -28, 120, 2000,{.minSpeed=72, .earlyExitRange=8});
    chassis.moveToPose(-8, -42, 140, 2000);
    lil_krith.set_value(true);


}
void autonSkills() { 
    pros::lcd::print(0, "Running Skills"); 
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
    {"Left Elim",  AutonRoutine::LeftElim,  leftElim},
    {"Right Elim", AutonRoutine::RightElim, rightElim},
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
    int index = 1;  // -1 = None
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