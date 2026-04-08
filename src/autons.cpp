#include "autons.hpp"
#include "main.h"

// Field and Robot Constants for Distance Odometry Reset (all in inches)
static constexpr double FIELD_IN               = 144.0;
static constexpr double ROBOT_HALF_W_IN        = 6.5;   // measure and update
static constexpr double ROBOT_HALF_L_IN        = 6.5;   // measure and update

// thresholds for validity checks
static constexpr double SUM_TOLERANCE_IN       = 3.5;
static constexpr int    MIN_CONFIDENCE         = 30;
static constexpr double MAX_CORRECTION_IN      = 5.0;

// Reads the current pose and corrects x/y using distance sensors.
// Snaps sensor assignments to the nearest cardinal heading so body-frame
// readings are correctly mapped to field axes.
// range: max distance (in) a sensor may read and still be used for correction.
// x, y: expected field position — used only as a Euclidean sanity gate.
void distanceReset(double range, double x, double y) {
    lemlib::Pose current = chassis.getPose();

    // Euclidean gate: skip if odom is already far from the expected position
    double dx = x - current.x;
    double dy = y - current.y;
    if (sqrt(dx * dx + dy * dy) > MAX_CORRECTION_IN) return;

    // Read sensors (mm → in)
    double L_in = dist_left.get()  / 25.4;
    double R_in = dist_right.get() / 25.4;
    double F_in = dist_front.get() / 25.4;
    double B_in = dist_back.get()  / 25.4;

    int  L_conf = dist_left.get_confidence();
    int  R_conf = dist_right.get_confidence();
    int  F_conf = dist_front.get_confidence();
    int  B_conf = dist_back.get_confidence();

    bool L_large = dist_left.get_object_size()  > 200;
    bool R_large = dist_right.get_object_size() > 200;
    bool F_large = dist_front.get_object_size() > 200;
    bool B_large = dist_back.get_object_size()  > 200;

    // Map body-frame sensors to field axes based on current heading.
    // LemLib: theta=0=north(+Y), theta=90=east(+X), clockwise positive.
    // Snap to nearest cardinal direction.
    int heading = (int)(std::round(current.theta / 90.0)) * 90;
    heading = ((heading % 360) + 360) % 360;

    double xNeg_in, xPos_in, yNeg_in, yPos_in;
    int    xNeg_conf, xPos_conf, yNeg_conf, yPos_conf;
    bool   xNeg_large, xPos_large, yNeg_large, yPos_large;
    double robot_half_x, robot_half_y;

    switch (heading) {
        case 0:   // north: L→x−  R→x+  B→y−  F→y+
            xNeg_in = L_in;  xPos_in = R_in;  yNeg_in = B_in;  yPos_in = F_in;
            xNeg_conf = L_conf; xPos_conf = R_conf; yNeg_conf = B_conf; yPos_conf = F_conf;
            xNeg_large = L_large; xPos_large = R_large; yNeg_large = B_large; yPos_large = F_large;
            robot_half_x = ROBOT_HALF_W_IN; robot_half_y = ROBOT_HALF_L_IN;
            break;
        case 90:  // east:  B→x−  F→x+  R→y−  L→y+
            xNeg_in = B_in;  xPos_in = F_in;  yNeg_in = R_in;  yPos_in = L_in;
            xNeg_conf = B_conf; xPos_conf = F_conf; yNeg_conf = R_conf; yPos_conf = L_conf;
            xNeg_large = B_large; xPos_large = F_large; yNeg_large = R_large; yPos_large = L_large;
            robot_half_x = ROBOT_HALF_L_IN; robot_half_y = ROBOT_HALF_W_IN;
            break;
        case 180: // south: R→x−  L→x+  F→y−  B→y+
            xNeg_in = R_in;  xPos_in = L_in;  yNeg_in = F_in;  yPos_in = B_in;
            xNeg_conf = R_conf; xPos_conf = L_conf; yNeg_conf = F_conf; yPos_conf = B_conf;
            xNeg_large = R_large; xPos_large = L_large; yNeg_large = F_large; yPos_large = B_large;
            robot_half_x = ROBOT_HALF_W_IN; robot_half_y = ROBOT_HALF_L_IN;
            break;
        case 270: // west:  F→x−  B→x+  L→y−  R→y+
            xNeg_in = F_in;  xPos_in = B_in;  yNeg_in = L_in;  yPos_in = R_in;
            xNeg_conf = F_conf; xPos_conf = B_conf; yNeg_conf = L_conf; yPos_conf = R_conf;
            xNeg_large = F_large; xPos_large = B_large; yNeg_large = L_large; yPos_large = R_large;
            robot_half_x = ROBOT_HALF_L_IN; robot_half_y = ROBOT_HALF_W_IN;
            break;
        default:
            return; // not aligned to a cardinal heading
    }

    // Validate X axis. Only require large object size from sensors within range
    // (far-wall sensors legitimately have small object size).
    bool x_sum_ok   = std::abs((xNeg_in + xPos_in) - (FIELD_IN - 2.0 * robot_half_x)) < SUM_TOLERANCE_IN;
    bool x_conf_ok  = xNeg_conf >= MIN_CONFIDENCE && xPos_conf >= MIN_CONFIDENCE;
    bool x_range_ok = (xNeg_in < range) || (xPos_in < range);
    bool x_size_ok  = (xNeg_in < range ? xNeg_large : true) && (xPos_in < range ? xPos_large : true);
    bool x_valid    = x_sum_ok && x_conf_ok && x_range_ok && x_size_ok;

    // Validate Y axis
    bool y_sum_ok   = std::abs((yNeg_in + yPos_in) - (FIELD_IN - 2.0 * robot_half_y)) < SUM_TOLERANCE_IN;
    bool y_conf_ok  = yNeg_conf >= MIN_CONFIDENCE && yPos_conf >= MIN_CONFIDENCE;
    bool y_range_ok = (yNeg_in < range) || (yPos_in < range);
    bool y_size_ok  = (yNeg_in < range ? yNeg_large : true) && (yPos_in < range ? yPos_large : true);
    bool y_valid    = y_sum_ok && y_conf_ok && y_range_ok && y_size_ok;

    if (!x_valid && !y_valid) return;

    double new_x = current.x;
    double new_y = current.y;

    if (x_valid) {
        double x_from_neg = -72.0 + xNeg_in + robot_half_x;
        double x_from_pos =  72.0 - xPos_in - robot_half_x;
        new_x = (x_from_neg + x_from_pos) / 2.0;
    }

    if (y_valid) {
        double y_from_neg = -72.0 + yNeg_in + robot_half_y;
        double y_from_pos =  72.0 - yPos_in - robot_half_y;
        new_y = (y_from_neg + y_from_pos) / 2.0;
    }

    chassis.setPose(new_x, new_y, current.theta);
    pros::delay(100);
}

// void moveToPointAndTurn(double x, double y, double timeout, double theta, lemlib::MoveToPointParams params = {}, bool async = true) {
//     float magnitude = sqrt(x * x + y * y);
//     float baseExit = std::min(magnitude/2, 20.0f);
//     float startSpeed = (magnitude < 24.0f) ? 80.0f : 127.0f;
//     float ratio = 1.0f;
//     for (int n = 0; baseExit / pow(2, n) >= 6; n++) {
//         moveToPoint(x, y, timeout, {.forwards = params.forwards, .maxSpeed = startSpeed*ratio, .minSpeed = 16*ratio, .earlyExitRange = baseExit*ratio}, async);
//         ratio *= 0.5;
//     }
//     chassis.turnToHeading(theta, 1000, {}, async);
// }

// void moveToPoint(double x, double y, double timeout, lemlib::MoveToPointParams params = {}, bool async = true) {
//     float magnitude = sqrt(x * x + y * y);
//     float baseExit = std::min(magnitude/2, 20.0f);
//     float startSpeed = (magnitude < 24.0f) ? 80.0f : 127.0f;
//     float ratio = 1.0f;
//     for (int n = 0; baseExit / pow(2, n) >= 6 ; n++) {
//         moveToPoint(x, y, timeout, {.forwards = params.forwards, .maxSpeed = startSpeed*ratio, .minSpeed = 16*ratio, .earlyExitRange = baseExit*ratio});
//         ratio *= 0.5;
//     }
//     moveToPoint(x, y, timeout, {.forwards = params.forwards, .maxSpeed = startSpeed*ratio, .minSpeed = params.minSpeed, .earlyExitRange = params.earlyExitRange}, async);
// }

void moveToPoint(double x, double y, double timeout, lemlib::MoveToPointParams params = {}, bool async = true) {
    lemlib::Pose currentPose = chassis.getPose();
    double deltaX = x-currentPose.x;
    double deltaY = y-currentPose.y;
    float magnitude = sqrt(deltaX * deltaX + deltaY * deltaY);
    float baseExit = std::min(magnitude/2, 18.0f);
    float startSpeed = std::clamp(magnitude * 3.0f, 70.0f, 127.0f);
    chassis.moveToPoint(x, y, timeout, {.forwards = params.forwards, .maxSpeed = startSpeed, .minSpeed = 16, .earlyExitRange = baseExit});
    chassis.moveToPoint(x, y, timeout, {.forwards = params.forwards, .maxSpeed = std::clamp(startSpeed/2.1f, 30.0f, 90.0f), .minSpeed = params.minSpeed, .earlyExitRange = params.earlyExitRange}, async);
}

// Global selected auton
AutonRoutine selectedAuton = AutonRoutine::None;
//15.5 x 13.6
// ------------------------
// Autonomous routines
// ------------------------
void leftFourBall()   { 
    chassis.setPose(48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    moveToPoint(48, -47, 2000, {.forwards = false, .minSpeed =20});
    chassis.turnToHeading(90, 700);
    moveToPoint(64, -48.5, 500);
    moveToPoint( 27, -49, 2000, {.forwards = false} ,false);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(500);
    distanceReset(40, 26, -49);

    chassis.turnToHeading(30, 600);
    moveToPoint(36, -39, 2000);
    chassis.turnToHeading(90, 400);
    moveToPoint(10, -39, 5000, {.forwards = false, .minSpeed = 10});
    chassis.turnToHeading(135 ,200, {.maxSpeed = 67});
    intake.set_state_and_move(Intake::State::NONE);

}

void leftSeven() {
    chassis.setPose(48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    moveToPoint(48, -47, 2000, {.forwards = false, .minSpeed =20});
    chassis.turnToHeading(90, 700);
    moveToPoint(64, -48.5, 600);
    moveToPoint( 26, -49, 2000, {.forwards = false});
    pros::delay(500);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(600);
    distanceReset(38, 26, -49);
    wing.set_value(true);

    chassis.turnToHeading(-10, 900);
    moveToPoint(20, -24, 2000);
    pros::Task([] {
        pros::delay(100);
        lil_krith.set_value(true);
        pros::delay(400);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(135, 1000);
    moveToPoint(8.2, -8.2, 2000,{.forwards = false});
    chassis.turnToHeading(135, 200);
    pros::delay(700);
    intake.set_state_and_move(Intake::State::OUTTAKING);
    pros::delay(200);
    intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    pros::delay(1000);
    intake.set_state_and_move(Intake::State::NONE);

    
    moveToPoint(36, -37.6, 2000); 
    chassis.turnToHeading(90, 700);
    wing.set_value(false);
    moveToPoint(13.5, -38, 5000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3}, false);
    chassis.turnToHeading(135, 200, {.maxSpeed = 67});

}

void leftSevenRush()   { 
    chassis.setPose(50, -14, -90, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::Task([] {
        pros::delay(700);
        lil_krith.set_value(true);
    });
    chassis.moveToPoint(18, -21, 2000, { .minSpeed =20});
    //chassis.turnToHeading(-225,500);
    moveToPoint(46, -48, 2000, {.minSpeed = 10, .earlyExitRange = 2.5});
    chassis.turnToHeading(90, 1000);
    moveToPoint(63, -48, 620, {.maxSpeed=50});
    moveToPoint(18, -48.3, 2000, {.forwards = false});
    pros::delay(300);   
    wing.set_value(false);
    lil_krith.set_value(false);
    pros::delay(1000);
    distanceReset(40, 26, -49);

    chassis.turnToHeading(30, 600);
    moveToPoint(36, -38, 2000);
    chassis.turnToHeading(90, 400);
    moveToPoint(10, -38, 5000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 6});
    chassis.turnToHeading(135 ,200, {.maxSpeed = 42});
    intake.set_state_and_move(Intake::State::NONE);
}

void leftAWP() {
    chassis.setPose(48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    moveToPoint(48, -47, 2000, {.forwards = false, .minSpeed =20});
    chassis.turnToHeading(90, 700);
    moveToPoint(64, -48.5, 600);
    moveToPoint( 26, -49, 2000, {.forwards = false});
    pros::delay(500);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(700);
    distanceReset(38, 26, -49);
    wing.set_value(true);

    chassis.turnToHeading(-10, 900);
    moveToPoint(20, -24, 2000);
    pros::Task([] {
        pros::delay(100);
        lil_krith.set_value(true);
        pros::delay(400);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(135, 1000);
    moveToPoint(8.2, -8.2, 2000,{.forwards = false});
    pros::delay(400);
    intake.set_state_and_move(Intake::State::OUTTAKING);
    pros::delay(200);
    intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    pros::delay(1000);
    intake.set_state_and_move(Intake::State::INTAKING);
    
    moveToPoint(24, -24, 2000);
    chassis.turnToHeading(0, 700);
    moveToPoint(24, 24, 2000); 
    pros::Task([] {
        pros::delay(400);
        lil_krith.set_value(true);
        pros::delay(500);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(-135, 500);
    moveToPoint(13.2, 13.4, 2000,{ .maxSpeed = 70});
    intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
    
}
void rightFourBall()  { 
    chassis.setPose(-48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(true);
    moveToPoint(-48, -47, 2000, {.forwards = false, .minSpeed =20, .earlyExitRange = 6});
    chassis.turnToHeading(-90, 700);
    moveToPoint(-64, -48.5, 470);
    moveToPoint(-24, -49, 2000, {.forwards = false});
    pros::delay(530);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(400);
    distanceReset(40, -26, -48.5);

    chassis.turnToHeading(-150, 600);
    moveToPoint(-36, -57.5, 2000);
    chassis.turnToHeading(-90, 700);
    moveToPoint(-8, -57, 5000, {.forwards = false});
    chassis.turnToHeading(-45,200, {.maxSpeed = 67});
   
};
void rightSeven() {
    chassis.setPose(-48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(true);
    moveToPoint(-48, -47, 2000, {.forwards = false, .minSpeed =20, .earlyExitRange = 6});
    chassis.turnToHeading(-90, 700);
    moveToPoint(-68, -48.5, 570, {.maxSpeed = 55});
    moveToPoint(-24, -49, 2000, {.forwards = false});
    pros::delay(530);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(400);
    distanceReset(40, -26, -48.5);

    //middle goal
    chassis.turnToHeading(0, 600);
    wing.set_value(true);
    moveToPoint(-24, -24, 2000, {.maxSpeed = 80});
    pros::Task([] {
        pros::delay(400);
        lil_krith.set_value(true);
        pros::delay(400);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(45, 600);
    pros::Task([] {
        pros::delay(100);
        intake.set_state_and_move(Intake::State::NONE);
    });
    moveToPoint(-8, -8, 1000);
    moveToPoint(-9, -9, 400, {.forwards = false});
    intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
    pros::delay(1200);
    chassis.turnToHeading(45, 200);
    moveToPoint(-39.5, -36.5, 1000, {.forwards = false, .maxSpeed = 70});
    wing.set_value(false);
    chassis.turnToHeading(90, 700);
    moveToPoint(-6, -38.5, 1000);
    chassis.turnToHeading(45, 300, {.maxSpeed = 60});

}

void rightAWP() {
    chassis.setPose(-48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    moveToPoint(-48, 5.5, 700);
    moveToPoint(-48, -44, 2000, {.forwards = false, .minSpeed = 1, .earlyExitRange = 8});
    pros::Task([] {
        pros::delay(100);
        lil_krith.set_value(true);
    });
    chassis.turnToHeading(-90, 800);
    moveToPoint(-64, -48, 600);
    //pros::delay(70);
    
    moveToPoint(-23, -48.25, 700, {.forwards = false}, false);
    // pros::delay(530);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(800);
    distanceReset(38, -26, -48.5);

    chassis.turnToHeading(5, 1000);
    wing.set_value(true);
    pros::Task([] {
        pros::delay(2100);
        lil_krith.set_value(true);
    });
    moveToPoint(-24, 30, 2000);
    chassis.turnToHeading(-45, 400);
    moveToPoint(-50, 52.5, 2000, {.minSpeed =10, .earlyExitRange = 6});
    chassis.turnToHeading(-93, 700);
    moveToPoint(-26, 53, 700, {.forwards = false} ,false);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::delay(1000);
    distanceReset(38, -26, 49);

    moveToPoint(-70, 48.5, 1000);
    wing.set_value(true);
    moveToPoint(-9.8, 9.8, 2000,{.forwards = false});
    pros::delay(700);
    intake.set_state_and_move(Intake::State::OUTTAKING);
    pros::delay(200);
    intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    pros::delay(1000);
    intake.set_state_and_move(Intake::State::INTAKING);


}

void skills() { 
    chassis.setPose(48, 15.5, 180, false);
    pros::lcd::print(0, "Running Skills");
    pros::delay(300);
    wing.set_value(true);
    lil_krith.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    moveToPoint(48, 48, 2000, {.forwards = false});
    chassis.turnToHeading(90, 700);
    chassis.moveToPoint(67, 50, 1000, {.maxSpeed = 60});                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    pros::delay(2000);
    chassis.moveToPoint(67, 50, 1000, {.maxSpeed = 60});
    pros::delay(500);

    moveToPoint(30, 63, 2000, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(90, 700);
    moveToPoint(-30, 63, 2000, {.forwards = false, .minSpeed = 20});
    lil_krith.set_value(false);
    moveToPoint(-48, 50, 2000, {.forwards = false,  .minSpeed = 20});
    chassis.turnToHeading(-90, 700);
    moveToPoint(-20, 50, 2500, {.forwards = false, .maxSpeed = 70});
    pros::delay(400);
    wing.set_value(false);
    lil_krith.set_value(true);
    pros::delay(1000);
    
    
// match loader far sides
    chassis.moveToPoint(-67, 47, 2000, {.maxSpeed = 55});
    pros::delay(200);
    wing.set_value(true);
    pros::delay(1800);
    chassis.moveToPoint(-67, 47, 2000, {.maxSpeed = 55});
    pros::delay(700);
    chassis.moveToPoint(-20, 50, 2500, {.forwards = false, .maxSpeed = 70});
    pros::delay(700);
    wing.set_value(false);
    pros::delay(1000);
    distanceReset(38, -26, 48);

    moveToPoint(-44, 48, 1500);
    wing.set_value(true);
    chassis.turnToHeading(-183,900);
    moveToPoint(-48, -57, 2000);
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(-67, -57, 2000, {.maxSpeed = 55});
    pros::delay(2000);
    chassis.moveToPoint(-67, -57, 2000, {.maxSpeed = 55});
    pros::delay(500);

    moveToPoint(-30, -68, 2000, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(-90, 700);
    moveToPoint(30, -68, 2000, {.forwards = false, .minSpeed = 20});
    lil_krith.set_value(false);
    moveToPoint(48, -53, 2000, {.forwards = false,  .minSpeed = 20});
    chassis.turnToHeading(90, 700);
    moveToPoint(26, -53, 2500, {.forwards = false});
    pros::delay(400);
    wing.set_value(false);
    lil_krith.set_value(true);
    pros::delay(1000);

    chassis.moveToPoint(67, -48, 2000, {.maxSpeed = 55});
    wing.set_value(true);
    pros::delay(2000);
    chassis.moveToPoint(67, -48, 2000, {.maxSpeed = 55});
    pros::delay(500);
    moveToPoint(26, -53, 2500, {.forwards = false});
    wing.set_value(false);
    pros::delay(1000);
    wing.set_value(true);
    distanceReset(38, 26, -48);

    lil_krith.set_value(false);
    chassis.moveToPose(64,14, 0, 10000, {.minSpeed=127, .earlyExitRange = 4});
    chassis.moveToPoint(64, -5, 10000, {.minSpeed = 127});

//     //work on whole route after, just park for now
//     chassis.turnToHeading(-180,900);
//     chassis.turnToHeading(-270,700);
//     chassis.moveToPose(53.5, 33, -270, 2000);
//     lil_krith.set_value(false);
//     moveToPoint(70, 20, 2000, {.maxSpeed = 70});
//     pros::delay(300);
//     chassis.turnToHeading(-180, 700);
//     pros::delay(300);
//     lil_krith.set_value(true);
//     pros::delay(300);
//     moveToPoint(70, -40, 1000, {.minSpeed = 120});
//     pros::delay(300);
//     lil_krith.set_value(false);


   //aligning to far left side 
    /*wing.set_value(true);
    chassis.turnToHeading(-180, 700);
    pros::delay(300);
    chassis.moveToPose(-34,-54, -180, 2000, {.maxSpeed =100});
    chassis.turnToHeading(-90,700);
    moveToPoint(0, -54, 1200, {.forwards = false});
// match loader far left side
    chassis.moveToPose(-50, -54,-90, 1000, {.maxSpeed = 70});
    pros::delay(1300);
    moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(-50, -54, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(-40, -54, 700, {.forwards = false, .maxSpeed = 70});
    chassis.turnToHeading(-90,600);
// getting to close left side
    lil_krith.set_value(false);
    chassis.moveToPose(-24, -67, -90, 2000, {.forwards = false});
    chassis.moveToPose(48, -64 , -90, 2000, {.forwards = false});
    /*chassis.moveToPose(57, -51.5, -90, 2000, {.forwards = false});
    chassis.turnToHeading(92, 700);
    pros::delay(1000);
    moveToPoint(10, -51.5, 3500, {.forwards = false, .maxSpeed = 60});
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
    moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(82, -51, 200, {.maxSpeed = 70});
    pros::delay(300);
    moveToPoint(82, -51, 200, {.maxSpeed = 70});
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
    moveToPoint(0, 48, 3000);
}
void ang_pid_tuning_test() {
    chassis.setPose(0, 0, 0); 
    pros::delay(100);
    chassis.turnToHeading(90, 2000);
}
void slewTest() {
    chassis.setPose(0, 0, 0); 
    pros::delay(100);
    moveToPoint(0, 60, 3000);
}

//example use
void exampleDistanceResetAuton() {
    // Starting near a wall or corner
    chassis.setPose(0, 0, 0); 
    pros::delay(100);

    // move to a position somewhere
    moveToPoint(24, 24, 2000);
    
    // somewhere where it stops is best so that the distance measurement isnt noisy
    pros::delay(300);

    // pass the expected Pose (24, 24, 0). 
    // the sensors won't correct if you're > 5 inches off since then something is wrong with measurement
    distanceReset(40, 24, 24);

    // move in reverse somewhere else now that odom is corrected
    moveToPoint(0, 0, 2000, {.forwards = false});
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
    {"Left 7 wing",  AutonRoutine::LeftSevenRush,  leftSevenRush},
    {"Left Solo AWP",  AutonRoutine::LeftAWP,  leftAWP},
    {"Right Solo AWP", AutonRoutine::RightAWP, rightAWP},
    {"Skills",     AutonRoutine::Skills,    skills},
    {"Linear PID", AutonRoutine::LinPID, lin_pid_tuning_test},
    {"Angular PID", AutonRoutine::AngPID, ang_pid_tuning_test},
    {"Slew Test", AutonRoutine::SlewTest, slewTest},
};

static constexpr int AUTON_COUNT =
    sizeof(autons) / sizeof(autons[0]);

// ------------------------
// Autonomous selector task
// ------------------------
void autonSelectorTask(void*) {
    int index = 4;  // -1 = None
    uint8_t lastButtons = pros::lcd::read_buttons();

    while (true) {
        pros::lcd::clear_line(0);
        pros::lcd::clear_line(1);
        pros::lcd::clear_line(2);

        if (index == -1) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Vertical Wheel: %i", verticalEnc.get_position());
            pros::lcd::print(4, "Horizontal Wheel: %i", horizontalEnc.get_position());
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