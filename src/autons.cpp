#include "autons.hpp"
#include "main.h"

// Field and Robot Constants for Distance Odometry Reset (all in inches)
static constexpr double FIELD_IN               = 144.0;
static constexpr double ROBOT_HALF_W_IN        = 6.5;   // measure and update
static constexpr double ROBOT_HALF_L_IN        = 6.5;   // measure and update

// gotta fill in sensor offsets from tracking center
// Positive X is toward the right side of the robot
// Positive Y is toward the front of the robot
static constexpr double LEFT_SENSOR_OFFSET_IN  = 0.0;
static constexpr double RIGHT_SENSOR_OFFSET_IN = 0.0;
static constexpr double FRONT_SENSOR_OFFSET_IN = 0.0;
static constexpr double BACK_SENSOR_OFFSET_IN  = 0.0;

// thresholds for validity checks
static constexpr double SUM_TOLERANCE_IN       = 3.5;
static constexpr int    MIN_CONFIDENCE         = 30;
static constexpr double MAX_CORRECTION_IN      = 5.0;

void distanceReset(double range, double x, double y, double theta) {
    lemlib::Pose current = chassis.getPose();
    
    // euclidean 
    double dx = x - current.x, dy = y - current.y;
    if (sqrt(dx*dx + dy*dy) > MAX_CORRECTION_IN) return;

    // front is at 0 degrees, right is 90, back 180, left 270 relative to robot front
    struct SensorMapping { pros::Distance& ref; double offset; double angle_offset; };
    SensorMapping robot_sensors[] = {
        {dist_front, FRONT_SENSOR_OFFSET_IN, 0.0},
        {dist_right, RIGHT_SENSOR_OFFSET_IN, 90.0},
        {dist_back,  BACK_SENSOR_OFFSET_IN,  180.0},
        {dist_left,  LEFT_SENSOR_OFFSET_IN,  270.0}
    };

    // 0:+Y (north), 1:+X (east), 2:-Y (south), 3:-X (west)
    double wall_dist[4] = {-1.0, -1.0, -1.0, -1.0};
    
    // assign sensor to direciton based on heading
    for (auto& s : robot_sensors) {
        // find global heading of a specific sensor
        double sensor_global_angle = fmod(current.theta + s.angle_offset, 360.0);
        if (sensor_global_angle < 0) sensor_global_angle += 360.0;

        // determine(0, 90, 180, 270) this sensor is currently facing
        int direction = (int)(round(sensor_global_angle / 90.0)) % 4;
        double diff = fabs(sensor_global_angle - (direction * 90.0));
        if (diff > 45) diff = 90.0 - diff; // wrap around

        // only use sensor if it is aligned within 10 degrees of a wall and sees a large object (can remove this part if it doesnt work)
        if (diff < 10.0 && s.ref.get_confidence() >= MIN_CONFIDENCE && s.ref.get_object_size() > 200) {
            wall_dist[direction] = s.ref.get() / 25.4;
        }
    }

    // 4. calculate position based on aligned sensors
    double new_x = current.x;
    double new_y = current.y;
    bool x_valid = false, y_valid = false;

    // validate and calculate X-axis (global east (+X) and global west (-X) sensors)
    if (wall_dist[1] > 0 && wall_dist[3] > 0) { 
        double x_from_right =  72.0 - (wall_dist[1] + ROBOT_HALF_W_IN + RIGHT_SENSOR_OFFSET_IN); 
        double x_from_left  = -72.0 + (wall_dist[3] + ROBOT_HALF_W_IN + LEFT_SENSOR_OFFSET_IN);
        
        double expected_x_sum = FIELD_IN - 2.0 * ROBOT_HALF_W_IN;
        if (fabs((wall_dist[1] + wall_dist[3]) - expected_x_sum) < SUM_TOLERANCE_IN) {
            new_x = (x_from_left + x_from_right) / 2.0;
            x_valid = (wall_dist[1] < range || wall_dist[3] < range);
        }
    }

    // validate and calculate Y-axis (global north (+Y) and global south (-Y) sensors)
    if (wall_dist[0] > 0 && wall_dist[2] > 0) { 
        double y_from_front =  72.0 - (wall_dist[0] + ROBOT_HALF_L_IN + FRONT_SENSOR_OFFSET_IN);
        double y_from_back  = -72.0 + (wall_dist[2] + ROBOT_HALF_L_IN + BACK_SENSOR_OFFSET_IN);
        
        double expected_y_sum = FIELD_IN - 2.0 * ROBOT_HALF_L_IN;
        if (fabs((wall_dist[0] + wall_dist[2]) - expected_y_sum) < SUM_TOLERANCE_IN) {
            new_y = (y_from_back + y_from_front) / 2.0;
            y_valid = (wall_dist[0] < range || wall_dist[2] < range);
        }
    }

    // 5. finalize position update
    if (x_valid || y_valid) {
        chassis.setPose(new_x, new_y, theta);
        pros::delay(100);
    }
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
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(true);
    moveToPoint(48, -45, 2000, {.forwards = false, .minSpeed =20, .earlyExitRange = 6});
    chassis.turnToHeading(90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(68, -48.5, 420, {.maxSpeed = 55});
    moveToPoint(24, -49, 1500, {.forwards = false});
    pros::delay(480);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(260);
    distanceReset(40, 26, -48.5, 90);

    chassis.turnToHeading(30, 600, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(36, -36, 2000, {.minSpeed = 20, .earlyExitRange = 6});
    chassis.turnToHeading(90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(8, -37, 5000, {.forwards = false});
    chassis.turnToHeading(135, 400, {.maxSpeed = 67});
    intake.set_state_and_move(Intake::State::NONE);

}

void leftSeven() {
    chassis.setPose(48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4+3 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(true);
    moveToPoint(48, -45, 2000, {.forwards = false, .minSpeed =20, .earlyExitRange = 6});
    chassis.turnToHeading(90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(68, -48.5, 420, {.maxSpeed = 55});
    moveToPoint(24, -49, 1500, {.forwards = false});
    pros::delay(480);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(260);
    distanceReset(40, 26, -48.5, 90);


    //middle goal
    chassis.turnToHeading(0, 600, {.minSpeed = 20, .earlyExitRange = 10});
    wing.set_value(true);
    moveToPoint(24, -24, 1000);
    pros::Task([] {
        pros::delay(400);
        lil_krith.set_value(true);
        pros::delay(400);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(135, 600, {.minSpeed = 20, .earlyExitRange = 10});
    pros::Task([] {
        pros::delay(400);
        intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
        pros::delay(150);
        intake.set_state_and_move(Intake::State::NONE);
        pros::delay(550);
        intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    });
    moveToPoint(8.67, -8.67, 1000, {.forwards = false});
    pros::delay(700);
    chassis.turnToHeading(150, 200, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(28.5, -36.6, 1000, {.minSpeed = 1, .earlyExitRange = 6});
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::NONE);
    chassis.turnToHeading(90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(8, -37, 5000, {.forwards = false});
    chassis.turnToHeading(135, 400, {.maxSpeed = 67});
}

void leftSevenRush()   { 
    chassis.setPose(50, -14, -90, false);
    pros::lcd::print(0, "Running 7 Ball Wing");
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
    distanceReset(40, 26, -49,  90);

    chassis.turnToHeading(30, 600);
    moveToPoint(36, -38, 2000);
    chassis.turnToHeading(90, 400);
    moveToPoint(10, -38, 5000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 6});
    chassis.turnToHeading(135, 400, {.maxSpeed = 42});
    intake.set_state_and_move(Intake::State::NONE);
}

void leftAWP() {
    chassis.setPose(48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4+3+3 SAWP");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    //moveToPoint(48, 5.5, 700);
    lil_krith.set_value(true);
    moveToPoint(48, -45, 2000, {.forwards = false, .minSpeed =20, .earlyExitRange = 6});
    chassis.turnToHeading(90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(68, -48.5, 420, {.maxSpeed = 55});
    moveToPoint(24, -49, 1500, {.forwards = false});
    pros::delay(480);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(260);
    distanceReset(40, 26, -48.5, 90);


    //middle high goal
    chassis.turnToHeading(0, 600, {.minSpeed = 20, .earlyExitRange = 10});
    wing.set_value(true);
    moveToPoint(24, -24, 1000);
    pros::Task([] {
        pros::delay(200);
        lil_krith.set_value(true);
        pros::delay(400);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(135, 600);
    pros::Task([] {
        pros::delay(400);
        intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
        pros::delay(150);
        intake.set_state_and_move(Intake::State::NONE);
        pros::delay(550);
        intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    });
    moveToPoint(8.67, -8.67, 1000, {.forwards = false});
    pros::delay(900);
    moveToPoint(24, -24, 1000);
    
    //middle low goal
    chassis.turnToHeading(-10, 600);
    wing.set_value(true);
    moveToPoint(24, 24, 1000);
    pros::Task([] {
        pros::delay(200);
        lil_krith.set_value(true);
        pros::delay(400);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(-135, 600);
    pros::Task([] {
        pros::delay(100);
        intake.set_state_and_move(Intake::State::NONE);
         pros::delay(800);
        intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
    });
    moveToPoint(7.8, 8, 1000);
    pros::delay(700);
    moveToPoint(7.8, 8, 1000);
    
}
void rightFourBall()  { 
    chassis.setPose(-48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(true);
    moveToPoint(-48, -45, 2000, {.forwards = false, .minSpeed =20, .earlyExitRange = 6});
    chassis.turnToHeading(-90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(-68, -48.5, 420, {.maxSpeed = 55});
    moveToPoint(-24, -49, 1500, {.forwards = false});
    pros::delay(480);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(260);
    distanceReset(40, -26, -48.5, -90);


    chassis.turnToHeading(-150, 600, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(-36, -61, 2000, {.minSpeed = 20, .earlyExitRange = 6});
    chassis.turnToHeading(-90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(-8, -60, 5000, {.forwards = false});
    chassis.turnToHeading(-45, 400, {.maxSpeed = 67});
    intake.set_state_and_move(Intake::State::NONE);
   
};
void rightSeven() {
    chassis.setPose(-48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4+3 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(true);
    moveToPoint(-48, -45, 2000, {.forwards = false, .minSpeed =20, .earlyExitRange = 6});
    chassis.turnToHeading(-90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(-68, -48.5, 420, {.maxSpeed = 55});
    moveToPoint(-24, -49, 1500, {.forwards = false});
    pros::delay(480);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(260);
    distanceReset(40, -26, -48.5, -90);


    //middle goal
    chassis.turnToHeading(0, 600, {.minSpeed = 20, .earlyExitRange = 10});
    wing.set_value(true);
    moveToPoint(-24, -24, 1000);
    pros::Task([] {
        pros::delay(400);
        lil_krith.set_value(true);
        pros::delay(400);
        lil_krith.set_value(false);
    });
    chassis.turnToHeading(45, 600, {.minSpeed = 20, .earlyExitRange = 10});
    pros::Task([] {
        pros::delay(100);
        intake.set_state_and_move(Intake::State::NONE);
         pros::delay(800);
        intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
    });
    moveToPoint(-8, -8, 1000);
    pros::delay(700);
    chassis.turnToHeading(30, 200, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(-28.5, -36.6, 1000, {.forwards = false, .minSpeed = 1, .earlyExitRange = 6});
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::NONE);
    chassis.turnToHeading(90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(-6, -38.5, 1000);
    chassis.turnToHeading(45, 400, {.maxSpeed = 60});

}

void rightAWP() {
    chassis.setPose(-48, -15.5, 0, false);
    pros::lcd::print(0, "Running 4 Ball Wing");
    pros::delay(100);
    wing.set_value(true);
    intake.set_state_and_move(Intake::State::INTAKING);
    moveToPoint(-48, 5.5, 700);
    lil_krith.set_value(true);
    moveToPoint(-48, -45, 2000, {.forwards = false, .minSpeed =20, .earlyExitRange = 6});
    chassis.turnToHeading(-90, 700, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(-68, -48.5, 420, {.maxSpeed = 55});
    moveToPoint(-24, -49, 1500, {.forwards = false});
    pros::delay(480);
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    lil_krith.set_value(false);
    pros::delay(260);
    distanceReset(40, -26, -48.5, -90);


    chassis.turnToHeading(10, 1000,{.minSpeed = 20, .earlyExitRange = 10});
    wing.set_value(true);
    pros::Task([] {
        pros::delay(2200);
        lil_krith.set_value(true);
    });
    moveToPoint(-24, 24, 2000, {.minSpeed = 50, .earlyExitRange = 6});
    chassis.turnToHeading(-45, 400, {.minSpeed = 20, .earlyExitRange = 10});
    moveToPoint(-48, 49, 2000, {.minSpeed = 20, .earlyExitRange = 6});
    chassis.turnToHeading(-90, 700, {.minSpeed = 20, .earlyExitRange = 10}); 
    moveToPoint(-24, 50.5, 500, {.forwards = false});
    wing.set_value(false);
    intake.set_state_and_move(Intake::State::INTAKING);
    pros::delay(800);
    distanceReset(40, -26, 48.5, 90);


    wing.set_value(true);
    moveToPoint(-71, 48, 1000, {.maxSpeed = 50});
    pros::Task([] {
        pros::delay(1700);
        intake.set_state_and_move(Intake::State::OUTTAKING_SLOW);
        pros::delay(150);
        lil_krith.set_value(false);
        intake.set_state_and_move(Intake::State::NONE);
        pros::delay(450);
        intake.set_state_and_move(Intake::State::MIDDLE_AUTO);
    });
    moveToPoint(-10, 7.5, 1000, {.forwards = false});


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
    distanceReset(38, -26, 48, -90);

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
    distanceReset(38, 26, -48, 90);

    lil_krith.set_value(false);
    chassis.moveToPose(64, 14, 0, 10000, {.minSpeed=127, .earlyExitRange = 4});
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
    distanceReset(40, 24, 24, 0);

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