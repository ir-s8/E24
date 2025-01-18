#include "main.h"
#include "lemlib/api.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(LEFT_FRONT, pros::E_MOTOR_GEARSET_06); // left front motor. port 12, reversed
pros::Motor lM(LEFT_MIDDLE, pros::E_MOTOR_GEARSET_06); // left middle motor. port 11, reversed
pros::Motor lB(LEFT_BACK, pros::E_MOTOR_GEARSET_06); // left back motor. port 1, reversed
pros::Motor rF(RIGHT_FRONT, pros::E_MOTOR_GEARSET_06); // right front motor. port 2
pros::Motor rM(RIGHT_MIDDLE, pros::E_MOTOR_GEARSET_06); // right middle motor. port 11
pros::Motor rB(RIGHT_BACK, pros::E_MOTOR_GEARSET_06); // right back motor. port 13

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// Inertial Sensor on port 13
pros::Imu imu(IMU_PORT);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
//pros::Rotation horizontalEnc(15, true);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              6 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(16, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
                                            2, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
/*
    pros::Task screenTask([&]() {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            //lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
    });
*/
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */


//map points down


void mg_rush_map_left() {
    chassis.moveToPoint(0, -25, 2000, {.forwards=false, .minSpeed=80});
    chassis.moveToPose(10, -51, -20, 1000, {.forwards=false, .maxSpeed=110});;
    chassis.waitUntilDone();
    clamp.set_state(1);     //grab mg #1
    
    pros::delay(280);
    intake.move_voltage(12000);
    pros::delay(800);
    chassis.moveToPoint(12, -32, 1000, {.maxSpeed=48}); //go to bottom red
    intake.move_voltage(0);     //
    chassis.waitUntilDone();    //
    intake.move_voltage(12000);
    chassis.moveToPoint(16, -28, 1000, {.maxSpeed=48});     //intake bottom red
    chassis.waitUntilDone();

    pros::delay(800);       //score
    intake.move_voltage(0);     //

    chassis.turnTo(36, -60, 1000); //face ladder
    intake.move_voltage(0);
    clamp.set_state(0);
    pros::delay(500);           //release clamp

    chassis.moveToPoint(36, -42, 1000, {.maxSpeed=64}); //go to ladder
} 

//x coor * -1
void mg_rush_map_right() {
    chassis.moveToPoint(0, -36, 2000, {.forwards=false, .minSpeed=80});
    chassis.moveToPose(-12, -60, 45, 1000, {.forwards=false});
    clamp.set_state(1);     //grab mg #1
    
    intake.move_voltage(12000);
    chassis.moveToPoint(-16, -36, 1000, {.maxSpeed=64}); //go to bottom red
    chassis.waitUntilDone();
    chassis.moveToPoint(-24, -24, 1000, {.maxSpeed=64}); //intake + st
    pros::delay(500);

    chassis.turnTo(-36, -60, 1000); //face ladder
    intake.move_voltage(0);
    chassis.waitUntilDone();
    clamp.set_state(0);     //drop mg
    pros::delay(200);
    
    chassis.moveToPoint(-36, -60, 1000); //go to ladder
} 

        
void awp_p_left() {
    chassis.moveToPoint(-24, -24, 1000, {.forwards=false});
    chassis.waitUntilDone();
    clamp.set_state(1);

    chassis.moveToPoint(0, -48, 1000);
    intake.move_voltage(12000);
    chassis.moveToPoint(4, -53, 1000);
    
}

     

    




void autonomous() {
    //chassis.moveToPose(24, 48, 45, 1000, {.maxSpeed=127});
    //chassis.turnTo(90, 0, 9000);

    mg_rush_map_left();
}

#define MATH_E  2.718281828459045235360
#define SCALE   1

int scaler(int input) {
    if (!input) {
        return 0;
    }

    float o_input = input / 127.0;

    if (o_input < 0) {
        return -127 * (1 - powl(MATH_E, -1 * o_input * SCALE)) / (1 - powl(MATH_E, SCALE));
    }
    return 127 * (1 - powl(MATH_E, o_input * SCALE)) / (1 - powl(MATH_E, SCALE));
}

void opcontrol() {
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        //leftY = scaler(leftY);
        //rightX = scaler(rightX);

        update_intake();
        clamp.driver_update_toggle();        

        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX);

        // delay to save resources
        pros::delay(10);
    }
}
