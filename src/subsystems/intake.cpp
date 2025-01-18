#include "main.h"
#include "subsystems/intake.hpp"

pros::Motor intake1(INTAKE_PORT1, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor intake2(INTAKE_PORT2, pros::E_MOTOR_GEAR_BLUE, false);

pros::MotorGroup intake({intake1, intake2});

void update_intake() {
    int input_1 = controller.get_digital(BUTTON_INTAKE);
    int input_2 = controller.get_digital(BUTTON_OUTTAKE);

    if (input_1) {
        intake.move_velocity(600);
    } else if (input_2){
        intake.move_velocity(-600);
    } else {
        intake.move_velocity(0);
    } 
}
