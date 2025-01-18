#include "main.h"
#include "subsystems/intake.hpp"
#include "subsystems/lb_mech.hpp"

pros::Motor intake(INTAKE_PORT, pros::E_MOTOR_GEAR_BLUE, false);
//extern int LB_STATE;

void update_intake() {
    int input_1 = controller.get_digital(BUTTON_INTAKE);
    int input_2 = controller.get_digital(BUTTON_OUTTAKE);

    if (input_1) {
        intake.move_velocity(600);
        //pros::delay(5);
        //if (if_motor_stall(intake));
          //  intake.move_velocity(0);
            //LB_STATE = LB_ARMED;
       // }
    } else if (input_2){
        intake.move_velocity(-600);
    } else {
        intake.move_velocity(0);
    } 
}
