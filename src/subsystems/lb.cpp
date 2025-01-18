#include "main.h"
#include "subsystems/lb_mech.hpp"
/*
enum {
    LB_LOWERED,
    LB_ARMED,
    LB_UP,
    LB_EXTENDED,
};
*/
pros::Motor lb_mech(LB_PORT, pros::E_MOTOR_GEAR_RED, false);
//int LB_STATE = LB_LOWERED;

bool if_motor_stall(pros::Motor& m) {
    return m.is_stopped() && (m.get_voltage() > 2000);
}
/*
//add toggle
void update_lb() {
    int input_1 = controller.get_digital(BUTTON_LB_OUT);
    int input_2 = controller.get_digital(BUTTON_LB_IN);

    if (input_1 && LB_STATE == LB_LOWERD) {
        lb_mech.move_absolute(77);
        LB_STATE = LB_ARMED;
    } else if (input_1 && LB_STATE == LB_ARMED) {
        lb_mech.move_absolute(1200);
        LB_STATE = LB_EXTENDED;
    } else if (input_2 && LB_STATE == LB_EXTENDED) {
        LB_STATE = LB_LOWERED;
        lb_mech.move_absolute(0);
    }
}
*/
void update_lb_simple() {

    int input_1 = controller.get_digital(BUTTON_LB_OUT);
    int input_2 = controller.get_digital(BUTTON_LB_IN);

    int input_3 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool motor_move = false;
        
    if (input_1) {
        lb_mech.move_velocity(100);
    } else if (input_2) {
        lb_mech.move_velocity(-100);
    } else {
        lb_mech.move_velocity(0);
    }
}
