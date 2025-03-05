#include "main.h"
#include "subsystems/lb_mech.hpp"

pros::Motor lb_mech(LB_PORT, pros::E_MOTOR_GEAR_RED, false, pros::E_MOTOR_ENCODER_DEGREES);
int LB_STATE = LB_LOWERED;

bool if_motor_stall(pros::Motor& m) {
    return m.is_stopped() && (m.get_voltage() > 2000);
}

int li1 = 0;
int li2 = 0;

void update_lb() {
    int input1 = controller.get_digital(BUTTON_LB_OUT);
    int input2 = controller.get_digital(BUTTON_LB_IN);

    bool toggle_1 = (input1 == 1 && li1 == 0);
    bool toggle_2 = (input2 != li2);

    //std::cout << toggle_1 << " " << toggle_2 << "\n";
    //pros::lcd::print(5, "toggle1: %s, toggle2: %s", toggle_1 ? "true" : "false", toggle_2 ? "true" : "false");

    li1 = input1; 
    li2 = input2; 

    if (toggle_1 && LB_STATE == LB_LOWERED) {
        lb_mech.move_absolute(-26, 28);  //-23
        LB_STATE = LB_ARMED1;
        //std::cout << "lower -> armed" << std::endl;
        return;
    } else if (toggle_1 && LB_STATE == LB_ARMED1) {
        lb_mech.move_absolute(-100, 88);
        LB_STATE = LB_ARMED2;
        //std::cout << "armed -> extended" << std::endl;
        return;
    } else if (toggle_1 && LB_STATE == LB_ARMED2) {
        lb_mech.move_absolute(-180, 88);
        LB_STATE = LB_EXTENDED;
        //std::cout << "armed -> extended" << std::endl;
        return;
    } 
    else if (toggle_2) {
        LB_STATE = LB_LOWERED;
        lb_mech.move_absolute(0-8, -64);
        //std::cout << "extended -> lowered" << std::endl;
        return;
    } 
    
}

void update_lb_simple() {
    int input_1 = controller.get_digital(BUTTON_LB_OUT);
    int input_2 = controller.get_digital(BUTTON_LB_IN);

    if (input_1) {
        lb_mech.move_velocity(100);
    } else if (input_2) {
        lb_mech.move_velocity(-100);
    } else {
        lb_mech.move_velocity(0);
    }
}
