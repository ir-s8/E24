#include "subsystems/pneumatics.hpp"

pneumaticSys::pneumaticSys(char port, pros::controller_digital_e_t _button_out, pros::controller_digital_e_t _button_in)
        : last_input(0), piston_state(0), piston(port), button_out(_button_out), button_in(_button_in) {} 

pneumaticSys::pneumaticSys(char adi_port, pros::controller_digital_e_t _button)
        : last_input(0), piston_state(0), piston(adi_port), button_toggle(_button) {} 

int pneumaticSys::get_state() const {
    return piston_state;
}

void pneumaticSys::set_state(int state) {
    piston.set_value(state);
    piston_state = state;
}

void pneumaticSys::driver_update() {
    int input_out = controller.get_digital(button_out);
    int input_in = controller.get_digital(button_in);

    if (input_out) {
        piston.set_value(1);
    } else if (input_in) {
        piston.set_value(0);
    }
}

void pneumaticSys::driver_update_toggle() {
    int input = controller.get_digital(button_toggle);

    if (input && !last_input) {
        if (!piston_state) {
            piston.set_value(1);
            piston_state = 1;
        } else {
            piston.set_value(0);
            piston_state = 0;
        }
    }

    last_input = input;
}

pneumaticSys::~pneumaticSys() = default;
