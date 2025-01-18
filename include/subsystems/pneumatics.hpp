#ifndef __PNEUMATICS_HPP__
#define __PNEUMATICS_HPP__

#include "main.h"

class pneumaticSys {
    int last_input;
    int piston_state;

    pros::ADIDigitalOut piston;
    pros::controller_digital_e_t button_out;
    pros::controller_digital_e_t button_in;
    pros::controller_digital_e_t button_toggle;

public:
    pneumaticSys(char, pros::controller_digital_e_t, pros::controller_digital_e_t);
    pneumaticSys(char, pros::controller_digital_e_t);

    int get_state() const;
    void set_state(int);

    void driver_update();
    void driver_update_toggle();

    ~pneumaticSys();
};

#endif
