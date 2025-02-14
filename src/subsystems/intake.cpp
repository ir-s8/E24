#include "main.h"
#include "subsystems/intake.hpp"
#include "subsystems/lb_mech.hpp"

extern int AC;

pros::Motor intake(INTAKE_PORT, pros::E_MOTOR_GEAR_BLUE, false);
pros::OpticalSensor color_sensor(OP_PORT,
                                OpticalSensorOutput ioutput = OpticalSensorOutput::hue,
                                bool idisableGestures = true,
                                std::unique_ptr<Filter> ifilter = std::make_unique<PassthroughFilter>()
)

void update_intake() {
    int input_1 = controller.get_digital(BUTTON_INTAKE);
    int input_2 = controller.get_digital(BUTTON_OUTTAKE);
    int c = color_sensor.getHue();

    if (abs(AC - c) > 25) {
        intake.move_velocity(0);
    }
    //set timer for 200 millisec
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
