#include "main.h"
#include "subsystems/intake.hpp"
#include "subsystems/lb_mech.hpp"
using namespace okapi;

extern int AC;

pros::Motor intake(INTAKE_PORT, pros::E_MOTOR_GEAR_BLUE, false);
pros::Optical color_sensor(COLOR_PORT);
Timer timer = Timer(2000);

bool enable_color_task = true;

//color sort define red and blue
#define RED_COLOR 10
#define BLUE_COLOR 200
int opposing_alliance_color = RED_COLOR; //CHANGE THIS WHEN YOU SWITCH ALLIANCE

/*
pros::OpticalSensor color_sensor(OP_PORT,
                                OpticalSensorOutput ioutput = OpticalSensorOutput::hue,
                                bool idisableGestures = true,
                                std::unique_ptr<Filter> ifilter = std::make_unique<PassthroughFilter>()
)
*/

static bool toggle = false;
int speed = 12000;
int last_input=0;

bool color_disabled = false;

void update_intake() {
    int input_1 = controller.get_digital(BUTTON_INTAKE);
    int input_2 = controller.get_digital(BUTTON_OUTTAKE);
    int input_3 = controller.get_digital(BUTTON_DISABLE);

    int color = color_sensor.get_hue();
    bool isRed = abs(color-opposing_alliance_color) < 50;
    
    if(toggle==true && timer.isDone()){
      speed = 12000;
      toggle=false;
    }
    if(isRed){ 
      toggle=true;
      timer.reset();
      speed=6000;
    
    }
    if(input_3!=last_input){
      color_disabled = !color_disabled;
    } else if (input_1) {
        intake.move_voltage((color_disabled) ? 12000 : speed);
        
    } else if (input_2){
        intake.move_voltage(-speed);
    } else {
        intake.move_voltage(0);
    } 
    last_input=input_3;
}


pros::Task intake_task([&]() {
  if(enable_color_task=true){
    intake.move_voltage(speed);
    int color = color_sensor.get_hue();
    bool isColor = abs(color-opposing_alliance_color) < 50;
    
    if(toggle==true && timer.getTimePassed() - start_time > 0.2 ){
      speed = 12000;
      start_time=timer.getTimePassed();
      toggle=false;
    }
    if(isColor){ 
      toggle=true; 
      start_time = timer.getTimePassed();
      speed /= 2;
    }
  }
});

void stop_intake(){
  enable_color_task=false;
  speed=0;
}
