#ifndef __LB_H__
#define __LB_H__

extern pros::Motor lb_mech;
extern bool LB_ARMED;

/*
enum {
    LB_LOWERED,
    LB_ARMED,
    LB_UP,
    LB_EXTENDED,
};
*/

void update_lb();
void update_lb_simple();
bool if_motor_stall(pros::Motor&);

#endif
