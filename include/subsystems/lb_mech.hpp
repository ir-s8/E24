#ifndef __LB_H__
#define __LB_H__

extern pros::Motor lb_mech;

extern int LB_STATE;
enum {
    LB_LOWERED,
    LB_ARMED1,
    LB_ARMED2,
    LB_EXTENDED,
};

void update_lb();
void update_lb_simple();
bool if_motor_stall(pros::Motor&);

#endif
