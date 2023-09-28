#ifndef REMOTECONTROL_REMOTE_CONTROL_H
#define REMOTECONTROL_REMOTE_CONTROL_H

#include "vector.h"

typedef struct RemoteControl {
    double l1, l2, d1, d2, d;
    double w1, w2, w3;
    Vec3d a, b, c;
    Vec3d dp1, dp2, dp3;
    Vec3d da, db, dc;
    Vec3d center;
    unsigned error_flag;
} RemoteControl;

extern void rc_init(RemoteControl *rc, double l1, double l2, double d1, double d2);

extern void solution(RemoteControl *rc);

#endif //REMOTECONTROL_REMOTE_CONTROL_H
