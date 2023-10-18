#ifndef REMOTECONTROL_REMOTE_CONTROL_H
#define REMOTECONTROL_REMOTE_CONTROL_H

#include "vector.h"

typedef struct RemoteControl {
    Vec3d a, b, c;
    Vec3d dp1, dp2, dp3;
    Vec3d da, db, dc;
    Vec3d center;
    double l1, l2, d1, d2, d;
    double w1, w2, w3;
    unsigned error_flag;
} RemoteControl;

extern RemoteControl *rc_init(double d1, double d2, double l1, double l2);

extern void solution(RemoteControl *rc);

extern void chan_axis(RemoteControl *rc);

#endif //REMOTECONTROL_REMOTE_CONTROL_H
