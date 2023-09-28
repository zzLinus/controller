#include "remote_control.h"

#include <stdio.h>
#include <math.h>

void rc_init(RemoteControl *rc, double d1, double d2, double l1, double l2) {
    rc->d1 = d1;
    rc->d2 = d2;
    rc->l1 = l1;
    rc->l2 = l2;
    rc->d = d1 - d2;
    vector_init(&rc->dp1, 0.0, 0.0, 1.0);
    vector_init(&rc->dp2, 0.0, cos(-M_PI / 6), sin(-M_PI / 6));
    vector_init(&rc->dp3, 0.0, cos(-M_PI * 5 / 6), sin(-M_PI * 5 / 6));
    mul(rc->d, &rc->dp1, &rc->da);
    mul(rc->d, &rc->dp2, &rc->db);
    mul(rc->d, &rc->dp3, &rc->dc);
    rc->error_flag = 0;
}

double get_circumradius(double a, double b, double c) {
    double p = (a + b + c) * 0.5;
    if(p <= a || p <= b || p <= c) {
        return 0.0;
    }
    else {
        return a * b * c * 0.25 / sqrt(p * (p - a) * (p - b) * (p - c));
    }
}

void get_joint_coordinate(RemoteControl *rc) {
    mul(rc->l1 * cos(M_PI - rc->w1), &rc->dp1, &rc->a);
    mul(rc->l1 * cos(M_PI - rc->w2), &rc->dp2, &rc->b);
    mul(rc->l1 * cos(M_PI - rc->w3), &rc->dp3, &rc->c);
    rc->a.x += rc->l1 * sin(M_PI - rc->w1);
    rc->b.x += rc->l1 * sin(M_PI - rc->w2);
    rc->c.x += rc->l1 * sin(M_PI - rc->w3);
    add(&rc->a, &rc->da, &rc->a);
    add(&rc->b, &rc->db, &rc->b);
    add(&rc->c, &rc->dc, &rc->c);
}

void get_center_point(RemoteControl *rc) {
    static Vec3d pab, pbc, pca;
    static double ab, bc, ca;
    static Vec3d rbc, rh;
    sub(&rc->b, &rc->a, &pab);
    sub(&rc->c, &rc->b, &pbc);
    sub(&rc->a, &rc->c, &pca);
    ab = norm(&pab);
    bc = norm(&pbc);
    ca = norm(&pca);

    double R = get_circumradius(ab, bc, ca);

    if(R == 0) {
        rc->error_flag = 1;
        return;
    }

    mul(dot(&pca, &pbc) / (bc * bc), &pbc, &rbc);
    sub(&pca, &rbc, &rbc);
    mul(1.0 / norm(&rbc), &rbc, &rbc);
    mul(sqrt(R * R - 0.25 * bc * bc), &rbc, &rbc);

    get_plane_normal_vector(&pab, &pbc, &rh);
    if(rh.x < 0) {
        mul(-1.0, &rh, &rh);
    }
    mul(sqrt(rc->l2 * rc->l2 - R * R), &rh, &rh);

    add(&rc->b, &rc->c, &rc->center);
    mul(0.5, &rc->center, &rc->center);
    add(&rc->center, &rbc, &rc->center);
    add(&rc->center, &rh, &rc->center);
}

void solution(RemoteControl *rc) {
    if(rc == NULL) {
        return;
    }
    get_joint_coordinate(rc);
    get_center_point(rc);
    if(vector_error_flag) {
        vector_error_flag = 0;
        rc->error_flag = 1;
    }
}

/*** demo ***/

//int main(void) {
//    RemoteControl rc;
//    rc_init(&rc, 5, 2, 4, 7);
//    rc.w1 = M_PI * 9 / 13;
//    rc.w2 = M_PI * 3 / 4;
//    rc.w3 = M_PI * 5 / 11;
//    solution(&rc);
//    if(rc.error_flag) {
//        printf("Error!\n");
//    }
//    else {
//        printf("center_point: (%lf, %lf, %lf)\n", rc.center.x, rc.center.y, rc.center.z);
//    }
//}
