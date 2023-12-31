#include "controller.h"

#include <math.h>
#include <stdio.h>

#include "com.h"
#include "ge.h"
#include "usart.h"

RemoteControl *rc_init(double d1, double d2, double l1, double l2)
{
    RemoteControl *rc;
    rc = malloc(sizeof(RemoteControl));
    rc->d1 = d1;
    rc->d2 = d2;
    rc->l1 = l1;
    rc->l2 = l2;
    rc->d = d1 - d2;
    rc_vector_init(&rc->dp1, 0.0, 0.0, 1.0);
    rc_vector_init(&rc->dp2, 0.0, cos(-M_PI / 6), sin(-M_PI / 6));
    rc_vector_init(&rc->dp3, 0.0, cos(-M_PI * 5 / 6), sin(-M_PI * 5 / 6));
    rc_mul(rc->d, &rc->dp1, &rc->da);
    rc_mul(rc->d, &rc->dp2, &rc->db);
    rc_mul(rc->d, &rc->dp3, &rc->dc);
    rc->error_flag = 0;
    return rc;
}

double get_circumradius(double a, double b, double c)
{
    double p = (a + b + c) * 0.5;
    if (p <= a || p <= b || p <= c)
    {
        return 0.0;
    }
    else
    {
        return a * b * c * 0.25 / sqrt(p * (p - a) * (p - b) * (p - c));
    }
}

void get_joint_coordinate(RemoteControl *rc)
{
    rc_mul(rc->l1 * cos(M_PI - rc->w1), &rc->dp1, &rc->a);
    rc_mul(rc->l1 * cos(M_PI - rc->w2), &rc->dp2, &rc->b);
    rc_mul(rc->l1 * cos(M_PI - rc->w3), &rc->dp3, &rc->c);
    rc->a.x = rc->l1 * sin(M_PI - rc->w1);
    rc->b.x = rc->l1 * sin(M_PI - rc->w2);
    rc->c.x = rc->l1 * sin(M_PI - rc->w3);
    rc_add(&rc->a, &rc->da, &rc->a);
    rc_add(&rc->b, &rc->db, &rc->b);
    rc_add(&rc->c, &rc->dc, &rc->c);
}

//#define ALGEBRAIC_SOLUTION
#define GEOMETRIC_SOLUTION

void get_center_point(RemoteControl *rc) {

#ifdef ALGEBRAIC_SOLUTION
    static Matrix mat;
    matrix_init(&mat, 2, 4);
    mat.val[0][0] = 2 * (rc->b.y - rc->a.y);
    mat.val[0][1] = 2 * (rc->b.z - rc->a.z);
    mat.val[0][2] = 2 * (rc->b.x - rc->a.x);
    mat.val[0][3] = (rc->b.x * rc->b.x + rc->b.y * rc->b.y + rc->b.z * rc->b.z) -
                    (rc->a.x * rc->a.x + rc->a.y * rc->a.y + rc->a.z * rc->a.z);
    mat.val[1][0] = 2 * (rc->c.y - rc->b.y);
    mat.val[1][1] = 2 * (rc->c.z - rc->b.z);
    mat.val[1][2] = 2 * (rc->c.x - rc->b.x);
    mat.val[1][3] = (rc->c.x * rc->c.x + rc->c.y * rc->c.y + rc->c.z * rc->c.z) -
                    (rc->b.x * rc->b.x + rc->b.y * rc->b.y + rc->b.z * rc->b.z);
    GE(&mat);

    double k1 = -mat.val[0][2], k2 = -mat.val[1][2];
    double b1 = mat.val[0][3], b2 = mat.val[1][3];
    double a = 1 + k1 * k1 + k2 * k2;
    double b = -2 * (rc->a.x + k1 * (rc->a.y - b1) + k2 * (rc->a.z - b2));
    double c = rc->a.x * rc->a.x + (rc->a.y - b1) * (rc->a.y - b1) + (rc->a.z - b2) * (rc->a.z - b2) - rc->l2 * rc->l2;

    double delta = b * b - 4 * a * c;
    rc->center.x = -b / (2 * a) + fabs(sqrt(delta) / (2 * a));
    rc->center.y = k1 * rc->center.x + b1;
    rc->center.z = k2 * rc->center.x + b2;
#endif
#ifdef GEOMETRIC_SOLUTION
    static Vec3d pab, pbc, pca;
    static double ab, bc, ca;
    static Vec3d rbc, rh;
    rc_sub(&rc->b, &rc->a, &pab);
    rc_sub(&rc->c, &rc->b, &pbc);
    rc_sub(&rc->a, &rc->c, &pca);
    ab = rc_norm(&pab);
    bc = rc_norm(&pbc);
    ca = rc_norm(&pca);

    double R = get_circumradius(ab, bc, ca);
    if (R < 1e-6) {
        rc->error_flag = 1;
        return;
    }

    rc_mul(rc_dot(&pca, &pbc) / (bc * bc), &pbc, &rbc);
    rc_sub(&pca, &rbc, &rbc);
    rc_mul(1.0 / rc_norm(&rbc), &rbc, &rbc);
    rc_mul(sqrt(R * R - 0.25 * bc * bc), &rbc, &rbc);

    rc_get_plane_normal_vector(&pab, &pbc, &rh);
    if (rh.x < 0) {
        rc_mul(-1.0, &rh, &rh);
    }
    rc_mul(sqrt(rc->l2 * rc->l2 - R * R), &rh, &rh);

    rc_add(&rc->b, &rc->c, &rc->center);
    rc_mul(0.5, &rc->center, &rc->center);
    rc_add(&rc->center, &rbc, &rc->center);
    rc_add(&rc->center, &rh, &rc->center);
#endif
}

void rm_clean(RemoteControl *rc)
{
    free(rc);
}

void solution(RemoteControl *rc)
{
    if (rc == NULL)
    {
        return;
    }
    get_joint_coordinate(rc);
    get_center_point(rc);
    if (vector_error_flag)
    {
        vector_error_flag = 0;
        rc->error_flag = 1;
    }
}

//  x -> z
// -z -> x
void chan_axis(RemoteControl *rc)
{
    double tmp;
    tmp = rc->a.x;
    rc->a.x = -rc->a.z;
    rc->a.z = tmp;

    tmp = rc->b.x;
    rc->b.x = -rc->b.z;
    rc->b.z = tmp;

    tmp = rc->c.x;
    rc->c.x = -rc->c.z;
    rc->c.z = tmp;

    tmp = rc->center.x;
    rc->center.x = -rc->center.z;
    rc->center.z = tmp;
}
