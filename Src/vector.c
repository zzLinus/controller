//
// Created by GST49 on 2023/9/23.
//

#include "vector.h"
#include "com.h"
#include "usart.h"
#include "math.h"

unsigned char vector_error_flag = 0;

void rc_vector_init(Vec3d *v, const double x, const double y, const double z) {
    v -> x = x;
    v -> y = y;
    v -> z = z;
}

void rc_add(const Vec3d *a, const Vec3d *b, Vec3d *ans) {
    ans -> x = a -> x + b -> x;
    ans -> y = a -> y + b -> y;
    ans -> z = a -> z + b -> z;
}

void rc_sub(const Vec3d *a, const Vec3d *b, Vec3d *ans) {
    ans -> x = a -> x - b -> x;
    ans -> y = a -> y - b -> y;
    ans -> z = a -> z - b -> z;
}

void rc_cross(const Vec3d *a, const Vec3d *b, Vec3d *ans) {
    if(a == ans || b == ans) {
        vector_error_flag = 1;
        return;
    }
    ans -> x = (a -> y * b -> z) - (a -> z * b -> y);
    ans -> y = (a -> z * b -> x) - (a -> x * b -> z);
    ans -> z = (a -> x * b -> y) - (a -> y * b -> x);
}

void rc_mul(const double a, const Vec3d *b, Vec3d *ans) {
    ans -> x = a * b -> x;
    ans -> y = a * b -> y;
    ans -> z = a * b -> z;
}

double rc_norm(const Vec3d *r) {
    double ans = (r -> x * r -> x) + (r -> y * r -> y) + (r -> z * r -> z);
    return sqrt(ans);
}

double rc_dot(const Vec3d *a, const Vec3d *b) {
    return a -> x * b -> x + a -> y * b -> y + a -> z * b -> z;
}

double rc_dis(const Vec3d *a, const Vec3d *b) {
    Vec3d dis_r;
    rc_sub(a, b, &dis_r);
    return rc_norm(&dis_r);
}

void rc_get_plane_normal_vector(const Vec3d *a, const Vec3d *b, Vec3d *ans) {
    rc_cross(a, b, ans);
    double r_norm = rc_norm(ans);
    if(r_norm == 0.0) {
        vector_error_flag = 0;
        return;
    }
    rc_mul(1.0 / r_norm, ans, ans);
}
