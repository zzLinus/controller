//
// Created by GST49 on 2023/9/23.
//

#ifndef REMOTECONTROL_VECTOR_H
#define REMOTECONTROL_VECTOR_H

typedef struct Vec3d {
    double x, y, z;
} Vec3d;

extern void rc_vector_init(Vec3d *v, double x, double y, double z);

extern void rc_add(const Vec3d *a, const Vec3d *b, Vec3d *ans);

extern void rc_sub(const Vec3d *a, const Vec3d *b, Vec3d *ans);

extern void rc_cross(const Vec3d *a, const Vec3d *b, Vec3d *ans);

extern void rc_mul(double a, const Vec3d *b, Vec3d *ans);

extern double rc_norm(const Vec3d *r);

extern double rc_dot(const Vec3d *a, const Vec3d *b);

extern double rc_dis(const Vec3d *a, const Vec3d *b);

extern void rc_get_plane_normal_vector(const Vec3d *a, const Vec3d *b, Vec3d *ans);

extern unsigned char vector_error_flag;

#endif //REMOTECONTROL_VECTOR_H
