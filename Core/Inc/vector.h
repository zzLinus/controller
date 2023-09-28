//
// Created by GST49 on 2023/9/23.
//

#ifndef REMOTECONTROL_VECTOR_H
#define REMOTECONTROL_VECTOR_H

typedef struct Vec3d {
    double x, y, z;
} Vec3d;

extern void vector_init(Vec3d *v, double x, double y, double z);

extern void add(const Vec3d *a, const Vec3d *b, Vec3d *ans);

extern void sub(const Vec3d *a, const Vec3d *b, Vec3d *ans);

extern void cross(const Vec3d *a, const Vec3d *b, Vec3d *ans);

extern void mul(double a, const Vec3d *b, Vec3d *ans);

extern double norm(const Vec3d *r);

extern double dot(const Vec3d *a, const Vec3d *b);

extern double dis(const Vec3d *a, const Vec3d *b);

extern void get_plane_normal_vector(const Vec3d *a, const Vec3d *b, Vec3d *ans);

extern unsigned char vector_error_flag;

#endif //REMOTECONTROL_VECTOR_H
