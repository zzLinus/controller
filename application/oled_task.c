/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled屏幕显示错误码
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "oled_task.h"

#include <math.h>

#include "CAN_receive.h"
#include "OLED.h"
#include "bsp_can.h"
#include "can.h"
#include "cmsis_os.h"
#include "com.h"
#include "controller.h"
#include "detect_task.h"
#include "ge.h"
#include "main.h"
#include "usart.h"
#include "voltage_task.h"

const double k = 200.0;
double angle[3] = {};
const int full_cir = 16384;
int zero_angle[3] = { 0 };

const double G1 = .8;
const double G = 2.60;
RemoteControl *rc;

double format(double w) {
    while(w < 0) {
        w += 2 * M_PI;
    }
    while(w >= 2 * M_PI) {
        w -= 2 * M_PI;
    }
    return w;
}

/**
 * @brief          oled task
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
/**
 * @brief          oled任务
 * @param[in]      pvParameters: NULL
 * @retval         none
 */

const double zero_angle_rad = (360 - 329.0023 + 180) / 180. * M_PI;

void controller_task(void const *argument) {  // !!!!!该任务已关闭!!!!!

    const double rc_d2 = 65 * 2.0 / sqrt(3.0) - 15 * sqrt(3.0) - 10;
    rc = rc_init(41, rc_d2, 90, 120);

    for (int i = 0; i < 500; ++i) {
        for (int j = 0; j < 3; ++j) {
            zero_angle[j] = (motor[j].encoder) % full_cir;
        }
        HAL_Delay(1);
    }

    float x = 3.14f * 0.5f;
    Matrix g_matrix;
    matrix_init(&g_matrix, 3, 4);
    while (1) {
        for (int i = 0; i < 3; i++) {
            angle[i] =
                ((double)(zero_angle[i] - motor[i].encoder) / full_cir) * 2.0 * M_PI;
            angle[i] = format(zero_angle_rad - angle[i]);
            /**cprintf(&huart1,"motoangle : %d",moto_chassis[i].angle);*/
            /**angle[i] = ((double)(zero_angle[i] - moto_chassis[i].angle) / full_cir) * 2.0 * M_PI;*/
        }
        /**cprintf(&huart1,"\n");*/
        /**cprintf(*/
        /**    &huart1,*/
        /**    "angle: (%lf, %lf, %lf) (%d,%d,%d)\n",*/
        /**    angle[0],*/
        /**    angle[1],*/
        /**    angle[2],*/
        /**    moto_chassis[0].angle,*/
        /**    moto_chassis[1].angle,*/
        /**    moto_chassis[2].angle);*/
        /**double a = (2 * M_PI + angle[0]+angle[1]+angle[2])/3;*/
        /**rc->w1 = a;*/
        /**rc->w2 = a;*/
        /**rc->w3 = a;*/
        rc->w1 = angle[0];
        rc->w2 = angle[1];
        rc->w3 = angle[2];
        solution(rc);

        if (rc->error_flag) {
            cprintf(&huart1, "Error!\n");
            rc->error_flag = 0;
        }
        else {
            /**cprintf(*/
            /**    &huart1,*/
            /**    "pos: (%lf, %lf, %lf) angle (%lf %lf %lf)\n",*/
            /**    rc->center.x,*/
            /**    rc->center.y,*/
            /**    rc->center.z,*/
            /**    angle[0],*/
            /**    angle[1],*/
            /**    angle[2]);*/
        }
        /**double *F ;*/
        double F1[3];
        /**Vec3d moto_rotevec[3] = { { 0, -1, 0 },*/
        /**                          { -sin(M_PI / 3), cos(M_PI / 3), 0 },*/
        /**                          { cos(M_PI / 6), sin(M_PI / 6), 0 } };*/

        Vec3d moto_rotevec[3] = { { 0, 1, 0 },
                                  { 0, -sin(M_PI / 6), -cos(M_PI / 6) },
                                  { 0, -cos(M_PI / 3), sin(M_PI / 3) } };
        Vec3d t_vec[3] = { 0 };
        Vec3d secarm_vec[3] = { 0 };

        /**cprintf(&huart1, "a : [%f %f %f]\n", rc->a.x,rc->a.y,rc->a.z);*/
        /**cprintf(&huart1, "b : [%f %f %f]\n", rc->b.x,rc->b.y,rc->b.z);*/
        /**cprintf(&huart1, "c : [%f %f %f]\n", rc->c.x,rc->c.y,rc->c.z);*/
        /**chan_axis(rc);*/
        /**cprintf(&huart1, "a : [%f %f %f]\n", rc->a.x,rc->a.y,rc->a.z);*/
        /**cprintf(&huart1, "b : [%f %f %f]\n", rc->b.x,rc->b.y,rc->b.z);*/
        /**cprintf(&huart1, "c : [%f %f %f]\n", rc->c.x,rc->c.y,rc->c.z);*/
        rc_cross(&rc->a, &moto_rotevec[0], &t_vec[0]);
        rc_cross(&rc->b, &moto_rotevec[1], &t_vec[1]);
        rc_cross(&rc->c, &moto_rotevec[2], &t_vec[2]);
        rc_mul(1.0 / rc_norm(&t_vec[0]), &t_vec[0], &t_vec[0]);
        rc_mul(1.0 / rc_norm(&t_vec[1]), &t_vec[1], &t_vec[1]);
        rc_mul(1.0 / rc_norm(&t_vec[2]), &t_vec[2], &t_vec[2]);
        /**cprintf(&huart1, "t1 : [%f %f %f]\n", t_vex[0].x,t_vex[0].y,t_vex[0].z);*/
        /**cprintf(&huart1, "t2 : [%f %f %f]\n", t_vex[1].x,t_vex[1].y,t_vex[1].z);*/
        /**cprintf(&huart1, "t3 : [%f %f %f]\n", t_vex[2].x,t_vex[2].y,t_vex[2].z);*/

        rc_sub(&rc->center, &rc->a, &secarm_vec[0]);
        rc_sub(&rc->center, &rc->b, &secarm_vec[1]);
        rc_sub(&rc->center, &rc->c, &secarm_vec[2]);
        rc_mul(1.0 / rc_norm(&secarm_vec[0]), &secarm_vec[0], &secarm_vec[0]);
        rc_mul(1.0 / rc_norm(&secarm_vec[1]), &secarm_vec[1], &secarm_vec[1]);
        rc_mul(1.0 / rc_norm(&secarm_vec[2]), &secarm_vec[2], &secarm_vec[2]);

        // all in one gravity compensation ----------------
        /**cprintf(*/
        /**    &huart1,*/
        /**    "pos: (%lf, %lf, %lf)\n",*/
        /**    -rc->center.z,*/
        /**    rc->center.y,*/
        /**    rc->center.x);*/

        g_matrix.val[0][0] = secarm_vec[0].x;
        g_matrix.val[0][1] = secarm_vec[1].x;
        g_matrix.val[0][2] = secarm_vec[2].x;
        g_matrix.val[0][3] = 0;

        g_matrix.val[1][0] = secarm_vec[0].y;
        g_matrix.val[1][1] = secarm_vec[1].y;
        g_matrix.val[1][2] = secarm_vec[2].y;
        g_matrix.val[1][3] = 0;

        g_matrix.val[2][0] = secarm_vec[0].z;
        g_matrix.val[2][1] = secarm_vec[1].z;
        g_matrix.val[2][2] = secarm_vec[2].z;
        g_matrix.val[2][3] = G;
        /**printf("--------------\n");*/
        /**for(int i = 0;i < 3;++i){*/
        /**    cprintf(&huart1,"| ");*/
        /**    for(int j = 0; j<4;++j){*/
        /**        cprintf(&huart1,"%f ",g_matrix[i][j]);*/
        /**    }*/
        /**    cprintf(&huart1,"|\n");*/
        /**}*/

//        if(!GE(&g_matrix)) {
//            while(1) {
//                cprintf(&huart1, "warning!!!!\n");
//            }
//        }
        // ------------------------------------------------------
        GE(&g_matrix);
        Vec3d f = { g_matrix.val[0][3], g_matrix.val[1][3], g_matrix.val[2][3] };

//        cprintf(&huart1, "%lf,%lf,%lf\n", f.x, f.y, f.z);
        cprintf(&huart1, "%lf,%lf,%lf\n",
                rc->center.x, rc->center.y, rc->center.z);
//        cprintf(&huart1, "%lf,%lf,%lf\n", rc->b.x, rc->b.y, rc->b.z);
//        cprintf(&huart1, "%lf,%lf,%lf\n", angle[0], angle[1], angle[2]);
//        cprintf(&huart1, "%d,%d,%d\n", (int)motor[0].encoder, (int)motor[1].encoder, (int)motor[2].encoder);
        f.x *= rc_dot(&secarm_vec[0], &t_vec[0]);
        f.y *= rc_dot(&secarm_vec[1], &t_vec[1]);
        f.z *= rc_dot(&secarm_vec[2], &t_vec[2]);

        Vec3d g = { 0, 0, G1 };
        F1[0] = rc_dot(&g, &t_vec[0]);
        F1[1] = rc_dot(&g, &t_vec[1]);
        F1[2] = rc_dot(&g, &t_vec[2]);

        double k1 = 0;
        double kp2 = 2;

        /**cprintf(&huart1,"solution : [%f %f %f]\n",f.x,f.y,f.z);*/
        motor[0].traget_torque = (F1[0] * 2) + (f.x * kp2);
        motor[1].traget_torque = (F1[1] * 2) + (f.y * kp2);
        motor[2].traget_torque = (F1[2] * 2) + (f.z * kp2);
        for (int i = 0; i < 3; i++) {
            if (motor[i].axis_current_stage != 8)
                cprintf(&huart1, "error motor id : %d\n", i);
        }
        x += 0.01f;
        HAL_Delay(1);
    }
}
