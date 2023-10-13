#include "main.h"

#include "bsp_can.h"
#include "can.h"
#include "cmsis_os.h"
#include "com.h"
#include "ge.h"
#include "gpio.h"
#include "i2c.h"
#include "math.h"
#include "oled.h"
#include "oledfont.h"
#include "pid.h"
#include "remote_control.h"
#include "stdlib.h"
#include "tim.h"
#include "usart.h"

uint8_t Receive_Var[2];
uint8_t Receive_Bluetooth;
uint8_t Receive_Bluetooth_Buff[8];
uint8_t Receive_Buffer[7];
uint8_t Receive_Flag_1 = FALSE;
uint8_t Receive_Flag_2 = FALSE;
uint8_t Begin_Flag = FALSE;

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

const double k = 200.0;
double angle[3] = {};
const int full_cir = 8192;
const int zero_angle[3] = { ((full_cir >> 1) + 4128) % full_cir,
                            ((full_cir >> 1) + 3302) % full_cir,
                            ((full_cir >> 1) + 3178) % full_cir };
const double G1 = 2.0;
const double G = 8.3;
RemoteControl *rc;

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART6_UART_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    HAL_CAN_Start(&hcan1);
    my_can_filter_init_recv_all(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_UART_Receive_IT(&huart1, &Receive_Bluetooth, 1);
    HAL_UART_Receive_IT(&huart6, &Receive_Var[0], 2);

    OLED_Init();
    OLED_FullyClear();
    OLED_DrawBMP(0, 0, 128, 64, (uint8_t *)BMP_2);
    OLED_RefreshRAM();
    HAL_Delay(1000);

    rc = rc_init(41, 42.5, 90, 120);
    while (1)
    {
        for (int i = 0; i < 3; i++)
        {
            angle[i] = ((double)(zero_angle[i] - moto_chassis[i].angle) / full_cir + (25.64 / 360)) * 2.0 * M_PI;
            /**angle[i] = ((double)(zero_angle[i] - moto_chassis[i].angle) / full_cir) * 2.0 * M_PI;*/
        }
        /**cprintf(*/
        /**    &huart1,*/
        /**    "angle: (%lf, %lf, %lf) (%d,%d,%d)\n",*/
        /**    angle[0],*/
        /**    angle[1],*/
        /**    angle[2],*/
        /**    moto_chassis[0].angle,*/
        /**    moto_chassis[1].angle,*/
        /**    moto_chassis[2].angle);*/
        rc->w1 = angle[0];
        rc->w2 = angle[1];
        rc->w3 = angle[2];
        solution(rc);

        if (rc->error_flag)
        {
            cprintf(&huart1, "Error!\n");
            rc->error_flag = 0;
        }
        else
        {
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

        double g_matrix[3][4] = {};
        /**double *F ;*/
        double F1[3];
        double *F;
        Vec3d moto_rotevec[3] = { { 0, -1, 0 },
                                  { -sin(M_PI / 3), cos(M_PI / 3), 0 },
                                  { cos(M_PI / 6), sin(M_PI / 6), 0 } };
        Vec3d t_vex[3] = { 0 };

        /**cprintf(&huart1, "a : [%f %f %f]\n", rc->a.x,rc->a.y,rc->a.z);*/
        /**cprintf(&huart1, "b : [%f %f %f]\n", rc->b.x,rc->b.y,rc->b.z);*/
        /**cprintf(&huart1, "c : [%f %f %f]\n", rc->c.x,rc->c.y,rc->c.z);*/
        chan_axis(rc);
        /**cprintf(&huart1, "a : [%f %f %f]\n", rc->a.x,rc->a.y,rc->a.z);*/
        /**cprintf(&huart1, "b : [%f %f %f]\n", rc->b.x,rc->b.y,rc->b.z);*/
        /**cprintf(&huart1, "c : [%f %f %f]\n", rc->c.x,rc->c.y,rc->c.z);*/
        cross(&rc->a, &moto_rotevec[0], &t_vex[0]);
        cross(&rc->b, &moto_rotevec[1], &t_vex[1]);
        cross(&rc->c, &moto_rotevec[2], &t_vex[2]);
        mul(1.0 / norm(&t_vex[0]), &t_vex[0], &t_vex[0]);
        mul(1.0 / norm(&t_vex[1]), &t_vex[1], &t_vex[1]);
        mul(1.0 / norm(&t_vex[2]), &t_vex[2], &t_vex[2]);
        /**cprintf(&huart1, "t1 : [%f %f %f]\n", t_vex[0].x,t_vex[0].y,t_vex[0].z);*/
        /**cprintf(&huart1, "t2 : [%f %f %f]\n", t_vex[1].x,t_vex[1].y,t_vex[1].z);*/
        /**cprintf(&huart1, "t3 : [%f %f %f]\n", t_vex[2].x,t_vex[2].y,t_vex[2].z);*/

        // all in one gravity compensation ----------------
        /**    cprintf(*/
        /**        &huart1,*/
        /**        "pos: (%lf, %lf, %lf)\n",*/
        /**        -rc->center.z,*/
        /**        rc->center.y,*/
        /**        rc->center.x);*/
        g_matrix[0][0] = t_vex[0].x;
        g_matrix[0][1] = t_vex[1].x;
        g_matrix[0][2] = t_vex[2].x;
        g_matrix[0][3] = 0;

        g_matrix[1][0] = t_vex[0].y;
        g_matrix[1][1] = t_vex[1].y;
        g_matrix[1][2] = t_vex[2].y;
        g_matrix[1][3] = 0;

        g_matrix[2][0] = t_vex[0].z;
        g_matrix[2][1] = t_vex[1].z;
        g_matrix[2][2] = t_vex[2].z;
				g_matrix[2][3] = G;
        /**for(int i = 0;i < 3;++i){*/
        /**    cprintf(&huart1,"| ");*/
        /**    for(int j = 0; j<4;++j){*/
        /**        cprintf(&huart1,"%f ",g_matrix[i][j]);*/
        /**    }*/
        /**    cprintf(&huart1,"|\n");*/
        /**}*/

        F = gaussian_elimination(g_matrix, 3, 4);
        // ------------------------------------------------------

        F1[0] = G1 * cos(M_PI - angle[0]);
        F1[1] = G1 * cos(M_PI - angle[1]);
        F1[2] = G1 * cos(M_PI - angle[2]);
        set_moto_current(&hcan1, (F1[0] + F[0]) * 1000, (F1[1] + F[1]) * 1000, (F1[2] + F[2]) * 1000, 0);

        free(F);
        HAL_Delay(1);
    }
    MX_FREERTOS_Init();
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
