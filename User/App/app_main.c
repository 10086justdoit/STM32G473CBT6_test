#include "app_main.h"
#include "main.h"

#include "bsp_uart.h"
#include "vofa_service.h"
#include "pid_controller.h"

#define APP_PID_TEST_DT_S       0.001f

static PID_Controller_t s_pid_test;
static float s_feedback = 0.0f;

void App_Init(void)
{
    BSP_UART_Init();
    VOFA_Service_Init();

    /*
     * 测试 PID：
     * target = 1.0
     * feedback 模拟被控对象，慢慢跟随 output。
     */
    PID_Controller_Init(&s_pid_test,
                        1.0f,       /* Kp */
                        2.0f,       /* Ki */
                        0.0f,       /* Kd */
                        -3.0f,      /* output min */
                        3.0f,       /* output max */
                        -1.0f,      /* integral min */
                        1.0f);      /* integral max */

    VOFA_Service_SendTextLine("PID test start");
}

void App_Start(void)
{
}

void App_Loop(void)
{
    float target;
    float output;

    target = 1.0f;

    output = PID_Controller_Update(&s_pid_test,
                                   target,
                                   s_feedback,
                                   APP_PID_TEST_DT_S);

    /*
     * 简单模拟一个一阶对象：
     * feedback 慢慢跟随 output。
     */
    s_feedback = s_feedback + 0.01f * (output - s_feedback);

    VOFA_Service_FireWater("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                           target,
                           s_feedback,
                           PID_Controller_GetError(&s_pid_test),
                           PID_Controller_GetOutput(&s_pid_test),
                           PID_Controller_GetPTerm(&s_pid_test),
                           PID_Controller_GetITerm(&s_pid_test));

    HAL_Delay(10);
}
