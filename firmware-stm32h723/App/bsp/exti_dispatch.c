#include "main.h"
#include "imu_mpu6500.h"
#include "buttons.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case IMU_INT_Pin:
            imu_read_dma_start();
            break;

        /* 6 nut bam - buttons_exti_handler() tu nhan dien pin nao trong so
         * 6 nut qua bang s_buttonPin[] trong buttons.c, nen o day chi can
         * liet ke du 6 case va goi chung 1 ham. Neu pin khong khop nut nao
         * buttons_exti_handler() se tu bo qua (khong lam gi). */
        case BTN1_Pin:    /* GPIOF1 */
        case BTN2_Pin:    /* GPIOE2 */
        case BTN3_Pin:    /* GPIOE3 */
        case BTN4_Pin:    /* GPIOE4 */
        case BTN5_Pin:    /* GPIOE5 */
        case BTN6_Pin:    /* GPIOE6 */
            buttons_exti_handler(GPIO_Pin);
            break;

        default:
            break;
    }
}
