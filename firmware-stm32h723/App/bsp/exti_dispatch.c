#include "main.h"
#include "imu_mpu6500.h"
#include "buttons.h"

/*
 * Central GPIO EXTI dispatch point.
 *
 * The HAL callback runs in interrupt context. Its responsibility is
 * limited to routing each interrupt source to the corresponding BSP
 * handler.
 *
 * Time-consuming processing and application-level event handling are
 * intentionally deferred to the appropriate subsystem.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case IMU_INT_Pin:
            /*
             * Start the IMU DMA read as soon as the data-ready
             * interrupt is received.
             */
            imu_read_dma_start();
            break;

        /*
         * All button EXTI sources share the same button handler.
         *
         * buttons_exti_handler() maps the GPIO pin to its button ID
         * and resets the corresponding debounce timer.
         */
        case BTN1_Pin:
        case BTN2_Pin:
        case BTN3_Pin:
        case BTN4_Pin:
        case BTN5_Pin:
        case BTN6_Pin:
            buttons_exti_handler(GPIO_Pin);
            break;

        default:
            /*
             * Ignore EXTI sources that are not handled by this
             * application-level dispatcher.
             */
            break;
    }
}
