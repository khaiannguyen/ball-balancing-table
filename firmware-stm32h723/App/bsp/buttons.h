#ifndef BSP_BUTTONS_H
#define BSP_BUTTONS_H

#include <stdint.h>
#include "cmsis_os2.h"

/* =========================================================
 * 6 nut vat ly -> button_id_t, khop truc tiep ten phan cung
 * that (main.h): BTN1 = GPIOF1 (EXTI Rising_Falling),
 * BTN2..BTN6 = GPIOE2..GPIOE6 (EXTI Falling).
 *
 * BTN_ID_1 la nut dac biet: EXTI ca Rising+Falling (de tinh
 * thoi gian giu), 5 nut con lai (BTN_ID_2..6) chi can EXTI Falling.
 *
 * Chuc nang cu the (LEFT/RIGHT/EXIT/UP/DOWN...) cho BTN2..BTN6 se
 * gan o buoc ghep ScreenManager_OnButton() sau, CHUA quyet dinh o
 * lop buttons.c/.h nay - lop nay chi lo dung: EXTI -> debounce -> event.
 * ========================================================= */
typedef enum
{
    BTN_ID_1 = 0,   /* GPIOF1 - nhan giu >= BTN_LONGPRESS_MS -> RUN/STOP toggle
                       nhan tha ra som hon -> short-press (vd ENTER) */
    BTN_ID_2,       /* GPIOE2 */
    BTN_ID_3,       /* GPIOE3 */
    BTN_ID_4,       /* GPIOE4 */
    BTN_ID_5,       /* GPIOE5 */
    BTN_ID_6,       /* GPIOE6 */
    BTN_ID_COUNT
} button_id_t;

typedef enum
{
    BTN_EVT_SHORT_PRESS = 0,  /* nut thuong: fire ngay luc nhan xuong (da debounce)
                                 BTN_ID_1: fire luc THA RA neu chua qua nguong giu */
    BTN_EVT_LONG_PRESS        /* CHI phat sinh cho BTN_ID_1, fire ngay khi dang giu
                                 vua qua nguong (khong can doi tha ra) */
} button_event_type_t;

typedef struct
{
    button_id_t         id;
    button_event_type_t type;
} button_event_t;

/* Nguong thoi gian giu de tinh la long-press (ms) - chi BTN_ID_1 dung */
#define BTN_LONGPRESS_MS   600u
/* Thoi gian debounce (ms) - theo khuyen nghi thiet ke muc 2 (20-30ms) */
#define BTN_DEBOUNCE_MS    25u

/* Do sau queue - CHI de tham khao/doi chieu, khong con dung de truyen
 * vao osMessageQueueNew() nua vi queue nay da duoc CubeMX tao san trong
 * main.c (16 phan tu, khop voi gia tri nay) - xem chu thich o buttons.c */
#define BTN_EVENT_QUEUE_LEN  16u

/* Queue chua button_event_t. Task_Button_UI la consumer DUY NHAT.
 * Bien nay DA duoc CubeMX dinh nghia + tao (osMessageQueueNew) trong
 * main.c (MX_FREERTOS_Init) - buttons.c CHI duoc dung qua extern nay,
 * KHONG duoc dinh nghia hay tao lai (se gay loi link "multiple
 * definition"). Duoc Put tu debounce timer callback (chay trong Timer
 * Service Task, KHONG phai ISR) nen dung CMSIS osMessageQueuePut binh
 * thuong, khong can ban FromISR. */
extern osMessageQueueId_t ButtonEventQueueHandle;

/* Goi 1 lan sau khi kernel objects san sang (vd dau StartDefaultTask,
 * SAU khi ButtonEventQueueHandle da duoc tao boi main.c/MX_FREERTOS_Init).
 * Tao 6 debounce timer + 1 longpress timer rieng cho BTN_ID_1 - KHONG
 * tao queue (queue da co san, xem chu thich tren). */
void buttons_init(void);

/* Goi tu HAL_GPIO_EXTI_Callback() trong exti_dispatch.c.
 * CHAY TRONG ISR CONTEXT - ben trong CHI duoc goi xTimerStartFromISR(),
 * KHONG duoc goi osMessageQueuePut()/osTimerStart() truc tiep o day. */
void buttons_exti_handler(uint16_t GPIO_Pin);

#endif /* BSP_BUTTONS_H */
