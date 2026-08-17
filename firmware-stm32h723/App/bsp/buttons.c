// App/bsp/buttons.c
#include "buttons.h"
#include "main.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <stdbool.h>

/* =========================================================
 * QUAN TRONG - LY DO DUNG xTimerStartFromISR() THAY VI osTimerStart():
 * Trong CMSIS-RTOS2 wrapper cua FreeRTOS (cmsis_os2.c), osTimerStart()
 * kiem tra IS_IRQ() va tra ve osErrorISR neu goi tu ISR - KHONG start
 * timer. Day la loi rat de gap va rat kho debug (khong HardFault, chi
 * don gian la nut khong bao gio phan hoi) neu goi nham trong
 * HAL_GPIO_EXTI_Callback(). Vi vay buttons_exti_handler() CHI duoc
 * goi xTimerStartFromISR() (FreeRTOS goc, ISR-safe that su).
 *
 * Nguoc lai, debounce timer callback va longpress timer callback chay
 * trong context cua "Timer Service Task" (task noi bo FreeRTOS) - DAY
 * LA TASK CONTEXT BINH THUONG, khong phai ISR - nen ben trong cac
 * callback nay duoc phep goi osMessageQueuePut()/xTimerStart() binh
 * thuong (khong can ban FromISR).
 * ========================================================= */

/* ButtonEventQueueHandle DA duoc CubeMX dinh nghia + tao trong main.c
 * (MX_FREERTOS_Init -> osMessageQueueNew(...)). File nay CHI dung qua
 * extern osMessageQueueId_t ButtonEventQueueHandle; da khai bao trong
 * buttons.h - KHONG duoc dinh nghia lai o day (gay loi "multiple
 * definition" luc link). */

/* ---- Bang anh xa GPIO_Pin -> button_id_t va port/pin de doc lai muc logic ----
 * Khop dung theo khai bao CubeMX that (main.h):
 *   BTN1 = GPIOF1   (EXTI1_IRQn,     Rising_Falling)
 *   BTN2 = GPIOE2   (EXTI2_IRQn,     Falling)
 *   BTN3 = GPIOE3   (EXTI3_IRQn,     Falling)
 *   BTN4 = GPIOE4   (EXTI4_IRQn,     Falling)
 *   BTN5 = GPIOE5   (EXTI9_5_IRQn,   Falling)
 *   BTN6 = GPIOE6   (EXTI9_5_IRQn,   Falling)
 * Tat ca deu GPIO_PULLUP -> active-low (nhan = GPIO_PIN_RESET), dung voi
 * gia dinh ReadPressed() ben duoi. */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} ButtonPin_t;

static const ButtonPin_t s_buttonPin[BTN_ID_COUNT] =
{
    [BTN_ID_1] = { BTN1_GPIO_Port, BTN1_Pin },
    [BTN_ID_2] = { BTN2_GPIO_Port, BTN2_Pin },
    [BTN_ID_3] = { BTN3_GPIO_Port, BTN3_Pin },
    [BTN_ID_4] = { BTN4_GPIO_Port, BTN4_Pin },
    [BTN_ID_5] = { BTN5_GPIO_Port, BTN5_Pin },
    [BTN_ID_6] = { BTN6_GPIO_Port, BTN6_Pin },
};

/* ---- Timer objects ---- */
static TimerHandle_t s_debounceTimer[BTN_ID_COUNT];   /* 1 one-shot / nut, period = BTN_DEBOUNCE_MS */
static TimerHandle_t s_btn1LongPressTimer;            /* rieng cho BTN_ID_1, period = BTN_LONGPRESS_MS */

/* ---- Trang thai "da xac nhan" (sau debounce) cua tung nut, dung de
 *      chi phan ung khi TRANG THAI THAT su doi (loai bounce) ---- */
static volatile bool s_confirmedPressed[BTN_ID_COUNT];
static volatile bool s_btn1LongPressFired;

static inline bool ReadPressed(button_id_t id)
{
    return HAL_GPIO_ReadPin(s_buttonPin[id].port, s_buttonPin[id].pin) == GPIO_PIN_RESET;
}

static button_id_t PinToButtonId(uint16_t GPIO_Pin)
{
    for (int i = 0; i < BTN_ID_COUNT; i++)
    {
        if (s_buttonPin[i].pin == GPIO_Pin)
        {
            return (button_id_t)i;
        }
    }
    return BTN_ID_COUNT;   /* khong khop -> goi la gia tri "invalid" */
}

/* =========================================================
 * Long-press timer callback (chi cho BTN_ID_1)
 * Chay trong Timer Service Task, KHONG phai ISR.
 * Neu timer nay no ra tuc la nut van dang bi giu lien tuc
 * (khong bi huy o Btn1DebounceCallback do tha ra som hon).
 * ========================================================= */
static void Btn1LongPressCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    s_btn1LongPressFired = true;

    button_event_t evt = { .id = BTN_ID_1, .type = BTN_EVT_LONG_PRESS };
    osMessageQueuePut(ButtonEventQueueHandle, &evt, 0, 0);
}

/* =========================================================
 * Debounce callback dung chung cho 5 nut thuong (LEFT/RIGHT/EXIT/UP/DOWN)
 *
 * QUAN TRONG - KHAC VOI BTN1: 5 nut nay chi cau hinh EXTI Falling
 * (xem bang anh xa GPIO o dau file), tuc la phan cung KHONG bao gio
 * sinh ngat luc tha nut ra. Vi vay KHONG the dung kieu debounce
 * "level-compare 2 chieu" (so sanh voi s_confirmedPressed[id] duoc
 * cap nhat ca luc nhan lan luc tha) nhu Btn1DebounceCallback - neu
 * lam vay s_confirmedPressed[id] se bi "ket" o gia tri true mai sau
 * lan nhan dau tien (khong co ngat nha de dua no ve false), khien
 * MOI LAN NHAN THU 2 tro di bi hieu nham la bounce va bi nuot mat
 * (trieu chung "nhan khong an" ban gap).
 *
 * Fix: coi day la debounce edge-based thuan tuy. Moi lan EXTI falling
 * that (do chinh phan cung sinh ra), buttons_exti_handler() da tu
 * Reset lai timer one-shot nay - neu bounce lien tuc, timer se lien
 * tuc bi Reset va CHI no sau khi tin hieu that su dung yen du
 * BTN_DEBOUNCE_MS (day la co che loc bounce chinh, khong can bang
 * bien state). Khi callback nay chay: neu doc lai pin van dang muc
 * "nhan" (an toan phong truong hop nhieu that qua ngan bi bo qua boi
 * NVIC/loc phan cung) thi bao SHORT_PRESS, khong can/khong duoc gan
 * lai s_confirmedPressed[id] vi bien do khong con y nghia doi voi
 * nhom nut nay nua.
 * ========================================================= */
static void GenericDebounceCallback(TimerHandle_t xTimer)
{
    button_id_t id = (button_id_t)(uintptr_t)pvTimerGetTimerID(xTimer);

    if (ReadPressed(id))
    {
        /* 5 nut nay chi can bao su kien luc NHAN XUONG, khong can biet
         * luc tha ra (dung cho dieu huong man hinh: 1 lan bam = 1 event) */
        button_event_t evt = { .id = id, .type = BTN_EVT_SHORT_PRESS };
        osMessageQueuePut(ButtonEventQueueHandle, &evt, 0, 0);
    }
    /* neu luc nay pin da tha ra roi (VD nhan rat ngan, thoi gian nhan <
     * BTN_DEBOUNCE_MS) thi coi nhu nhieu/qua ngan, bo qua, khong bao gi */
}

/* =========================================================
 * Debounce callback rieng cho BTN_ID_1 (co long-press)
 * ========================================================= */
static void Btn1DebounceCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    bool pressedNow = ReadPressed(BTN_ID_1);

    if (pressedNow == s_confirmedPressed[BTN_ID_1])
    {
        return;   /* bounce, bo qua */
    }
    s_confirmedPressed[BTN_ID_1] = pressedNow;

    if (pressedNow)
    {
        /* xac nhan vua nhan xuong that su -> bat dau dem gio long-press */
        s_btn1LongPressFired = false;
        xTimerStart(s_btn1LongPressTimer, 0);   /* task context binh thuong, khong can FromISR */
    }
    else
    {
        /* xac nhan vua tha ra that su -> huy dem gio long-press neu con chay */
        xTimerStop(s_btn1LongPressTimer, 0);

        if (!s_btn1LongPressFired)
        {
            /* tha ra truoc khi qua nguong giu -> day la short-press */
            button_event_t evt = { .id = BTN_ID_1, .type = BTN_EVT_SHORT_PRESS };
            osMessageQueuePut(ButtonEventQueueHandle, &evt, 0, 0);
        }
        /* neu s_btn1LongPressFired == true thi long-press da duoc ban
         * (bao) ngay luc dat nguong roi, tha ra sau do KHONG ban them
         * short-press nua (tranh 1 lan giu ban ra 2 event) */
    }
}

/* =========================================================
 * Public API
 * ========================================================= */
void buttons_init(void)
{
    /* KHONG goi osMessageQueueNew() o day nua - ButtonEventQueueHandle
     * da duoc CubeMX tao san trong main.c truoc khi cac task bat dau
     * chay (MX_FREERTOS_Init chay truoc osKernelStart). buttons_init()
     * chi can lo phan rieng cua minh: cac debounce timer + longpress
     * timer, khong dung toi viec tao queue. */

    for (int i = 0; i < BTN_ID_COUNT; i++)
    {
        s_confirmedPressed[i] = false;

        TimerCallbackFunction_t cb = (i == BTN_ID_1) ? Btn1DebounceCallback : GenericDebounceCallback;

        s_debounceTimer[i] = xTimerCreate(
                "btnDebounce",
                pdMS_TO_TICKS(BTN_DEBOUNCE_MS),
                pdFALSE,                       /* one-shot */
                (void *)(uintptr_t)i,           /* timer ID = button_id_t, doc lai trong callback */
                cb);
    }

    s_btn1LongPressFired = false;
    s_btn1LongPressTimer = xTimerCreate(
            "btn1LongPress",
            pdMS_TO_TICKS(BTN_LONGPRESS_MS),
            pdFALSE,                           /* one-shot */
            NULL,
            Btn1LongPressCallback);
}

void buttons_exti_handler(uint16_t GPIO_Pin)
{
    button_id_t id = PinToButtonId(GPIO_Pin);
    if (id == BTN_ID_COUNT)
    {
        return;   /* pin khong thuoc 6 nut - khong lam gi */
    }

    /* CHI duoc goi ham FromISR o day - dung xTimerStart()/osTimerStart()
     * se khong hoat dong (hoac tra loi osErrorISR) vi dang o ISR context. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTimerResetFromISR(s_debounceTimer[id], &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
