#include "screen.h"
#include "cmsis_os2.h"
#include <stddef.h>
#include <stdint.h>

/* Độ sâu tối đa từng thực sự cần: Home -> Shutdown -> Stop -> Fault = 3 lớp
 * lồng nhau, để dư 1 cho an toàn. */
#define SCREEN_STACK_DEPTH 4

static const Screen_t *currentScreen = NULL;
static const Screen_t *screenStack[SCREEN_STACK_DEPTH];
static uint8_t stackTop = 0;   /* số phần tử đang có trong stack */

/* =========================================================
 * MUTEX BAO TFT/SCREEN - FIX RACE CONDITION GIUA 2 TASK
 *
 * TRIEU CHUNG DA GAP: chuyen tu gauge screen sang Shutdown bang nut
 * EXIT bi CHONG CHEO chu/hinh cua ca 2 man hinh (anh chup thuc te cho
 * thay ca "ROLL/PITCH/BALL.../2.BALANCE/SHUTDOWN" cua gauge LAN
 * "BACK TO HOME AND SHUTDOWN?/NO" cua Shutdown cung xuat hien).
 *
 * NGUYEN NHAN GOC: Task_Button_UI (khi xu ly BUTTON_EXIT) goi THANG
 * ScreenManager_GotoAndRemember() -> goi onEnter() cua screen moi ->
 * VE TFT NGAY TAI DAY, trong context cua Task_Button_UI. Trong khi do
 * Task_Display chay vong lap rieng 25Hz, goi ScreenManager_Update() ->
 * update() cua screen HIEN TAI (co the van la gauge screen cu, dang
 * ve dang) - CUNG LUC do. 2 task cung ghi vao SPI/TFT khong dong bo
 * -> du lieu 2 man hinh dan xen nhau.
 *
 * FIX: dung 1 mutex duy nhat bao quanh TOAN BO 5 ham public ben duoi
 * (Goto/GotoAndRemember/GoBack/Update/OnButton) - day la "co chai" ma
 * MOI duong ve TFT tu ScreenManager deu phai di qua (onEnter/onExit
 * trong Goto, update() trong Update, va onButton() trong OnButton -
 * ke ca cac handler nhu screen_gauge_common.c goi truc tiep
 * DrawRealtimePart() ben trong onButton). Tai 1 thoi diem chi 1 task
 * duoc "o trong" 1 trong 5 ham nay -> khong con 2 task cung ve TFT.
 *
 * Mutex duoc tao lazy (giong pattern semaphore trong tft_service.c) -
 * an toan trong thuc te vi lan goi dau tien luon la Task_Display goi
 * ScreenManager_Goto(ScreenBoot_Get()) luc khoi dong (StartTaskDisplay),
 * truoc khi Task_Button_UI kip nhan bat ky nut nao.
 *
 * QUAN TRONG - PHAI DUNG osMutexRecursive: ScreenManager_OnButton() da
 * khoa mutex, nhung BEN TRONG onButton() cua tung screen (vd
 * screen_gauge_common.c case BUTTON_EXIT/LEFT/RIGHT) lai GOI TIEP
 * ScreenManager_Goto()/GotoAndRemember()/GoBack() - cac ham nay cung
 * co khoa CUNG mutex do. Voi mutex THUONG (non-recursive), 1 task tu
 * khoa lan 2 se TU KHOA CHET CHINH NO (deadlock vinh vien) - dung
 * trieu chung "bam chuyen screen thi dung hinh TFT va khong con nhan
 * EXTI nut nua" ban gap (Task_Button_UI dung hinh trong luc giu mutex
 * mai mai, khong bao gio xu ly duoc su kien nut tiep theo).
 * osMutexRecursive cho phep CHINH task dang giu mutex duoc khoa them
 * lan nua (dem so lan, phai Unlock du so lan Lock moi thuc su nha),
 * loai bo hoan toan deadlock nay ma van giu duoc tinh loai tru giua
 * 2 TASK KHAC NHAU (Task_Display vs Task_Button_UI) nhu thiet ke ban dau.
 * ========================================================= */
static osMutexId_t s_screenMutex;
static osMutexAttr_t s_screenMutexAttr = { .name = "screenMutex", .attr_bits = osMutexRecursive };

static inline void ScreenManager_EnsureMutexCreated(void)
{
    if (s_screenMutex == NULL)
    {
        s_screenMutex = osMutexNew(&s_screenMutexAttr);
    }
}

void ScreenManager_Lock(void)
{
    ScreenManager_EnsureMutexCreated();
    osMutexAcquire(s_screenMutex, osWaitForever);
}

void ScreenManager_Unlock(void)
{
    osMutexRelease(s_screenMutex);
}

/* =========================================================
 * Logic goc (khong doi ve hanh vi) - chi doi ten thanh "_Locked"
 * vi gio duoc goi TU BEN TRONG khoa mutex (ScreenManager_Goto cong
 * khai o duoi se lock roi moi goi ham nay).
 * ========================================================= */
static void ScreenManager_Goto_Locked(const Screen_t *next)
{
    if (currentScreen != NULL && currentScreen->onExit != NULL)
    {
        currentScreen->onExit();
    }

    currentScreen = next;

    if (currentScreen != NULL && currentScreen->onEnter != NULL)
    {
        currentScreen->onEnter();
    }
}

void ScreenManager_Goto(const Screen_t *next)
{
    ScreenManager_Lock();
    ScreenManager_Goto_Locked(next);
    ScreenManager_Unlock();
}

void ScreenManager_GotoAndRemember(const Screen_t *next)
{
    ScreenManager_Lock();

    if (stackTop < SCREEN_STACK_DEPTH)
    {
        screenStack[stackTop++] = currentScreen;
    }
    /* Nếu đầy stack (không nên xảy ra ở B5): vẫn goto, chỉ mất khả năng
     * quay lại 1 lớp xa nhất - an toàn hơn là kẹt cứng không goto được. */

    ScreenManager_Goto_Locked(next);

    ScreenManager_Unlock();
}

void ScreenManager_GoBack(void)
{
    ScreenManager_Lock();

    if (stackTop > 0)
    {
        const Screen_t *prev = screenStack[--stackTop];
        ScreenManager_Goto_Locked(prev);
    }

    ScreenManager_Unlock();
}

void ScreenManager_Update(void)
{
    ScreenManager_Lock();

    if (currentScreen != NULL && currentScreen->update != NULL)
    {
        currentScreen->update();
    }

    ScreenManager_Unlock();
}

void ScreenManager_OnButton(ButtonState_t evt)
{
    ScreenManager_Lock();

    if (currentScreen != NULL && currentScreen->onButton != NULL)
    {
        currentScreen->onButton(evt);
    }

    ScreenManager_Unlock();
}
