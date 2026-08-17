#include "screen_gauge_common.h"
#include "screen_shutdown.h"
#include "tft_service.h"
#include "ui_data.h"
#include "system_state.h"
#include "control_mode_manual.h"   /* THÊM Giai đoạn 3 - điều khiển Mode Manual qua UI */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* =========================================================
 * LAYOUT - 220x176 (ngang), không đổi so với bản gốc, chỉ
 * thêm 1 field hiển thị trạng thái SEL/NAV.
 * ========================================================= */

#define COLOR_BG      0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF

#define TXT_X         2
#define TXT_Y0        14
#define TXT_LINE_H    10

#define CIRCLE_CX     145
#define CIRCLE_CY     75
#define CIRCLE_R      72

#define BALL_PHYS_MAX 178.0f
#define BALL_STEP     10.0f

#define BALL_RADIUS 5

#define GUIDE_Y0      136
#define GUIDE_Y1      148

#define DOT_Y         168
#define DOT_X0        10
#define DOT_SPACING   16
#define DOT_R         5

#define BTN_MODE_X0   62
#define BTN_MODE_Y0   158
#define BTN_MODE_X1   150
#define BTN_MODE_Y1   175

#define BTN_SHUTDOWN_X0 156
#define BTN_SHUTDOWN_Y0 158
#define BTN_SHUTDOWN_X1 218
#define BTN_SHUTDOWN_Y1 175

/* THÊM Giai đoạn 3 - bước chỉnh us mỗi lần bấm LEFT/RIGHT ở Mode Manual
 * (MANUAL_SUB_MANUAL_STEP), khớp SERVO_TEST_MANUAL_STEP_US trong
 * B6_Control.md mục 4. */
#define MANUAL_STEP_US   10

/* ---- trạng thái điều hướng: SELECTED (đã chọn, robot đang dùng mode
 * này) <-> BROWSE (đang duyệt để đổi mode, LEFT/RIGHT mới có tác dụng
 * chuyển screen). Dùng chung 1 biến static cho cả 5 gauge screen vì
 * tại 1 thời điểm chỉ 1 trong 5 đang active. ---- */
typedef enum { NAV_SELECTED = 0, NAV_BROWSE } NavState_t;
static NavState_t navState = NAV_SELECTED;

static const char *bottomLabelStore[MODE_COUNT];
static uint8_t activeMode = 0;

static bool    ballWasDrawn = false;
static int16_t lastBallPx = 0, lastBallPy = 0;

/* =========================================================
 * STOP OVERLAY (THÊM) - thay cho việc chuyển sang ScreenStop
 * riêng: khi BTN1-long dừng khẩn cấp, vẽ ĐÈ hình tròn đỏ đặc +
 * chữ "STOP" trắng lên đúng vòng tròn (CIRCLE_CX/CY/R) đang có
 * trên gauge screen hiện tại, KHÔNG đổi currentScreen trong
 * ScreenManager (không GotoAndRemember/GoBack nữa). Lý do: người
 * dùng muốn vẫn thấy toàn bộ số liệu (Roll/Pitch/S1-3...) trong
 * lúc dừng khẩn cấp, chỉ có vùng vòng tròn đổi sang báo STOP.
 * ========================================================= */
#define STOP_CIRCLE_R  (CIRCLE_R - 25)   /* nhỏ hơn viền tròn 1 chút, không đè ra ngoài */
static bool stopOverlayActive = false;

/* =========================================================
 * GIAI DOAN 1 - DIRTY UPDATE
 *
 * Cache lai gia tri da ve len man hinh lan truoc. Moi lan
 * DrawRealtimePart() duoc goi, chi field nao THAY DOI so voi
 * cache moi bi FillRectangle+DrawText lai - giam manh so lan
 * ghi SPI (nguyen nhan chinh gay nhay khi ve dong).
 *
 * cacheValid = false bat buoc ve LAI TOAN BO field mot lan (vd
 * ngay sau khi vao screen / DrawStaticPart() chay) de dam bao
 * khong bi "thieu" gia tri do so sanh voi cache rac.
 * ========================================================= */
typedef struct
{
    bool valid;

    uint8_t  mode;
    uint8_t  navBrowse;     /* 0=SEL, 1=NAV */

    uint8_t  robotRunning;
    int16_t  imuRoll;
    int16_t  imuPitch;
    uint8_t  ballOn;

    int16_t  ballXShown;    /* gia tri da hien (desired hoac do thuc, tuy mode) */
    int16_t  ballYShown;
    uint8_t  showDesired;   /* de phat hien doi nguon x/y (desired <-> do thuc) */

    int16_t  ballVx;
    int16_t  ballVy;

    uint8_t  cameraOk;

    int16_t  servoUs[3];

    char     guideText[64];
} UiCache_t;

static UiCache_t lastUi;

static inline void InvalidateUiCache(void)
{
    lastUi.valid = false;
}

/* toạ độ ball desired đang chỉnh ở mode Position - chỉ có ý nghĩa khi
 * activeMode == MODE_POSITION && navState == NAV_SELECTED */
static float ballXDesired = 0.0f;
static float ballYDesired = 0.0f;

#define VAL_FIELD_H        8

#define W_STATE   28
#define W_ANGLE   24
#define W_ONOFF   18
#define W_BALLXY  30
#define W_OKERR   18
#define W_MODE    14
#define W_NAV     30

static void DrawValueField(int16_t x, int16_t y, uint16_t fieldWidth,
        const char *text, uint16_t fg, uint16_t bg)
{
    TFT_FillRectangle(x, y, x + fieldWidth - 1, y + VAL_FIELD_H - 1, bg);
    TFT_DrawTextFast(x, y, text, fg, bg, 1);
}

static void MapBallToPixel(float x, float y, int16_t *px, int16_t *py)
{
    float fx = x / BALL_PHYS_MAX;
    float fy = y / BALL_PHYS_MAX;

    if (fx > 1.0f)  fx = 1.0f;
    if (fx < -1.0f) fx = -1.0f;
    if (fy > 1.0f)  fy = 1.0f;
    if (fy < -1.0f) fy = -1.0f;

    *px = CIRCLE_CX + (int16_t)(fx * CIRCLE_R);
    *py = CIRCLE_CY - (int16_t)(fy * CIRCLE_R);
}

/* ---- Toa do 6 dinh luc giac, tinh 1 LAN duy nhat (dung chung giua
 * DrawStaticPart() va RepairStaticNear() ben duoi - khong tinh lai
 * trig moi frame). 1 dinh nam tren truc Y (dinh tren cung, goc -90 do). */
static int16_t s_hexX[6], s_hexY[6];
static bool    s_hexReady = false;

static void ComputeHexVertices(void)
{
    if (s_hexReady) return;
    static const float HEX_ANGLE0 = -1.5707963f; /* -90 deg, tren truc Y */
    int i;
    for (i = 0; i < 6; i++)
    {
        float ang = HEX_ANGLE0 + i * (3.14159265f / 3.0f); /* +60 deg moi buoc */
        /* Dung lroundf() thay vi "+0.5f roi ep kieu int" - cach cu lam tron
         * sai voi so am (cac dinh ben trai co cosf(ang) < 0), khien dinh
         * trai bi keo gan tam hon 1 pixel so voi dinh phai => luc giac
         * bi lech, canh phai trong xa tam hon canh trai. */
        s_hexX[i] = CIRCLE_CX + (int16_t)lroundf(55.0f * cosf(ang));
        s_hexY[i] = CIRCLE_CY + (int16_t)lroundf(55.0f * sinf(ang));
    }
    s_hexReady = true;
}

/* Khoang cach tu diem (px,py) toi doan thang (ax,ay)-(bx,by) - dung de
 * biet 1 canh luc giac co bi vung xoa bong de len hay khong, tranh phai
 * ve lai CA 6 canh khi chi 1 canh (hoac khong canh nao) bi anh huong. */
static float PointSegDist(float px, float py, float ax, float ay, float bx, float by)
{
    float dx = bx - ax, dy = by - ay;
    float len2 = dx*dx + dy*dy;
    float t = 0.0f;
    if (len2 > 0.0001f)
    {
        t = ((px-ax)*dx + (py-ay)*dy) / len2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    float cx = ax + t*dx, cy = ay + t*dy;
    float ex = px-cx, ey = py-cy;
    return sqrtf(ex*ex + ey*ey);
}

/* ---- VA LAI phan tinh (crosshair/vong tron/luc giac) CHI TAI VUNG
 * bi TFT_FillCircleFast() xoa de len khi bong roi khoi 1 vi tri (bx,by)
 * ban kinh BALL_RADIUS - KHONG ve lai toan bo moi frame nua. Chi goi
 * ham nay dung 1 lan ngay sau khi xoa bong cu (xem GIAI DOAN 1 ben duoi),
 * va chi ve lai DUNG canh/duong thuc su bi vung xoa do de len. */
static void RepairStaticNear(int16_t bx, int16_t by)
{
    const float R = (float)(BALL_RADIUS + 1); /* +1 chong sai so lam tron */

    /* QUAN TRONG: thu tu ve lai o day phai KHOP voi thu tu ve trong
     * DrawStaticPart() - vong tron do (duoi cung) -> crosshair trang
     * (de len vong tron) -> luc giac do (tren cung). Neu dao nguoc thu
     * tu (vd ve crosshair truoc roi vong tron sau), tai 4 diem giao
     * giua crosshair va vong tron (dinh tren/duoi/trai/phai cua vong
     * tron) mau sac se bi LAT NGUOC: dang le phai la trang thi lai bi
     * ve de thanh do, de lai vet loi vinh vien tai dung 4 diem do. */

    /* vong tron vien: chi ve lai (ca duong tron) neu vung xoa dung sat
     * bien - hiem khi xay ra vi bong thuong o gan giua, khong sat bien. */
    float distToCenter = sqrtf((float)(bx-CIRCLE_CX)*(bx-CIRCLE_CX) + (float)(by-CIRCLE_CY)*(by-CIRCLE_CY));
    if (fabsf(distToCenter - (float)CIRCLE_R) <= R)
    {
        TFT_DrawCircle(CIRCLE_CX, CIRCLE_CY, CIRCLE_R, TFT_COLOR_RED);
    }
    /* crosshair ngang: y = CIRCLE_CY */
    if (fabsf((float)by - (float)CIRCLE_CY) <= R)
    {
        TFT_DrawLine(bx-BALL_RADIUS-1, CIRCLE_CY, bx+BALL_RADIUS+1, CIRCLE_CY, COLOR_WHITE);
    }
    /* crosshair doc: x = CIRCLE_CX */
    if (fabsf((float)bx - (float)CIRCLE_CX) <= R)
    {
        TFT_DrawLine(CIRCLE_CX, by-BALL_RADIUS-1, CIRCLE_CX, by+BALL_RADIUS+1, COLOR_WHITE);
    }
    /* luc giac: chi ve lai DUNG canh nao bi de len, khong ve ca 6 canh */
    ComputeHexVertices();
    {
        int i;
        for (i = 0; i < 6; i++)
        {
            int j = (i + 1) % 6;
            if (PointSegDist((float)bx, (float)by, (float)s_hexX[i], (float)s_hexY[i],
                              (float)s_hexX[j], (float)s_hexY[j]) <= R)
            {
                TFT_DrawLine(s_hexX[i], s_hexY[i], s_hexX[j], s_hexY[j], TFT_COLOR_RED);
            }
        }
    }
}

static void DrawStaticPart(void)
{
    TFT_FillScreen(COLOR_BG);

    TFT_DrawTextFast(2, 2, "PINGPONG-TABLE", COLOR_WHITE, COLOR_BG, 1);

    TFT_DrawCircle(CIRCLE_CX, CIRCLE_CY, CIRCLE_R, TFT_COLOR_RED);

    TFT_DrawLine(CIRCLE_CX-CIRCLE_R-4, CIRCLE_CY, CIRCLE_CX+CIRCLE_R+4, CIRCLE_CY, COLOR_WHITE);
    TFT_DrawLine(CIRCLE_CX, CIRCLE_CY-CIRCLE_R-4, CIRCLE_CX, CIRCLE_CY+CIRCLE_R+4, COLOR_WHITE);

    /* ---- Luc giac deu mau do, 1 dinh nam tren truc Y (dinh tren cung) ---- */
    ComputeHexVertices();
    {
        int i;
        for (i = 0; i < 6; i++)
        {
            int j = (i + 1) % 6;
            TFT_DrawLine(s_hexX[i], s_hexY[i], s_hexX[j], s_hexY[j], TFT_COLOR_RED);
        }
    }

    uint16_t y = TXT_Y0;
    TFT_DrawTextFast(TXT_X,y,"State:",COLOR_WHITE,COLOR_BG,1); y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"Roll:",COLOR_WHITE,COLOR_BG,1);  y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"Pitch:",COLOR_WHITE,COLOR_BG,1); y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"Ball:",COLOR_WHITE,COLOR_BG,1);  y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"x:",COLOR_WHITE,COLOR_BG,1);     y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"y:",COLOR_WHITE,COLOR_BG,1);     y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"vx:",COLOR_WHITE,COLOR_BG,1);    y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"vy:",COLOR_WHITE,COLOR_BG,1);    y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"Cam:",COLOR_WHITE,COLOR_BG,1);   y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"S1:",COLOR_WHITE,COLOR_BG,1);    y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"S2:",COLOR_WHITE,COLOR_BG,1);    y+=TXT_LINE_H;
    TFT_DrawTextFast(TXT_X,y,"S3:",COLOR_WHITE,COLOR_BG,1);

    TFT_DrawTextFast(TXT_X, GUIDE_Y0, "instructions:", COLOR_WHITE, COLOR_BG, 1);

    TFT_FillRectangle(BTN_MODE_X0, BTN_MODE_Y0, BTN_MODE_X1, BTN_MODE_Y1, TFT_COLOR_BLUE);

    if (bottomLabelStore[activeMode] != NULL)
    {
        uint16_t tw, th;
        TFT_GetTextExtent(bottomLabelStore[activeMode], 1, &tw, &th);
        TFT_DrawTextFast(
                BTN_MODE_X0+((BTN_MODE_X1-BTN_MODE_X0)-tw)/2,
                BTN_MODE_Y0+6,
                bottomLabelStore[activeMode],
                COLOR_WHITE, TFT_COLOR_BLUE, 1);
    }

    TFT_FillRectangle(BTN_SHUTDOWN_X0, BTN_SHUTDOWN_Y0, BTN_SHUTDOWN_X1, BTN_SHUTDOWN_Y1, TFT_COLOR_GREEN);
    TFT_DrawTextFast(165, 163, "Shutdown", COLOR_WHITE, TFT_COLOR_GREEN, 1);

    TFT_FillCircle(DOT_X0, DOT_Y, DOT_R, COLOR_WHITE);
    TFT_FillCircle(DOT_X0+DOT_SPACING, DOT_Y, DOT_R, TFT_COLOR_RED);
    TFT_FillCircle(DOT_X0+2*DOT_SPACING, DOT_Y, DOT_R, COLOR_WHITE);

    ballWasDrawn = false;

    /* man hinh tinh vua duoc ve lai toan bo -> cache cu khong con
     * dung nua, bat DrawRealtimePart() ve lai TOAN BO field 1 lan */
    InvalidateUiCache();
}

static void DrawRealtimePart(void)
{
    char buf[24];
    bool force = !lastUi.valid;

    /* ---- MODE ---- */
    if (force)
    {
        TFT_DrawTextFast(180, 2, "MODE", COLOR_YELLOW, COLOR_BG, 1);
    }

    if (force || (lastUi.mode != g_uiData.mode))
    {
        snprintf(buf,sizeof(buf),"%d",g_uiData.mode);
        DrawValueField(210, 2, W_MODE, buf, COLOR_YELLOW, COLOR_BG);
        lastUi.mode = g_uiData.mode;
    }

    /* ---- chỉ báo trạng thái điều hướng: NAV / SEL ---- */
    uint8_t navBrowse = (navState == NAV_BROWSE) ? 1 : 0;
    if (force || (lastUi.navBrowse != navBrowse))
    {
        DrawValueField(
                196, 12, W_NAV,
                navBrowse ? "NAV" : "SEL",
                navBrowse ? TFT_COLOR_CYAN : COLOR_YELLOW,
                COLOR_BG);
        lastUi.navBrowse = navBrowse;
    }

    uint16_t y = TXT_Y0;

    /* ---- State (RUN/STOP) ---- */
    if (force || (lastUi.robotRunning != (uint8_t)g_uiData.robotRunning))
    {
        DrawValueField(40,y,W_STATE, g_uiData.robotRunning?"RUN":"STOP", COLOR_YELLOW, COLOR_BG);
        lastUi.robotRunning = (uint8_t)g_uiData.robotRunning;
    }
    y+=TXT_LINE_H;

    /* ---- Roll ---- */
    {
        int16_t rollI = (int16_t)g_uiData.imuRoll;   /* so sanh theo gia tri se HIEN (da lam tron) */
        if (force || (lastUi.imuRoll != rollI))
        {
            snprintf(buf,sizeof(buf),"%.0f",g_uiData.imuRoll);
            DrawValueField(40,y,W_ANGLE,buf,COLOR_YELLOW,COLOR_BG);
            lastUi.imuRoll = rollI;
        }
    }
    y+=TXT_LINE_H;

    /* ---- Pitch ---- */
    {
        int16_t pitchI = (int16_t)g_uiData.imuPitch;
        if (force || (lastUi.imuPitch != pitchI))
        {
            snprintf(buf,sizeof(buf),"%.0f",g_uiData.imuPitch);
            DrawValueField(40,y,W_ANGLE,buf,COLOR_YELLOW,COLOR_BG);
            lastUi.imuPitch = pitchI;
        }
    }
    y+=TXT_LINE_H;

    /* ---- Ball ON/OFF ---- */
    if (force || (lastUi.ballOn != (uint8_t)g_uiData.ballOn))
    {
        DrawValueField(40,y,W_ONOFF, g_uiData.ballOn?"ON":"OFF", COLOR_YELLOW, COLOR_BG);
        lastUi.ballOn = (uint8_t)g_uiData.ballOn;
    }
    y+=TXT_LINE_H;

    /* x/y: ở mode Position + đang SELECTED, hiện setpoint đang chỉnh
     * (ballXDesired/ballYDesired) thay vì vị trí bóng đo được, để
     * người dùng thấy ngay giá trị mình vừa bấm - các mode khác vẫn
     * hiện toạ độ bóng thật như cũ. */
    bool showDesired = (activeMode == MODE_POSITION) && (navState == NAV_SELECTED);
    uint8_t showDesiredFlag = showDesired ? 1 : 0;
    /* nguon du lieu doi (desired <-> do thuc) -> phai coi la "thay doi"
     * du gia tri so lam tron co the trung, de tranh hien nham gia tri cu. */
    bool sourceChanged = (lastUi.showDesired != showDesiredFlag);

    /* ---- x ---- */
    {
        float xF = showDesired ? ballXDesired : g_uiData.ballX;
        int16_t xI = (int16_t)xF;
        if (force || sourceChanged || (lastUi.ballXShown != xI))
        {
            snprintf(buf,sizeof(buf),"%.0f", xF);
            DrawValueField(20,y,W_BALLXY,buf,COLOR_YELLOW,COLOR_BG);
            lastUi.ballXShown = xI;
        }
    }
    y+=TXT_LINE_H;

    /* ---- y ---- */
    {
        float yF = showDesired ? ballYDesired : g_uiData.ballY;
        int16_t yI = (int16_t)yF;
        if (force || sourceChanged || (lastUi.ballYShown != yI))
        {
            snprintf(buf,sizeof(buf),"%.0f", yF);
            DrawValueField(20,y,W_BALLXY,buf,COLOR_YELLOW,COLOR_BG);
            lastUi.ballYShown = yI;
        }
    }
    y+=TXT_LINE_H;

    lastUi.showDesired = showDesiredFlag;

    /* ---- vx ---- */
    {
        int16_t vxI = (int16_t)g_uiData.ballVx;
        if (force || (lastUi.ballVx != vxI))
        {
            snprintf(buf,sizeof(buf),"%.0f",g_uiData.ballVx);
            DrawValueField(20,y,W_BALLXY,buf,COLOR_YELLOW,COLOR_BG);
            lastUi.ballVx = vxI;
        }
    }
    y+=TXT_LINE_H;

    /* ---- vy ---- */
    {
        int16_t vyI = (int16_t)g_uiData.ballVy;
        if (force || (lastUi.ballVy != vyI))
        {
            snprintf(buf,sizeof(buf),"%.0f",g_uiData.ballVy);
            DrawValueField(20,y,W_BALLXY,buf,COLOR_YELLOW,COLOR_BG);
            lastUi.ballVy = vyI;
        }
    }
    y+=TXT_LINE_H;

    /* ---- Cam OK/OFF ---- */
    if (force || (lastUi.cameraOk != (uint8_t)g_uiData.cameraOk))
    {
        DrawValueField(25,y,W_OKERR, g_uiData.cameraOk?"OK":"OFF", COLOR_YELLOW, COLOR_BG);
        lastUi.cameraOk = (uint8_t)g_uiData.cameraOk;
    }
    y+=TXT_LINE_H;

    /* ---- S1/S2/S3: giá trị servo thật (µs) ---- */
    for (uint8_t i = 0; i < 3; i++)
    {
        int16_t sv = (int16_t)g_uiData.servoUs[i];
        if (force || (lastUi.servoUs[i] != sv))
        {
            snprintf(buf,sizeof(buf),"%d",g_uiData.servoUs[i]);
            DrawValueField(20,y,W_BALLXY,buf,COLOR_YELLOW,COLOR_BG);
            lastUi.servoUs[i] = sv;
        }
        y+=TXT_LINE_H;
    }

    /* ---- guide text ---- */
    if (force || (strncmp(lastUi.guideText, g_uiData.guideText, sizeof(lastUi.guideText)-1) != 0))
    {
        TFT_FillRectangle(TXT_X, GUIDE_Y1, TXT_X+(TFT_WIDTH-2*TXT_X)-1, GUIDE_Y1+VAL_FIELD_H-1, COLOR_BG);
        TFT_DrawTextFast(TXT_X, GUIDE_Y1, g_uiData.guideText, COLOR_YELLOW, COLOR_BG, 1);
        strncpy(lastUi.guideText, g_uiData.guideText, sizeof(lastUi.guideText)-1);
        lastUi.guideText[sizeof(lastUi.guideText)-1] = '\0';
    }

    /* GHI CHU: crosshair + luc giac da duoc ve san trong DrawStaticPart()
     * (chi 1 lan khi vao man hinh), nen KHONG ve lai o day (ham chay
     * lap 25Hz) - tranh ton thoi gian SPI moi frame va giu Task_Button_UI
     * bi cho lau khi tranh chap ScreenManager_Lock(). */

    /* ---- GIAI DOAN 1: DIRTY BALL ----
     * Chi xoa+ve lai bong neu VI TRI PIXEL thuc su thay doi, hoac
     * trang thai ballOn vua bat/tat. Neu bong dung yen (px/py khong
     * doi giua 2 lan goi) thi KHONG ghi gi len man hinh ca - day chinh
     * la nguyen nhan chinh gay "nhay" khi xoa cu roi ve moi trong luc
     * bong khong di chuyen (hoac di chuyen rat cham). */
    /* Khi STOP overlay đang bật, vùng vòng tròn đang là hình tròn đỏ +
     * chữ STOP - KHÔNG được vẽ/xoá bóng đè lên đó, để dành nguyên vẹn
     * cho tới khi ClearStopOverlay() vẽ lại từ đầu. */
    if (!stopOverlayActive)
    {
        if (g_uiData.ballOn)
        {
            int16_t px, py;
            MapBallToPixel(g_uiData.ballX, g_uiData.ballY, &px, &py);

            bool posChanged = (!ballWasDrawn) || (px != lastBallPx) || (py != lastBallPy);

            if (posChanged)
            {
                if (ballWasDrawn)
                {
                	TFT_FillCircleFast(lastBallPx,
                	                   lastBallPy,
                	                   BALL_RADIUS,
                	                   COLOR_BG,
                	                   COLOR_BG);
                	/* Vung vua xoa (COLOR_BG) co the da de len crosshair/vong
                	 * tron/luc giac - VA LAI CHI DUNG phan bi de, khong ve lai
                	 * toan bo (xem RepairStaticNear). Neu bong dung yen thi
                	 * khoi nay khong chay - dung nguyen ly dirty ball da co. */
                	RepairStaticNear(lastBallPx, lastBallPy);
                }
                TFT_FillCircleFast(px,
                                   py,
                                   BALL_RADIUS,
                                   COLOR_YELLOW,
                                   COLOR_BG);
                lastBallPx = px; lastBallPy = py; ballWasDrawn = true;
            }
            /* posChanged == false -> bong dung yen, khong dong gi den SPI */
        }
        else if (ballWasDrawn)
        {
            /* ballOn vua tat -> xoa bong 1 lan duy nhat, sau do thoi */
        	TFT_FillCircleFast(lastBallPx,
        	                   lastBallPy,
        	                   BALL_RADIUS,
        	                   COLOR_BG,
        	                   COLOR_BG);
        	RepairStaticNear(lastBallPx, lastBallPy);
            ballWasDrawn = false;
        }
    }

    lastUi.valid = true;
}

/* =========================================================
 * STOP OVERLAY - vẽ / xoá (THÊM)
 * ========================================================= */
static void DrawStopOverlay(void)
{
    /* Cùng kỹ thuật với screen_stop.c cũ: FillCircleFast nền đặc +
     * DrawTextFast đè giữa - đủ nhanh để không nhấp nháy, và đè kín
     * hoàn toàn crosshair + bóng cũ trong vùng vòng tròn. */
    TFT_FillCircleFast(CIRCLE_CX, CIRCLE_CY, STOP_CIRCLE_R, TFT_COLOR_RED, COLOR_BG);

    uint16_t tw, th;
    TFT_GetTextExtent("STOP", 2, &tw, &th);
    TFT_DrawTextFast(CIRCLE_CX - tw/2, CIRCLE_CY - th/2, "stop",
                      TFT_COLOR_WHITE, TFT_COLOR_RED, 2);

    /* bóng/crosshair trong vùng này coi như "chưa từng vẽ" - tránh vẽ
     * nhầm lên hình STOP nếu update() bị gọi trước khi overlay tắt */
    ballWasDrawn = false;
}

static void ClearStopOverlay(void)
{
    /* FillCircleFast(STOP) đã xoá mất viền tròn đỏ + crosshair trắng
     * trong vùng đó -> phải vẽ lại TOÀN BỘ phần tĩnh (DrawStaticPart
     * tự InvalidateUiCache() bên trong), rồi ép realtime vẽ lại hết. */
    DrawStaticPart();
    DrawRealtimePart();
}

/* API công khai - gọi từ Task_Button_UI (HandleBtn1Long) thay cho
 * ScreenManager_GotoAndRemember(ScreenStop_Get()) / ScreenManager_GoBack()
 * cũ. Tự khoá ScreenManager mutex (screen.h) vì hàm này được gọi từ
 * Task_Button_UI, khác task với Task_Display đang gọi ScreenManager_Update()
 * 25Hz - không khoá sẽ tái diễn đúng race chồng chéo hình đã gặp ở B5
 * (xem comment đầu screen_manager.c). */
void ScreenGauge_SetStopped(bool stopped)
{
    if (stopped == stopOverlayActive) return;   /* không đổi trạng thái - bỏ qua */

    ScreenManager_Lock();

    stopOverlayActive = stopped;
    if (stopOverlayActive) DrawStopOverlay();
    else                   ClearStopOverlay();

    ScreenManager_Unlock();
}

static void EnterForMode(uint8_t mode)
{
    activeMode = mode;

    if (mode == MODE_POSITION)
    {
        /* nạp lại setpoint thật đang có (vd sau khi reset MCU, hoặc do
         * CAN 0x204/nơi khác đã set trước đó) thay vì luôn bắt đầu từ 0 */
        setpoint_t sp;
        if (setpoint_get(&sp))
        {
            ballXDesired = sp.Ballx_d;
            ballYDesired = sp.Bally_d;
        }
    }

    DrawStaticPart();
    DrawRealtimePart();

    /* Khôi phục STOP overlay nếu đang dừng khẩn cấp và ta vừa quay lại
     * gauge screen (vd GoBack() sau khi Fault tự hết) - onEnter() không
     * đi qua ScreenGauge_SetStopped() nên phải tự vẽ lại ở đây. */
    if (stopOverlayActive) DrawStopOverlay();
}

static void OnEnter0(void) { EnterForMode(0); }
static void OnEnter1(void) { EnterForMode(1); }
static void OnEnter2(void) { EnterForMode(2); }
static void OnEnter3(void) { EnterForMode(3); }
/* THÊM Giai đoạn 3 - Mode Manual (index 4). guideText ban đầu do
 * control_mode_manual_enter() (Task_ControlLoop, khi setpoint.mode vừa đổi
 * sang OPMODE_MANUAL) tự ghi qua g_uiData.guideText - EnterForMode() ở đây
 * chỉ lo phần vẽ màn hình, không tự đặt guideText để tránh 2 nơi cùng ghi
 * đè nhau (UI thread vs Task_ControlLoop thread). */
static void OnEnter4(void) { EnterForMode(4); }

static void Update(void) { DrawRealtimePart(); }

static const Screen_t* GaugeScreenArray(uint8_t mode);

static inline float ClampBall(float v)
{
    if (v > BALL_PHYS_MAX)  return BALL_PHYS_MAX;
    if (v < -BALL_PHYS_MAX) return -BALL_PHYS_MAX;
    return v;
}

/* Ghi ballXDesired/ballYDesired (đã clamp) ra setpoint_t thật (system_state.h),
 * giữ nguyên mode/Roll_d/Pitch_d/Height_d đang có (read-modify-write, giống
 * cách task_can_rx.c xử lý CAN_ID_ATTITUDE_DESIRED). Nếu miss mutex (busy),
 * bỏ qua lần này - lần bấm nút kế tiếp sẽ thử lại, không áp dụng nửa vời. */
static void PublishBallDesired(void)
{
    setpoint_t sp;
    if (setpoint_get(&sp))
    {
        sp.Ballx_d = ballXDesired;
        sp.Bally_d = ballYDesired;
        setpoint_set(&sp);
    }
}

/* ---- THÊM Giai đoạn 3: điều khiển Mode Manual qua nút UI ----
 * Quy ước (đề xuất, CẦN BẠN XÁC NHẬN lại theo cảm giác bấm thật trên bàn):
 *   - Sub-state đang IDLE/DONE (chưa chạy việc gì / vừa xong 1 việc):
 *       UP/DOWN  = duyệt qua lại 3 việc {Deadband, Manual step, Sweep log}
 *                  và BẮT ĐẦU CHẠY NGAY việc vừa chọn (không cần bước
 *                  "commit" riêng, khác hẳn cơ chế NAV/SEL đổi mode ở
 *                  trên vì đây là 3 CÔNG CỤ trong cùng 1 mode, không phải
 *                  đổi mode robot).
 *   - Sub-state đang MANUAL_SUB_MANUAL_STEP (đang chỉnh tay):
 *       LEFT/RIGHT = -/+ MANUAL_STEP_US cho servo đang chọn
 *       UP/DOWN    = đổi servo đang chỉnh (S1 -> S2 -> S3 -> S1)
 *   - Sub-state đang DEADBAND_SCAN/SWEEP_LOG (đang tự động chạy):
 *       mọi nút (trừ ENTER/EXIT vốn dùng cho điều hướng mode) không có
 *       tác dụng - đợi tự xong (chuyển về DONE). */
static void HandleManualButton(ButtonState_t evt)
{
    manual_sub_state_t sub = control_mode_manual_get_sub_state();

    if (sub == MANUAL_SUB_MANUAL_STEP)
    {
        static uint8_t s_manual_ch = 1;   /* 1..3, chỉ dùng để hiển thị thứ tự xoay vòng ở UI */

        switch (evt.button)
        {
        case BUTTON_LEFT:
            control_mode_manual_adjust(-MANUAL_STEP_US);
            break;
        case BUTTON_RIGHT:
            control_mode_manual_adjust(+MANUAL_STEP_US);
            break;
        case BUTTON_UP:
        case BUTTON_DOWN:
            s_manual_ch = (s_manual_ch % 3) + 1;   /* 1->2->3->1, cả UP/DOWN đều xoay tới - đơn giản hoá vì chỉ có 3 lựa chọn */
            control_mode_manual_select_channel(s_manual_ch);
            break;
        default:
            break;
        }
        return;
    }

    if (sub == MANUAL_SUB_IDLE || sub == MANUAL_SUB_DONE)
    {
        static manual_sub_state_t s_pick = MANUAL_SUB_MANUAL_STEP;

        if (evt.button == BUTTON_UP || evt.button == BUTTON_DOWN)
        {
            /* Chỉ còn 2 lựa chọn (Deadband Scan đã bỏ - Giai đoạn 4, cảm
             * biến nhiễu đo không chính xác) - UP/DOWN xoay qua lại 2 việc. */
            switch (s_pick)
            {
                case MANUAL_SUB_MANUAL_STEP: s_pick = MANUAL_SUB_MANUAL_STEP;   break;
                default:                     s_pick = MANUAL_SUB_SWEEP_LOG; break;
            }
            control_mode_manual_select_substate(s_pick);
        }
    }
    /* sub == DEADBAND_SCAN/SWEEP_LOG: không xử lý nút gì, đang tự động chạy */
}

static void OnButton(ButtonState_t evt)
{
    if (evt.event != BUTTON_EVENT_PRESS)
    {
        return;
    }

    /* THÊM Giai đoạn 3: khi đang ở Mode Manual + đã SELECTED (robot đang
     * thật sự dùng mode này, không phải đang browse để đổi mode khác),
     * chuyển hướng LEFT/RIGHT/UP/DOWN sang điều khiển Manual thay vì hành
     * vi mặc định (vốn dành cho Ball X/Y ở Position). ENTER/EXIT vẫn xử lý
     * như cũ bên dưới (đổi mode / shutdown) - không chặn ở đây. */
    if (activeMode == MODE_MANUAL && navState == NAV_SELECTED &&
        (evt.button == BUTTON_LEFT || evt.button == BUTTON_RIGHT ||
         evt.button == BUTTON_UP   || evt.button == BUTTON_DOWN))
    {
        HandleManualButton(evt);
        DrawRealtimePart();
        return;
    }

    switch (evt.button)
    {
    case BUTTON_ENTER:
        if (navState == NAV_SELECTED)
        {
            /* SELECTED -> BROWSE: bắt đầu cho phép LEFT/RIGHT đổi screen */
            navState = NAV_BROWSE;
        }
        else
        {
            /* BROWSE -> SELECTED: COMMIT mode hiện tại - đây là điểm
             * DUY NHẤT ghi mode thật ra ngoài (setpoint_set),
             * đúng nguyên tắc "chỉ đổi state robot khi đã xác nhận". */
            navState = NAV_SELECTED;
            g_uiData.mode = activeMode;   /* cache hiển thị - sẽ bị ghi đè đúng giá
                                              trị bởi UiData_SyncFromSystemState()
                                              mỗi vòng Task_Display, không sao */
            {
                /* COMMIT mode thật vào setpoint_t.mode (system_state.h) - đây là
                 * điểm DUY NHẤT ghi mode ra ngoài UI, đúng khi đã xác nhận
                 * (BROWSE -> SELECTED). Task_CAN_TX (0x103) và Task_ControlLoop
                 * đọc field này qua setpoint_get(). */
                setpoint_t sp;
                if (setpoint_get(&sp))
                {
                    sp.mode = activeMode;
                    setpoint_set(&sp);
                }
            }
        }
        DrawRealtimePart();
        break;

    case BUTTON_LEFT:
        if (navState == NAV_BROWSE)
        {
            uint8_t next = (activeMode == 0) ? (MODE_COUNT - 1) : (activeMode - 1);
            ScreenManager_Goto(GaugeScreenArray(next));
        }
        else if (activeMode == MODE_POSITION)
        {
            ballXDesired = ClampBall(ballXDesired - BALL_STEP);
            PublishBallDesired();
            DrawRealtimePart();
        }
        break;

    case BUTTON_RIGHT:
        if (navState == NAV_BROWSE)
        {
            uint8_t next = (activeMode + 1) % MODE_COUNT;
            ScreenManager_Goto(GaugeScreenArray(next));
        }
        else if (activeMode == MODE_POSITION)
        {
            ballXDesired = ClampBall(ballXDesired + BALL_STEP);
            PublishBallDesired();
            DrawRealtimePart();
        }
        break;

    case BUTTON_UP:
        if (navState == NAV_SELECTED && activeMode == MODE_POSITION)
        {
            ballYDesired = ClampBall(ballYDesired + BALL_STEP);
            PublishBallDesired();
            DrawRealtimePart();
        }
        /* NAV_BROWSE hoặc mode khác Position: không làm gì, đúng chốt #1 */
        break;

    case BUTTON_DOWN:
        if (navState == NAV_SELECTED && activeMode == MODE_POSITION)
        {
            ballYDesired = ClampBall(ballYDesired - BALL_STEP);
            PublishBallDesired();
            DrawRealtimePart();
        }
        break;

    case BUTTON_EXIT:
        ScreenManager_GotoAndRemember(ScreenShutdown_Get());
        break;

    default:
        break;
    }
}

static const Screen_t gaugeScreens[MODE_COUNT] = {
    { .onEnter=OnEnter0, .onExit=NULL, .update=Update, .onButton=OnButton },
    { .onEnter=OnEnter1, .onExit=NULL, .update=Update, .onButton=OnButton },
    { .onEnter=OnEnter2, .onExit=NULL, .update=Update, .onButton=OnButton },
    { .onEnter=OnEnter3, .onExit=NULL, .update=Update, .onButton=OnButton },
    { .onEnter=OnEnter4, .onExit=NULL, .update=Update, .onButton=OnButton },   /* THÊM Giai đoạn 3 */
};

static const Screen_t* GaugeScreenArray(uint8_t mode)
{
    return &gaugeScreens[mode % MODE_COUNT];
}

const Screen_t* ScreenGauge_Get(uint8_t mode, const char *bottomLabel)
{
    if (mode < MODE_COUNT)
    {
        bottomLabelStore[mode] = bottomLabel;
    }
    return &gaugeScreens[mode % MODE_COUNT];
}
