#include "screen_gauge_common.h"
#include "screen_shutdown.h"
#include "tft_service.h"
#include "ui_data.h"
#include "system_state.h"
#include "control_mode_manual.h"   /* ADDED Phase 3 - Manual mode control via UI */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* =========================================================
 * LAYOUT - 220x176 (landscape), unchanged from the original,
 * only adds one field to show SEL/NAV state.
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

/* ADDED Phase 3 - us step applied per LEFT/RIGHT press in Manual mode
 * (MANUAL_SUB_MANUAL_STEP), matches SERVO_TEST_MANUAL_STEP_US in
 * B6_Control.md section 4. */
#define MANUAL_STEP_US   10

/* ---- Navigation state: SELECTED (committed, robot is actually running
 * this mode) <-> BROWSE (browsing to switch mode; LEFT/RIGHT only change
 * screens while in this state). One static variable shared across all
 * 5 gauge screens since only one of the five is ever active at a time. ---- */
typedef enum { NAV_SELECTED = 0, NAV_BROWSE } NavState_t;
static NavState_t navState = NAV_SELECTED;

static const char *bottomLabelStore[MODE_COUNT];
static uint8_t activeMode = 0;

static bool    ballWasDrawn = false;
static int16_t lastBallPx = 0, lastBallPy = 0;

/* =========================================================
 * STOP OVERLAY (ADDED) - replaces the old approach of switching to a
 * separate ScreenStop: on a BTN1-long emergency stop, a solid red
 * circle + white "STOP" text is drawn ON TOP of the existing circle
 * (CIRCLE_CX/CY/R) on the current gauge screen, WITHOUT changing
 * ScreenManager's currentScreen (no more GotoAndRemember/GoBack).
 * Reason: the user wants to keep seeing all the readouts (Roll/Pitch/
 * S1-3...) during an emergency stop - only the circle area switches
 * to the STOP indicator.
 * ========================================================= */
#define STOP_CIRCLE_R  (CIRCLE_R - 25)   /* slightly smaller than the ring so it doesn't spill over */
static bool stopOverlayActive = false;

/* =========================================================
 * PHASE 1 - DIRTY UPDATE
 *
 * Caches the values last drawn to the screen. On each call to
 * DrawRealtimePart(), only the fields that CHANGED relative to the
 * cache get FillRectangle+DrawText'd again - this sharply cuts down
 * the number of SPI writes (the main cause of flicker on redraw).
 *
 * cacheValid = false forces every field to be redrawn once (e.g.
 * right after entering the screen / after DrawStaticPart() runs) so
 * nothing is "missed" by comparing against stale cache data.
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

    int16_t  ballXShown;    /* value currently shown (desired or measured, depending on mode) */
    int16_t  ballYShown;
    uint8_t  showDesired;   /* to detect a switch of x/y source (desired <-> measured) */

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

/* Desired ball coordinates currently being adjusted in Position mode -
 * only meaningful when activeMode == MODE_POSITION && navState == NAV_SELECTED */
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

/* ---- Coordinates of the 6 hexagon vertices, computed ONCE (shared
 * between DrawStaticPart() and RepairStaticNear() below - avoids
 * redoing trig every frame). One vertex sits on the Y axis (topmost
 * vertex, -90 degrees). ---- */
static int16_t s_hexX[6], s_hexY[6];
static bool    s_hexReady = false;

static void ComputeHexVertices(void)
{
    if (s_hexReady) return;
    static const float HEX_ANGLE0 = -1.5707963f; /* -90 deg, on the Y axis */
    int i;
    for (i = 0; i < 6; i++)
    {
        float ang = HEX_ANGLE0 + i * (3.14159265f / 3.0f); /* +60 deg per step */
        /* Use lroundf() instead of "+0.5f then cast to int" - the old
         * rounding was wrong for negative values (left-side vertices have
         * cosf(ang) < 0), which pulled the left vertices 1 pixel closer to
         * center than the right ones, skewing the hexagon so the right
         * edge sat farther from center than the left. */
        s_hexX[i] = CIRCLE_CX + (int16_t)lroundf(55.0f * cosf(ang));
        s_hexY[i] = CIRCLE_CY + (int16_t)lroundf(55.0f * sinf(ang));
    }
    s_hexReady = true;
}

/* Distance from point (px,py) to the line segment (ax,ay)-(bx,by) - used
 * to check whether a hexagon edge was overwritten by the ball-erase area,
 * so we don't have to redraw ALL 6 edges when only one (or none) was
 * actually affected. */
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

/* ---- REPAIRS the static drawing (crosshair/circle/hexagon) ONLY IN THE
 * AREA that TFT_FillCircleFast() overwrote when the ball moved away from
 * position (bx,by), radius BALL_RADIUS - no more full redraw every frame.
 * Called exactly once right after erasing the old ball (see PHASE 1
 * below), and only redraws the specific edges/lines actually covered
 * by that erase area. */
static void RepairStaticNear(int16_t bx, int16_t by)
{
    const float R = (float)(BALL_RADIUS + 1); /* +1 to absorb rounding error */

    /* IMPORTANT: the redraw order here must MATCH the draw order in
     * DrawStaticPart() - red circle (bottom) -> white crosshair (drawn
     * over the circle) -> red hexagon (top). If the order were reversed
     * (e.g. crosshair first, then circle), the 4 points where the
     * crosshair crosses the circle (its top/bottom/left/right vertices)
     * would have their colors FLIPPED: what should be white would get
     * painted red instead, leaving a permanent artifact at those exact
     * 4 points. */

    /* Circle outline: only redraw (the arc) if the erase area sits right
     * against the border - rare, since the ball usually stays near center,
     * not right at the edge. */
    float distToCenter = sqrtf((float)(bx-CIRCLE_CX)*(bx-CIRCLE_CX) + (float)(by-CIRCLE_CY)*(by-CIRCLE_CY));
    if (fabsf(distToCenter - (float)CIRCLE_R) <= R)
    {
        TFT_DrawCircle(CIRCLE_CX, CIRCLE_CY, CIRCLE_R, TFT_COLOR_RED);
    }
    /* horizontal crosshair: y = CIRCLE_CY */
    if (fabsf((float)by - (float)CIRCLE_CY) <= R)
    {
        TFT_DrawLine(bx-BALL_RADIUS-1, CIRCLE_CY, bx+BALL_RADIUS+1, CIRCLE_CY, COLOR_WHITE);
    }
    /* vertical crosshair: x = CIRCLE_CX */
    if (fabsf((float)bx - (float)CIRCLE_CX) <= R)
    {
        TFT_DrawLine(CIRCLE_CX, by-BALL_RADIUS-1, CIRCLE_CX, by+BALL_RADIUS+1, COLOR_WHITE);
    }
    /* hexagon: only redraw the SPECIFIC edge(s) that were overwritten */
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

    /* ---- Regular red hexagon, one vertex on the Y axis (topmost) ---- */
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

    /* the static screen was just fully redrawn -> the old cache is no
     * longer valid, force DrawRealtimePart() to redraw EVERY field once */
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

    /* ---- navigation state indicator: NAV / SEL ---- */
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
        int16_t rollI = (int16_t)g_uiData.imuRoll;   /* compare against the value that will be DISPLAYED (already rounded) */
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

    /* x/y: in Position mode while SELECTED, show the setpoint currently
     * being adjusted (ballXDesired/ballYDesired) instead of the measured
     * ball position, so the user immediately sees the value they just
     * changed - other modes still show the real ball coordinates as
     * before. */
    bool showDesired = (activeMode == MODE_POSITION) && (navState == NAV_SELECTED);
    uint8_t showDesiredFlag = showDesired ? 1 : 0;
    /* the data source switched (desired <-> measured) -> must be treated
     * as "changed" even if the rounded value happens to match, to avoid
     * showing a stale value. */
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

    /* ---- S1/S2/S3: actual servo values (µs) ---- */
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

    /* NOTE: the crosshair + hexagon were already drawn in DrawStaticPart()
     * (once, on screen entry), so they are NOT redrawn here (this
     * function runs in a 25Hz loop) - avoids burning SPI time every
     * frame and keeps Task_Button_UI from waiting too long when it
     * contends for ScreenManager_Lock(). */

    /* ---- PHASE 1: DIRTY BALL ----
     * Only erases+redraws the ball if its PIXEL POSITION actually
     * changed, or ballOn was just toggled. If the ball is stationary
     * (px/py unchanged between calls), NOTHING is written to the
     * screen at all - this is exactly what was causing the flicker
     * from erase-then-redraw while the ball wasn't moving (or moving
     * very slowly). */
    /* While the STOP overlay is active, the circle area currently shows
     * the red circle + STOP text - the ball must NOT be drawn/erased
     * over it, so it stays intact until ClearStopOverlay() redraws
     * from scratch. */
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
                	/* The area just erased (COLOR_BG) may have overwritten the
                	 * crosshair/circle/hexagon - REPAIR ONLY the affected part,
                	 * not a full redraw (see RepairStaticNear). If the ball is
                	 * stationary this block doesn't run at all - it relies on
                	 * the dirty-ball logic already in place. */
                	RepairStaticNear(lastBallPx, lastBallPy);
                }
                TFT_FillCircleFast(px,
                                   py,
                                   BALL_RADIUS,
                                   COLOR_YELLOW,
                                   COLOR_BG);
                lastBallPx = px; lastBallPy = py; ballWasDrawn = true;
            }
            /* posChanged == false -> ball is stationary, no SPI traffic at all */
        }
        else if (ballWasDrawn)
        {
            /* ballOn was just turned off -> erase the ball exactly once, then stop */
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
 * STOP OVERLAY - draw / clear (ADDED)
 * ========================================================= */
static void DrawStopOverlay(void)
{
    /* Same technique as the old screen_stop.c: solid FillCircleFast
     * background + centered DrawTextFast on top - fast enough to avoid
     * flicker, and fully covers the crosshair + old ball inside the
     * circle area. */
    TFT_FillCircleFast(CIRCLE_CX, CIRCLE_CY, STOP_CIRCLE_R, TFT_COLOR_RED, COLOR_BG);

    uint16_t tw, th;
    TFT_GetTextExtent("STOP", 2, &tw, &th);
    TFT_DrawTextFast(CIRCLE_CX - tw/2, CIRCLE_CY - th/2, "stop",
                      TFT_COLOR_WHITE, TFT_COLOR_RED, 2);

    /* treat the ball/crosshair in this area as "never drawn" - avoids
     * accidentally drawing over the STOP graphic if update() gets called
     * before the overlay is cleared */
    ballWasDrawn = false;
}

static void ClearStopOverlay(void)
{
    /* FillCircleFast(STOP) wiped out the red ring + white crosshair in
     * that area -> the entire static part must be redrawn (DrawStaticPart
     * calls InvalidateUiCache() internally), then force a full realtime
     * redraw as well. */
    DrawStaticPart();
    DrawRealtimePart();
}

/* Public API - called from Task_Button_UI (HandleBtn1Long) in place of
 * the old ScreenManager_GotoAndRemember(ScreenStop_Get()) /
 * ScreenManager_GoBack(). Locks the ScreenManager mutex (screen.h)
 * itself, since this function is called from Task_Button_UI, a
 * different task than Task_Display which calls ScreenManager_Update()
 * at 25Hz - without the lock, the same overlapping-draw race seen in
 * B5 would resurface (see the comment at the top of screen_manager.c). */
void ScreenGauge_SetStopped(bool stopped)
{
    if (stopped == stopOverlayActive) return;   /* state unchanged - nothing to do */

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
        /* reload the real setpoint currently in effect (e.g. after an
         * MCU reset, or if CAN 0x204/something else already set it)
         * instead of always starting from 0 */
        setpoint_t sp;
        if (setpoint_get(&sp))
        {
            ballXDesired = sp.Ballx_d;
            ballYDesired = sp.Bally_d;
        }
    }

    DrawStaticPart();
    DrawRealtimePart();

    /* Restore the STOP overlay if an emergency stop is active and we
     * just returned to a gauge screen (e.g. GoBack() after a Fault
     * cleared on its own) - onEnter() doesn't go through
     * ScreenGauge_SetStopped(), so it has to be redrawn here explicitly. */
    if (stopOverlayActive) DrawStopOverlay();
}

static void OnEnter0(void) { EnterForMode(0); }
static void OnEnter1(void) { EnterForMode(1); }
static void OnEnter2(void) { EnterForMode(2); }
static void OnEnter3(void) { EnterForMode(3); }
/* ADDED Phase 3 - Manual mode (index 4). The initial guideText is written
 * by control_mode_manual_enter() itself (Task_ControlLoop, when
 * setpoint.mode just switched to OPMODE_MANUAL) via g_uiData.guideText -
 * EnterForMode() here only handles drawing the screen and does not set
 * guideText itself, to avoid two sides writing it at the same time (UI
 * thread vs Task_ControlLoop thread). */
static void OnEnter4(void) { EnterForMode(4); }

static void Update(void) { DrawRealtimePart(); }

static const Screen_t* GaugeScreenArray(uint8_t mode);

static inline float ClampBall(float v)
{
    if (v > BALL_PHYS_MAX)  return BALL_PHYS_MAX;
    if (v < -BALL_PHYS_MAX) return -BALL_PHYS_MAX;
    return v;
}

/* Writes ballXDesired/ballYDesired (already clamped) out to the real
 * setpoint_t (system_state.h), preserving the existing mode/Roll_d/
 * Pitch_d/Height_d (read-modify-write, same approach task_can_rx.c uses
 * for CAN_ID_ATTITUDE_DESIRED). If the mutex is missed (busy), this
 * update is simply skipped - the next button press will retry, so
 * nothing gets applied half-way. */
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

/* ---- ADDED Phase 3: Manual mode control via UI buttons ----
 * Convention (proposed - NEEDS YOUR CONFIRMATION against how it actually
 * feels pressing the real buttons on the table):
 *   - Sub-state is IDLE/DONE (nothing running / a task just finished):
 *       UP/DOWN  = cycle through the 3 tasks {Deadband, Manual step,
 *                  Sweep log} and START RUNNING the newly selected task
 *                  IMMEDIATELY (no separate "commit" step, unlike the
 *                  NAV/SEL mode-switch mechanism above, since these are
 *                  3 TOOLS within the same mode, not a robot mode
 *                  change).
 *   - Sub-state is MANUAL_SUB_MANUAL_STEP (manual adjustment in progress):
 *       LEFT/RIGHT = -/+ MANUAL_STEP_US on the currently selected servo
 *       UP/DOWN    = switch which servo is being adjusted (S1 -> S2 -> S3 -> S1)
 *   - Sub-state is DEADBAND_SCAN/SWEEP_LOG (running automatically):
 *       no button (other than ENTER/EXIT, reserved for mode navigation)
 *       has any effect - wait for it to finish on its own (back to DONE). */
static void HandleManualButton(ButtonState_t evt)
{
    manual_sub_state_t sub = control_mode_manual_get_sub_state();

    if (sub == MANUAL_SUB_MANUAL_STEP)
    {
        static uint8_t s_manual_ch = 1;   /* 1..3, only used to track the UI's round-robin order */

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
            s_manual_ch = (s_manual_ch % 3) + 1;   /* 1->2->3->1, both UP and DOWN advance - simplified since there are only 3 choices */
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
            /* Only 2 choices remain (Deadband Scan was dropped in Phase 4 -
             * the sensor's measurement noise made it unreliable) - UP/DOWN
             * toggles between the 2 remaining tasks. */
            switch (s_pick)
            {
                case MANUAL_SUB_MANUAL_STEP: s_pick = MANUAL_SUB_MANUAL_STEP;   break;
                default:                     s_pick = MANUAL_SUB_SWEEP_LOG; break;
            }
            control_mode_manual_select_substate(s_pick);
        }
    }
    /* sub == DEADBAND_SCAN/SWEEP_LOG: no buttons handled, running automatically */
}

static void OnButton(ButtonState_t evt)
{
    if (evt.event != BUTTON_EVENT_PRESS)
    {
        return;
    }

    /* ADDED Phase 3: while in Manual mode and SELECTED (the robot is
     * actually running this mode, not just browsing to switch to another
     * one), redirect LEFT/RIGHT/UP/DOWN to Manual mode control instead of
     * the default behavior (which is meant for Ball X/Y in Position mode).
     * ENTER/EXIT still fall through to the handling below as usual
     * (mode switch / shutdown) - not intercepted here. */
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
            /* SELECTED -> BROWSE: start allowing LEFT/RIGHT to change screens */
            navState = NAV_BROWSE;
        }
        else
        {
            /* BROWSE -> SELECTED: COMMIT the current mode - this is the
             * ONE place that writes the real mode out (setpoint_set),
             * following the rule "only change robot state once confirmed". */
            navState = NAV_SELECTED;
            g_uiData.mode = activeMode;   /* display cache - will be overwritten with
                                              the correct value by
                                              UiData_SyncFromSystemState() on the
                                              next Task_Display loop, which is fine */
            {
                /* COMMIT the real mode into setpoint_t.mode (system_state.h) -
                 * this is the ONE place the UI writes mode out, and only once
                 * confirmed (BROWSE -> SELECTED). Task_CAN_TX (0x103) and
                 * Task_ControlLoop read this field via setpoint_get(). */
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
        /* NAV_BROWSE or a mode other than Position: no-op, matches checkpoint #1 */
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
    { .onEnter=OnEnter4, .onExit=NULL, .update=Update, .onButton=OnButton },   /* ADDED Phase 3 */
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
