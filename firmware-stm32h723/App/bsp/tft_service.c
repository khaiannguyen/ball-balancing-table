#include "tft_service.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

extern SPI_HandleTypeDef hspi1;

#define TFT_SPI (&hspi1)

#define ILI9225_GRAM_WRITE 0x22

volatile uint8_t dmaDone=0;


/* =========================================================
 * Semaphore bao SPI-DMA line complete.
 *
 * LY DO doi tu bien co "dmaDone" + vong lap spin (busy-wait/taskYIELD)
 * sang osSemaphoreId_t: ca 2 cach cu deu sai trong truong hop nay.
 *
 *  - osDelay(1) (ban dau): task bi dua RA KHOI ready-list hoan toan
 *    trong it nhat 1 tick -> moi task khac (bat ke priority nao) chac
 *    chan co co hoi chay. Nhung cham: toi thieu 1ms/dong du DMA that
 *    su chi mat vai chuc us.
 *
 *  - busy-wait / taskYIELD (ban sua truoc): task VAN nam trong
 *    ready-list, chi "nhuong luot". Neu Task_Display co priority >=
 *    Timer Service Task (noi chay debounce timer callback cua nut
 *    bam), taskYIELD() VO DUNG - scheduler van chon lai Task_Display
 *    ngay vi no la task ready co priority cao nhat. Timer Service Task
 *    khong bao gio duoc chay -> debounce callback khong chay -> nut
 *    bam khong bao gio duoc enqueue -> toan bo he thong nut "chet".
 *    Day chinh la nguyen nhan that cua loi "nap firmware moi thi bam
 *    nut khong con tac dung".
 *
 * Semaphore giai quyet dut diem: osSemaphoreAcquire() dua task RA
 * KHOI ready-list (giong osDelay ve mat "nhuong CPU that"), nhung
 * thuc day NGAY khi DMA callback release semaphore - khong can cho
 * du 1 tick nhu osDelay(1). Vua nhanh vua khong phu thuoc priority
 * cua task nao khac.
 *
 * QUAN TRONG - TAI SAO PHAI DUNG xSemaphoreGiveFromISR() CHU KHONG
 * PHAI osSemaphoreRelease(): dung Y HET ly do buttons.c da giai thich
 * cho xTimerStartFromISR() vs osTimerStart() - CMSIS-RTOS2 wrapper
 * (cmsis_os2.c) trong project nay KHONG tu dong chuyen sang ban
 * "FromISR" khi phat hien dang chay trong ISR (IS_IRQ()), ma mot so
 * ham (da xac nhan voi osTimerStart, gia dinh tuong tu voi
 * osSemaphoreRelease) se TRA VE LOI va KHONG thuc hien hanh dong that
 * su neu bi goi tu ISR. HAL_SPI_TxCpltCallback() chay trong ISR context
 * (DMA complete interrupt) - neu goi osSemaphoreRelease() o day, no co
 * the am tham that bai (semaphore khong bao gio duoc release that),
 * khien TFT_WaitDmaLine() LUON LUON phai cho het timeout 50ms moi dong
 * thay vi duoc danh thuc ngay khi DMA xong - ket qua: ve 1 man hinh
 * (176 dong FillScreen + nhieu FillRectangle khac) co the mat toi HANG
 * CHUC GIAY, tao cam giac "bam nut khong an" du thuc ra MCU van dang
 * chay, chi cuc ky cham.
 *
 * Fix: goi thang ham FreeRTOS goc xSemaphoreGiveFromISR() (bypass CMSIS
 * wrapper hoan toan), dung portYIELD_FROM_ISR de context-switch ngay
 * neu can - day la cach ISR-safe THAT SU, khong phu thuoc hanh vi cua
 * wrapper.
 * ========================================================= */
static osSemaphoreId_t s_tftDmaSem;
static osSemaphoreAttr_t s_tftDmaSemAttr = { .name = "tftDmaSem" };

/* Goi 1 lan duy nhat truoc khi dung TFT_FillScreen/TFT_FillRectangle
 * (vd trong TFT_Init()). Neu chua goi, cac ham do se tu tao lazy o
 * lan dau (xem TFT_WaitDmaLine() ben duoi) de tranh crash neu ai quen
 * goi TFT_Init() truoc. */
static inline void TFT_EnsureDmaSemCreated(void)
{
    if (s_tftDmaSem == NULL)
    {
        s_tftDmaSem = osSemaphoreNew(1, 0, &s_tftDmaSemAttr);
    }
}

/* Cho 1 dong DMA xong - dung o ca TFT_FillScreen va TFT_FillRectangle */
static inline void TFT_WaitDmaLine(void)
{
    TFT_EnsureDmaSemCreated();
    osSemaphoreAcquire(s_tftDmaSem, 50);   /* timeout 50ms - tranh treo cung neu DMA loi */
}

/* buffer 1 dòng */
static uint16_t lineBuffer[TFT_WIDTH]
__attribute__((aligned(32)));


/*=========================================================
GPIO
=========================================================*/

static inline void TFT_CS_LOW(void)
{
    HAL_GPIO_WritePin(
            TFT_CS_GPIO_Port,
            TFT_CS_Pin,
            GPIO_PIN_RESET);
}

static inline void TFT_CS_HIGH(void)
{
    HAL_GPIO_WritePin(
            TFT_CS_GPIO_Port,
            TFT_CS_Pin,
            GPIO_PIN_SET);
}

static inline void TFT_DC_LOW(void)
{
    HAL_GPIO_WritePin(
            TFT_DC_GPIO_Port,
            TFT_DC_Pin,
            GPIO_PIN_RESET);
}

static inline void TFT_DC_HIGH(void)
{
    HAL_GPIO_WritePin(
            TFT_DC_GPIO_Port,
            TFT_DC_Pin,
            GPIO_PIN_SET);
}

static inline void TFT_RST_LOW(void)
{
    HAL_GPIO_WritePin(
            TFT_RST_GPIO_Port,
            TFT_RST_Pin,
            GPIO_PIN_RESET);
}

static inline void TFT_RST_HIGH(void)
{
    HAL_GPIO_WritePin(
            TFT_RST_GPIO_Port,
            TFT_RST_Pin,
            GPIO_PIN_SET);
}


/*=========================================================
DMA callback
=========================================================*/

void HAL_SPI_TxCpltCallback(
        SPI_HandleTypeDef *hspi)
{
    if(hspi==TFT_SPI)
    {
        dmaDone=1;

        /* ISR context - PHAI dung ham FreeRTOS goc xSemaphoreGiveFromISR(),
         * KHONG dung osSemaphoreRelease() (CMSIS wrapper co the am tham
         * fail khi goi tu ISR - xem giai thich chi tiet o dinh file, cung
         * loai loi voi osTimerStart() ma buttons.c da gap). s_tftDmaSem
         * (osSemaphoreId_t) chinh la SemaphoreHandle_t goc trong CMSIS-
         * RTOS2-cho-FreeRTOS nen cast thang duoc, khong can ham chuyen doi. */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR((SemaphoreHandle_t)s_tftDmaSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/*=========================================================
SPI register write
=========================================================*/

static void TFT_WriteRegister(
        uint8_t reg,
        uint16_t value)
{
    uint8_t cmd[2];
    uint8_t data[2];

    cmd[0]=0;
    cmd[1]=reg;

    data[0]=(value>>8);
    data[1]=(value&0xFF);

    TFT_CS_LOW();

    TFT_DC_LOW();

    HAL_SPI_Transmit(
            TFT_SPI,
            cmd,
            2,
            HAL_MAX_DELAY);

    TFT_DC_HIGH();

    HAL_SPI_Transmit(
            TFT_SPI,
            data,
            2,
            HAL_MAX_DELAY);

    TFT_CS_HIGH();
}


/*=========================================================
GRAM write
=========================================================*/

static void TFT_BeginGramWrite(void)
{
    uint8_t cmd[2];

    cmd[0]=0;
    cmd[1]=ILI9225_GRAM_WRITE;

    TFT_CS_LOW();

    TFT_DC_LOW();

    HAL_SPI_Transmit(
            TFT_SPI,
            cmd,
            2,
            HAL_MAX_DELAY);

    TFT_DC_HIGH();
}


/*=========================================================
Window
=========================================================*/

void TFT_SetWindow(
        uint16_t x0,
        uint16_t y0,
        uint16_t x1,
        uint16_t y1)
{
    /* ---- xoay 90 độ: logic (x,y) ngang -> vật lý (px,py) dọc ----
     * physical_x = logic_y
     * physical_y = (PHYS_HEIGHT-1) - logic_x
     *
     * Nếu màn hình hiện ra bị NGƯỢC (úp ngược) hoặc LẬT GƯƠNG,
     * đổi 2 dòng dưới sang bản "rotation khác" ở cuối file (xem chú thích).
     */
    uint16_t px0 = y0;
    uint16_t px1 = y1;

    uint16_t py0 = (TFT_PHYS_HEIGHT-1) - x1;
    uint16_t py1 = (TFT_PHYS_HEIGHT-1) - x0;

    TFT_WriteRegister(0x36,px1);
    TFT_WriteRegister(0x37,px0);

    TFT_WriteRegister(0x38,py1);
    TFT_WriteRegister(0x39,py0);

    TFT_WriteRegister(0x20,px0);
    TFT_WriteRegister(0x21,py0);
}


/*=========================================================
INIT
=========================================================*/

void TFT_Init(void)
{
    //printf("TFT START\r\n");

    TFT_RST_HIGH();
    HAL_Delay(1);

    TFT_RST_LOW();
    HAL_Delay(50);

    TFT_RST_HIGH();
    HAL_Delay(150);

    TFT_WriteRegister(0x10,0x0000);
    TFT_WriteRegister(0x11,0x0000);
    TFT_WriteRegister(0x12,0x0000);
    TFT_WriteRegister(0x13,0x0000);
    TFT_WriteRegister(0x14,0x0000);

    HAL_Delay(40);

    TFT_WriteRegister(0x11,0x0018);
    TFT_WriteRegister(0x12,0x6121);
    TFT_WriteRegister(0x13,0x006F);
    TFT_WriteRegister(0x14,0x495F);
    TFT_WriteRegister(0x10,0x0800);

    HAL_Delay(10);

    TFT_WriteRegister(0x11,0x103B);

    HAL_Delay(50);

    TFT_WriteRegister(0x01,0x011C);
    TFT_WriteRegister(0x02,0x0100);
    TFT_WriteRegister(0x03,0x1038);

    TFT_WriteRegister(0x07,0x0012);

    HAL_Delay(50);

    TFT_WriteRegister(0x07,0x1017);

    HAL_Delay(50);


    //printf("TFT INIT DONE\r\n");
}


/*=========================================================
FillScreen
=========================================================*/

void TFT_FillScreen(
        uint16_t color)
{
    uint32_t y;
    uint32_t i;

    color=
    (color<<8)|
    (color>>8);

    for(i=0;i<TFT_WIDTH;i++)
    {
        lineBuffer[i]=color;
    }

#if (__DCACHE_PRESENT==1)

    SCB_CleanDCache_by_Addr(
            (uint32_t*)lineBuffer,
            sizeof(lineBuffer));

#endif

    TFT_SetWindow(
            0,
            0,
            TFT_WIDTH-1,
            TFT_HEIGHT-1);

    TFT_BeginGramWrite();

    for(y=0;y<TFT_HEIGHT;y++)
    {
        dmaDone=0;

        HAL_SPI_Transmit_DMA(
                TFT_SPI,
                (uint8_t*)lineBuffer,
                TFT_WIDTH*2);

        TFT_WaitDmaLine();
    }

    TFT_CS_HIGH();
}



/*=========================================================
Pixel test
=========================================================*/

void TFT_TestPixel(void)
{
    TFT_SetWindow(
            50,
            50,
            50,
            50);

    TFT_BeginGramWrite();

    uint8_t pixel[2];

    pixel[0]=0xF8;
    pixel[1]=0x00;

    HAL_SPI_Transmit(
            TFT_SPI,
            pixel,
            2,
            HAL_MAX_DELAY);

    TFT_CS_HIGH();
}
/* =========================================================
 * THÊM VÀO tft_service.c
 * (đặt sau các hàm TFT_SetWindow / TFT_BeginGramWrite hiện có)
 * ========================================================= */



/*=========================================================
  Ghi 1 pixel trực tiếp (không DMA, dùng cho hình nhỏ)
=========================================================*/

void TFT_DrawPixel(
        uint16_t x,
        uint16_t y,
        uint16_t color)
{
    if((x>=TFT_WIDTH)||(y>=TFT_HEIGHT))
    {
        return;
    }

    uint8_t data[2];

    /* swap byte giống TFT_FillScreen đang làm */
    data[0]=(color>>8);
    data[1]=(color&0xFF);

    TFT_SetWindow(x,y,x,y);

    TFT_BeginGramWrite();

    HAL_SPI_Transmit(
            TFT_SPI,
            data,
            2,
            HAL_MAX_DELAY);

    TFT_CS_HIGH();
}


/*=========================================================
  Line - Bresenham
=========================================================*/

void TFT_DrawLine(
        uint16_t x0,
        uint16_t y0,
        uint16_t x1,
        uint16_t y1,
        uint16_t color)
{
    int16_t dx = (int16_t)x1-(int16_t)x0;
    int16_t dy = (int16_t)y1-(int16_t)y0;

    int16_t sx = (dx>=0)?1:-1;
    int16_t sy = (dy>=0)?1:-1;

    dx=abs(dx);
    dy=abs(dy);

    int16_t err = (dx>dy)?dx:dy;

    int16_t x=x0;
    int16_t y=y0;

    int16_t ex=0;
    int16_t ey=0;

    for(;;)
    {
        TFT_DrawPixel((uint16_t)x,(uint16_t)y,color);

        if(x==(int16_t)x1 && y==(int16_t)y1)
        {
            break;
        }

        ex += dx;
        ey += dy;

        if(ex > err)
        {
            x += sx;
            ex -= err;
        }

        if(ey > err)
        {
            y += sy;
            ey -= err;
        }
    }
}


/*=========================================================
  Rectangle (viền)
=========================================================*/

void TFT_DrawRectangle(
        uint16_t x0,
        uint16_t y0,
        uint16_t x1,
        uint16_t y1,
        uint16_t color)
{
    TFT_DrawLine(x0,y0,x1,y0,color);
    TFT_DrawLine(x0,y1,x1,y1,color);
    TFT_DrawLine(x0,y0,x0,y1,color);
    TFT_DrawLine(x1,y0,x1,y1,color);
}


/*=========================================================
  Rectangle (đặc) - dùng buffer 1 dòng + DMA cho nhanh
=========================================================*/

void TFT_FillRectangle(
        uint16_t x0,
        uint16_t y0,
        uint16_t x1,
        uint16_t y1,
        uint16_t color)
{
    uint16_t xs = (x0<x1)?x0:x1;
    uint16_t xe = (x0<x1)?x1:x0;
    uint16_t ys = (y0<y1)?y0:y1;
    uint16_t ye = (y0<y1)?y1:y0;

    uint16_t w = xe-xs+1;
    uint16_t y;
    uint16_t i;

    uint16_t swapped = (color<<8)|(color>>8);

    /* dùng lineBuffer tĩnh sẵn có trong tft_service.c */
    for(i=0;i<w;i++)
    {
        lineBuffer[i]=swapped;
    }

#if (__DCACHE_PRESENT==1)
    SCB_CleanDCache_by_Addr(
            (uint32_t*)lineBuffer,
            w*sizeof(uint16_t));
#endif

    TFT_SetWindow(xs,ys,xe,ye);

    TFT_BeginGramWrite();

    for(y=ys;y<=ye;y++)
    {
        dmaDone=0;

        HAL_SPI_Transmit_DMA(
                TFT_SPI,
                (uint8_t*)lineBuffer,
                w*2);

        TFT_WaitDmaLine();
    }

    TFT_CS_HIGH();
}


/*=========================================================
  Rounded rectangle - viền và đặc (góc tròn bằng cung tròn)
=========================================================*/

static void TFT_DrawCircleHelper(
        uint16_t x0, uint16_t y0,
        uint16_t r,
        uint8_t cornermask,
        uint16_t color)
{
    int16_t f = 1-(int16_t)r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2*(int16_t)r;
    int16_t x = 0;
    int16_t y = r;

    while(x<y)
    {
        if(f>=0)
        {
            y--;
            ddF_y+=2;
            f+=ddF_y;
        }
        x++;
        ddF_x+=2;
        f+=ddF_x;

        if(cornermask & 0x1) /* top-right */
        {
            TFT_DrawPixel(x0+x,y0-y,color);
            TFT_DrawPixel(x0+y,y0-x,color);
        }
        if(cornermask & 0x2) /* top-left */
        {
            TFT_DrawPixel(x0-x,y0-y,color);
            TFT_DrawPixel(x0-y,y0-x,color);
        }
        if(cornermask & 0x4) /* bottom-right */
        {
            TFT_DrawPixel(x0+x,y0+y,color);
            TFT_DrawPixel(x0+y,y0+x,color);
        }
        if(cornermask & 0x8) /* bottom-left */
        {
            TFT_DrawPixel(x0-x,y0+y,color);
            TFT_DrawPixel(x0-y,y0+x,color);
        }
    }
}

void TFT_DrawRoundRect(
        uint16_t x0, uint16_t y0,
        uint16_t x1, uint16_t y1,
        uint16_t radius,
        uint16_t color)
{
    uint16_t w = x1-x0+1;
    uint16_t h = y1-y0+1;

    TFT_DrawLine(x0+radius, y0, x1-radius, y0, color);          /* top    */
    TFT_DrawLine(x0+radius, y1, x1-radius, y1, color);          /* bottom */
    TFT_DrawLine(x0, y0+radius, x0, y1-radius, color);          /* left   */
    TFT_DrawLine(x1, y0+radius, x1, y1-radius, color);          /* right  */

    (void)w;(void)h;

    TFT_DrawCircleHelper(x0+radius, y0+radius, radius, 0x2, color); /* top-left  */
    TFT_DrawCircleHelper(x1-radius, y0+radius, radius, 0x1, color); /* top-right */
    TFT_DrawCircleHelper(x0+radius, y1-radius, radius, 0x8, color); /* bot-left  */
    TFT_DrawCircleHelper(x1-radius, y1-radius, radius, 0x4, color); /* bot-right */
}

static void TFT_FillCircleHelper(
        uint16_t x0, uint16_t y0,
        uint16_t r,
        uint8_t cornermask,
        int16_t delta,
        uint16_t color)
{
    int16_t f = 1-(int16_t)r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2*(int16_t)r;
    int16_t x = 0;
    int16_t y = r;

    while(x<y)
    {
        if(f>=0)
        {
            y--;
            ddF_y+=2;
            f+=ddF_y;
        }
        x++;
        ddF_x+=2;
        f+=ddF_x;

        if(cornermask & 0x1)
        {
            TFT_DrawLine(x0+x, y0-y, x0+x, y0+y+delta, color);
            TFT_DrawLine(x0+y, y0-x, x0+y, y0+x+delta, color);
        }
        if(cornermask & 0x2)
        {
            TFT_DrawLine(x0-x, y0-y, x0-x, y0+y+delta, color);
            TFT_DrawLine(x0-y, y0-x, x0-y, y0+x+delta, color);
        }
    }
}

void TFT_FillRoundRect(
        uint16_t x0, uint16_t y0,
        uint16_t x1, uint16_t y1,
        uint16_t radius,
        uint16_t color)
{
    uint16_t h = y1-y0+1;

    TFT_FillRectangle(x0+radius, y0, x1-radius, y1, color);

    TFT_FillCircleHelper(x1-radius, y0+radius, radius, 0x1,
            (int16_t)h-2*(int16_t)radius-1, color);
    TFT_FillCircleHelper(x0+radius, y0+radius, radius, 0x2,
            (int16_t)h-2*(int16_t)radius-1, color);
}


/*=========================================================
  Circle - midpoint algorithm
=========================================================*/

void TFT_DrawCircle(
        uint16_t x0,
        uint16_t y0,
        uint16_t radius,
        uint16_t color)
{
    int16_t f = 1-(int16_t)radius;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2*(int16_t)radius;
    int16_t x = 0;
    int16_t y = radius;

    TFT_DrawPixel(x0, y0+radius, color);
    TFT_DrawPixel(x0, y0-radius, color);
    TFT_DrawPixel(x0+radius, y0, color);
    TFT_DrawPixel(x0-radius, y0, color);

    while(x<y)
    {
        if(f>=0)
        {
            y--;
            ddF_y+=2;
            f+=ddF_y;
        }
        x++;
        ddF_x+=2;
        f+=ddF_x;

        TFT_DrawPixel(x0+x, y0+y, color);
        TFT_DrawPixel(x0-x, y0+y, color);
        TFT_DrawPixel(x0+x, y0-y, color);
        TFT_DrawPixel(x0-x, y0-y, color);
        TFT_DrawPixel(x0+y, y0+x, color);
        TFT_DrawPixel(x0-y, y0+x, color);
        TFT_DrawPixel(x0+y, y0-x, color);
        TFT_DrawPixel(x0-y, y0-x, color);
    }
}

void TFT_FillCircle(
        uint16_t x0,
        uint16_t y0,
        uint16_t radius,
        uint16_t color)
{
    TFT_DrawLine(x0, y0-radius, x0, y0+radius, color);

    TFT_FillCircleHelper(x0, y0, radius, 0x3, 0, color);
}


/* =========================================================
 * FILL CIRCLE NHANH - dung DMA theo tung dong thay vi ve tung
 * pixel/line rieng le.
 *
 * TFT_FillCircle() ban goc dung TFT_DrawLine (ve tung pixel qua
 * TFT_DrawPixel, moi pixel 1 lan SetWindow+SPI_Transmit rieng) lap
 * lai nhieu lan qua TFT_FillCircleHelper - voi ban kinh vai chuc
 * pixel se ton HANG NGAN giao dich SPI rieng le, rat cham va de
 * gay nhay/giat khi ve (dac biet man hinh STOP thay doi toan bo
 * ngay khi vao, khong co dirty-update nao giup duoc o day).
 *
 * TFT_FillCircleFast() tinh san moi "dong ngang" cua hinh tron
 * (dua vao phuong trinh duong tron, x^2+y^2<=r^2, chi dung so
 * nguyen - khong can <math.h>/sqrt), dung 1 buffer dong (lineBuffer
 * co san) roi GUI 1 LAN DMA CHO CA DONG - giam tu hang ngan giao
 * dich xuong con (2*r+1) lan DMA (vd r=50 -> 101 lan, thay vi hang
 * ngan lan).
 *
 * Yeu cau: vung ben ngoai hinh tron nhung nam TRONG hinh vuong bao
 * (bounding box) se bi ve DE bang bgColor - chi dung ham nay khi
 * nen xung quanh la mau dac biet trong (vd sau TFT_FillScreen()),
 * KHONG dung cho hinh tron ve chong len chi tiet khac (vd bong ban
 * ping-pong de len duong crosshair) vi se de mat chi tiet do.
 * ========================================================= */
bool TFT_FillCircleFast(
        uint16_t cx, uint16_t cy,
        uint16_t radius,
        uint16_t color,
        uint16_t bgColor)
{
    if (radius == 0)
    {
        return true;
    }

    uint16_t diameter = 2*radius + 1;

    if ((cx < radius) || (cy < radius) ||
        ((uint32_t)cx + radius >= TFT_WIDTH) ||
        ((uint32_t)cy + radius >= TFT_HEIGHT) ||
        (diameter > TFT_WIDTH))
    {
        /* vuot man hinh hoac vuot buffer dong - fallback ve ham cu
         * (cham hon nhung an toan, khong tran bo nho) */
        TFT_FillCircle(cx, cy, radius, color);
        return false;
    }

    uint16_t x0 = cx - radius;
    uint16_t y0 = cy - radius;

    uint16_t colorSwapped = (color<<8)|(color>>8);
    uint16_t bgSwapped    = (bgColor<<8)|(bgColor>>8);

    TFT_SetWindow(x0, y0, x0+diameter-1, y0+diameter-1);
    TFT_BeginGramWrite();

    int32_t r2 = (int32_t)radius * (int32_t)radius;

    for (int16_t dy = -(int16_t)radius; dy <= (int16_t)radius; dy++)
    {
        int32_t dy2 = (int32_t)dy * (int32_t)dy;

        /* tim dx lon nhat sao cho dx^2+dy^2 <= r^2 - vong lap don
         * gian dung so nguyen, tong chi phi ca vong ngoai ~O(r^2),
         * qua nho so voi thoi gian truyen SPI nen khong dang lo */
        int16_t dxMax = 0;
        while (((int32_t)(dxMax+1)*(dxMax+1) + dy2) <= r2)
        {
            dxMax++;
        }

        uint16_t left  = (uint16_t)(radius - dxMax);
        uint16_t right = (uint16_t)(radius + dxMax);

        for (uint16_t i = 0; i < diameter; i++)
        {
            lineBuffer[i] = ((i >= left) && (i <= right)) ? colorSwapped : bgSwapped;
        }

#if (__DCACHE_PRESENT==1)
        SCB_CleanDCache_by_Addr((uint32_t*)lineBuffer, diameter*sizeof(uint16_t));
#endif

        dmaDone = 0;

        HAL_SPI_Transmit_DMA(
                TFT_SPI,
                (uint8_t*)lineBuffer,
                diameter*2);

        TFT_WaitDmaLine();
    }

    TFT_CS_HIGH();

    return true;
}


/*=========================================================
  FONT 5x7 (chuẩn GLCD, mỗi ký tự 5 byte, bit0=trên)
  Chỉ gồm ký tự thường dùng cho UI: space, số, hoa, thường,
  thêm vào nếu cần ký tự khác.
=========================================================*/


static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' 0x20 */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '[' (dùng tạm hình '(' ) */
    {0x02,0x04,0x08,0x10,0x20}, /* '\' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ']' (dùng tạm hình ')' ) */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */

    /* ---- chữ thường a-z: dùng lại glyph chữ hoa tương ứng ---- */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'a' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'b' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'c' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'd' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'e' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'f' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'g' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'h' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'i' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'j' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'k' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'l' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'm' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'n' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'o' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'p' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'r' */
    {0x46,0x49,0x49,0x49,0x31}, /* 's' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 't' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'u' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'v' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'w' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'x' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'z' */
};

#define FONT_FIRST_CHAR 0x20
#define FONT_LAST_CHAR  0x7A  /* 'z' - đã mở rộng từ 0x5A */
#define FONT_W 5
#define FONT_H 7

/*=========================================================
  Vẽ 1 ký tự, scale = phóng to nguyên lần (1,2,3...)
=========================================================*/

void TFT_DrawChar(
        uint16_t x, uint16_t y,
        char c,
        uint16_t fgColor,
        uint16_t bgColor,
        uint8_t scale)
{
    if(scale==0)
    {
        scale=1;
    }

    if((c<FONT_FIRST_CHAR)||(c>FONT_LAST_CHAR))
    {
        c=' ';
    }

    const uint8_t *glyph = font5x7[c-FONT_FIRST_CHAR];

    for(uint8_t col=0; col<FONT_W; col++)
    {
        uint8_t line = glyph[col];

        for(uint8_t row=0; row<FONT_H; row++)
        {
            uint16_t color = (line & (1<<row)) ? fgColor : bgColor;

            if(scale==1)
            {
                TFT_DrawPixel(x+col, y+row, color);
            }
            else
            {
                TFT_FillRectangle(
                        x+col*scale,
                        y+row*scale,
                        x+col*scale+scale-1,
                        y+row*scale+scale-1,
                        color);
            }
        }
    }

    /* cột đệm giữa ký tự (bg) */
    if(scale==1)
    {
        TFT_DrawLine(x+FONT_W, y, x+FONT_W, y+FONT_H-1, bgColor);
    }
    else
    {
        TFT_FillRectangle(
                x+FONT_W*scale,
                y,
                x+FONT_W*scale+scale-1,
                y+FONT_H*scale-1,
                bgColor);
    }
}


void TFT_DrawText(
        uint16_t x, uint16_t y,
        const char *str,
        uint16_t fgColor,
        uint16_t bgColor,
        uint8_t scale)
{
    if(scale==0)
    {
        scale=1;
    }

    uint16_t cx = x;

    while(*str)
    {
        TFT_DrawChar(cx, y, *str, fgColor, bgColor, scale);

        cx += (FONT_W+1)*scale;

        str++;
    }
}


void TFT_GetTextExtent(
        const char *str,
        uint8_t scale,
        uint16_t *w,
        uint16_t *h)
{
    if(scale==0)
    {
        scale=1;
    }

    uint16_t len = 0;

    while(str[len]!='\0')
    {
        len++;
    }

    if(w) { *w = len*(FONT_W+1)*scale; }
    if(h) { *h = FONT_H*scale; }
}


/* =========================================================
 * GIAI DOAN 2 - FONT SPRITE + 1 LAN DMA
 *
 * TFT_DrawChar/TFT_DrawText (ban goc, van giu nguyen o tren, dung
 * lam fallback) ve tung pixel (scale=1) hoac tung o (scale>1) bang
 * TFT_DrawPixel/TFT_FillRectangle rieng le - moi ky tu scale=1 ton
 * toi 35 giao dich SPI (SetWindow+Transmit) tach roi nhau. Vi
 * ILI9225 khong co chan TE/VSYNC dong bo voi MCU, thoi gian ve keo
 * dai kieu nay lam "lo" trang thai ve dang do ra man hinh -> nhay.
 *
 * TFT_DrawTextFast() dung 1 buffer RAM (sprite) de dung san CA
 * CHUOI ky tu (gom ca cot dem giua chu = mau nen), roi chi
 * TFT_SetWindow() + 1 LAN DMA DUY NHAT cho toan bo chuoi - giam so
 * giao dich SPI tu ~35*len xuong con 1 lan transmit.
 *
 * Gioi han: buffer sprite kich thuoc co dinh
 * TEXT_SPRITE_MAX_W x TEXT_SPRITE_MAX_H (du cho toan bo label/
 * value-field dang dung trong UI hien tai, TFT_WIDTH ngang x
 * FONT_H*3 cao - du du cho scale 1..3). Neu chuoi/toa do vuot qua
 * (vd goi voi scale qua lon hoac x+w > TFT_WIDTH), ham se KHONG
 * tran buffer - tu dong fallback ve TFT_DrawText() (cham hon nhung
 * an toan) va tra ve false de noi goi biet neu can kiem tra.
 * ========================================================= */

#define TEXT_SPRITE_MAX_W   TFT_WIDTH
#define TEXT_SPRITE_MAX_H   (FONT_H*3)

/* =========================================================
 * FIX LAT/NGUOC CHU
 *
 * TFT_FillRectangle dung MAU DONG NHAT cho ca dong nen du thu tu
 * auto-increment that cua GRAM (quyet dinh boi thanh ghi Entry Mode
 * 0x03 = 0x1038 trong TFT_Init(), phoi hop voi phep xoay 90 do trong
 * TFT_SetWindow) co bi dao nguoc so voi gia dinh ban dau, anh van
 * dung (moi pixel cung mau). TFT_DrawTextFast ghi du lieu KHAC NHAU
 * theo tung pixel nen thu tu sai se lo ra ngay - day chinh la ly do
 * chu bi nguoc/lat khi test tren panel that.
 *
 * Thay vi phai doc datasheet de tinh chinh xac AM/ID bit, dung 1 co
 * co the thu-sai truc tiep tren board that: doi so duoi day roi build
 * lai, thu tuan tu 0 -> 1 -> 2 -> 3 cho den khi chu hien DUNG CHIEU:
 *   0 = khong lat gi ca
 *   1 = chi lat NGANG (trai-phai) - DA XAC NHAN DUNG tren board that
 *   2 = chi lat DOC (tren-duoi)
 *   3 = lat CA HAI (xoay 180 do)
 * ========================================================= */
#define TEXT_SPRITE_FLIP_MODE   1

static uint16_t textSpriteBuf[TEXT_SPRITE_MAX_W * TEXT_SPRITE_MAX_H]
__attribute__((aligned(32)));

bool TFT_DrawTextFast(
        uint16_t x, uint16_t y,
        const char *str,
        uint16_t fgColor,
        uint16_t bgColor,
        uint8_t scale)
{
    if (scale == 0)
    {
        scale = 1;
    }

    uint16_t len = 0;
    while (str[len] != '\0')
    {
        len++;
    }

    if (len == 0)
    {
        return true; /* chuoi rong - khong co gi de ve */
    }

    uint16_t charAdvance = (FONT_W + 1) * scale;
    uint16_t w = len * charAdvance;
    uint16_t h = FONT_H * scale;

    if ((w > TEXT_SPRITE_MAX_W) || (h > TEXT_SPRITE_MAX_H) ||
        ((uint32_t)x + w > TFT_WIDTH) || ((uint32_t)y + h > TFT_HEIGHT))
    {
        /* vuot buffer/man hinh - fallback an toan, khong tran bo nho */
        TFT_DrawText(x, y, str, fgColor, bgColor, scale);
        return false;
    }

#if   (TEXT_SPRITE_FLIP_MODE == 1)
    #define SPRITE_R(r)  (r)
    #define SPRITE_C(c)  ((w-1)-(c))
#elif (TEXT_SPRITE_FLIP_MODE == 2)
    #define SPRITE_R(r)  ((h-1)-(r))
    #define SPRITE_C(c)  (c)
#elif (TEXT_SPRITE_FLIP_MODE == 3)
    #define SPRITE_R(r)  ((h-1)-(r))
    #define SPRITE_C(c)  ((w-1)-(c))
#else
    #define SPRITE_R(r)  (r)
    #define SPRITE_C(c)  (c)
#endif

    uint16_t fgSwapped = (fgColor << 8) | (fgColor >> 8);
    uint16_t bgSwapped = (bgColor << 8) | (bgColor >> 8);

    for (uint16_t ci = 0; ci < len; ci++)
    {
        char c = str[ci];
        if ((c < FONT_FIRST_CHAR) || (c > FONT_LAST_CHAR))
        {
            c = ' ';
        }
        const uint8_t *glyph = font5x7[c - FONT_FIRST_CHAR];

        uint16_t colBase = ci * charAdvance;

        for (uint8_t col = 0; col < FONT_W; col++)
        {
            uint8_t line = glyph[col];

            for (uint8_t row = 0; row < FONT_H; row++)
            {
                uint16_t px = (line & (1 << row)) ? fgSwapped : bgSwapped;

                for (uint8_t sy = 0; sy < scale; sy++)
                {
                    uint16_t rowIdx = row * scale + sy;

                    for (uint8_t sx = 0; sx < scale; sx++)
                    {
                        uint16_t colIdx = colBase + col * scale + sx;
                        textSpriteBuf[(uint32_t)SPRITE_R(rowIdx) * w + SPRITE_C(colIdx)] = px;
                    }
                }
            }
        }

        /* cot dem giua ky tu - luon la mau nen, tren toan bo chieu cao */
        for (uint16_t row = 0; row < h; row++)
        {
            for (uint8_t sx = 0; sx < scale; sx++)
            {
                uint16_t colIdx = colBase + FONT_W * scale + sx;
                textSpriteBuf[(uint32_t)SPRITE_R(row) * w + SPRITE_C(colIdx)] = bgSwapped;
            }
        }
    }

#undef SPRITE_R
#undef SPRITE_C

#if (__DCACHE_PRESENT==1)
    SCB_CleanDCache_by_Addr(
            (uint32_t*)textSpriteBuf,
            (uint32_t)w * h * sizeof(uint16_t));
#endif

    TFT_SetWindow(x, y, x + w - 1, y + h - 1);
    TFT_BeginGramWrite();

    dmaDone = 0;

    HAL_SPI_Transmit_DMA(
            TFT_SPI,
            (uint8_t*)textSpriteBuf,
            (uint32_t)w * h * 2);

    TFT_WaitDmaLine();

    TFT_CS_HIGH();

    return true;
}
