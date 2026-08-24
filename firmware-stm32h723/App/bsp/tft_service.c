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

volatile uint8_t dmaDone = 0;

/* ------------------------------------------------------------------
 * SPI DMA completion synchronization.
 *
 * TFT rendering functions start an SPI DMA transfer and then wait
 * for the corresponding completion event before reusing the shared
 * DMA buffer.
 *
 * A semaphore is used instead of busy-waiting so that the rendering
 * task leaves the Ready state while waiting for DMA completion.
 * This prevents the display task from unnecessarily consuming CPU
 * time or starving lower-priority RTOS services.
 *
 * The completion callback executes in interrupt context. The native
 * FreeRTOS xSemaphoreGiveFromISR() API is therefore used directly
 * instead of the CMSIS-RTOS wrapper.
 *
 * portYIELD_FROM_ISR() allows an awakened higher-priority task to
 * resume immediately after the DMA interrupt when required.
 * ------------------------------------------------------------------ */
static osSemaphoreId_t s_tftDmaSem;

static osSemaphoreAttr_t s_tftDmaSemAttr =
{
    .name = "tftDmaSem"
};

/**
 * @brief Ensure that the TFT DMA completion semaphore exists.
 *
 * The semaphore is created lazily as a defensive measure so that
 * DMA-based drawing functions remain safe even if they are called
 * before TFT_Init().
 */
static inline void TFT_EnsureDmaSemCreated(void)
{
    if (s_tftDmaSem == NULL)
    {
        s_tftDmaSem =
            osSemaphoreNew(
                1,
                0,
                &s_tftDmaSemAttr
            );
    }
}

/**
 * @brief Wait for completion of one SPI DMA transfer.
 *
 * A finite timeout prevents the calling task from remaining blocked
 * indefinitely if the SPI peripheral or DMA controller fails.
 */
static inline void TFT_WaitDmaLine(void)
{
    TFT_EnsureDmaSemCreated();

    osSemaphoreAcquire(
        s_tftDmaSem,
        50
    );
}

/*
 * Shared line buffer used by DMA-based rendering functions.
 *
 * The buffer is aligned to a 32-byte Cortex-M7 cache line because
 * SPI DMA transfers require explicit D-Cache maintenance on systems
 * where the data cache is enabled.
 */
static uint16_t lineBuffer[TFT_WIDTH]
__attribute__((aligned(32)));


/* ------------------------------------------------------------------
 * Display control signals
 * ------------------------------------------------------------------ */

static inline void TFT_CS_LOW(void)
{
    HAL_GPIO_WritePin(
        TFT_CS_GPIO_Port,
        TFT_CS_Pin,
        GPIO_PIN_RESET
    );
}

static inline void TFT_CS_HIGH(void)
{
    HAL_GPIO_WritePin(
        TFT_CS_GPIO_Port,
        TFT_CS_Pin,
        GPIO_PIN_SET
    );
}

static inline void TFT_DC_LOW(void)
{
    HAL_GPIO_WritePin(
        TFT_DC_GPIO_Port,
        TFT_DC_Pin,
        GPIO_PIN_RESET
    );
}

static inline void TFT_DC_HIGH(void)
{
    HAL_GPIO_WritePin(
        TFT_DC_GPIO_Port,
        TFT_DC_Pin,
        GPIO_PIN_SET
    );
}

static inline void TFT_RST_LOW(void)
{
    HAL_GPIO_WritePin(
        TFT_RST_GPIO_Port,
        TFT_RST_Pin,
        GPIO_PIN_RESET
    );
}

static inline void TFT_RST_HIGH(void)
{
    HAL_GPIO_WritePin(
        TFT_RST_GPIO_Port,
        TFT_RST_Pin,
        GPIO_PIN_SET
    );
}


/* ------------------------------------------------------------------
 * SPI DMA completion callback
 * ------------------------------------------------------------------ */

/**
 * @brief Handle completion of an SPI DMA transfer for the TFT.
 *
 * This callback executes in interrupt context.
 *
 * The DMA completion semaphore is therefore released through the
 * ISR-safe FreeRTOS API. If a higher-priority task is waiting for
 * the transfer, the scheduler is requested to switch to it
 * immediately.
 */
void HAL_SPI_TxCpltCallback(
    SPI_HandleTypeDef *hspi)
{
    if (hspi == TFT_SPI)
    {
        dmaDone = 1;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR(
            (SemaphoreHandle_t)s_tftDmaSem,
            &xHigherPriorityTaskWoken
        );

        portYIELD_FROM_ISR(
            xHigherPriorityTaskWoken
        );
    }
}


/* ------------------------------------------------------------------
 * ILI9225 register write
 * ------------------------------------------------------------------ */

static void TFT_WriteRegister(
    uint8_t reg,
    uint16_t value)
{
    uint8_t cmd[2];
    uint8_t data[2];

    cmd[0] = 0;
    cmd[1] = reg;

    data[0] = (value >> 8);
    data[1] = (value & 0xFF);

    TFT_CS_LOW();

    TFT_DC_LOW();

    HAL_SPI_Transmit(
        TFT_SPI,
        cmd,
        2,
        HAL_MAX_DELAY
    );

    TFT_DC_HIGH();

    HAL_SPI_Transmit(
        TFT_SPI,
        data,
        2,
        HAL_MAX_DELAY
    );

    TFT_CS_HIGH();
}


/* ------------------------------------------------------------------
 * GRAM write
 *
 * Starts a sequential write transaction to the ILI9225 display RAM.
 * The caller must configure the active address window before starting
 * the GRAM write.
 * ------------------------------------------------------------------ */

static void TFT_BeginGramWrite(void)
{
    uint8_t cmd[2];

    cmd[0] = 0;
    cmd[1] = ILI9225_GRAM_WRITE;

    TFT_CS_LOW();

    TFT_DC_LOW();

    HAL_SPI_Transmit(
        TFT_SPI,
        cmd,
        2,
        HAL_MAX_DELAY
    );

    TFT_DC_HIGH();
}


/* ------------------------------------------------------------------
 * Display address window
 *
 * The application uses a logical 220 x 176 coordinate system while
 * the physical ILI9225 panel is 176 x 220 pixels.
 *
 * The current display orientation applies a 90-degree coordinate
 * transformation:
 *
 *     physical_x = logical_y
 *     physical_y = (physical_height - 1) - logical_x
 *
 * The transformed coordinates are written to the ILI9225 GRAM
 * address registers.
 * ------------------------------------------------------------------ */

void TFT_SetWindow(
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1)
{
    uint16_t px0 = y0;
    uint16_t px1 = y1;

    uint16_t py0 =
        (TFT_PHYS_HEIGHT - 1) - x1;

    uint16_t py1 =
        (TFT_PHYS_HEIGHT - 1) - x0;

    TFT_WriteRegister(
        0x36,
        px1
    );

    TFT_WriteRegister(
        0x37,
        px0
    );

    TFT_WriteRegister(
        0x38,
        py1
    );

    TFT_WriteRegister(
        0x39,
        py0
    );

    TFT_WriteRegister(
        0x20,
        px0
    );

    TFT_WriteRegister(
        0x21,
        py0
    );
}


/* ------------------------------------------------------------------
 * ILI9225 initialization
 * ------------------------------------------------------------------ */

/**
 * @brief Initialize the ILI9225 display controller.
 *
 * Performs the required hardware reset sequence and applies the
 * controller configuration used by the current panel.
 *
 * The register configuration establishes the required power,
 * timing, entry-mode and display-operation settings.
 */
void TFT_Init(void)
{
    TFT_RST_HIGH();

    HAL_Delay(1);

    TFT_RST_LOW();

    HAL_Delay(50);

    TFT_RST_HIGH();

    HAL_Delay(150);

    TFT_WriteRegister(
        0x10,
        0x0000
    );

    TFT_WriteRegister(
        0x11,
        0x0000
    );

    TFT_WriteRegister(
        0x12,
        0x0000
    );

    TFT_WriteRegister(
        0x13,
        0x0000
    );

    TFT_WriteRegister(
        0x14,
        0x0000
    );

    HAL_Delay(40);

    TFT_WriteRegister(
        0x11,
        0x0018
    );

    TFT_WriteRegister(
        0x12,
        0x6121
    );

    TFT_WriteRegister(
        0x13,
        0x006F
    );

    TFT_WriteRegister(
        0x14,
        0x495F
    );

    TFT_WriteRegister(
        0x10,
        0x0800
    );

    HAL_Delay(10);

    TFT_WriteRegister(
        0x11,
        0x103B
    );

    HAL_Delay(50);

    TFT_WriteRegister(
        0x01,
        0x011C
    );

    TFT_WriteRegister(
        0x02,
        0x0100
    );

    TFT_WriteRegister(
        0x03,
        0x1038
    );

    TFT_WriteRegister(
        0x07,
        0x0012
    );

    HAL_Delay(50);

    TFT_WriteRegister(
        0x07,
        0x1017
    );

    HAL_Delay(50);
}


/* ------------------------------------------------------------------
 * Full-screen fill
 *
 * A single line buffer is reused for every display row.
 *
 * The CPU prepares one complete row, cleans the corresponding
 * D-Cache region, and then transfers the row through SPI DMA.
 * ------------------------------------------------------------------ */

void TFT_FillScreen(
    uint16_t color)
{
    uint32_t y;
    uint32_t i;

    color =
        (color << 8) |
        (color >> 8);

    for (i = 0; i < TFT_WIDTH; i++)
    {
        lineBuffer[i] = color;
    }

#if (__DCACHE_PRESENT == 1)

    /*
     * The line buffer is written by the CPU and consumed by SPI DMA.
     * Clean the D-Cache so that DMA reads the latest buffer contents.
     */
    SCB_CleanDCache_by_Addr(
        (uint32_t *)lineBuffer,
        sizeof(lineBuffer)
    );

#endif

    TFT_SetWindow(
        0,
        0,
        TFT_WIDTH - 1,
        TFT_HEIGHT - 1
    );

    TFT_BeginGramWrite();

    for (y = 0; y < TFT_HEIGHT; y++)
    {
        dmaDone = 0;

        HAL_SPI_Transmit_DMA(
            TFT_SPI,
            (uint8_t *)lineBuffer,
            TFT_WIDTH * 2
        );

        TFT_WaitDmaLine();
    }

    TFT_CS_HIGH();
}


/* ------------------------------------------------------------------
 * Low-level display communication diagnostic
 * ------------------------------------------------------------------ */

/**
 * @brief Draw one red diagnostic pixel at a fixed location.
 *
 * This function is intended for low-level display bring-up and SPI
 * communication verification.
 */
void TFT_TestPixel(void)
{
    TFT_SetWindow(
        50,
        50,
        50,
        50
    );

    TFT_BeginGramWrite();

    uint8_t pixel[2];

    pixel[0] = 0xF8;
    pixel[1] = 0x00;

    HAL_SPI_Transmit(
        TFT_SPI,
        pixel,
        2,
        HAL_MAX_DELAY
    );

    TFT_CS_HIGH();
}


/* ------------------------------------------------------------------
 * Basic graphics primitives
 * ------------------------------------------------------------------ */

/**
 * @brief Draw one pixel directly to the display.
 *
 * This primitive uses a blocking SPI transfer and is intended for
 * small or infrequent drawing operations.
 *
 * Coordinates outside the logical display area are ignored.
 */
void TFT_DrawPixel(
    uint16_t x,
    uint16_t y,
    uint16_t color)
{
    if ((x >= TFT_WIDTH) ||
        (y >= TFT_HEIGHT))
    {
        return;
    }

    uint8_t data[2];

    data[0] = (color >> 8);
    data[1] = (color & 0xFF);

    TFT_SetWindow(
        x,
        y,
        x,
        y
    );

    TFT_BeginGramWrite();

    HAL_SPI_Transmit(
        TFT_SPI,
        data,
        2,
        HAL_MAX_DELAY
    );

    TFT_CS_HIGH();
}


/**
 * @brief Draw a line using Bresenham rasterization.
 *
 * The algorithm uses integer arithmetic and therefore avoids
 * floating-point operations in the pixel-generation path.
 */
void TFT_DrawLine(
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    uint16_t color)
{
    int16_t dx =
        (int16_t)x1 -
        (int16_t)x0;

    int16_t dy =
        (int16_t)y1 -
        (int16_t)y0;

    int16_t sx =
        (dx >= 0) ? 1 : -1;

    int16_t sy =
        (dy >= 0) ? 1 : -1;

    dx = abs(dx);
    dy = abs(dy);

    int16_t err =
        (dx > dy) ? dx : dy;

    int16_t x = x0;
    int16_t y = y0;

    int16_t ex = 0;
    int16_t ey = 0;

    for (;;)
    {
        TFT_DrawPixel(
            (uint16_t)x,
            (uint16_t)y,
            color
        );

        if ((x == (int16_t)x1) &&
            (y == (int16_t)y1))
        {
            break;
        }

        ex += dx;
        ey += dy;

        if (ex > err)
        {
            x += sx;
            ex -= err;
        }

        if (ey > err)
        {
            y += sy;
            ey -= err;
        }
    }
}


/**
 * @brief Draw a rectangle outline.
 *
 * The rectangle edges are rendered using the Bresenham line
 * primitive.
 */
void TFT_DrawRectangle(
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    uint16_t color)
{
    TFT_DrawLine(
        x0,
        y0,
        x1,
        y0,
        color
    );

    TFT_DrawLine(
        x0,
        y1,
        x1,
        y1,
        color
    );

    TFT_DrawLine(
        x0,
        y0,
        x0,
        y1,
        color
    );

    TFT_DrawLine(
        x1,
        y0,
        x1,
        y1,
        color
    );
}


/**
 * @brief Draw a filled rectangle using a reusable DMA line buffer.
 *
 * The rectangle is rendered one horizontal row at a time. The shared
 * line buffer avoids allocating memory proportional to the complete
 * rectangle area.
 *
 * D-Cache maintenance is performed before DMA reads the buffer.
 */
void TFT_FillRectangle(
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    uint16_t color)
{
    uint16_t xs =
        (x0 < x1) ? x0 : x1;

    uint16_t xe =
        (x0 < x1) ? x1 : x0;

    uint16_t ys =
        (y0 < y1) ? y0 : y1;

    uint16_t ye =
        (y0 < y1) ? y1 : y0;

    uint16_t w =
        xe - xs + 1;

    uint16_t y;
    uint16_t i;

    uint16_t swapped =
        (color << 8) |
        (color >> 8);

    /*
     * Prepare one complete horizontal row.
     */
    for (i = 0; i < w; i++)
    {
        lineBuffer[i] = swapped;
    }

#if (__DCACHE_PRESENT == 1)

    /*
     * Ensure that SPI DMA sees the CPU-written pixel data.
     */
    SCB_CleanDCache_by_Addr(
        (uint32_t *)lineBuffer,
        w * sizeof(uint16_t)
    );

#endif

    TFT_SetWindow(
        xs,
        ys,
        xe,
        ye
    );

    TFT_BeginGramWrite();

    for (y = ys; y <= ye; y++)
    {
        dmaDone = 0;

        HAL_SPI_Transmit_DMA(
            TFT_SPI,
            (uint8_t *)lineBuffer,
            w * 2
        );

        TFT_WaitDmaLine();
    }

    TFT_CS_HIGH();
}

/*=========================================================
 * Rounded rectangle rendering
 *
 * The corner geometry is generated using the midpoint-circle
 * algorithm. A bit mask selects which corner arcs are rendered.
 *=========================================================*/

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

        if(cornermask & 0x1) /* Top-right corner */
        {
            TFT_DrawPixel(x0+x,y0-y,color);
            TFT_DrawPixel(x0+y,y0-x,color);
        }

        if(cornermask & 0x2) /* Top-left corner */
        {
            TFT_DrawPixel(x0-x,y0-y,color);
            TFT_DrawPixel(x0-y,y0-x,color);
        }

        if(cornermask & 0x4) /* Bottom-right corner */
        {
            TFT_DrawPixel(x0+x,y0+y,color);
            TFT_DrawPixel(x0+y,y0+x,color);
        }

        if(cornermask & 0x8) /* Bottom-left corner */
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

    TFT_DrawLine(
            x0+radius,
            y0,
            x1-radius,
            y0,
            color);          /* Top */

    TFT_DrawLine(
            x0+radius,
            y1,
            x1-radius,
            y1,
            color);          /* Bottom */

    TFT_DrawLine(
            x0,
            y0+radius,
            x0,
            y1-radius,
            color);          /* Left */

    TFT_DrawLine(
            x1,
            y0+radius,
            x1,
            y1-radius,
            color);          /* Right */

    (void)w;
    (void)h;

    TFT_DrawCircleHelper(
            x0+radius,
            y0+radius,
            radius,
            0x2,
            color);          /* Top-left */

    TFT_DrawCircleHelper(
            x1-radius,
            y0+radius,
            radius,
            0x1,
            color);          /* Top-right */

    TFT_DrawCircleHelper(
            x0+radius,
            y1-radius,
            radius,
            0x8,
            color);          /* Bottom-left */

    TFT_DrawCircleHelper(
            x1-radius,
            y1-radius,
            radius,
            0x4,
            color);          /* Bottom-right */
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
            TFT_DrawLine(
                    x0+x,
                    y0-y,
                    x0+x,
                    y0+y+delta,
                    color);

            TFT_DrawLine(
                    x0+y,
                    y0-x,
                    x0+y,
                    y0+x+delta,
                    color);
        }

        if(cornermask & 0x2)
        {
            TFT_DrawLine(
                    x0-x,
                    y0-y,
                    x0-x,
                    y0+y+delta,
                    color);

            TFT_DrawLine(
                    x0-y,
                    y0-x,
                    x0-y,
                    y0+x+delta,
                    color);
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

    /*
     * Fill the central rectangular region first. The corner helpers
     * then extend the fill into the rounded end regions.
     */
    TFT_FillRectangle(
            x0+radius,
            y0,
            x1-radius,
            y1,
            color);

    TFT_FillCircleHelper(
            x1-radius,
            y0+radius,
            radius,
            0x1,
            (int16_t)h-2*(int16_t)radius-1,
            color);

    TFT_FillCircleHelper(
            x0+radius,
            y0+radius,
            radius,
            0x2,
            (int16_t)h-2*(int16_t)radius-1,
            color);
}


/*=========================================================
 * Circle rendering
 *
 * Uses the integer midpoint-circle algorithm to avoid
 * floating-point operations in the rasterization path.
 *=========================================================*/

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

    TFT_DrawPixel(
            x0,
            y0+radius,
            color);

    TFT_DrawPixel(
            x0,
            y0-radius,
            color);

    TFT_DrawPixel(
            x0+radius,
            y0,
            color);

    TFT_DrawPixel(
            x0-radius,
            y0,
            color);

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

        TFT_DrawPixel(
                x0+x,
                y0+y,
                color);

        TFT_DrawPixel(
                x0-x,
                y0+y,
                color);

        TFT_DrawPixel(
                x0+x,
                y0-y,
                color);

        TFT_DrawPixel(
                x0-x,
                y0-y,
                color);

        TFT_DrawPixel(
                x0+y,
                y0+x,
                color);

        TFT_DrawPixel(
                x0-y,
                y0+x,
                color);

        TFT_DrawPixel(
                x0+y,
                y0-x,
                color);

        TFT_DrawPixel(
                x0-y,
                y0-x,
                color);
    }
}


void TFT_FillCircle(
        uint16_t x0,
        uint16_t y0,
        uint16_t radius,
        uint16_t color)
{
    TFT_DrawLine(
            x0,
            y0-radius,
            x0,
            y0+radius,
            color);

    TFT_FillCircleHelper(
            x0,
            y0,
            radius,
            0x3,
            0,
            color);
}


/* =========================================================
 * DMA-optimized filled circle
 *
 * The optimized path renders one horizontal scanline at a time
 * using the shared DMA line buffer.
 *
 * Compared with the general TFT_FillCircle() implementation,
 * this reduces the number of individual SPI transactions by
 * transferring one complete scanline per DMA operation.
 *
 * The horizontal extent of each scanline is calculated from:
 *
 *     x^2 + y^2 <= r^2
 *
 * using integer arithmetic only.
 *
 * Pixels inside the circle are written with color. Pixels inside
 * the bounding box but outside the circle are written with
 * bgColor.
 *
 * Therefore, the complete bounding box is overwritten. Callers
 * must only use this function when replacing the background inside
 * that region is acceptable.
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

    /*
     * The optimized path requires the complete bounding box to
     * remain inside the display and fit inside the shared line
     * buffer. The general implementation is used as a safe fallback
     * when these constraints are not satisfied.
     */
    if ((cx < radius) || (cy < radius) ||
        ((uint32_t)cx + radius >= TFT_WIDTH) ||
        ((uint32_t)cy + radius >= TFT_HEIGHT) ||
        (diameter > TFT_WIDTH))
    {
        TFT_FillCircle(
                cx,
                cy,
                radius,
                color);

        return false;
    }

    uint16_t x0 = cx - radius;
    uint16_t y0 = cy - radius;

    uint16_t colorSwapped =
            (color<<8)|(color>>8);

    uint16_t bgSwapped =
            (bgColor<<8)|(bgColor>>8);

    TFT_SetWindow(
            x0,
            y0,
            x0+diameter-1,
            y0+diameter-1);

    TFT_BeginGramWrite();

    int32_t r2 =
            (int32_t)radius *
            (int32_t)radius;

    for (int16_t dy = -(int16_t)radius;
         dy <= (int16_t)radius;
         dy++)
    {
        int32_t dy2 =
                (int32_t)dy *
                (int32_t)dy;

        /*
         * Determine the maximum horizontal extent satisfying the
         * circle equation. The calculation intentionally uses
         * integer arithmetic to avoid floating-point dependencies.
         */
        int16_t dxMax = 0;

        while (((int32_t)(dxMax+1)*(dxMax+1) + dy2) <= r2)
        {
            dxMax++;
        }

        uint16_t left =
                (uint16_t)(radius-dxMax);

        uint16_t right =
                (uint16_t)(radius+dxMax);

        /*
         * Populate the complete scanline. Pixels outside the circle
         * receive the caller-provided background color.
         */
        for (uint16_t i = 0;
             i < diameter;
             i++)
        {
            lineBuffer[i] =
                    ((i >= left) && (i <= right))
                    ? colorSwapped
                    : bgSwapped;
        }

#if (__DCACHE_PRESENT==1)

        /*
         * The line buffer is modified by the CPU and consumed by
         * the SPI DMA controller. Clean the D-Cache before transfer
         * so that DMA reads the latest buffer contents.
         */
        SCB_CleanDCache_by_Addr(
                (uint32_t*)lineBuffer,
                diameter*sizeof(uint16_t));

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
 * 5x7 bitmap font
 *
 * Each glyph contains five columns. Bit 0 represents the
 * top pixel of the corresponding column.
 *
 * The table contains the character set required by the
 * embedded user interface.
 *=========================================================*/

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
    {0x00,0x1C,0x22,0x41,0x00}, /* '[' - mapped to parenthesis glyph */
    {0x02,0x04,0x08,0x10,0x20}, /* '\' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ']' - mapped to parenthesis glyph */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */

    /*
     * Lowercase glyphs intentionally reuse the corresponding
     * uppercase bitmap to keep the font representation compact.
     */
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
#define FONT_LAST_CHAR  0x7A  /* 'z' */
#define FONT_W 5
#define FONT_H 7

/*=========================================================
 * Character rendering
 *
 * Draws one 5x7 bitmap glyph with an integer scale factor.
 * A scale of 1 renders individual pixels; larger scales expand
 * each glyph pixel into a square block.
 *=========================================================*/

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

    /*
     * Add one background-colored column between adjacent glyphs.
     * The spacing column is scaled together with the character.
     */
    if(scale==1)
    {
        TFT_DrawLine(
                x+FONT_W,
                y,
                x+FONT_W,
                y+FONT_H-1,
                bgColor);
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
        TFT_DrawChar(
                cx,
                y,
                *str,
                fgColor,
                bgColor,
                scale);

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

    if(w)
    {
        *w = len*(FONT_W+1)*scale;
    }

    if(h)
    {
        *h = FONT_H*scale;
    }
}


/* =========================================================
 * Sprite-based text rendering
 *
 * TFT_DrawChar() and TFT_DrawText() provide the general-purpose
 * rendering path, but each glyph is transferred through multiple
 * small SPI operations.
 *
 * TFT_DrawTextFast() instead builds the complete text region in
 * RAM and transfers the resulting pixel buffer in a single DMA
 * transaction. This reduces SPI transaction overhead and avoids
 * exposing partially rendered text to the display.
 *
 * The sprite buffer includes the background-colored spacing column
 * between characters so that the complete text bounding box is
 * rendered deterministically.
 *
 * The buffer dimensions are fixed to the display width and three
 * font heights, covering the text sizes currently required by the
 * user interface.
 *
 * If the requested text exceeds the sprite buffer or display
 * boundaries, TFT_DrawTextFast() falls back to TFT_DrawText().
 * This preserves memory safety at the cost of lower rendering
 * performance.
 * ========================================================= */

#define TEXT_SPRITE_MAX_W   TFT_WIDTH
#define TEXT_SPRITE_MAX_H   (FONT_H*3)


/* =========================================================
 * Sprite orientation compensation
 *
 * The ILI9225 GRAM address increment direction is coupled to the
 * Entry Mode configuration and the logical-to-physical coordinate
 * transformation used by TFT_SetWindow().
 *
 * Solid-color rendering does not expose an incorrect pixel ordering
 * because every transferred pixel has the same value. Text rendering
 * does expose the ordering because neighboring pixels contain
 * different values.
 *
 * TEXT_SPRITE_FLIP_MODE compensates for this relationship when the
 * sprite is mapped into the display memory.
 *
 * Supported modes:
 *
 *   0 = no axis inversion
 *   1 = horizontal inversion
 *   2 = vertical inversion
 *   3 = horizontal and vertical inversion
 *
 * The selected mode is part of the display-orientation configuration
 * and must remain consistent with the TFT_SetWindow() coordinate
 * mapping and ILI9225 Entry Mode configuration.
 * ========================================================= */

#define TEXT_SPRITE_FLIP_MODE   1


static uint16_t textSpriteBuf[
        TEXT_SPRITE_MAX_W * TEXT_SPRITE_MAX_H]
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
        /* Empty string: no display update is required. */
        return true;
    }

    uint16_t charAdvance =
            (FONT_W + 1) * scale;

    uint16_t w =
            len * charAdvance;

    uint16_t h =
            FONT_H * scale;

    /*
     * Validate both the sprite-buffer capacity and the requested
     * display region before writing any pixel data.
     *
     * Falling back to the non-sprite renderer prevents buffer
     * overruns and keeps the API safe for unexpected text sizes
     * or coordinates.
     */
    if ((w > TEXT_SPRITE_MAX_W) ||
        (h > TEXT_SPRITE_MAX_H) ||
        ((uint32_t)x + w > TFT_WIDTH) ||
        ((uint32_t)y + h > TFT_HEIGHT))
    {
        TFT_DrawText(
                x,
                y,
                str,
                fgColor,
                bgColor,
                scale);

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


    /*
     * Convert RGB565 values to the byte order expected by the SPI
     * transfer path. The converted values can then be written
     * directly into the DMA source buffer.
     */
    uint16_t fgSwapped =
            (fgColor << 8) |
            (fgColor >> 8);

    uint16_t bgSwapped =
            (bgColor << 8) |
            (bgColor >> 8);


    /*
     * Rasterize every character directly into the sprite buffer.
     *
     * Each logical font pixel is expanded according to the requested
     * scale factor. The orientation macros map the logical text
     * coordinates into the memory order required by the display.
     */
    for (uint16_t ci = 0; ci < len; ci++)
    {
        char c = str[ci];

        if ((c < FONT_FIRST_CHAR) ||
            (c > FONT_LAST_CHAR))
        {
            c = ' ';
        }

        const uint8_t *glyph =
                font5x7[c - FONT_FIRST_CHAR];

        uint16_t colBase =
                ci * charAdvance;

        for (uint8_t col = 0;
             col < FONT_W;
             col++)
        {
            uint8_t line =
                    glyph[col];

            for (uint8_t row = 0;
                 row < FONT_H;
                 row++)
            {
                uint16_t px =
                        (line & (1 << row))
                        ? fgSwapped
                        : bgSwapped;

                /*
                 * Expand one bitmap pixel into a scale x scale
                 * block while applying the selected sprite
                 * orientation mapping.
                 */
                for (uint8_t sy = 0;
                     sy < scale;
                     sy++)
                {
                    uint16_t rowIdx =
                            row * scale + sy;

                    for (uint8_t sx = 0;
                         sx < scale;
                         sx++)
                    {
                        uint16_t colIdx =
                                colBase +
                                col * scale +
                                sx;

                        textSpriteBuf[
                            (uint32_t)SPRITE_R(rowIdx) * w +
                            SPRITE_C(colIdx)
                        ] = px;
                    }
                }
            }
        }


        /*
         * Fill the inter-character spacing column with the background
         * color across the complete text height. This guarantees that
         * the sprite contains a deterministic background between glyphs
         * instead of leaving stale data from a previous rendering.
         */
        for (uint16_t row = 0;
             row < h;
             row++)
        {
            for (uint8_t sx = 0;
                 sx < scale;
                 sx++)
            {
                uint16_t colIdx =
                        colBase +
                        FONT_W * scale +
                        sx;

                textSpriteBuf[
                    (uint32_t)SPRITE_R(row) * w +
                    SPRITE_C(colIdx)
                ] = bgSwapped;
            }
        }
    }


#undef SPRITE_R
#undef SPRITE_C


#if (__DCACHE_PRESENT==1)

    /*
     * The sprite buffer is written by the CPU and subsequently read
     * by the SPI DMA controller. Clean the D-Cache so that DMA sees
     * the latest pixel data in memory.
     */
    SCB_CleanDCache_by_Addr(
            (uint32_t*)textSpriteBuf,
            (uint32_t)w * h * sizeof(uint16_t));

#endif


    /*
     * Configure one display window covering the complete rendered
     * text region, then transfer the entire sprite through one DMA
     * transaction.
     */
    TFT_SetWindow(
            x,
            y,
            x + w - 1,
            y + h - 1);

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


