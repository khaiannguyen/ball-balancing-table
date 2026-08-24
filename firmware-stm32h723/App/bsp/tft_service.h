#ifndef TFT_SERVICE_H
#define TFT_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * ILI9225 display geometry.
 *
 * The physical panel is 176 x 220 pixels. The application exposes
 * a logical 220 x 176 coordinate system because the display is
 * operated in a 90-degree rotated orientation.
 *
 * Application-level drawing functions use:
 *
 *     x = 0 .. TFT_WIDTH  - 1
 *     y = 0 .. TFT_HEIGHT - 1
 *
 * TFT_SetWindow() performs the logical-to-physical coordinate
 * transformation required by the selected display orientation.
 */
#define TFT_WIDTH        220
#define TFT_HEIGHT       176

#define TFT_PHYS_WIDTH   176
#define TFT_PHYS_HEIGHT  220

/*
 * Common RGB565 colors used by the UI and diagnostic functions.
 */
#define TFT_COLOR_BLACK     0x0000
#define TFT_COLOR_WHITE     0xFFFF
#define TFT_COLOR_RED       0xF800
#define TFT_COLOR_GREEN     0x07E0
#define TFT_COLOR_BLUE      0x001F
#define TFT_COLOR_YELLOW    0xFFE0
#define TFT_COLOR_CYAN      0x07FF
#define TFT_COLOR_MAGENTA   0xF81F

/**
 * @brief Initialize the ILI9225 display controller.
 *
 * Performs the hardware reset sequence and applies the controller
 * initialization registers required by the selected panel and
 * display orientation.
 *
 * This function should be called once during system initialization.
 */
void TFT_Init(void);

/**
 * @brief Configure the active GRAM write window.
 *
 * Subsequent GRAM write operations are directed to the inclusive
 * rectangle:
 *
 *     (x0, y0) .. (x1, y1)
 *
 * Coordinates use the application's logical display orientation.
 * The function converts them to the physical ILI9225 coordinate
 * system internally.
 */
void TFT_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
 * @brief Fill the complete display with one RGB565 color.
 *
 * Pixel data is transferred using SPI DMA one display line at a time.
 * The function blocks until all DMA transfers have completed.
 *
 * @param color RGB565 color value.
 */
void TFT_FillScreen(uint16_t color);

/**
 * @brief Draw a diagnostic pixel at a fixed display location.
 *
 * This function is intended for low-level display bring-up and
 * communication diagnostics.
 */
void TFT_TestPixel(void);

/**
 * @brief Handle completion of a TFT SPI DMA transfer.
 *
 * This function is intended to be called from the application's
 * HAL_SPI_TxCpltCallback().
 *
 * @param hspi SPI peripheral that generated the DMA completion.
 */
void TFT_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);

/* ------------------------------------------------------------------
 * Basic graphics primitives
 * ------------------------------------------------------------------ */

/**
 * @brief Draw one pixel.
 *
 * Coordinates outside the logical display area are ignored.
 */
void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw a line using the Bresenham rasterization algorithm.
 */
void TFT_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Draw a rectangle outline.
 *
 * The rectangle coordinates are inclusive.
 */
void TFT_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Draw a filled rectangle.
 *
 * The implementation uses a reusable line buffer and SPI DMA to
 * reduce the number of CPU-driven SPI transactions.
 */
void TFT_FillRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Draw a rounded rectangle outline.
 */
void TFT_DrawRoundRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t radius, uint16_t color);

/**
 * @brief Draw a filled rounded rectangle.
 */
void TFT_FillRoundRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t radius, uint16_t color);

/**
 * @brief Draw a circle outline using an integer midpoint algorithm.
 */
void TFT_DrawCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t color);

/**
 * @brief Draw a filled circle.
 *
 * This is the general-purpose implementation and may perform many
 * individual SPI transactions for larger circles.
 *
 * Use TFT_FillCircleFast() when the background-overwrite constraint
 * is acceptable and higher rendering performance is required.
 */
void TFT_FillCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t color);

/**
 * @brief Draw a filled circle using line-buffered SPI DMA.
 *
 * The circle is rendered one horizontal scanline at a time using the
 * shared DMA line buffer. This significantly reduces the number of
 * SPI transactions compared with TFT_FillCircle().
 *
 * The area inside the circle's bounding box but outside the circle
 * is explicitly written with bgColor. Therefore this function is
 * appropriate only when the entire bounding box may safely be
 * overwritten.
 *
 * Do not use this function to draw over existing graphics that must
 * be preserved.
 *
 * If the circle does not fit completely within the display or the
 * internal line buffer, the function falls back to TFT_FillCircle()
 * and returns false.
 *
 * @param cx      Circle center X coordinate.
 * @param cy      Circle center Y coordinate.
 * @param radius  Circle radius in pixels.
 * @param color   Circle RGB565 color.
 * @param bgColor  Background RGB565 color used outside the circle.
 *
 * @return true if the DMA-optimized implementation was used;
 *         false if the function used the fallback implementation.
 */
bool TFT_FillCircleFast(uint16_t cx, uint16_t cy, uint16_t radius, uint16_t color, uint16_t bgColor);

/* ------------------------------------------------------------------
 * Text rendering
 * ------------------------------------------------------------------ */

/*
 * The built-in font uses a fixed 5 x 7 pixel glyph with integer
 * scaling.
 *
 * Supported glyphs cover the character range required by the current
 * UI.
 */

/**
 * @brief Draw one character using the built-in 5 x 7 font.
 *
 * @param x        Character origin X coordinate.
 * @param y        Character origin Y coordinate.
 * @param c        Character to render.
 * @param fgColor  Foreground RGB565 color.
 * @param bgColor  Background RGB565 color.
 * @param scale    Integer glyph scale. Values below 1 are treated as 1.
 */
void TFT_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fgColor, uint16_t bgColor, uint8_t scale);

/**
 * @brief Draw a null-terminated string using the built-in font.
 *
 * Characters are rendered sequentially with one background column
 * between adjacent glyphs.
 */
void TFT_DrawText(uint16_t x, uint16_t y, const char *str, uint16_t fgColor, uint16_t bgColor, uint8_t scale);

/**
 * @brief Draw a string using a RAM sprite and one SPI DMA transfer.
 *
 * The complete text region is first rendered into an internal sprite
 * buffer and then transferred to the display as a single DMA operation.
 *
 * This reduces SPI transaction overhead and minimizes visible
 * intermediate rendering when frequently changing UI values.
 *
 * If the requested text exceeds the internal sprite buffer or display
 * boundaries, the function falls back to TFT_DrawText() and returns
 * false.
 *
 * @return true if the sprite/DMA path was used;
 *         false if the fallback renderer was required.
 */
bool TFT_DrawTextFast(uint16_t x, uint16_t y, const char *str, uint16_t fgColor, uint16_t bgColor, uint8_t scale);

/**
 * @brief Calculate the rendered dimensions of a text string.
 *
 * @param str   Null-terminated string.
 * @param scale Integer font scale.
 * @param w     Output width in pixels. May be NULL.
 * @param h     Output height in pixels. May be NULL.
 */
void TFT_GetTextExtent(const char *str, uint8_t scale, uint16_t *w, uint16_t *h);

#ifdef __cplusplus
}
#endif

#endif /* TFT_SERVICE_H */
