#ifndef TFT_SERVICE_H
#define TFT_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* lấy SPI_HandleTypeDef, GPIO_TypeDef và các define
                        TFT_DC_Pin / TFT_RST_Pin / TFT_CS_Pin... */
#include <stdint.h>

/* Độ phân giải vật lý của panel, theo datasheet 176RGBx220 */
#define TFT_WIDTH   220
#define TFT_HEIGHT  176

#define TFT_PHYS_WIDTH   176
#define TFT_PHYS_HEIGHT  220

/* Vài màu RGB565 dựng sẵn để test */
#define TFT_COLOR_BLACK     0x0000
#define TFT_COLOR_WHITE     0xFFFF
#define TFT_COLOR_RED       0xF800
#define TFT_COLOR_GREEN     0x07E0
#define TFT_COLOR_BLUE      0x001F
#define TFT_COLOR_YELLOW    0xFFE0
#define TFT_COLOR_CYAN      0x07FF
#define TFT_COLOR_MAGENTA   0xF81F

/* Reset + chạy power-on sequence của ILI9225, gọi 1 lần lúc khởi động */
void TFT_Init(void);

/* Set vùng GRAM (x0,y0)-(x1,y1) sẽ được ghi tiếp theo */
void TFT_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/* Tô toàn màn hình 1 màu duy nhất, dùng SPI DMA (blocking tới khi xong) */
void TFT_FillScreen(uint16_t color);

void TFT_TestPixel(void);

/* Gọi hàm này từ trong HAL_SPI_TxCpltCallback() của project (xem ghi chú
   trong tft_service.c) để báo DMA đã gửi xong 1 chunk dữ liệu cho TFT */
void TFT_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* TFT_SERVICE_H */

/* =========================================================
 * THÊM VÀO tft_service.h
 * Các hàm graphics cơ bản: pixel, line, rect, circle, text
 * ========================================================= */

#ifndef TFT_SERVICE_GFX_H
#define TFT_SERVICE_GFX_H

#include <stdint.h>
#include <stdbool.h>
void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

void TFT_DrawLine(uint16_t x0, uint16_t y0,
                   uint16_t x1, uint16_t y1,
                   uint16_t color);

void TFT_DrawRectangle(uint16_t x0, uint16_t y0,
                        uint16_t x1, uint16_t y1,
                        uint16_t color);

void TFT_FillRectangle(uint16_t x0, uint16_t y0,
                        uint16_t x1, uint16_t y1,
                        uint16_t color);

void TFT_DrawRoundRect(uint16_t x0, uint16_t y0,
                        uint16_t x1, uint16_t y1,
                        uint16_t radius,
                        uint16_t color);

void TFT_FillRoundRect(uint16_t x0, uint16_t y0,
                        uint16_t x1, uint16_t y1,
                        uint16_t radius,
                        uint16_t color);

void TFT_DrawCircle(uint16_t x0, uint16_t y0,
                     uint16_t radius,
                     uint16_t color);

void TFT_FillCircle(uint16_t x0, uint16_t y0,
                     uint16_t radius,
                     uint16_t color);

/* Fill hinh tron NHANH - dung DMA theo tung dong (thay vi tung
 * pixel/line rieng le nhu TFT_FillCircle). Chi dung khi vung ben
 * ngoai hinh tron (trong bounding box) co the ve de bang bgColor
 * ma khong lam mat chi tiet nao khac (vd man hinh vua duoc
 * TFT_FillScreen() truoc do) - KHONG dung khi hinh tron de len
 * chi tiet co san (vd bong de len duong crosshair). Neu vuot man
 * hinh/buffer dong, tu dong fallback ve TFT_FillCircle() va tra ve
 * false. */
bool TFT_FillCircleFast(uint16_t cx, uint16_t cy,
                     uint16_t radius,
                     uint16_t color,
                     uint16_t bgColor);

/* ---- Text (font 5x7, scale nguyên) ---- */
void TFT_DrawChar(uint16_t x, uint16_t y,
                   char c,
                   uint16_t fgColor,
                   uint16_t bgColor,
                   uint8_t scale);

void TFT_DrawText(uint16_t x, uint16_t y,
                   const char *str,
                   uint16_t fgColor,
                   uint16_t bgColor,
                   uint8_t scale);

/* ---- GIAI DOAN 2: font sprite + 1 lan DMA ----
 * Ve ca chuoi vao 1 buffer RAM roi ghi ra panel bang DUY NHAT 1 lan
 * SPI DMA, thay vi ~35 giao dich SPI rieng le cho MOI ky tu nhu
 * TFT_DrawText() o tren. Dung cho cac vung text can cap nhat nhieu
 * (gia tri so, gia tri thay doi lien tuc...) de giam nhay.
 *
 * Neu chuoi/toa do vuot qua buffer noi bo (xem TEXT_SPRITE_MAX_W/H
 * trong tft_service.c) hoac vuot mep man hinh, ham tu dong fallback
 * ve TFT_DrawText() (cham hon nhung an toan, khong tran bo nho) va
 * tra ve false - gia tri tra ve co the bo qua neu khong can kiem tra. */
bool TFT_DrawTextFast(uint16_t x, uint16_t y,
                   const char *str,
                   uint16_t fgColor,
                   uint16_t bgColor,
                   uint8_t scale);

/* Đo kích thước chuỗi (dùng để căn giữa text trong nút) */
void TFT_GetTextExtent(const char *str, uint8_t scale,
                        uint16_t *w, uint16_t *h);

#endif /* TFT_SERVICE_GFX_H */
