#include "panel_display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>

static const char* TAG = "PANEL";

// ============================================================
// CONSTRUCTOR
// ============================================================
PanelDisplay::PanelDisplay(Hub75Driver* disp, int width, int height)
    : display(disp), panel_width(width), panel_height(height) {
}

// ============================================================
// HELPER — avance horizontal de un carácter
// ============================================================
int PanelDisplay::char_advance(char c) const {
    if (c == ' ') return SPACE_WIDTH;
    int idx = get_char_index(c);
    return (idx >= 0) ? FONT_8X16[idx].width + CHAR_SPACING : 0;
}

// ============================================================
// HELPER — limpiar y voltear buffer
// ============================================================
void PanelDisplay::clear_screen() {
    display->clear();
    display->flip_buffer();
}

// ============================================================
// REMAPEO FÍSICO — TU VERSIÓN ORIGINAL (sin tocar)
// ============================================================
void PanelDisplay::remap_coords(int x_want, int y_want,
                                 int& x_send, int& y_send) {
    int bloque_y8     = y_want / 8;
    int y_in_bloque   = y_want % 8;
    int bloque_x8     = x_want / 8;
    int x_in_bloque   = x_want % 8;
    int sub_bloque_y4 = y_in_bloque / 4;
    int y_offset      = y_in_bloque % 4;

    if (sub_bloque_y4 == 0) {
        if (bloque_x8 == 0) {
            y_send = bloque_y8 * 8 + 4 + y_offset;
            x_send = 0 + x_in_bloque;
        }
        else if (bloque_x8 == 1) {
            y_send = bloque_y8 * 8 + 4 + y_offset;
            x_send = 16 + x_in_bloque;
        }
        else if (bloque_x8 == 2) {
            y_send = bloque_y8 * 8 + y_offset;
            x_send = 0 + x_in_bloque;
        }
        else {
            y_send = bloque_y8 * 8 + y_offset;
            x_send = 16 + x_in_bloque;
        }
    }
    else {
        if (bloque_x8 == 0) {
            y_send = bloque_y8 * 8 + 4 + y_offset;
            x_send = 8 + x_in_bloque;
        }
        else if (bloque_x8 == 1) {
            y_send = bloque_y8 * 8 + 4 + y_offset;
            x_send = 24 + x_in_bloque;
        }
        else if (bloque_x8 == 2) {
            y_send = bloque_y8 * 8 + y_offset;
            x_send = 8 + x_in_bloque;
        }
        else {
            y_send = bloque_y8 * 8 + y_offset;
            x_send = 24 + x_in_bloque;
        }
    }
}

// ============================================================
// PRIMITIVAS — PIXEL
// ============================================================
void PanelDisplay::draw_pixel(int x, int y,
                               uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || x >= panel_width ||
        y < 0 || y >= panel_height) return;

    int xs, ys;
    remap_coords(x, y, xs, ys);
    display->set_pixel(xs, ys, r, g, b);
}

void PanelDisplay::draw_pixel(int x, int y, Color::RGB color) {
    draw_pixel(x, y, color.r, color.g, color.b);
}

// ============================================================
// PRIMITIVAS — LÍNEA (Bresenham)
// ============================================================
void PanelDisplay::draw_line(int x0, int y0, int x1, int y1,
                              uint8_t r, uint8_t g, uint8_t b) {
    int dx  =  abs(x1 - x0);
    int dy  = -abs(y1 - y0);
    int sx  = (x0 < x1) ? 1 : -1;
    int sy  = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        draw_pixel(x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void PanelDisplay::draw_line(int x0, int y0, int x1, int y1,
                              Color::RGB color) {
    draw_line(x0, y0, x1, y1, color.r, color.g, color.b);
}

// ============================================================
// PRIMITIVAS — RECTÁNGULO (solo borde)
// ============================================================
void PanelDisplay::draw_rect(int x, int y, int w, int h,
                              uint8_t r, uint8_t g, uint8_t b) {
    draw_line(x,         y,         x + w - 1, y,         r, g, b);
    draw_line(x,         y + h - 1, x + w - 1, y + h - 1, r, g, b);
    draw_line(x,         y,         x,         y + h - 1, r, g, b);
    draw_line(x + w - 1, y,         x + w - 1, y + h - 1, r, g, b);
}

void PanelDisplay::draw_rect(int x, int y, int w, int h,
                              Color::RGB color) {
    draw_rect(x, y, w, h, color.r, color.g, color.b);
}

// ============================================================
// PRIMITIVAS — RECTÁNGULO RELLENO
// ============================================================
void PanelDisplay::fill_rect(int x, int y, int w, int h,
                              uint8_t r, uint8_t g, uint8_t b) {
    for (int row = y; row < y + h; row++) {
        draw_line(x, row, x + w - 1, row, r, g, b);
    }
}

void PanelDisplay::fill_rect(int x, int y, int w, int h,
                              Color::RGB color) {
    fill_rect(x, y, w, h, color.r, color.g, color.b);
}

// ============================================================
// SPRITES — bitmap 1bpp, bit7 = píxel izquierdo
// ============================================================
void PanelDisplay::draw_sprite(const Sprite* sprite, int x, int y,
                                uint8_t r, uint8_t g, uint8_t b) {
    if (sprite == nullptr || sprite->data == nullptr) return;

    int bytes_per_row = (sprite->width + 7) / 8;

    for (int row = 0; row < sprite->height; row++) {
        for (int col = 0; col < sprite->width; col++) {
            int byte_idx = row * bytes_per_row + (col / 8);
            int bit_idx  = 7 - (col % 8);
            if (sprite->data[byte_idx] & (1 << bit_idx)) {
                draw_pixel(x + col, y + row, r, g, b);
            }
        }
    }
}

void PanelDisplay::draw_sprite(const Sprite* sprite, int x, int y,
                                Color::RGB color) {
    draw_sprite(sprite, x, y, color.r, color.g, color.b);
}

// ============================================================
// TEXTO — CARÁCTER INDIVIDUAL (tu lógica original)
// ============================================================
void PanelDisplay::draw_char(char c, int x_offset, int y_offset,
                              uint8_t r, uint8_t g, uint8_t b) {
    int char_idx = get_char_index(c);
    if (char_idx < 0) {
        ESP_LOGD(TAG, "Char no soportado: '%c' (0x%02X)", c, (uint8_t)c);
        return;
    }

    const Font_Char* font_char = &FONT_8X16[char_idx];
    int xs, ys;

    for (int y = 0; y < 16; y++) {
        uint8_t row = font_char->data[y];
        for (int x = 0; x < font_char->width; x++) {
            if (row & (0x80 >> x)) {
                int screen_x = x_offset + x;
                int screen_y = y_offset + y;

                if (screen_x >= 0 && screen_x < panel_width &&
                    screen_y >= 0 && screen_y < panel_height) {
                    remap_coords(screen_x, screen_y, xs, ys);
                    display->set_pixel(xs, ys, r, g, b);
                }
            }
        }
    }
}

void PanelDisplay::draw_char(char c, int x, int y, Color::RGB color) {
    draw_char(c, x, y, color.r, color.g, color.b);
}

// ============================================================
// TEXTO — CADENA COMPLETA (tu lógica original)
// ============================================================
void PanelDisplay::draw_text(const char* text, int x_offset, int y_offset,
                              uint8_t r, uint8_t g, uint8_t b) {
    if (text == nullptr) return;

    int current_x = x_offset;
    for (int i = 0; text[i] != '\0'; i++) {
        draw_char(text[i], current_x, y_offset, r, g, b);

        int char_idx = get_char_index(text[i]);
        if (char_idx >= 0) {
            if (text[i] == ' ') {
                current_x += 6;
            } else {
                current_x += FONT_8X16[char_idx].width + 1;
            }
        }
    }
}

void PanelDisplay::draw_text(const char* text, int x, int y,
                              Color::RGB color) {
    draw_text(text, x, y, color.r, color.g, color.b);
}

// ============================================================
// ALINEACIÓN — CENTRADO
// Si no cabe → scroll automático
// ============================================================
void PanelDisplay::draw_text_centered(const char* text, int y,
                                       uint8_t r, uint8_t g, uint8_t b,
                                       volatile bool* keep_running) {
    if (text == nullptr) return;

    int text_width = get_text_width(text);

    if (text_width <= panel_width) {
        // ✅ Cabe → centrar estático
        int x = (panel_width - text_width) / 2;
        draw_text(text, x, y, r, g, b);
    } else {
        // ⚠️ No cabe → scroll automático
        scroll_text(text, r, g, b, keep_running);
    }
}

void PanelDisplay::draw_text_centered(const char* text, int y,
                                       Color::RGB color,
                                       volatile bool* keep_running) {
    draw_text_centered(text, y, color.r, color.g, color.b, keep_running);
}

// ============================================================
// ALINEACIÓN — DERECHA
// ============================================================
void PanelDisplay::draw_text_right(const char* text, int y,
                                    uint8_t r, uint8_t g, uint8_t b) {
    if (text == nullptr) return;
    int x = panel_width - get_text_width(text);
    draw_text(text, x, y, r, g, b);
}

void PanelDisplay::draw_text_right(const char* text, int y,
                                    Color::RGB color) {
    draw_text_right(text, y, color.r, color.g, color.b);
}

// ============================================================
// PRINTF-STYLE
// ============================================================
void PanelDisplay::draw_printf(int x, int y,
                                uint8_t r, uint8_t g, uint8_t b,
                                const char* fmt, ...) {
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    draw_text(buf, x, y, r, g, b);
}

void PanelDisplay::draw_printf(int x, int y, Color::RGB color,
                                const char* fmt, ...) {
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    draw_text(buf, x, y, color.r, color.g, color.b);
}

void PanelDisplay::draw_printf_centered(int y,
                                         uint8_t r, uint8_t g, uint8_t b,
                                         const char* fmt, ...) {
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    draw_text_centered(buf, y, r, g, b);
}

void PanelDisplay::draw_printf_centered(int y, Color::RGB color,
                                         const char* fmt, ...) {
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    draw_text_centered(buf, y, color.r, color.g, color.b);
}

// ============================================================
// DEGRADADO — interpolación lineal letra por letra
// ============================================================
void PanelDisplay::draw_text_gradient(const char* text, int x, int y,
                                       Color::RGB color_start,
                                       Color::RGB color_end) {
    if (text == nullptr) return;

    int len = (int)strlen(text);
    if (len == 0) return;

    int current_x = x;

    for (int i = 0; text[i] != '\0'; i++) {
        float t = (len > 1) ? (float)i / (float)(len - 1) : 0.0f;

        uint8_t r = (uint8_t)(color_start.r + t * ((int)color_end.r - color_start.r));
        uint8_t g = (uint8_t)(color_start.g + t * ((int)color_end.g - color_start.g));
        uint8_t b = (uint8_t)(color_start.b + t * ((int)color_end.b - color_start.b));

        if (current_x >= panel_width) break;
        draw_char(text[i], current_x, y, r, g, b);

        int char_idx = get_char_index(text[i]);
        if (char_idx >= 0) {
            current_x += (text[i] == ' ') ? 6 : FONT_8X16[char_idx].width + 1;
        }
    }
}

// ============================================================
// UTILIDAD — ancho total de un texto (tu lógica original)
// ============================================================
int PanelDisplay::get_text_width(const char* text) {
    if (text == nullptr || text[0] == '\0') return 0;

    int width = 0;
    for (int i = 0; text[i] != '\0'; i++) {
        int char_idx = get_char_index(text[i]);
        if (char_idx >= 0) {
            width += FONT_8X16[char_idx].width + 1;
        }
    }
    return width > 0 ? width - 1 : 0;
}

// ============================================================
// ANIMACIÓN — SCROLL HORIZONTAL (tu lógica original)
// ============================================================
void PanelDisplay::scroll_text(const char* text,
                                uint8_t r, uint8_t g, uint8_t b,
                                volatile bool* keep_running) {
    if (text == nullptr) return;

    int text_width = get_text_width(text);
    if (text_width <= 0) return;

    for (int offset = panel_width; offset > -text_width; offset--) {
        if (should_stop(keep_running)) {
            clear_and_flip();
            return;
        }
        display->clear();
        draw_text(text, offset, 0, r, g, b);
        display->flip_buffer();
        vTaskDelay(pdMS_TO_TICKS(SCROLL_DELAY_MS));
    }

    clear_and_flip();
}

void PanelDisplay::scroll_text(const char* text, Color::RGB color,
                                volatile bool* keep_running) {
    scroll_text(text, color.r, color.g, color.b, keep_running);
}

// ============================================================
// ANIMACIÓN — PARPADEO (tu lógica original + keep_running)
// ============================================================
void PanelDisplay::blink_text(const char* text, int x, int y,
                               uint8_t r, uint8_t g, uint8_t b,
                               int times, int delay_ms,
                               volatile bool* keep_running) {
    if (text == nullptr) return;

    for (int i = 0; i < times; i++) {
        if (should_stop(keep_running)) { clear_and_flip(); return; }

        // ON
        display->clear();
        draw_text(text, x, y, r, g, b);
        display->flip_buffer();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        if (should_stop(keep_running)) { clear_and_flip(); return; }

        // OFF
        clear_and_flip();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void PanelDisplay::blink_text(const char* text, int x, int y,
                               Color::RGB color,
                               int times, int delay_ms,
                               volatile bool* keep_running) {
    blink_text(text, x, y, color.r, color.g, color.b,
               times, delay_ms, keep_running);
}

// ============================================================
// ANIMACIÓN — FADE IN
// ============================================================
void PanelDisplay::fade_in_text(const char* text, int x, int y,
                                 Color::RGB color,
                                 int steps, int delay_ms,
                                 volatile bool* keep_running) {
    if (text == nullptr) return;

    for (int step = 1; step <= steps; step++) {
        if (should_stop(keep_running)) { clear_and_flip(); return; }

        float t   = (float)step / (float)steps;
        uint8_t r = (uint8_t)(color.r * t);
        uint8_t g = (uint8_t)(color.g * t);
        uint8_t b = (uint8_t)(color.b * t);

        display->clear();
        draw_text(text, x, y, r, g, b);
        display->flip_buffer();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// ============================================================
// ANIMACIÓN — FADE OUT
// ============================================================
void PanelDisplay::fade_out_text(const char* text, int x, int y,
                                  Color::RGB color,
                                  int steps, int delay_ms,
                                  volatile bool* keep_running) {
    if (text == nullptr) return;

    for (int step = steps; step >= 0; step--) {
        if (should_stop(keep_running)) { clear_and_flip(); return; }

        float t   = (float)step / (float)steps;
        uint8_t r = (uint8_t)(color.r * t);
        uint8_t g = (uint8_t)(color.g * t);
        uint8_t b = (uint8_t)(color.b * t);

        display->clear();
        draw_text(text, x, y, r, g, b);
        display->flip_buffer();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    clear_and_flip();
}

// ============================================================
// ANIMACIÓN — TYPEWRITER
// ============================================================
void PanelDisplay::typewriter_text(const char* text, int x, int y,
                                    Color::RGB color,
                                    int delay_ms,
                                    volatile bool* keep_running) {
    if (text == nullptr) return;

    int  len = (int)strlen(text);
    char buf[64];

    for (int i = 1; i <= len; i++) {
        if (should_stop(keep_running)) { clear_and_flip(); return; }

        memcpy(buf, text, i);
        buf[i] = '\0';

        display->clear();
        draw_text(buf, x, y, color);
        display->flip_buffer();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// ============================================================
// ANIMACIÓN — BOUNCE
// ============================================================
void PanelDisplay::bounce_text(const char* text, Color::RGB color,
                                volatile bool* keep_running) {
    if (text == nullptr) return;

    int text_width = get_text_width(text);
    if (text_width <= 0) return;

    int x_min = (text_width < panel_width) ? 0
                                           : -(text_width - panel_width);
    int x_max = (text_width < panel_width) ? panel_width - text_width
                                           : 0;

    int x   = x_max;
    int dir = -1;

    while (true) {
        if (should_stop(keep_running)) { clear_and_flip(); return; }

        display->clear();
        draw_text(text, x, 0, color);
        display->flip_buffer();
        vTaskDelay(pdMS_TO_TICKS(SCROLL_DELAY_MS));

        x += dir;
        if (x <= x_min) { x = x_min; dir = +1; }
        if (x >= x_max) { x = x_max; dir = -1; }
    }
}
