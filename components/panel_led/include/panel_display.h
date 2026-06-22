#ifndef PANEL_DISPLAY_H
#define PANEL_DISPLAY_H

#include <stdint.h>
#include <stdarg.h>
#include "hub75.h"
#include "font_8x16.h"

// ============================================================
// PALETA DE COLORES PREDEFINIDOS
// ============================================================
namespace Color {
    struct RGB { uint8_t r, g, b; };

    constexpr RGB RED     = {255, 0,   0  };
    constexpr RGB GREEN   = {0,   255, 0  };
    constexpr RGB BLUE    = {0,   0,   255};
    constexpr RGB WHITE   = {255, 255, 255};
    constexpr RGB YELLOW  = {255, 255, 0  };
    constexpr RGB CYAN    = {0,   255, 255};
    constexpr RGB MAGENTA = {255, 0,   255};
    constexpr RGB ORANGE  = {255, 128, 0  };
    constexpr RGB OFF     = {0,   0,   0  };
}

// ============================================================
// ESTRUCTURA SPRITE / ÍCONO
// ============================================================
typedef struct {
    uint8_t  width;
    uint8_t  height;
    const uint8_t* data;   // ceil(width/8) bytes por fila, bit7 = píxel izquierdo
} Sprite;

// ============================================================
// CLASE PRINCIPAL
// ============================================================
class PanelDisplay {
private:
    Hub75Driver* display;

    int panel_width;
    int panel_height;

    // ── Constantes de comportamiento ────────────────────────
    static constexpr int CHAR_SPACING    = 1;
    static constexpr int SPACE_WIDTH     = 6;
    static constexpr int SCROLL_DELAY_MS = 40;
    static constexpr int FADE_STEPS      = 8;
    static constexpr int FADE_DELAY_MS   = 30;
    static constexpr int TYPEWRITER_MS   = 120;

    // ── Helpers internos ────────────────────────────────────
    void remap_coords(int x_want, int y_want, int& x_send, int& y_send);
    int  char_advance(char c) const;

    // ── Macro interna para check de stop ────────────────────
    // Retorna true si debe detenerse
    inline bool should_stop(volatile bool* keep_running) const {
        return (keep_running != nullptr && !(*keep_running));
    }

    // ── Limpia y apaga el panel ──────────────────────────────
    inline void clear_and_flip() {
        display->clear();
        display->flip_buffer();
    }

public:
    // ── Constructor ─────────────────────────────────────────
    PanelDisplay(Hub75Driver* disp, int width = 32, int height = 16);

    // ════════════════════════════════════════════════════════
    // PRIMITIVAS DE DIBUJO
    // ════════════════════════════════════════════════════════

    void draw_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);
    void draw_pixel(int x, int y, Color::RGB color);

    void draw_line(int x0, int y0, int x1, int y1,
                   uint8_t r, uint8_t g, uint8_t b);
    void draw_line(int x0, int y0, int x1, int y1, Color::RGB color);

    void draw_rect(int x, int y, int w, int h,
                   uint8_t r, uint8_t g, uint8_t b);
    void draw_rect(int x, int y, int w, int h, Color::RGB color);

    void fill_rect(int x, int y, int w, int h,
                   uint8_t r, uint8_t g, uint8_t b);
    void fill_rect(int x, int y, int w, int h, Color::RGB color);

    // ════════════════════════════════════════════════════════
    // SPRITES / ÍCONOS
    // ════════════════════════════════════════════════════════

    void draw_sprite(const Sprite* sprite, int x, int y,
                     uint8_t r, uint8_t g, uint8_t b);
    void draw_sprite(const Sprite* sprite, int x, int y, Color::RGB color);

    // ════════════════════════════════════════════════════════
    // TEXTO BÁSICO
    // ════════════════════════════════════════════════════════

    void draw_char(char c, int x, int y, uint8_t r, uint8_t g, uint8_t b);
    void draw_char(char c, int x, int y, Color::RGB color);

    void draw_text(const char* text, int x, int y,
                   uint8_t r, uint8_t g, uint8_t b);
    void draw_text(const char* text, int x, int y, Color::RGB color);

    // ════════════════════════════════════════════════════════
    // ALINEACIÓN
    // ════════════════════════════════════════════════════════

    // Centrado — si no cabe hace scroll automático
    void draw_text_centered(const char* text, int y,
                            uint8_t r, uint8_t g, uint8_t b,
                            volatile bool* keep_running = nullptr);
    void draw_text_centered(const char* text, int y,
                            Color::RGB color,
                            volatile bool* keep_running = nullptr);

    // Alineado a la derecha
    void draw_text_right(const char* text, int y,
                         uint8_t r, uint8_t g, uint8_t b);
    void draw_text_right(const char* text, int y, Color::RGB color);

    // ════════════════════════════════════════════════════════
    // PRINTF-STYLE
    // ════════════════════════════════════════════════════════

    void draw_printf(int x, int y,
                     uint8_t r, uint8_t g, uint8_t b,
                     const char* fmt, ...);
    void draw_printf(int x, int y, Color::RGB color,
                     const char* fmt, ...);

    void draw_printf_centered(int y,
                              uint8_t r, uint8_t g, uint8_t b,
                              const char* fmt, ...);
    void draw_printf_centered(int y, Color::RGB color,
                              const char* fmt, ...);

    // ════════════════════════════════════════════════════════
    // DEGRADADO DE COLOR
    // ════════════════════════════════════════════════════════

    void draw_text_gradient(const char* text, int x, int y,
                            Color::RGB color_start, Color::RGB color_end);

    // ════════════════════════════════════════════════════════
    // UTILIDADES
    // ════════════════════════════════════════════════════════

    int  get_text_width(const char* text);
    void clear_screen();

    // ════════════════════════════════════════════════════════
    // ANIMACIONES — todas con keep_running para poder detenerlas
    // ════════════════════════════════════════════════════════

    void scroll_text(const char* text,
                     uint8_t r, uint8_t g, uint8_t b,
                     volatile bool* keep_running = nullptr);
    void scroll_text(const char* text,
                     Color::RGB color,
                     volatile bool* keep_running = nullptr);

    void blink_text(const char* text, int x, int y,
                    uint8_t r, uint8_t g, uint8_t b,
                    int times    = 4,
                    int delay_ms = 250,
                    volatile bool* keep_running = nullptr);
    void blink_text(const char* text, int x, int y,
                    Color::RGB color,
                    int times    = 4,
                    int delay_ms = 250,
                    volatile bool* keep_running = nullptr);

    void fade_in_text(const char* text, int x, int y,
                      Color::RGB color,
                      int steps    = FADE_STEPS,
                      int delay_ms = FADE_DELAY_MS,
                      volatile bool* keep_running = nullptr);

    void fade_out_text(const char* text, int x, int y,
                       Color::RGB color,
                       int steps    = FADE_STEPS,
                       int delay_ms = FADE_DELAY_MS,
                       volatile bool* keep_running = nullptr);

    void typewriter_text(const char* text, int x, int y,
                         Color::RGB color,
                         int delay_ms = TYPEWRITER_MS,
                         volatile bool* keep_running = nullptr);

    void bounce_text(const char* text,
                     Color::RGB color,
                     volatile bool* keep_running = nullptr);
};

#endif // PANEL_DISPLAY_H
