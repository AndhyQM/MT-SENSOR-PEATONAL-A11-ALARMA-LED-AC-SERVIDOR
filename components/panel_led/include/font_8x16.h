#ifndef FONT_8X16_H
#define FONT_8X16_H

#include <stdint.h>

// Estructura para cada carácter 8x16
typedef struct {
    uint8_t width;      // Ancho real usado (máximo 8)
    uint8_t data[16];   // 16 filas, cada byte = 8 píxeles
} Font_Char;

// ─── Declaración extern: UNA sola instancia en font_8x16.cpp ───
// Evita duplicar ~700 bytes en RAM por cada .cpp que incluya este header
extern const Font_Char FONT_8X16[];
extern const int       FONT_8X16_COUNT;

// Convierte un char a su índice en FONT_8X16[]
// Retorna -1 si el carácter no está soportado
int get_char_index(char c);

#endif // FONT_8X16_H
