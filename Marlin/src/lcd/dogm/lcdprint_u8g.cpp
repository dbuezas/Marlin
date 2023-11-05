/**
 * @file    lcdprint_u8g.cpp
 * @brief   LCD print api for u8glib
 * @author  Yunhui Fu (yhfudev@gmail.com)
 * @version 1.0
 * @date    2016-08-19
 * @copyright GPL/BSD
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_U8GLIB

#include "marlinui_DOGM.h"

#include "../marlinui.h"
#include "../../MarlinCore.h"

#include "u8g_fontutf8.h"
#include "../lcdprint.h"

int lcd_glyph_height() {
  // Ascent is the distance from the baseline to the top of the glyph
  int fontAscent = u8g.getFontAscent();
  
  // Descent is the distance from the baseline to the bottom of the glyph,
  // it is returned as a negative value so we subtract it to get the total height
  int fontDescent = u8g.getFontDescent();
  
  // The total glyph height is the ascent minus the descent (which is negative)
 return fontAscent - fontDescent;
}

void lcd_moveto(const lcd_uint_t col, const lcd_uint_t row) { u8g.setCursor(col, row); }

void lcd_put_int(const int i) { u8g.print(i); }

// return < 0 on error
// return the number of pixels advanced
void unicodeToUtf8(const uint32_t &c, char *utf8) {
  if (c <= 0x7F) {
    // ASCII
    utf8[0] = c;
    utf8[1] = '\0';
  } else if (c <= 0x7FF) {
    // Two-byte sequence
    utf8[0] = 0xC0 | ((c >> 6) & 0x1F);
    utf8[1] = 0x80 | (c & 0x3F);
    utf8[2] = '\0';
  } else if (c <= 0xFFFF) {
    // Three-byte sequence
    utf8[0] = 0xE0 | ((c >> 12) & 0x0F);
    utf8[1] = 0x80 | ((c >> 6) & 0x3F);
    utf8[2] = 0x80 | (c & 0x3F);
    utf8[3] = '\0';
  } else if (c <= 0x10FFFF) {
    // Four-byte sequence
    utf8[0] = 0xF0 | ((c >> 18) & 0x07);
    utf8[1] = 0x80 | ((c >> 12) & 0x3F);
    utf8[2] = 0x80 | ((c >> 6) & 0x3F);
    utf8[3] = 0x80 | (c & 0x3F);
    utf8[4] = '\0';
  }
}

int lcd_put_lchar_max(const lchar_t &c, const pixel_len_t max_length) {
    char utf8[5];
    unicodeToUtf8(c, utf8);
    u8g.print(utf8);
    return u8g.getUTF8Width(utf8); // Get the width of the single character
}

/**
 * @return output width in pixels
 */
int lcd_put_u8str_max(const char * utf8_str, const pixel_len_t max_length) {
  int initialX = u8g.getCursorX();
  u8g.print(utf8_str); // U8g2 handles UTF-8 natively
  int width = u8g.getCursorX() - initialX; // Calculate the width of the printed string
  return width; // Return the width of the string that was printed
}

/**
 * @return output width in pixels
 */
int lcd_put_u8str_max_P(PGM_P utf8_pstr, const pixel_len_t max_length) {
  return lcd_put_u8str_max(utf8_pstr, max_length);
}


#endif // HAS_MARLINUI_U8GLIB
