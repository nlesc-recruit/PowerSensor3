#!/usr/bin/env python3
#
# This script generates font maps for the PowerSensor3 firmware.
# These are used to speed up the display by transfering the data of an entire
# character instead of writing pixel-by-pixel

import sys
import numpy as np


# RGB 565 colors as used by the display. Defining the colours used in PowerSensor
# See https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST77xx.h
COLOURS = {
    "red": 0xF800,
    "green": 0x07E0,
    "blue": 0x001F,
    "yellow": 0xFFE0
}


# This is the font as defined in the Adafruit GFX library.
# It is an 5x7 ASCII font, stored as 5 bytes per character
# See https://github.com/adafruit/Adafruit-GFX-Library/blob/master/glcdfont.c
FONT = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x5B, 0x4F, 0x5B, 0x3E, 0x3E, 0x6B,
                 0x4F, 0x6B, 0x3E, 0x1C, 0x3E, 0x7C, 0x3E, 0x1C, 0x18, 0x3C, 0x7E, 0x3C,
                 0x18, 0x1C, 0x57, 0x7D, 0x57, 0x1C, 0x1C, 0x5E, 0x7F, 0x5E, 0x1C, 0x00,
                 0x18, 0x3C, 0x18, 0x00, 0xFF, 0xE7, 0xC3, 0xE7, 0xFF, 0x00, 0x18, 0x24,
                 0x18, 0x00, 0xFF, 0xE7, 0xDB, 0xE7, 0xFF, 0x30, 0x48, 0x3A, 0x06, 0x0E,
                 0x26, 0x29, 0x79, 0x29, 0x26, 0x40, 0x7F, 0x05, 0x05, 0x07, 0x40, 0x7F,
                 0x05, 0x25, 0x3F, 0x5A, 0x3C, 0xE7, 0x3C, 0x5A, 0x7F, 0x3E, 0x1C, 0x1C,
                 0x08, 0x08, 0x1C, 0x1C, 0x3E, 0x7F, 0x14, 0x22, 0x7F, 0x22, 0x14, 0x5F,
                 0x5F, 0x00, 0x5F, 0x5F, 0x06, 0x09, 0x7F, 0x01, 0x7F, 0x00, 0x66, 0x89,
                 0x95, 0x6A, 0x60, 0x60, 0x60, 0x60, 0x60, 0x94, 0xA2, 0xFF, 0xA2, 0x94,
                 0x08, 0x04, 0x7E, 0x04, 0x08, 0x10, 0x20, 0x7E, 0x20, 0x10, 0x08, 0x08,
                 0x2A, 0x1C, 0x08, 0x08, 0x1C, 0x2A, 0x08, 0x08, 0x1E, 0x10, 0x10, 0x10,
                 0x10, 0x0C, 0x1E, 0x0C, 0x1E, 0x0C, 0x30, 0x38, 0x3E, 0x38, 0x30, 0x06,
                 0x0E, 0x3E, 0x0E, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F,
                 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14,
                 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62, 0x36, 0x49,
                 0x56, 0x20, 0x50, 0x00, 0x08, 0x07, 0x03, 0x00, 0x00, 0x1C, 0x22, 0x41,
                 0x00, 0x00, 0x41, 0x22, 0x1C, 0x00, 0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x08,
                 0x08, 0x3E, 0x08, 0x08, 0x00, 0x80, 0x70, 0x30, 0x00, 0x08, 0x08, 0x08,
                 0x08, 0x08, 0x00, 0x00, 0x60, 0x60, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02,
                 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x72, 0x49,
                 0x49, 0x49, 0x46, 0x21, 0x41, 0x49, 0x4D, 0x33, 0x18, 0x14, 0x12, 0x7F,
                 0x10, 0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x31, 0x41,
                 0x21, 0x11, 0x09, 0x07, 0x36, 0x49, 0x49, 0x49, 0x36, 0x46, 0x49, 0x49,
                 0x29, 0x1E, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x40, 0x34, 0x00, 0x00,
                 0x00, 0x08, 0x14, 0x22, 0x41, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x41,
                 0x22, 0x14, 0x08, 0x02, 0x01, 0x59, 0x09, 0x06, 0x3E, 0x41, 0x5D, 0x59,
                 0x4E, 0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E,
                 0x41, 0x41, 0x41, 0x22, 0x7F, 0x41, 0x41, 0x41, 0x3E, 0x7F, 0x49, 0x49,
                 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x3E, 0x41, 0x41, 0x51, 0x73,
                 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x20, 0x40,
                 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40,
                 0x40, 0x7F, 0x02, 0x1C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E,
                 0x41, 0x41, 0x41, 0x3E, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51,
                 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x26, 0x49, 0x49, 0x49, 0x32,
                 0x03, 0x01, 0x7F, 0x01, 0x03, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x1F, 0x20,
                 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14,
                 0x63, 0x03, 0x04, 0x78, 0x04, 0x03, 0x61, 0x59, 0x49, 0x4D, 0x43, 0x00,
                 0x7F, 0x41, 0x41, 0x41, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41,
                 0x41, 0x7F, 0x04, 0x02, 0x01, 0x02, 0x04, 0x40, 0x40, 0x40, 0x40, 0x40,
                 0x00, 0x03, 0x07, 0x08, 0x00, 0x20, 0x54, 0x54, 0x78, 0x40, 0x7F, 0x28,
                 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x28, 0x38, 0x44, 0x44, 0x28,
                 0x7F, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x08, 0x7E, 0x09, 0x02, 0x18,
                 0xA4, 0xA4, 0x9C, 0x78, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D,
                 0x40, 0x00, 0x20, 0x40, 0x40, 0x3D, 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00,
                 0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x78, 0x04, 0x78, 0x7C, 0x08,
                 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38, 0xFC, 0x18, 0x24, 0x24,
                 0x18, 0x18, 0x24, 0x24, 0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48,
                 0x54, 0x54, 0x54, 0x24, 0x04, 0x04, 0x3F, 0x44, 0x24, 0x3C, 0x40, 0x40,
                 0x20, 0x7C, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x3C, 0x40, 0x30, 0x40, 0x3C,
                 0x44, 0x28, 0x10, 0x28, 0x44, 0x4C, 0x90, 0x90, 0x90, 0x7C, 0x44, 0x64,
                 0x54, 0x4C, 0x44, 0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x77, 0x00,
                 0x00, 0x00, 0x41, 0x36, 0x08, 0x00, 0x02, 0x01, 0x02, 0x04, 0x02, 0x3C,
                 0x26, 0x23, 0x26, 0x3C, 0x1E, 0xA1, 0xA1, 0x61, 0x12, 0x3A, 0x40, 0x40,
                 0x20, 0x7A, 0x38, 0x54, 0x54, 0x55, 0x59, 0x21, 0x55, 0x55, 0x79, 0x41,
                 0x22, 0x54, 0x54, 0x78, 0x42,
                 0x21, 0x55, 0x54, 0x78, 0x40, 0x20, 0x54, 0x55, 0x79, 0x40, 0x0C, 0x1E,
                 0x52, 0x72, 0x12, 0x39, 0x55, 0x55, 0x55, 0x59, 0x39, 0x54, 0x54, 0x54,
                 0x59, 0x39, 0x55, 0x54, 0x54, 0x58, 0x00, 0x00, 0x45, 0x7C, 0x41, 0x00,
                 0x02, 0x45, 0x7D, 0x42, 0x00, 0x01, 0x45, 0x7C, 0x40, 0x7D, 0x12, 0x11,
                 0x12, 0x7D,
                 0xF0, 0x28, 0x25, 0x28, 0xF0, 0x7C, 0x54, 0x55, 0x45, 0x00, 0x20, 0x54,
                 0x54, 0x7C, 0x54, 0x7C, 0x0A, 0x09, 0x7F, 0x49, 0x32, 0x49, 0x49, 0x49,
                 0x32, 0x3A, 0x44, 0x44, 0x44, 0x3A,
                 0x32, 0x4A, 0x48, 0x48, 0x30, 0x3A, 0x41, 0x41, 0x21, 0x7A, 0x3A, 0x42,
                 0x40, 0x20, 0x78, 0x00, 0x9D, 0xA0, 0xA0, 0x7D, 0x3D, 0x42, 0x42, 0x42,
                 0x3D,
                 0x3D, 0x40, 0x40, 0x40, 0x3D, 0x3C, 0x24, 0xFF, 0x24, 0x24, 0x48, 0x7E,
                 0x49, 0x43, 0x66, 0x2B, 0x2F, 0xFC, 0x2F, 0x2B, 0xFF, 0x09, 0x29, 0xF6,
                 0x20, 0xC0, 0x88, 0x7E, 0x09, 0x03, 0x20, 0x54, 0x54, 0x79, 0x41, 0x00,
                 0x00, 0x44, 0x7D, 0x41, 0x30, 0x48, 0x48, 0x4A, 0x32, 0x38, 0x40, 0x40,
                 0x22, 0x7A, 0x00, 0x7A, 0x0A, 0x0A, 0x72, 0x7D, 0x0D, 0x19, 0x31, 0x7D,
                 0x26, 0x29, 0x29, 0x2F, 0x28, 0x26, 0x29, 0x29, 0x29, 0x26, 0x30, 0x48,
                 0x4D, 0x40, 0x20, 0x38, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
                 0x38, 0x2F, 0x10, 0xC8, 0xAC, 0xBA, 0x2F, 0x10, 0x28, 0x34, 0xFA, 0x00,
                 0x00, 0x7B, 0x00, 0x00, 0x08, 0x14, 0x2A, 0x14, 0x22, 0x22, 0x14, 0x2A,
                 0x14, 0x08, 0x55, 0x00, 0x55, 0x00, 0x55,
                 0xAA, 0x55, 0xAA, 0x55, 0xAA,
                 0xFF, 0x55, 0xFF, 0x55, 0xFF,
                 0x00, 0x00, 0x00, 0xFF, 0x00, 0x10, 0x10, 0x10, 0xFF, 0x00, 0x14, 0x14,
                 0x14, 0xFF, 0x00, 0x10, 0x10, 0xFF, 0x00, 0xFF, 0x10, 0x10, 0xF0, 0x10,
                 0xF0, 0x14, 0x14, 0x14, 0xFC, 0x00, 0x14, 0x14, 0xF7, 0x00, 0xFF, 0x00,
                 0x00, 0xFF, 0x00, 0xFF, 0x14, 0x14, 0xF4, 0x04, 0xFC, 0x14, 0x14, 0x17,
                 0x10, 0x1F, 0x10, 0x10, 0x1F, 0x10, 0x1F, 0x14, 0x14, 0x14, 0x1F, 0x00,
                 0x10, 0x10, 0x10, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x10, 0x10, 0x10,
                 0x10, 0x1F, 0x10, 0x10, 0x10, 0x10, 0xF0, 0x10, 0x00, 0x00, 0x00, 0xFF,
                 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFF, 0x10, 0x00,
                 0x00, 0x00, 0xFF, 0x14, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x1F,
                 0x10, 0x17, 0x00, 0x00, 0xFC, 0x04, 0xF4, 0x14, 0x14, 0x17, 0x10, 0x17,
                 0x14, 0x14, 0xF4, 0x04, 0xF4, 0x00, 0x00, 0xFF, 0x00, 0xF7, 0x14, 0x14,
                 0x14, 0x14, 0x14, 0x14, 0x14, 0xF7, 0x00, 0xF7, 0x14, 0x14, 0x14, 0x17,
                 0x14, 0x10, 0x10, 0x1F, 0x10, 0x1F, 0x14, 0x14, 0x14, 0xF4, 0x14, 0x10,
                 0x10, 0xF0, 0x10, 0xF0, 0x00, 0x00, 0x1F, 0x10, 0x1F, 0x00, 0x00, 0x00,
                 0x1F, 0x14, 0x00, 0x00, 0x00, 0xFC, 0x14, 0x00, 0x00, 0xF0, 0x10, 0xF0,
                 0x10, 0x10, 0xFF, 0x10, 0xFF, 0x14, 0x14, 0x14, 0xFF, 0x14, 0x10, 0x10,
                 0x10, 0x1F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x10, 0xFF, 0xFF, 0xFF, 0xFF,
                 0xFF, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x38, 0x44, 0x44,
                 0x38, 0x44, 0xFC, 0x4A, 0x4A, 0x4A, 0x34,
                 0x7E, 0x02, 0x02, 0x06, 0x06, 0x02, 0x7E, 0x02, 0x7E, 0x02, 0x63, 0x55,
                 0x49, 0x41, 0x63, 0x38, 0x44, 0x44, 0x3C, 0x04, 0x40, 0x7E, 0x20, 0x1E,
                 0x20, 0x06, 0x02, 0x7E, 0x02, 0x02, 0x99, 0xA5, 0xE7, 0xA5, 0x99, 0x1C,
                 0x2A, 0x49, 0x2A, 0x1C, 0x4C, 0x72, 0x01, 0x72, 0x4C, 0x30, 0x4A, 0x4D,
                 0x4D, 0x30, 0x30, 0x48, 0x78, 0x48, 0x30, 0xBC, 0x62, 0x5A, 0x46, 0x3D,
                 0x3E, 0x49, 0x49, 0x49, 0x00, 0x7E, 0x01, 0x01, 0x01, 0x7E, 0x2A, 0x2A,
                 0x2A, 0x2A, 0x2A, 0x44, 0x44, 0x5F, 0x44, 0x44, 0x40, 0x51, 0x4A, 0x44,
                 0x40, 0x40, 0x44, 0x4A, 0x51, 0x40, 0x00, 0x00, 0xFF, 0x01, 0x03, 0xE0,
                 0x80, 0xFF, 0x00, 0x00, 0x08, 0x08, 0x6B, 0x6B, 0x08, 0x36, 0x12, 0x36,
                 0x24, 0x36, 0x06, 0x0F, 0x09, 0x0F, 0x06, 0x00, 0x00, 0x18, 0x18, 0x00,
                 0x00, 0x00, 0x10, 0x10, 0x00, 0x30, 0x40, 0xFF, 0x01, 0x01, 0x00, 0x1F,
                 0x01, 0x01, 0x1E, 0x00, 0x19, 0x1D, 0x17, 0x12, 0x00, 0x3C, 0x3C, 0x3C,
                 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00], dtype=np.uint8)


if __name__ == '__main__':
    DEBUG = False

    # process the font array
    # it is an ascii 5x7 font, using 5 bytes per char
    # the characters are rotated 90 deg clockwise
    ncol = 5
    nrow = 8
    nbyte = 5
    # extract bits and create sub-array per char
    bitmaps = np.unpackbits(FONT.reshape(-1, nbyte), axis=1)
    bitmaps = bitmaps.reshape(-1, ncol, nrow)
    # de-rotate and reshape back to one array per char
    bitmaps = np.rot90(bitmaps, axes=(1, 2))

    # a font map is created for the following characters only
    chars = "./0123456789"
    nchar = len(chars)
    # create a size-1 fontmap for each colour, and a size-5 one for yellow only
    size = 5
    fontmap_names = COLOURS.keys()
    # number of entries in each fontmap
    fontmap_size = nchar * nrow * ncol
    fontmap = np.zeros((len(fontmap_names), nchar, nbyte * 8), dtype=np.uint16)
    fontmap_large = np.zeros((nchar, nbyte * 8 * size * size), dtype=np.uint16)

    # size-1 fontmaps
    for idx_fontmap, fontmap_name in enumerate(fontmap_names):
        if DEBUG:
            print(fontmap_name, file=sys.stderr)
        # create a map for each character
        for idx_char, char in enumerate(chars):
            # select the right bitmap and create version with rgb565 colours
            bitmap = bitmaps[ord(char)].ravel().astype(bool)
            # set colour
            fontmap[idx_fontmap][idx_char][bitmap] = COLOURS[fontmap_name]
            if DEBUG:
                print(fontmap[idx_fontmap][idx_char].reshape(nrow, ncol), file=sys.stderr)
                print('\n\n', file=sys.stderr)

    # size-5 fontmap (yellow only)
    fontmap_large_colour = "yellow"
    colour = COLOURS[fontmap_large_colour]
    if DEBUG:
        print("yellow-large", file=sys.stderr)
    # create a map for each character
    for idx_char, char in enumerate(chars):
        # select the right bitmap and create version with rgb565 colours
        bitmap = bitmaps[ord(char)].astype(bool)
        # create scaled bitmap
        bitmap = bitmap.repeat(size, axis=0).repeat(size, axis=1).ravel().astype(bool)
        fontmap_large[idx_char][bitmap] = colour
        if DEBUG:
            print(fontmap_large[idx_char].reshape(nrow * size, ncol * size), file=sys.stderr)
            print('\n\n', file=sys.stderr)

    # flip endianness of the font maps
    fontmap.byteswap(inplace=True)
    fontmap_large.byteswap(inplace=True)

    # create the fontmap array for use in the C++ firmware
    prefix = "static const uint16_t fontMap[] PROGMEM = {\n    "
    suffix = "};"
    fontmap_str = np.array2string(fontmap.ravel(), separator=', ', formatter={'int': hex}, threshold=np.inf)
    # index newlines with 4 spaces instead of one and convert the hex values to upper case
    fontmap_str = fontmap_str.replace('\n ', '\n    ')
    # define full C++ array, removing the [] from the numpy string
    fontmap_str = prefix + fontmap_str[1:-1] + suffix

    # repeat for the large-character fontmap
    prefix = "static const uint16_t fontMapLarge[] PROGMEM = {\n    "
    suffix = "};"
    fontmap_large_str = np.array2string(fontmap_large.ravel(), separator=', ', formatter={'int': hex}, threshold=np.inf)
    # index newlines with 4 spaces instead of one and convert the hex values to upper case
    fontmap_large_str = fontmap_large_str.replace('\n ', '\n    ')
    # define full C++ array, removing the [] from the numpy string
    fontmap_large_str = prefix + fontmap_large_str[1:-1] + suffix

    # create mapping for color to start of that fontmap
    color_mapping = "static const std::map<uint16_t, uint16_t> colorMap{\n  "
    for idx, color in enumerate(COLOURS.keys()):
        color_mapping += f"{{ST77XX_{color.upper()}, {fontmap_size * idx}}}"
        if idx < len(COLOURS.keys()) - 1:
            color_mapping += ", "
    color_mapping += "\n};"

    # create fontmap for space: just zeroes at max font size
    fontmap_space = f"static const uint16_t fontMapSpace[FONT_NROW * {size} * FONT_NCOL * {size}] PROGMEM = {{0}};"

    # file header
    header = f"""/* Fontmaps automatically generated with create_fontmaps.py
* These are bitmaps for the following characters: {chars}
* The colour is included as well in RGB565 format, so actually each "bit"map is a map of uint16 values.
* A size-1 fontmap is available for the following colours: {', '.join([f'ST77XX_{c.upper()}' for c in COLOURS.keys()])}
* A size-5 fontmap is available for the following colour: ST77XX_{fontmap_large_colour.upper()}
* The colorMap map can be used to find the start of the size-1 font map corresponding to a specific colour
* Each size-1 character is stored as 40 bytes (ASCII 5x7 plus empty 8th line font).
* An array of zeroes at the largest font size is also intialized for the space character
* The font is based on the font available in the Adafruit GFX library, see
* https://github.com/adafruit/Adafruit-GFX-Library/blob/master/glcdfont.c
* Note: the characters in the Adafruit library are rotated by 90 degrees compared to this font
*/
#ifndef POWERSENSOR_FONTMAPS_H
#define POWERSENSOR_FONTMAPS_H
#define FONT_NROW {nrow}
#define FONT_NCOL {ncol}
#include <map>"""

    footer = "#endif  // POWERSENSOR_FONTMAPS_H"

    # print the complete file contents
    print(header)
    print('')
    print(color_mapping)
    print('')
    print(fontmap_space)
    print('')
    print(fontmap_str)
    print('')
    print(fontmap_large_str)
    print('')
    print(footer)
