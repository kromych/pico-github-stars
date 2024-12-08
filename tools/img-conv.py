#!/usr/bin/env python3
from PIL import Image
from pprint import pprint

def convert_image(image_path):
    im = Image.open(image_path)
    # 14x14 pixels, 26 symbols, 6 levels of brightness
    glyph_data =  [[[[0 for x in range(14)] for y in range(14)] for s in range(26)] for b in range(6)]
    palette_data = []
    palette_data565_be = []

    print('Image size:', im.size)
    print('Image mode:', im.mode)
    print('Image format:', im.format)
    print('Image info:', im.info)
    print('Image palette size:', len(im.getpalette("RGB")))

    for i in range(256):
        p = im.getpalette("RGB")[i*3:i*3+3]
        palette_data.append([hex(p[0]), hex(p[1]), hex(p[2])])
        # Convert to 16-bit RGB565 big-endian
        r = p[0] >> 3
        g = p[1] >> 2
        b = p[2] >> 3
        rgb565 = (r << 11) | (g << 5) | b
        rgb565_be = ((rgb565 & 0xff) << 8) | ((rgb565 & 0xff00) >> 8)
        palette_data565_be.append(hex(rgb565_be))

    im_data = list(im.getdata(0))
    # Place the image data into the glyph data, the image is 364x84 pixels
    
    for y in range(84):
        for x in range(364):
            # 14x14 pixels, 26 symbols, 6 levels of brightness
            brightness = y // 14
            symbol = x // 14
            glyph_y = y % 14
            glyph_x = x % 14
            glyph_data[brightness][symbol][glyph_y][glyph_x] = hex(im_data[y * 364 + x])

    print('# Palette data [[r, g, b]; 256]:')
    pprint(palette_data)
    print('# Palette data RGB565 big-endian [0x0000; 0xffff]:')
    pprint(palette_data565_be)
    print('# Glyph data, 14x14 pixels, 26 symbols, 6 levels of brightness [[[[palette_index] * 14] * 14] * 26] * 6:')
    pprint(glyph_data)

if __name__ == '__main__':
    convert_image('matrix.bmp')
