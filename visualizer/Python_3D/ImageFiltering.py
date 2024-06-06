#https://he-arc.github.io/livre-python/pillow/index.html

from PIL import Image, ImageFilter

img = Image.open("simulator\A010\data_a010_4.bmp")


width, height = img.size

WHITE_THRESHOLD = 100
PURPLE = (155, 89, 182)
BLUE = (52, 152, 219)

for x in range(width):
    for y in range(height):
        pixel = img.getpixel((x, y))
        # On vérifie que le pixel est blanc
        if all(channel < WHITE_THRESHOLD for channel in range(pixel)):
            if x + y <= width:
                img.putpixel((x, y), PURPLE)
            else:
                img.putpixel((x, y), BLUE)

img.show()
