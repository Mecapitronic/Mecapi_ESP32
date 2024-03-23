import py5

def draw_two_squares():
    py5.rect(py5.mouse_x, py5.mouse_y, 10, 10)
    py5.rect(py5.random_int(py5.width), py5.random_int(py5.height), 10, 10)


def setup():
    py5.size(300, 200)
    py5.rect_mode(py5.CENTER)

def draw():
    draw_two_squares()
    py5.square(py5.mouse_x, py5.mouse_y, 10)

py5.run_sketch()
