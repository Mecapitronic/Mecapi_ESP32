import lidar_data_parser
import py5

coords = None
def draw_two_squares():
    py5.rect(py5.mouse_x, py5.mouse_y, 10, 10)
    py5.rect(py5.random_int(py5.width), py5.random_int(py5.height), 10, 10)

# def draw():
#     draw_two_squares()
#     py5.square(py5.mouse_x, py5.mouse_y, 10)


def setup():
    global coords
    py5.size(1000, 1000)
    py5.rect_mode(py5.CENTER)
    coords = lidar_data_parser.get_lidar_point()

def test():
    for i in range(100):
        yield i

def draw():
    for i in test():
        print(i)
    # for i in coords:
    #     print(coords)
    # py5.rect(coords[0] / 2, coords[1] / 2, 10, 10)
    # print(coords[0], coords[1])

py5.run_sketch()
