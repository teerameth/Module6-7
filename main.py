import cv2
import imutilus
import v4l2capture
import select

width, height = 1920, 1080
full_screen = False
cap = v4l2capture.Video_device("/dev/video0")
size_x, size_y = cap.set_format(width, height, fourcc='MJPG')
cap.create_buffers(1)
cap.queue_all_buffers()
cap.start()
while True:
    select.select((cap,), (), ())
    image_data = cap.read_and_queue()
    img = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)