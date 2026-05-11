import cv2

IMAGE_PATH = "/home/thiwa/scan_data_zed/b1_r13_20260506_134428/color.png"

img = cv2.imread(IMAGE_PATH)
if img is None:
    raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

clone = img.copy()

crop_points = []
cropping = False


def mouse_callback(event, x, y, flags, param):
    global crop_points, cropping, clone

    if event == cv2.EVENT_LBUTTONDOWN:
        crop_points = [(x, y)]
        cropping = True

    elif event == cv2.EVENT_MOUSEMOVE and cropping:
        temp = clone.copy()
        cv2.rectangle(temp, crop_points[0], (x, y), (0, 255, 0), 2)
        cv2.imshow("Select Crop", temp)

    elif event == cv2.EVENT_LBUTTONUP:
        crop_points.append((x, y))
        cropping = False

        x1, y1 = crop_points[0]
        x2, y2 = crop_points[1]

        # Make sure coordinates are ordered correctly
        x_min = min(x1, x2)
        y_min = min(y1, y2)
        x_max = max(x1, x2)
        y_max = max(y1, y2)

        crop = clone[y_min:y_max, x_min:x_max]

        print("\nCrop coordinates:")
        print(f"x1 = {x_min}")
        print(f"y1 = {y_min}")
        print(f"x2 = {x_max}")
        print(f"y2 = {y_max}")

        print("\nCrop resolution:")
        print(f"width  = {x_max - x_min}")
        print(f"height = {y_max - y_min}")

        cv2.imshow("Cropped Image", crop)
        cv2.imwrite("cropped_image.png", crop)
        print("\nSaved as cropped_image.png")


cv2.namedWindow("Select Crop", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Select Crop", 900, 600)

cv2.setMouseCallback("Select Crop", mouse_callback)

print("Drag with left mouse button to select crop area.")
print("Press q to quit.")

while True:
    cv2.imshow("Select Crop", clone)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()