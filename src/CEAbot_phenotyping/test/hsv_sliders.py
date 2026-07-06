import cv2
import numpy as np

IMAGE_PATH = "b1_r17_20260706_153034/color.png"

img = cv2.imread(IMAGE_PATH)
if img is None:
    raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

USE_CROP = True
x1, y1 = 140, 162
x2, y2 = 1836, 900

if USE_CROP:
    img = img[y1:y2, x1:x2]

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

def nothing(x):
    pass

cv2.namedWindow("HSV Sliders", cv2.WINDOW_NORMAL)

# Initial green range
cv2.createTrackbar("H Min", "HSV Sliders", 22, 179, nothing)
cv2.createTrackbar("H Max", "HSV Sliders", 95, 179, nothing)

cv2.createTrackbar("S Min", "HSV Sliders", 27, 255, nothing)
cv2.createTrackbar("S Max", "HSV Sliders", 255, 255, nothing)

cv2.createTrackbar("V Min", "HSV Sliders", 0, 255, nothing)
cv2.createTrackbar("V Max", "HSV Sliders", 255, 255, nothing)

cv2.createTrackbar("Min Area", "HSV Sliders", 10000, 10000, nothing)

while True:
    h_min = cv2.getTrackbarPos("H Min", "HSV Sliders")
    h_max = cv2.getTrackbarPos("H Max", "HSV Sliders")
    s_min = cv2.getTrackbarPos("S Min", "HSV Sliders")
    s_max = cv2.getTrackbarPos("S Max", "HSV Sliders")
    v_min = cv2.getTrackbarPos("V Min", "HSV Sliders")
    v_max = cv2.getTrackbarPos("V Max", "HSV Sliders")
    min_area = cv2.getTrackbarPos("Min Area", "HSV Sliders")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    result = cv2.bitwise_and(img, img, mask=mask_clean)

    contours, _ = cv2.findContours(
        mask_clean,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    output = img.copy()

    valid_contours = []

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < min_area:
            continue

        valid_contours.append(cnt)

        x, y, w, h = cv2.boundingRect(cnt)
        M = cv2.moments(cnt)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            cv2.circle(output, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(
                output,
                f"Area:{int(area)}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2
            )

        cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)

    print_text = f"Lower HSV: [{h_min}, {s_min}, {v_min}]  Upper HSV: [{h_max}, {s_max}, {v_max}]"
    display = output.copy()
    cv2.putText(
        display,
        print_text,
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2
    )

    scale = 0.5

    def resize(img, scale):
        width = int(img.shape[1] * scale)
        height = int(img.shape[0] * scale)
        return cv2.resize(img, (width, height))

    cv2.imshow("Original / Detection", resize(display, scale))
    cv2.imshow("Mask", resize(mask_clean, scale))
    cv2.imshow("Segmented Result", resize(result, scale))

    key = cv2.waitKey(30) & 0xFF

    if key == ord("q"):
        break

    if key == ord("s"):
        print("\nBest HSV values:")
        print(f"lower_green = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper_green = np.array([{h_max}, {s_max}, {v_max}])")
        print(f"min_area = {min_area}")

cv2.destroyAllWindows()