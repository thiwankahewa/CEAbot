import cv2

# Open USB camera (0 = first camera)



cap = cv2.VideoCapture(
    "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam-B0578-2.3MP-GS_SN001-video-index0",
    cv2.CAP_V4L2
)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        break

    # Show frame
    cv2.imshow("USB Camera", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
