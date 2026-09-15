import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import RectangleSelector

IMAGE_PATH = "/home/thiwa/scan_data/b1_r12_20260911_110915/color.png"

def main():
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

    height, width = img.shape[:2]
    # Use Matplotlib for windows: headless OpenCV still supports image I/O.
    fig, ax = plt.subplots(figsize=(9, 6))
    fig.canvas.manager.set_window_title("Select Crop")
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB),
              extent=(0, width, height, 0))
    ax.set_title("Drag with left mouse button to crop. Press q to quit.")
    preview = None

    def select_crop(press, release):
        nonlocal preview
        if any(value is None for value in
               (press.xdata, press.ydata, release.xdata, release.ydata)):
            return

        x_min, x_max = sorted(
            max(0, min(width, round(x))) for x in (press.xdata, release.xdata)
        )
        y_min, y_max = sorted(
            max(0, min(height, round(y))) for y in (press.ydata, release.ydata)
        )
        if x_min == x_max or y_min == y_max:
            return

        crop = img[y_min:y_max, x_min:x_max]
        if not cv2.imwrite("cropped_image.png", crop):
            raise OSError("Could not save cropped_image.png")

        print("\nCrop coordinates:")
        print(f"x1 = {x_min}\ny1 = {y_min}\nx2 = {x_max}\ny2 = {y_max}")
        print("\nCrop resolution:")
        print(f"width  = {x_max - x_min}\nheight = {y_max - y_min}")
        print("\nSaved as cropped_image.png")

        if preview is None or not plt.fignum_exists(preview.number):
            preview, _ = plt.subplots()
            preview.canvas.manager.set_window_title("Cropped Image")
            preview.canvas.mpl_connect("key_press_event", on_key)
        preview_ax = preview.axes[0]
        preview_ax.clear()
        preview_ax.imshow(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        preview.canvas.draw_idle()
        preview.show()

    def on_key(event):
        if event.key == "q":
            plt.close(fig)
            if preview is not None:
                plt.close(preview)

    def on_close(event):
        if preview is not None:
            plt.close(preview)

    # Keep a reference so the selector stays active throughout plt.show().
    selector = RectangleSelector(
        ax, select_crop, button=[1], minspanx=1, minspany=1,
        spancoords="data", interactive=True,
    )
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", on_close)
    print("Drag with left mouse button to select crop area.")
    print("Press q to quit.")
    plt.show()
    selector.set_active(False)


if __name__ == "__main__":
    main()
