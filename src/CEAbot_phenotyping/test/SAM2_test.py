from ultralytics import SAM
import cv2

# Load SAM2 model
model = SAM("mobile_sam.pt")   # b = base model (fast)


# Run segmentation on image
results = model("/home/thiwa/scan_data_zed/b1_r12_20260504_170451/color.png", save=True)
'''r = results[0]
masks = r.masks.data.cpu().numpy()
for i, mask in enumerate(masks):
    mask_img = (mask * 255).astype("uint8")
    cv2.imwrite(f"mask_{i}.png", mask_img)'''