import os
import pandas as pd

base_path = "/home/thiwa/scan_data_zed"  # update if needed

all_areas = []
all_center_y = []

for folder in os.listdir(base_path):
    folder_path = os.path.join(base_path, folder)

    if not os.path.isdir(folder_path):
        print(f"Skipping {folder}: not a directory")
        continue

    csv_path = os.path.join(folder_path, "plant_coordinates_camera_frame.csv")

    if not os.path.exists(csv_path):
        print(f"Skipping {folder}: plant_coordinates_camera_frame.csv not found")
        continue

    try:
        df = pd.read_csv(csv_path)

        if "area_px" in df.columns and "center_y" in df.columns:
            areas = df["area_px"].dropna()
            all_areas.extend(areas.tolist())
            center_ys = df["center_y"].dropna()
            all_center_y.extend(center_ys.tolist())

    except Exception as e:
        print(f"Error in {folder}: {e}")

# Convert to pandas series for stats
all_areas = pd.Series(all_areas)
all_center_y = pd.Series(all_center_y)

print("\nOverall Area Stats:")
print(f"Total samples: {len(all_areas)}")
print(f"Average: {all_areas.mean():.2f}")
print(f"Min: {all_areas.min():.2f}")
print(f"Max: {all_areas.max():.2f}")

print("\nOverall Center Y Stats:")
print(f"Total samples: {len(all_center_y)}")
abs_center_y = all_center_y.abs()
print(f"Average (abs): {abs_center_y.mean():.2f}")
print(f"Min: {all_center_y.min():.2f}")
print(f"Max: {all_center_y.max():.2f}")
print(f"Range: {(all_center_y.max() - all_center_y.min()):.2f}")