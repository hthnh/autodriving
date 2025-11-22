import numpy as np
import pandas as pd
import json
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# === 1. Load data từ CSV ===
csv_file = 'mag_raw_data_20250610_170833.csv'  # Đổi tên file nếu cần
df = pd.read_csv(csv_file)
mag_array = df[['mag_x_raw', 'mag_y_raw', 'mag_z_raw']].to_numpy()

# === 2. Tính Hard-Iron Offset ===
min_vals = np.min(mag_array, axis=0)
max_vals = np.max(mag_array, axis=0)
hard_iron_offset = (max_vals + min_vals) / 2.0
corrected_data = mag_array - hard_iron_offset

# === 3. Soft-Iron Calibration bằng PCA ===
pca = PCA(n_components=3)
pca.fit(corrected_data)

scaling_matrix = np.diag(1 / np.sqrt(pca.explained_variance_))
rotation_matrix = pca.components_
soft_iron_matrix = rotation_matrix.T @ scaling_matrix @ rotation_matrix

# === 4. Áp dụng hiệu chỉnh soft-iron ===
calibrated_data = (soft_iron_matrix @ corrected_data.T).T

# === 5. Lưu kết quả sang file JSON ===
calibration_result = {
    "hard_iron_offset": hard_iron_offset.tolist(),
    "soft_iron_matrix": soft_iron_matrix.tolist()
}

with open("magnetometer_calibration.json", "w") as f:
    json.dump(calibration_result, f, indent=4)

print("✅ Calibration completed and saved to magnetometer_calibration.json")
print("Explained variance ratio:", pca.explained_variance_ratio_)


# === 6. (Tuỳ chọn) Vẽ 3D để kiểm tra trước/sau hiệu chỉnh ===
fig = plt.figure(figsize=(12, 6))

# Trước Soft-Iron Calibration
ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(*corrected_data.T, s=1)
ax1.set_title("Before Soft-Iron Calibration")

# Sau Soft-Iron Calibration
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(*calibrated_data.T, s=1, color='g')
ax2.set_title("After Soft-Iron Calibration")

plt.tight_layout()
plt.show()
