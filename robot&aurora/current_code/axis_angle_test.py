from scipy.spatial.transform import Rotation as R
import numpy as np

# r1 = R.from_rotvec(np.deg2rad([-64.52, 0, 168.04]))
# r1_quaternion = r1.as_quat()
# r_vec1 = r1.as_rotvec(degrees=True)
# r2 = R.from_rotvec(np.deg2rad([64.52, 0, -168.04]))
# r2_quaternion = r2.as_quat()
# r_vec2 = r2.as_rotvec(degrees=True)

# r1_quaternion_rounded = np.round(r1_quaternion, 4)
# r2_quaternion_rounded = np.round(r2_quaternion, 4)

# r_vec1_rounded = R.from_quat(r1_quaternion_rounded).as_rotvec(degrees=True)
# r_vec2_rounded = R.from_quat(r2_quaternion_rounded).as_rotvec(degrees=True)

# print("Rotation Vector 1:")
# print(r_vec1)
# print("Quaternion 1:")
# print(r1_quaternion)
# print("Rounded Rotation Vector 1:")
# print(r_vec1_rounded)

# print("Rotation Vector 2:")
# print(r_vec2)
# print("Quaternion 2:")
# print(r2_quaternion)
# print("Rounded Rotation Vector 2:")
# print(r_vec2_rounded)

# # 回転行列として比較（ほぼ等しいか）
# result = np.allclose(r1_quaternion_rounded, -r2_quaternion_rounded, atol=1e-4)
# print(f"Are the rotation vectors equal? {result}")

# r3 = R.from_quat([-0.3584, -0.0000, 0.9336, -0.00000000000000000000000000000000000000000000000001])
# r3_quaternion = r3.as_quat()
# r3_vec = r3.as_rotvec(degrees=True)

# r4 = R.from_quat([-0.3584, 0.0000, 0.9336, 0.0000000000000001])
# r4_quaternion = r4.as_quat()
# r4_vec = r4.as_rotvec(degrees=True)
# print("Rotation Vector 3:")
# print(r3_vec)
# # print("Quaternion 3:")
# # print(r3_quaternion)
# print("Rotation Vector 4:")
# print(r4_vec)
# # print("Quaternion 4:")
# # print(r4_quaternion)

def normalize_quat_sign(q):
    """Ensure the quaternion has a positive scalar part (w)."""
    q = np.array(q)
    if q[3] < 0:
        q = -q
    return q

q3 = normalize_quat_sign([-0.3584, -0.0000, 0.9336, -0.00000000000000000000000000000000000000000000000001])
r3 = R.from_quat(q3)
r3_vec = r3.as_rotvec(degrees=True)

q4 = normalize_quat_sign([-0.3584, 0.0000, 0.9336, 0.0000000000000000000000000000000000000000000000000001])
r4 = R.from_quat(q4)
r4_vec = r4.as_rotvec(degrees=True)

print(r3_vec)
print(r4_vec)
