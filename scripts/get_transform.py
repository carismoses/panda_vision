from rotation_util import *
import numpy as np

#t = [0.0966688, -0.0389088, -0.0112938]
#R = quat_to_rot([-0.187011, -0.184241, 0.67231, 0.692156])
t = [0.0373563, -0.0314662, 0.0603106]
R = quat_to_rot([-0.055435, -0.0635838, 0.70426, 0.704912])
hand_to_optical = Rt_to_pose_matrix(R, t)

t = [0.0106, 0.0175, 0.0125]
R = eul_to_rot([0, 0, 0])
screw_to_camera = Rt_to_pose_matrix(R, t)

t = [0, 0.015, 0]
R = eul_to_rot([0, 0, 0])
camera_to_color = Rt_to_pose_matrix(R, t)

t = [0, 0, 0]
R = eul_to_rot([-1.57079632679, 0, -1.57079632679])
color_to_optical = Rt_to_pose_matrix(R, t)

tmp = np.dot(np.linalg.inv(color_to_optical), np.linalg.inv(camera_to_color))
tmp = np.dot(tmp, np.linalg.inv(screw_to_camera))
hand_to_screw = np.dot(hand_to_optical, tmp)
# tmp = np.dot(np.linalg.inv(screw_to_camera), np.linalg.inv(camera_to_color))
# tmp = np.dot(tmp, np.linalg.inv(color_to_optical))
# hand_to_screw = np.dot(tmp, hand_to_optical)
# print(hand_to_screw)

R, t = pose_matrix_to_Rt(hand_to_screw)
r = rot_to_eul(R)

print('T:', t, 'R:', r)