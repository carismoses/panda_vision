""" Massachusetts Institute of Technology

Izzybrand, 2020
"""
import itertools
import pickle
import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import os
import sys
import rospkg

from cal import get_custom_intrinsics
from rotation_util import *
from rs_util import *

# create the aruco dictionary and default parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_params =  aruco.DetectorParameters_create()

# 8 x 3 matrix of -1, 1 to compute the corners of the blocks (used in draw_block)
signed_corners = np.array([c for c in itertools.product([-1, 1], repeat=3)])

camera_lookup = {'032622074588':'A', '028522072401':'B', '032622073024': 'C'}

def get_all_blocks_info():
    """ load all the block info files from the tags/ folder

    block info files are created by create_aruco_texture_for_block.
    They contain information about the aruco tags associated with the block
    and the poses of those tags in the block frame

    Returns:
        json -- {block_id: block_info}
    """
    all_blocks_info = {}

    rospack = rospkg.RosPack()
    folder_path = rospack.get_path('panda_vision') + '/tags'
    print(folder_path)
    for fname in os.listdir(folder_path):
        if fname.endswith('info.pkl'):
            block_id = int(fname.split('_')[1])
            with open(folder_path + '/' + fname, 'rb') as f:
                all_blocks_info[block_id] = pickle.load(f)

    return all_blocks_info

def draw_block(X_CO, dimensions, color_image, intrinsics, draw_axis=True):
    """ Draw dots at the corners of the block at the specified camera-frame
    pose and with the given dimensions.

    The top corners are lighter in color and the bottom corners are darker.

    Arguments:
        X_CO {np.ndarray} -- 4x4 pose of the object in the camera frame
        dimensions {np.ndarray} -- x,y,z dimensions of the object
        color_image {np.ndarray} -- the image into which to draw the object

    Optional
        draw_axis {bool} -- Draw the block axis at the COG. Default True

    Side effects:
        Modifies color_image
    """
    mtx, dist = intrinsics

    # get the translation from the COG to the box corner points
    t_OP = signed_corners * dimensions[None,:] / 2
    # and convert the corner points to camera frame
    t_CP = np.array([(X_CO @ Rt_to_pose_matrix(np.eye(3), t_OP_i))[:3,3] for t_OP_i in t_OP])
    # project the points into the image coordinates
    image_points, _ = cv2.projectPoints(t_CP, np.eye(3), np.zeros(3), mtx, dist)
    # and draw them into the image
    for corner, image_pt in zip(signed_corners, image_points):
        color = np.array([1.0,0.0,0.7])*100 + (corner[2] > 0) * 155
        cv2.circle(color_image, tuple(image_pt[0].astype(int)), 5, color, -1)

    if draw_axis:
        R_CO, t_CO = pose_matrix_to_Rt(X_CO)
        aruco.drawAxis(color_image, mtx, dist, R_CO, t_CO, dimensions.min()/2)

# def get_block_poses_in_camera_frame(ids, corners, all_blocks_info, color_image=None):
#     tag_id_to_block_pose = {}
#     for i in range(0, ids.size):
#         # pull out the info corresponding to this block
#         tag_id = ids[i][0]
#         block_id = tag_id // 6
#         try:
#             marker_info = all_blocks_info[block_id][tag_id]
#             # print(f'Detected {tag_id}, the {marker_info["name"]} face of block {block_id}')
#         except KeyError:
#             print(f'Failed to find the block info for {block_id}. Skipping.')
#             continue
#         # detect the aruco tag
#         marker_length = marker_info["marker_size_cm"]/100. # in meters
#         rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(
#             corners[i], marker_length, mtx, dist)
#         # pose of the tag in the camera frame
#         X_CT = Rt_to_pose_matrix(cv2.Rodrigues(rvec)[0], tvec)
#         # pose of the object in camera frame
#         #print(block_id, tag_id)
#         try:
#             X_TO = np.linalg.inv(marker_info["X_OT"])
#         except KeyError:
#             print(f'Failed to find the calibration info for tag {tag_id} on block {block_id}! Skipping.')
#             continue
#         X_CO = X_CT @ X_TO
#         # # draw axis for the aruco markers
#         tag_id_to_block_pose[tag_id] = X_CO
#
#         if color_image is not None:
#             aruco.drawAxis(color_image, mtx, dist, rvec, tvec, marker_length/2)
#
#     return tag_id_to_block_pose

# def combine_block_poses(block_poses_in_camera_frame):
#     # consolidate all the tags for each block into a list indexed by the block_id
#     block_poses = {}
#     for tag_id in block_poses_in_camera_frame.keys():
#         block_id = tag_id // 6
#         if block_id not in block_poses:
#             block_poses[block_id] = []
#         block_poses[block_id].append(block_poses_in_camera_frame[tag_id])
#
#     for block_id in block_poses.keys():
#         poses = block_poses[block_id]
#         block_poses[block_id] = mean_pose(poses)
#
#     return block_poses

def pnp_block_poses(ids, corners, all_blocks_info, intrinsics, color_image=None, min_tags=1):
    mtx, dist = intrinsics

    block_id_to_corners = {}
    corner_coeffs = np.array([[-1,1], [1, 1], [1, -1], [-1, -1]])/2
    for i in range(0, ids.size):
        # pull out the info corresponding to this block
        tag_id = ids[i][0]
        block_id = tag_id // 6
        img_corners = corners[i][0]
        try:
            marker_info = all_blocks_info[block_id][tag_id]
            # print(f'Detected {tag_id}, the {marker_info["name"]} face of block {block_id}')
        except KeyError:
            print(f'Failed to find the block info for {block_id}. Skipping.')
            continue

        try:
            tag_length = marker_info["marker_size_cm"]/100. # in meters
            X_OT = marker_info["X_OT"]
        except KeyError:
            print(f'Failed to find the calibration info for tag {tag_id} on block {block_id}! Skipping.')
            continue

        # create tag-frame poses of the corners
        X_TCs = np.zeros([4,4,4])
        X_TCs[:] = np.eye(4)
        X_TCs[:, :2, 3] = tag_length * corner_coeffs

        # and get the object frame poses
        X_OCs = np.einsum('ij,njk->nik', X_OT, X_TCs)
        obj_corners = X_OCs[:, :3, 3]

        if block_id not in block_id_to_corners:
            block_id_to_corners[block_id] = {'obj': obj_corners, 'img': img_corners}
        else:
            block_id_to_corners[block_id]['obj'] = np.vstack([block_id_to_corners[block_id]['obj'], obj_corners])
            block_id_to_corners[block_id]['img'] = np.vstack([block_id_to_corners[block_id]['img'], img_corners])

    block_poses = {}
    for block_id, corners in block_id_to_corners.items():
        #print(corners['obj'].shape, corners['img'].shape)
        if corners['img'].shape[0] < min_tags*4:
            continue
        _, rvec, tvec = cv2.solvePnP(corners['obj'], corners['img'], mtx, dist)
        block_poses[block_id] = Rt_to_pose_matrix(cv2.Rodrigues(rvec)[0], tvec[:,0])

    return block_poses


class BlockPoseEst:

    def __init__(self, callback=None, vis=False, serial_number=None, intrinsics=None, min_tags=1):
        self.callback = callback
        self.vis = vis
        self.all_blocks_info = get_all_blocks_info()
        self.min_tags = min_tags

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        if serial_number is not None:
            config.enable_device(serial_number)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        # Start streaming
        pipeline_profile = self.pipeline.start(config)

        # And get the device info
        self.serial_number = get_serial_number(pipeline_profile)
        print(f'Connected to {self.serial_number}')

        # get the camera intrinsics
        if intrinsics is None:
            print('Using default intrinsics from camera.')
            self.intrinsics = get_intrinsics(pipeline_profile)
        else:
            self.intrinsics = intrinsics

    def spin(self):
        try:
            while True:
                self.step()

        finally:
            # Stop streaming
            self.pipeline.stop()

    def step(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        # depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # convert image to numpy array
        color_image = np.asarray(color_frame.get_data())
        # convert to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # detect aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            gray_image, aruco_dict, parameters=aruco_params)
        if self.vis:
            aruco.drawDetectedMarkers(color_image, corners, ids)

        # if we've detected markers, then estimate their pose and draw frames
        if ids is not None:
            block_id_to_block_pose = pnp_block_poses(ids, corners, self.all_blocks_info, self.intrinsics, min_tags=self.min_tags)

            for block_id in block_id_to_block_pose.keys():
                X_CO = block_id_to_block_pose[block_id]

                # run the supplied callback
                if self.callback is not None:
                    self.callback(block_id, self.serial_number, X_CO)

                # if the visualizer is turned on, draw the block
                if self.vis:
                    dimensions = self.all_blocks_info[block_id]['dimensions']
                    draw_block(X_CO, dimensions, color_image, self.intrinsics)

        if self.vis:
            cv2.imshow('Aruco Frames', color_image)

        cv2.waitKey(1)

    def close(self):
        self.pipeline.stop()


def main():
    #if len(rs.context().devices):
    #    print('Could not find any Realsense Devices.')
    #    sys.exit(1)
    print('Listing available realsense devices...')
    serial_numbers = []
    for i, device in enumerate(rs.context().devices):
        serial_number = device.get_info(rs.camera_info.serial_number)
        serial_numbers.append(serial_number)
        print(f'{i+1}. {serial_number}')


    def print_callback(block_id, serial_number, X_CO):
        R_CO, T_CO = pose_matrix_to_Rt(X_CO)
        print(f'{block_id} at {T_CO}')

    bpe = BlockPoseEst(print_callback,
                       vis=True,
                       serial_number=serial_numbers[0],
                       intrinsics=get_custom_intrinsics(camera_lookup[serial_numbers[0]]))
    bpe.spin()


if __name__ == '__main__':
    main()
