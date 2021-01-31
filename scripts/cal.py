import numpy as np

def get_custom_intrinsics(camera_name):
    if camera_name == 'A':
        # Camera A: 1920 x 1080 Mike's Cal (camera_calibration ros)
        mtx = np.array([[1342.451693 ,   0.        , 1005.799801],
               [  0.        ,1339.678018, 545.304949],
               [  0.        ,   0.        ,   1.        ]])

        dist = np.array([[ 0.069644, -0.154332, -0.004702, 0.004893, 0.000000]])
    elif camera_name == 'B':
        # Camera B: 1920x1080 Mike's Cal (camera_calibration ros)
        mtx = np.array([[1349.544501 ,   0.        , 974.411300],
               [  0.        ,1348.501863, 530.852211],
               [  0.        ,   0.        ,   1.        ]])

        dist = np.array([[ 0.099147, -0.190634, 0.001793, 0.004096, 0.000000]])
    elif camera_name == 'C':
        # Camera C: 1920x1080 Mike's Cal (camera_calibration ros)
        mtx = np.array([[1296.617377 ,   0.        , 1010.359942],
              [  0.        ,1295.837058, 576.386493],
              [  0.        ,   0.        ,   1.        ]])

        dist = np.array([[0.104545, -0.148822, 0.000863, 0.003670, 0.000000]])
        return None
    else:
        raise NotImplementedError()
    return mtx, dist

########## 680x480 ##########

# Camera A: 640x480 Izzy's Cal
# mtx = np.array([[628.1604173 ,   0.        , 286.50564325],
#        [  0.        , 626.92951202, 250.89533263],
#        [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0.09078199, -0.01523302,  0.00307738, -0.01766021, -0.19786653]])

# Camera A: 640 x 480 Realsense Defaults
# mtx = np.array([[606.583374023438 ,   0.        , 331.773406982422],
#        [  0.        , 606.31201171875, 250.26139831543],
#        [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0., 0.,  0., 0., 0.]])

# Camera A: 640 x 480 Mike's Cal (camera_calibration ros)
# mtx = np.array([[603.306972 ,   0.        , 337.270625],
#        [  0.        , 603.468975, 247.853768],
#        [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0.115294, -0.241355, -0.002980, 0.003112, 0.000000]])

########## 1280x720 ##########

# Camera A: 1280 x 720 Realsense Defaults
# mtx = np.array([[909.875122070312 ,   0.        , 657.66015625],
#        [  0.        , 909.467956542969, 375.39208984375],
#        [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0., 0.,  0., 0., 0.]])

# Camera A: 1280 x 720 Mike's Cal (camera_calibration ros)
# mtx = np.array([[902.257272 ,   0.        , 664.567352],
#        [  0.        , 901.807867, 363.254469],
#        [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0.091499, -0.163526, -0.005734, 0.003308, 0.000000]])

########## 1920x1080 ##########

# Camera A: 1920x1080 Realsense Defaults
# mtx = np.array([[1364.81262207031 ,   0.        , 986.490234375],
#      [  0.        ,1364.20202636719, 563.088134765625],
#      [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0., 0.,  0., 0., 0.]])

# Camera A: 1920 x 1080 Mike's Cal (camera_calibration ros)
# mtx = np.array([[1342.451693 ,   0.        , 1005.799801],
#        [  0.        ,1339.678018, 545.304949],
#        [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0.069644, -0.154332, -0.004702, 0.004893, 0.000000]])

# Camera B: 1920x1080 Realsense Defaults
# mtx = np.array([[1377.68286132812 ,   0.        , 964.721923828125],
#      [  0.        ,1375.89331054688, 527.139770507812],
#      [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0., 0.,  0., 0., 0.]])

# Camera B: 1920x1080 Mike's Cal (camera_calibration ros)
# mtx = np.array([[1349.544501 ,   0.        , 974.411300],
#      [  0.        ,1348.501863, 530.852211],
#      [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[ 0.099147, -0.190634, 0.001793, 0.004096, 0.000000]])

# Camera C: 1920x1080 Mike's Cal (camera_calibration ros)
# mtx = np.array([[1296.617377 ,   0.        , 1010.359942],
#      [  0.        ,1295.837058, 576.386493],
#      [  0.        ,   0.        ,   1.        ]])
#
# dist = np.array([[0.104545 -0.148822 0.000863 0.003670 0.000000]])
