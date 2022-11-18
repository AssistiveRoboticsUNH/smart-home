import pyzed.sl as sl
import cv2
import numpy as np
import sys

def main():
    print('openning zed2')
    # zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    # init_params = sl.InitParameters()
    # init_params.sdk_verbose = False

    camera = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 15
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    init_params.depth_minimum_distance = 1000
    init_params.depth_maximum_distance = 40000
    init_params.camera_image_flip = sl.FLIP_MODE.AUTO
    init_params.depth_stabilization = True
    runtime_params = sl.RuntimeParameters(confidence_threshold=50, texture_confidence_threshold=100)

    err = camera.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(err)
        sys.exit()

    #
    # # Open the camera
    # err = zed.open(init_params)
    # if err != sl.ERROR_CODE.SUCCESS:
    #     print('error=',err)
    #     exit(1)
    #
    # # Get camera information (ZED serial number)
    # zed_serial = zed.get_camera_information().serial_number
    # print("Hello! This is my serial number: {0}".format(zed_serial))

    # Close the camera
    # zed.close()

if __name__ == "__main__":
    main()