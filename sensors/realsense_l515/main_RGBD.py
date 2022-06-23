"""
Author: Mohsen Azimi
Date: September 01 2021
"""
import time
import cv2 as cv
from camera import L515
import pyrealsense2 as rs
import numpy as np
import open3d as o3d

point = (400, 300)


def mouse_coord(event, x, y, args, params):
    global point
    point = (x, y)


# Initialize Camera
camera = L515(read_bag=0, record_bag=0)
# camera.reset()
# camera.set_options()
i = 0
# try:
# Streaming loop

while True:
    # This call waits until a new coherent set of frames is available on a device maintain frame timing

    # read camera data
    f = camera.get_frame()
    color_image = f.color_image
    depth_image = f.depth_image
    ir_image = f.ir_image
    accel = f.accel
    gyro = f.gyro

    # print(gyro)
    pcd = f.point_cloud

    depth_clipped = camera.clip_distance(depth_image, color_image, 1, 3)

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_image_colorised = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=.03), cv.COLORMAP_JET)

    # # Flip image ?
    # color_image = cv.flip(color_image, 1)
    # depth_image_colorised = cv.flip(depth_image_colorised, 1)
    # infrared = cv.flip(infrared, 1)

    if camera.enable_rgbd:
        cv.namedWindow('Color', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('Depth', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('depth_clipped', cv.WINDOW_AUTOSIZE)
        # cv.namedWindow('IR', cv.WINDOW_AUTOSIZE)
        # cv.imshow('IR', infrared)

        # cv.setMouseCallback('Depth', mouse_coord)  # to show distance on mouse
        # # Show distance for a specific point
        # cv.circle(depth_image_colorised, point, 5, (0, 0, 255))
        # distance = depth_image[point[1], point[0]] * camera.depth_scale
        #
        # cv.putText(depth_image_colorised, f'{distance:.3f} m', (point[0], point[1] - 20), cv.FONT_HERSHEY_PLAIN, 2,
        #            (0, 255, 255), 4)

        cv.imshow('Color', color_image)
        cv.imshow('Depth', depth_image_colorised)
        cv.imshow('depth_clipped', depth_clipped)

        # # save to png
        # if camera.save_png:
        #     if i % 1 == 0:
        #         cv.imwrite('outputs/depth/depth_' + str(i) + '.png', depth_image_colorised)
        #         cv.imwrite('outputs/depth_clipped/depth_clipped_' + str(i) + '.png', depth_clipped)
        #         cv.imwrite('outputs/color/color_' + str(i) + '.png', color_image)
        #
        # i += 1
        # print(i)
        # print(pcd)
        # o3d.visualization.draw_geometries(pcd, zoom=.8)

    if cv.waitKey(1) & 0xff == 27:  # 27 = ESC
        break

# finally:
# Stop streaming
camera.pipeline.stop()
