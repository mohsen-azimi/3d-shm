"""
3D-SHM
Copyright (c) Mohsen Azimi, 2022
"""

import argparse
import cv2
import numpy as np

from sensors.realsense_l515.camera import L515
import utils.cv2_utils as utils


def get_args_parser():
    parser = argparse.ArgumentParser('Set Robot parameters', add_help=False)


    -- Camera (RGB-d)
    parser.add_argument('--camera', default='l515', type=str,
                        help="Select the sensor type")
    parser.add_argument('--imshow', default=False, type=bool,
                        help="Show RGB-D frames")


    # Python
    parser.add_argument('--seed', default=66, type=int)
    parser.add_argument('--output_dir', default='output/',
                        help='path where to save')
    # --model
    parser.add_argument('--detector', default='ArUco',
                        help='select the detector, ArUco, yolov5s, etc.')

    # more parser: https://github.com/mohsen-azimi/detr/blob/3513d7d3a4aaee1da9aa0e22c365ffb64922eb15/main_face.py#L20
    return parser


def main(args):
    print(args)
    if args.detector == 'ArUco':
        obj_detector = utils.ArUco_marker()

    camera = L515(read_bag=0, record_bag=0)

    while True:
        # This call waits until a new coherent set of frames is available on a device maintain frame timing

        frame = camera.get_frame()

        color_image = frame.color_image
        depth_image = frame.depth_image
        # ir_image = frame.ir_image
        # accel = frame.accel
        # gyro = frame.gyro

        # print(gyro)
        # pcd = frame.point_cloud

        depth_clipped = camera.clip_distance(depth_image, color_image, 1, 3)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_image_colorised = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=.03), cv2.COLORMAP_JET)

        # # Flip image ?
        # color_image = cv.flip(color_image, 1)
        # depth_image_colorised = cv.flip(depth_image_colorised, 1)
        # infrared = cv.flip(infrared, 1)

        (corners, ids, rejected) = obj_detector.detect_obj(color_image)
        # print(corners)
        if ids is not None:
            if [1] in ids:

                idx = np.where(ids == 1)
                # print(idx[0][0])
                marker_corner = corners[idx[0][0]][idx[1][0]]
                # print(idx, marker_corner)

                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                # Convert the (x,y) coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # Draw the bounding box of the ArUco detection
                cv2.line(color_image, top_left, top_right, (0, 255, 0), 2)
                cv2.line(color_image, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(color_image, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(color_image, bottom_left, top_left, (0, 255, 0), 2)

                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(color_image, (center_x, center_y), 6, (0, 0, 255), -1)

                # print(depth_image.shape, center_x, camera.depth_scale)

                # Draw the ArUco marker ID on the video frame
                # The ID is always located at the top_left of the ArUco marker
                distance = depth_image[center_y-1, center_x-1] * camera.depth_scale
                cv2.putText(color_image,  f'{distance:0.2f}',
                            (top_left[0], top_left[1] - 25),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            2.5, (0, 255, 0), 2)


                # show the bbox



        if camera.enable_rgbd:
            utils.im_show(640, 480, ('color_image', depth_image),  show_bbox=False)

            # for (marker_corner, marker_id) in zip(corners, ids):
            #     # print(marker_id, "-----")
            #     if marker_id[0] == 1:
            #         # print(robot.moves[marker_id[0] - 1])
            #         break



            #
            # # cv.namedWindow('IR', cv.WINDOW_AUTOSIZE)
            # # cv.imshow('IR', infrared)
            #
            # # cv.setMouseCallback('Depth', mouse_coord)  # to show distance on mouse
            # # # Show distance for a specific point
            # # cv.circle(depth_image_colorised, point, 5, (0, 0, 255))
            # # distance = depth_image[point[1], point[0]] * camera.depth_scale
            # #
            # # cv.putText(depth_image_colorised, f'{distance:.3f} m', (point[0], point[1] - 20), cv.FONT_HERSHEY_PLAIN, 2,
            # #            (0, 255, 255), 4)
            #

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

        if cv2.waitKey(1) & 0xff == 27:  # 27 = ESC
            break

    # finally:
    # Stop streaming
    camera.pipeline.stop()



if __name__ == '__main__':
    parser = argparse.ArgumentParser('3D-SHM', parents=[get_args_parser()])
    args = parser.parse_args()
    # if args.output_dir:
    #     Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    main(args)
