import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
from os import makedirs
from os.path import exists, join
import shutil
import json

import random
from itertools import count
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

matplotlib.use('TkAgg')

plt.style.use('fivethirtyeight')

x_vals = []  # [0, 1, 2, 3, 4, 5]
y_vals = []  # [0, 1, 3, 2, 3, 5]


index = count()


def animate(i):
    x_vals.append(next(index))
    y_vals.append(random.randint(0, 5))
    plt.cla()
    plt.plot(x_vals, y_vals)


ani = FuncAnimation(plt.gcf(), animate, interval=10)

plt.tight_layout()
plt.show()


# def make_clean_folder(path_folder):
#     if not exists(path_folder):
#         makedirs(path_folder)
#     else:
#         user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
#         if user_input.lower() == 'y':
#             shutil.rmtree(path_folder)
#             makedirs(path_folder)
#         else:
#             exit()


# def save_intrinsic_as_json(filename, frame):
#     intrinsics = frame.profile.as_video_stream_profile().intrinsics
#     with open(filename, 'w') as outfile:
#         json.dump(
#             {
#                 'width':
#                     intrinsics.width,
#                 'height':
#                     intrinsics.height,
#                 'intrinsic_matrix': [
#                     intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
#                     intrinsics.ppy, 1
#                 ]
#             },
#             outfile,
#             indent=4)


