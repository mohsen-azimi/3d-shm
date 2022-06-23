import open3d as o3d
import numpy as np
import matplotlib.pylab as plt
import copy
import os
import sys


pcd = o3d.io.read_point_cloud('out.ply')

# print(pcd)
# print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries(
#     [pcd],
#     zoom=0.3,
#     front=[0.4, -.2, -.8],
#     lookat=[2.5, 2.0, 1.5],
#     up=[-.05, -.9, .2]
# )
# o3d.visualization.draw_geometries([pcd])




# voxel downsampling
dwnsmpld_pcd = pcd.voxel_down_sample(voxel_size=0.02)
# o3d.visualization.draw_geometries([dwnsmpld_pcd])

# voxel normal estimation
dwnsmpld_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.1, max_nn=30))

o3d.visualization.draw_geometries([dwnsmpld_pcd], point_show_normal=True)
