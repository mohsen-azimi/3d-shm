
import pyrealsense2 as rs
import numpy as np



colorizer = None
align_to_depth = None
align_to_color = None
pointcloud = rs.pointcloud()

class L515ImagePacket:
    """
    Class that contains image and associated processing data.
    """

    @property
    def frame_id(self):
        return self._frame_id

    @property
    def timestamp(self):
        return self._timestamp

    @property
    def image_color(self):
        return self._image_color

    @property
    def image_depth(self):
        return self._image_depth

    @property
    def image_color_aligned(self):
        return self._image_color_aligned

    @property
    def image_depth_aligned(self):
        return self._image_depth_aligned

    @property
    def image_depth_colorized(self):
        if not self._image_depth_colorized:
            self._image_depth_colorized = cv2.applyColorMap(self.image_depth, cv2.COLORMAP_JET);
        return self._image_depth_colorized

    @property
    def intrinsics(self):
        return self._intrinsics

    @property
    def pointcloud(self):
        return self._pointcloud

    @property
    def pointcloud_texture(self):
        return self._pointcloud_texture

    def _rs_intrinsics_to_opencv_matrix(self, rs_intrinsics):
        fx = rs_intrinsics.fx
        fy = rs_intrinsics.fy
        cx = rs_intrinsics.ppx
        cy = rs_intrinsics.ppy
        s = 0  # skew
        return np.array([fx, s, cx,
                         0, fy, cy,
                         0, 0, 1]).reshape(3, 3)

    def __init__(self, frame_set, frame_id=None, timestamp=None, *args, **kwargs):
        global colorizer
        if not colorizer:
            colorizer = rs.colorizer()
            colorizer.set_option(rs.option.color_scheme, 0)

        global align_to_depth
        if not align_to_depth:
            align_to_depth = rs.align(rs.stream.depth)

        global align_to_color
        if not align_to_color:
            align_to_color = rs.align(rs.stream.color)

        global pointcloud
        if not pointcloud:
             pointcloud = rs.pointcloud()

        # Get intrinsics
        profile = frame_set.get_profile()
        video_stream_profile = profile.as_video_stream_profile()
        rs_intrinsics = video_stream_profile.get_intrinsics()
        self._intrinsics = self._rs_intrinsics_to_opencv_matrix(rs_intrinsics)

        # Get pointcloud
        depth_frame = frame_set.get_depth_frame()
        color_frame = frame_set.get_color_frame()
        pointcloud.map_to(color_frame)
        points = pointcloud.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices())
        points_arr = vtx.view(np.float32).reshape(vtx.shape + (-1,)).copy()
        self._pointcloud = points_arr

        # Get pointcloud texture mapping
        tex = np.asanyarray(points.get_texture_coordinates())
        color_map_arr = tex.view(np.float32).reshape(tex.shape + (-1,)).copy()
        self._pointcloud_texture = color_map_arr

        # Extract color image
        color_frame = frame_set.get_color_frame()
        self._image_color = np.asanyarray(color_frame.get_data()).copy()

        # Extract depth image
        depth_frame = frame_set.get_depth_frame()
        self._image_depth = np.asanyarray(depth_frame.get_data()).copy()

        # Align the color frame to depth frame and extract color image
        color_frame_aligned = align_to_depth.process(frame_set).get_color_frame()
        self._image_color_aligned = np.asanyarray(color_frame_aligned.get_data()).copy()

        # Align the depth frame to color frame and extract depth image
        depth_frame_aligned = align_to_color.process(frame_set).get_depth_frame()
        self._image_depth_aligned = np.asanyarray(depth_frame_aligned.get_data()).copy()

        self._image_depth_colorized = None
        if frame_id:
            self._frame_id = frame_id
        else:
            self._frame_id = frame_set.frame_number
        if timestamp:
            self._timestamp = timestamp
        else:
            self._timestamp = frame_set.timestamp
        self.__dict__.update(kwargs)
