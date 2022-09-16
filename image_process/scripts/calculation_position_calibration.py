import pyrealsense2 as rs
import numpy as np
import cv2
import dlib
import os

LIP_MARGIN = 0.3  # Marginal rate for lip-only image.
RESIZE = (64, 64)  # Final image size

# Face detector and landmark detector
face_detector = dlib.get_frontal_face_detector()
landmark_detector = dlib.shape_predictor("detector/shape_predictor_68_face_landmarks.dat")


def shape_to_list(shape):
    coords = []
    for i in range(0, 68):
        coords.append((shape.part(i).x, shape.part(i).y))
    return coords


class plane_vector_finder:
    def __init__(self):
        pass

    def fit_plane(self, depth_intrin, depth_frame, lip_landmarks):
        # output: a point in the plane, the normal of the plane (camera coordinate)
        from skspatial.objects import Plane, Points
        from skspatial.plotting import plot_3d

        # get 3d points in camera coordinate
        pts = np.zeros([len(lip_landmarks), 3])
        for i in range(0, len(lip_landmarks)):
            x, y = lip_landmarks[i]
            dis = depth_frame.get_distance(x, y)  # get depth at (x,y)
            camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], dis)
            pts[i, :] = camera_coordinate

        # fit plane
        points = Points(pts)
        plane = Plane.best_fit(points)
        print(plane.point, plane.normal)

        return plane.point, plane.normal

    def find_max_depth_point(self, plane_point, plane_normal, depth_intrin, depth_frame, crop_pos):
        # output:

        # get ROI 3d points in camera coordinate
        camera_pts_list = []
        for x in range(crop_pos[0], crop_pos[1]):
            for y in range(crop_pos[2], crop_pos[3]):
                dis = depth_frame.get_distance(x, y)
                camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], dis)
                camera_pts_list.append(camera_coordinate)

        # get distance of points to the plane
        camera_pts = np.array(camera_pts_list)
        pts_vector = camera_pts - plane_point
        dis_to_plane = pts_vector.dot(plane_normal) / (np.linalg.norm(plane_normal))
        max_index = np.argmax(dis_to_plane)
        max_depth_point = camera_pts[max_index, :]
        return max_depth_point

    def find_all(self, color_frame, depth_frame, lip_landmarks, crop_pos):
        # get camera intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # get the lip plane
        plane_point, plane_normal = self.fit_plane(depth_intrin, depth_frame, lip_landmarks)

        # get max_depth_point
        max_depth_point = self.find_max_depth_point(plane_point, plane_normal, depth_intrin, depth_frame, crop_pos)

        # get vector
        in_vector = max_depth_point - plane_point

        # convert point to pixel coordinate
        max_depth_point_pixel = rs.rs2_project_point_to_pixel(depth_intrin, max_depth_point)
        plane_point_pixel = rs.rs2_project_point_to_pixel(depth_intrin, plane_point)
        return plane_point, max_depth_point, in_vector, max_depth_point_pixel, plane_point_pixel


if __name__ == "__main__":
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, '/media/lishoujie/视频材料/核酸/722/real/20220722_171037.bag')
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    align_to = rs.stream.color
    align = rs.align(align_to)
    profile = pipeline.start(config)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # color_image = color_image[...,::-1]  # something wrong here
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
            align_to = rs.stream.color
            # align = rs.align(align_to)

            # plane and vector finder
            finder = plane_vector_finder()

            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)  # Convert image into grayscale
            frame_buffer = gray  # Add image to the frame buffer
            frame_buffer_color = color_image
            landmark_buffer = []
            image = frame_buffer
            face_rects = face_detector(image, 1)
            if len(face_rects) < 1:
                print("No face detected: ")
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                cv2.namedWindow("color_image", cv2.WINDOW_AUTOSIZE)
                cv2.imshow("color_image", color_image)
                cv2.namedWindow("depth_colormap", cv2.WINDOW_AUTOSIZE)
                cv2.imshow("depth_colormap", depth_colormap)
                key = cv2.waitKey(1)
                continue
            if len(face_rects) > 1:  # Too many face detected
                print("Too many face: ")
                break
            rect = face_rects[0]  # Proper number of face
            landmark = landmark_detector(image, rect)  # Detect face landmarks
            landmark = shape_to_list(landmark)
            cropped_buffer = []
            lip_landmark = landmark[48:68]  # Landmark corresponding to lip
            lip_x = sorted(lip_landmark, key=lambda pointx: pointx[0])  # Lip landmark sorted for determining lip region
            lip_y = sorted(lip_landmark, key=lambda pointy: pointy[1])
            x_add = int((-lip_x[0][0] + lip_x[-1][0]) * LIP_MARGIN)  # Determine Margins for lip-only image
            y_add = int((-lip_y[0][1] + lip_y[-1][1]) * LIP_MARGIN)
            crop_pos = (
                lip_x[0][0] - x_add,
                lip_x[-1][0] + x_add,
                lip_y[0][1] - y_add,
                lip_y[-1][1] + y_add,
            )  # Crop image
            color_image = cv2.rectangle(
                img=color_image,
                pt1=(crop_pos[0], crop_pos[2]),
                pt2=(crop_pos[1], crop_pos[3]),
                color=(0, 255, 0),
                thickness=3,
            )
            # color_image = cv2.circle(
            #     img=color_image,
            #     center=(int((crop_pos[0] + crop_pos[1]) / 2), int((crop_pos[2] + crop_pos[3]) / 2)),
            #     radius=5,
            #     color=(0, 0, 255),
            #     thickness=5,
            # )

            # find plane and vector
            plane_point, max_depth_point, in_vector, max_depth_point_pixel, plane_point_pixel = finder.find_all(
                color_frame, aligned_depth_frame, lip_landmark, crop_pos
            )

            # draw max_depth_point and in_vector
            if max_depth_point[2] > 0:
                color_image = cv2.circle(
                    img=color_image,
                    center=(int(plane_point_pixel[0]), int((plane_point_pixel[1]))),
                    radius=5,
                    color=(0, 0, 255),
                    thickness=5,
                )

            # cropped = frame_buffer_color[crop_pos[2]:crop_pos[3],crop_pos[0]:crop_pos[1]]
            # cropped_depth = depth_image [crop_pos[2]:crop_pos[3],crop_pos[0]:crop_pos[1]]

            # cropped = cv2.resize(cropped,(RESIZE[0],RESIZE[1]),interpolation=cv2.INTER_CUBIC)        # Resize
            # cropped_buffer.append(cropped)

            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)  # Convert image into grayscale
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            depth_colormap = cv2.rectangle(
                img=depth_colormap,
                pt1=(crop_pos[0], crop_pos[2]),
                pt2=(crop_pos[1], crop_pos[3]),
                color=(0, 255, 0),
                thickness=3,
            )
            depth_colormap = cv2.circle(
                img=depth_colormap,
                center=(int((crop_pos[0] + crop_pos[1]) / 2), int((crop_pos[2] + crop_pos[3]) / 2)),
                radius=5,
                color=(0, 0, 255),
                thickness=5,
            )
            # images = np.hstack((color_image, depth_colormap))

            cv2.namedWindow("color_image", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("color_image", color_image)
            cv2.namedWindow("depth_colormap", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("depth_colormap", depth_colormap)
            key = cv2.waitKey(1)
            if key & 0xFF == ord("q") or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        pipeline.stop()

