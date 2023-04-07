import sys
import cv2
import copy
import rospy
import torch

import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
import matplotlib.cm as cm

from time import time
from feature_match import VisualTracker
from utils.parameter import frame2tensor, readParameters, make_matching_plot_fast
from utils.camera_model import PinholeCamera
from utils.matching import Matching

cur_un_pts_map = {}
prev_un_pts_map = {}

cur_time = 0.0
prev_time = 0.0

init = True


def img_callback(img_msg, param):

    global prev_un_pts_map, cur_un_pts_map, cur_time, prev_time, \
        keys, init, last_data, last_conver_img

    cur_time = img_msg.header.stamp.to_sec()

    bridge = CvBridge()
    conver_img = bridge.imgmsg_to_cv2(img_msg, "mono8")

    if init:
        init = False
    else:
        frame_tensor = frame2tensor(conver_img, "cuda")
        pred = matching({**last_data, 'image1': frame_tensor})
        kpts0 = last_data['keypoints0'][0].cpu().numpy()
        kpts1 = pred['keypoints1'][0].cpu().numpy()
        matches = pred['matches0'][0].cpu().numpy()
        confidence = pred['matching_scores0'][0].cpu().detach().numpy()
        valid = matches > -1
        mkpts0 = kpts0[valid]
        mkpts1 = kpts1[matches[valid]]
        ids = matches[valid]

        if True:

            feature_points = PointCloud()
            id_of_point = ChannelFloat32()
            camera_id = ChannelFloat32()
            u_of_point = ChannelFloat32()
            v_of_point = ChannelFloat32()
            velocity_x_of_point = ChannelFloat32()
            velocity_y_of_point = ChannelFloat32()
            feature_points.header = img_msg.header
            feature_points.header.frame_id = "world"

            cur_pts = mkpts1
            for i in range(len(cur_pts)):
                b = CamearIntrinsicParam.liftProjective(cur_pts[i])
                un_pts = Point32()
                un_pts.x = b[0] / b[2]
                un_pts.y = b[1] / b[2]
                un_pts.z = 1

                feature_points.points.append(un_pts)
                id_of_point.values.append(ids[i])
                camera_id.values.append(0)
                u_of_point.values.append(cur_pts[i][0])
                v_of_point.values.append(cur_pts[i][1])

                cur_un_pts_map[ids[i]] = un_pts

            if len(prev_un_pts_map) == 0:
                for index in range(len(cur_un_pts_map)):
                    velocity_x_of_point.values.append(0.0)
                    velocity_y_of_point.values.append(0.0)
            else:
                dt = cur_time - prev_time
                for i in range(len(ids)):
                    if ids[i] in prev_un_pts_map:
                        v_x = (cur_un_pts_map[ids[i]].x -
                               prev_un_pts_map[ids[i]].x) / dt
                        v_y = (cur_un_pts_map[ids[i]].y -
                               prev_un_pts_map[ids[i]].y) / dt
                        velocity_x_of_point.values.append(v_x)
                        velocity_y_of_point.values.append(v_y)
                    else:
                        velocity_x_of_point.values.append(0.0)
                        velocity_y_of_point.values.append(0.0)

            feature_points.channels.append(id_of_point)
            feature_points.channels.append(camera_id)
            feature_points.channels.append(u_of_point)
            feature_points.channels.append(v_of_point)
            feature_points.channels.append(velocity_x_of_point)
            feature_points.channels.append(velocity_y_of_point)

            prev_un_pts_map = cur_un_pts_map.copy()
            prev_time = cur_time

            pub_img.publish(feature_points)

            ptr_toImageMsg = Image()

            ptr_toImageMsg.header = img_msg.header
            ptr_toImageMsg.height = param.height * param.scale
            ptr_toImageMsg.width = param.width * param.scale
            ptr_toImageMsg.encoding = 'bgr8'

            ptr_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

            for pt in kpts0:
                pt2 = (int(round(pt[0])), int(round(pt[1])))
                cv2.circle(ptr_image, pt2, 2, (0, 255, 0), thickness=2)

            ptr_toImageMsg.data = np.array(ptr_image).tobytes()
            pub_match.publish(ptr_toImageMsg)

            # debug matching
            # valid = matches > -1
            # mkpts0 = kpts0[valid]
            # mkpts1 = kpts1[matches[valid]]
            # k_thresh = matching.superpoint.config['keypoint_threshold']
            # m_thresh = matching.superglue.config['match_threshold']
            # color = cm.jet(confidence[valid])
            # text = [
            #     'SuperGlue',
            #     'Keypoints: {}:{}'.format(len(kpts0), len(kpts1)),
            #     'Matches: {}'.format(len(mkpts0))
            # ]
            # small_text = [
            #     'Keypoint Threshold: {:.4f}'.format(k_thresh),
            #     'Match Threshold: {:.2f}'.format(m_thresh),
            #     'Image Pair: {:06}:{:06}'.format(0, 0),
            # ]
            # out = make_matching_plot_fast(
            #     last_conver_img, conver_img, kpts0, kpts1, mkpts0, mkpts1, color, text,
            #     path=None, small_text=small_text)
            # cv2.imshow('SuperGlue matches', out)
            # key = chr(cv2.waitKey(1) & 0xFF)

    frame_tensor = frame2tensor(conver_img, "cuda")
    last_data = matching.superpoint({'image': frame_tensor})
    last_data = {k+'0': last_data[k] for k in keys}
    last_data['image0'] = frame_tensor

    last_conver_img = conver_img


#####################################################################
###########################   main入口  ##############################
#####################################################################
if __name__ == '__main__':

    rospy.init_node('feature_tracker', anonymous=False)

    ############## 加载参数 #################
    Option_Param = readParameters()
    print(Option_Param)

    config = {
        'superpoint': {
            'nms_radius': Option_Param.nms_dist,
            'keypoint_threshold': Option_Param.conf_thresh,
            'max_keypoints': Option_Param.max_cnt
        },
        'superglue': {
            'weights': Option_Param.superglue,
            'sinkhorn_iterations': Option_Param.sinkhorn_iterations,
            'match_threshold': Option_Param.match_threshold,
        }
    }
    matching = Matching(config).eval().to("cuda")

    keys = ['keypoints', 'scores', 'descriptors']

    CamearIntrinsicParam = PinholeCamera(
        fx=378.60955810546875, fy=378.60955810546875, cx=315.99139404296875, cy=231.82102966308594,
        k1=0.0, k2=0.0, p1=0.0, p2=0.0
    )
    FeatureParameters = VisualTracker(Option_Param, CamearIntrinsicParam)

    sub_img = rospy.Subscriber("/camera/infra1/image_rect_raw",
                               Image, img_callback, FeatureParameters,  queue_size=1000)

    pub_img = rospy.Publisher(
        "/feature_tracker/feature", PointCloud, queue_size=1000)
    pub_match = rospy.Publisher(
        "/feature_tracker/feature_img", Image, queue_size=1000)

    rospy.spin()
