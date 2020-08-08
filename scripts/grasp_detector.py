#! /usr/bin/env python

camera_intr_filename = "/home/bostoncleek/gqcnn_ws/src/gqcnn_demo/config/calib/kinect.intr"
color_img_filename = "/home/bostoncleek/gqcnn_ws/src/gqcnn_demo/data/images/object_rgb.png"
depth_img_filename = "/home/bostoncleek/gqcnn_ws/src/gqcnn_demo/data/images/object_depth.png"

model_name = "GQCNN-4.0-PJ"
model_dir = "/home/bostoncleek/dex-net/deps/gqcnn/models/GQCNN-4.0-PJ"
config_filename = "/home/bostoncleek/dex-net/deps/gqcnn/cfg/examples/replication/dex-net_4.0_pj.yaml"

import json
import os
import time
import cv2
import numpy as np
from matplotlib import pyplot as plt

from autolab_core import YamlConfig, Logger
from perception import (BinaryImage, CameraIntrinsics, ColorImage, DepthImage,
                        RgbdImage)
from visualization import Visualizer2D as vis

from gqcnn.grasping import (RobustGraspingPolicy,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState,
                            FullyConvolutionalGraspingPolicyParallelJaw,
                            FullyConvolutionalGraspingPolicySuction)
from gqcnn.utils import GripperMode


if __name__ == "__main__":

    # model weights
    model_config = json.load(open(os.path.join(model_dir, "config.json"), "r"))

    try:
        gqcnn_config = model_config["gqcnn"]
        gripper_mode = gqcnn_config["gripper_mode"]
    except KeyError:
        gqcnn_config = model_config["gqcnn_config"]
        input_data_mode = gqcnn_config["input_data_mode"]
        if input_data_mode == "tf_image":
            gripper_mode = GripperMode.LEGACY_PARALLEL_JAW
        elif input_data_mode == "tf_image_suction":
            gripper_mode = GripperMode.LEGACY_SUCTION
        elif input_data_mode == "suction":
            gripper_mode = GripperMode.SUCTION
        elif input_data_mode == "multi_suction":
            gripper_mode = GripperMode.MULTI_SUCTION
        elif input_data_mode == "parallel_jaw":
            gripper_mode = GripperMode.PARALLEL_JAW
        else:
            raise ValueError(
                "Input data mode {} not supported!".format(input_data_mode))

    # policy params
    config = YamlConfig(config_filename)
    policy_config = config["policy"]
    policy_config["metric"]["gqcnn_model"] = model_dir

    # # sensor
    camera_intr = CameraIntrinsics.load(camera_intr_filename)

    # images
    color_cvmat = cv2.imread(color_img_filename)
    depth_cvmat = cv2.imread(depth_img_filename) # load as 8UC3
    depth_cvmat = cv2.cvtColor(depth_cvmat, cv2.COLOR_BGR2GRAY) # 8UC1
    depth_cvmat = depth_cvmat.astype(np.float32) * 1.0/255.0 # 32FC1

    # cv2.imshow('img', color_cvmat)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # create wrapped BerkeleyAutomation/perception RGB and depth images
    color_im = ColorImage(color_cvmat, frame=camera_intr.frame, encoding="bgr8")
    depth_im = DepthImage(depth_cvmat, frame=camera_intr.frame)

    # check image sizes.
    if (color_im.height != depth_im.height or color_im.width != depth_im.width):
        msg = ("Color image and depth image must be the same shape! Color"
               " is %d x %d but depth is %d x %d") % (
                   color_im.height, color_im.width, depth_im.height,
                   depth_im.width)
        print(msg)

    # assume not mask is provided
    # segmask = depth_im.invalid_pixel_mask().inverse()
    segmask = BinaryImage(255 *
                          np.ones(depth_im.shape).astype(np.uint8),
                          frame=color_im.frame)

    # inpaint images.
    # color_im = color_im.inpaint(
    #     rescale_factor=config["inpaint_rescale_factor"])
    # depth_im = depth_im.inpaint(
    #     rescale_factor=config["inpaint_rescale_factor"])

    # Aggregate color and depth images into a single
    # BerkeleyAutomation/perception `RgbdImage`.
    rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)

    state = RgbdImageState(rgbd_im, camera_intr, segmask=segmask)

    # vis.imshow(segmask)
    # vis.imshow(rgbd_im)
    # vis.show()

    policy = CrossEntropyRobustGraspingPolicy(policy_config)

    # Query policy.
    policy_start = time.time()
    action = policy(state)
    print("Planning took %.3f sec" % (time.time() - policy_start))

    print("Gripper pose: ", action.grasp.pose())

    # Vis final grasp.
    if policy_config["vis"]["final_grasp"]:
        vis.figure(size=(10, 10))
        vis.imshow(rgbd_im.depth,
                   vmin=policy_config["vis"]["vmin"],
                   vmax=policy_config["vis"]["vmax"])
        vis.grasp(action.grasp, scale=2.5, show_center=False, show_axis=True)
        vis.title("Planned grasp at depth {0:.3f}m with Q={1:.3f}".format(
            action.grasp.depth, action.q_value))
        vis.show()











#
