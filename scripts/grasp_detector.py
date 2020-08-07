#! /usr/bin/env python

camera_intr_filename = "/home/bostoncleek/gqcnn_ws/src/gqcnn_demo/config/calib/kinect.intr"
color_img_filename = "/home/bostoncleek/gqcnn_ws/src/gqcnn_demo/data/images/object_rgb.jpg"
depth_img_filename = "/home/bostoncleek/gqcnn_ws/src/gqcnn_demo/data/images/object_depth.jpg"

model_name = "GQCNN-4.0-PJ"
model_dir = "/home/bostoncleek/dex-net/deps/gqcnn/models/GQCNN-4.0-PJ"
config_filename = "/home/bostoncleek/dex-net/deps/gqcnn/cfg/examples/replication/dex-net_4.0_pj.yaml"

import json
import os
import cv2


from autolab_core import YamlConfig, Logger
from perception import (BinaryImage, CameraIntrinsics, ColorImage, DepthImage, RgbdImage)


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

    inpaint_rescale_factor = config["inpaint_rescale_factor"]

    # sensor
    camera_intr = CameraIntrinsics.load(camera_intr_filename)

    # images
    color_cvmat = cv2.imread(color_img_filename)
    depth_cvmat = cv2.imread(depth_img_filename)

    # create wrapped BerkeleyAutomation/perception RGB and depth images
    color_im = ColorImage(color_cvmat, frame=camera_intr.frame, encoding="rgb8")
    depth_im = ColorImage(depth_cvmat, frame=camera_intr.frame, encoding="rgb8")

    # check image sizes.
    if (color_im.height != depth_im.height or color_im.width != depth_im.width):
        msg = ("Color image and depth image must be the same shape! Color"
               " is %d x %d but depth is %d x %d") % (
                   color_im.height, color_im.width, depth_im.height,
                   depth_im.width)
        print(msg)



















#
