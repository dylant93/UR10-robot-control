# -*- coding: utf-8 -*-
"""
Created on Thu Jun  3 02:22:03 2021

@author: dylantan1993@gmail.com

"""
import argparse
import open3d as o3d
import os
import json
import sys
from os.path import exists, isfile, join, splitext, dirname, basename
from warnings import warn

pwd = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(pwd, '..'))
# from initialize_config import initialize_config
def set_default_value(config, key, value):
    if key not in config:
        config[key] = value
        
def initialize_config(config):

    # set default parameters if not specified
    set_default_value(config, "depth_map_type", "redwood")
    set_default_value(config, "n_frames_per_fragment", 100)
    set_default_value(config, "n_keyframes_per_n_frame", 5)
    set_default_value(config, "min_depth", 0.3)
    set_default_value(config, "max_depth", 3.0)
    set_default_value(config, "voxel_size", 0.05)
    set_default_value(config, "max_depth_diff", 0.07)
    set_default_value(config, "depth_scale", 1000)
    set_default_value(config, "preference_loop_closure_odometry", 0.1)
    set_default_value(config, "preference_loop_closure_registration", 5.0)
    set_default_value(config, "tsdf_cubic_size", 3.0)
    set_default_value(config, "icp_method", "color")
    set_default_value(config, "global_registration", "ransac")
    set_default_value(config, "python_multi_threading", True)

    # `slac` and `slac_integrate` related parameters.
    # `voxel_size` and `min_depth` paramters from previous section,
    # are also used in `slac` and `slac_integrate`.
    set_default_value(config, "max_iterations", 5)
    set_default_value(config, "sdf_trunc", 0.04)
    set_default_value(config, "block_count", 40000)
    set_default_value(config, "distance_threshold", 0.07)
    set_default_value(config, "fitness_threshold", 0.3)
    set_default_value(config, "regularizer_weight", 1)
    set_default_value(config, "method", "slac")
    set_default_value(config, "device", "CPU:0")
    set_default_value(config, "save_output_as", "pointcloud")
    set_default_value(config, "folder_slac", "slac/")
    set_default_value(config, "template_optimized_posegraph_slac",
                      "optimized_posegraph_slac.json")

    # path related parameters.
    set_default_value(config, "folder_fragment", "fragments/")
    set_default_value(config, "subfolder_slac",
                      "slac/%0.3f/" % config["voxel_size"])
    set_default_value(config, "template_fragment_posegraph",
                      "fragments/fragment_%03d.json")
    set_default_value(config, "template_fragment_posegraph_optimized",
                      "fragments/fragment_optimized_%03d.json")
    set_default_value(config, "template_fragment_pointcloud",
                      "fragments/fragment_%03d.ply")
    set_default_value(config, "folder_scene", "scene/")
    set_default_value(config, "template_global_posegraph",
                      "scene/global_registration.json")
    set_default_value(config, "template_global_posegraph_optimized",
                      "scene/global_registration_optimized.json")
    set_default_value(config, "template_refined_posegraph",
                      "scene/refined_registration.json")
    set_default_value(config, "template_refined_posegraph_optimized",
                      "scene/refined_registration_optimized.json")
    set_default_value(config, "template_global_mesh", "scene/integrated.ply")
    set_default_value(config, "template_global_traj", "scene/trajectory.log")

    if config["path_dataset"].endswith(".bag"):
        assert os.path.isfile(config["path_dataset"]), (
            f"File {config['path_dataset']} not found.")
        print("Extracting frames from RGBD video file")
        config["path_dataset"], config["path_intrinsic"], config[
            "depth_scale"] = extract_rgbd_frames(config["path_dataset"])

def extract_rgbd_frames(rgbd_video_file):
    """
    Extract color and aligned depth frames and intrinsic calibration from an
    RGBD video file (currently only RealSense bag files supported). Folder
    structure is:
        <directory of rgbd_video_file/<rgbd_video_file name without extension>/
            {depth/00000.jpg,color/00000.png,intrinsic.json}
    """
    frames_folder = join(dirname(rgbd_video_file),
                         basename(splitext(rgbd_video_file)[0]))
    path_intrinsic = join(frames_folder, "intrinsic.json")
    if isfile(path_intrinsic):
        warn(f"Skipping frame extraction for {rgbd_video_file} since files are"
             " present.")
    else:
        rgbd_video = o3d.t.io.RGBDVideoReader.create(rgbd_video_file)
        rgbd_video.save_frames(frames_folder)
    with open(path_intrinsic) as intr_file:
        intr = json.load(intr_file)
    depth_scale = intr["depth_scale"]
    return frames_folder, path_intrinsic, depth_scale

class ReaderWithCallback:

    def __init__(self, input, output):
        self.flag_exit = False
        self.flag_play = True
        self.input = input
        self.output = output

        self.reader = o3d.io.AzureKinectMKVReader()
        self.reader.open(self.input)
        if not self.reader.is_opened():
            raise RuntimeError("Unable to open file {}".format(args.input))

    def escape_callback(self, vis):
        self.flag_exit = True
        return False

    def space_callback(self, vis):
        if self.flag_play:
            print('Playback paused, press [SPACE] to continue.')
        else:
            print('Playback resumed, press [SPACE] to pause.')
        self.flag_play = not self.flag_play
        return False

    def run(self):
        glfw_key_escape = 256
        glfw_key_space = 32
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.register_key_callback(glfw_key_escape, self.escape_callback)
        vis.register_key_callback(glfw_key_space, self.space_callback)

        vis_geometry_added = False
        vis.create_window('reader', 1920, 540)

        print(
            "MKV reader initialized. Press [SPACE] to pause/start, [ESC] to exit."
        )

        if self.output is not None:
            abspath = os.path.abspath(self.output)
            metadata = self.reader.get_metadata()
            o3d.io.write_azure_kinect_mkv_metadata(
                '{}/intrinsic.json'.format(abspath), metadata)

            config = {
                'path_dataset': abspath,
                'path_intrinsic': '{}/intrinsic.json'.format(abspath)
            }
            initialize_config(config)
            with open('{}/config.json'.format(abspath), 'w') as f:
                json.dump(config, f, indent=4)

        idx = 0
        while not self.reader.is_eof() and not self.flag_exit:
            if self.flag_play:
                rgbd = self.reader.next_frame()
                if rgbd is None:
                    continue

                if not vis_geometry_added:
                    vis.add_geometry(rgbd)
                    vis_geometry_added = True

                if self.output is not None:
                    color_filename = '{0}/color/{1:05d}.jpg'.format(
                        self.output, idx)
                    print('Writing to {}'.format(color_filename))
                    o3d.io.write_image(color_filename, rgbd.color)

                    depth_filename = '{0}/depth/{1:05d}.png'.format(
                        self.output, idx)
                    print('Writing to {}'.format(depth_filename))
                    o3d.io.write_image(depth_filename, rgbd.depth)
                    idx += 1

            try:
                vis.update_geometry(rgbd)
            except NameError:
                pass
            vis.poll_events()
            vis.update_renderer()

        self.reader.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv reader.')
    parser.add_argument('--input',
                        type=str,
                        required=True,
                        help='input mkv file')
    parser.add_argument('--output',
                        type=str,
                        help='output path to store color/ and depth/ images')
    args = parser.parse_args()

    if args.input is None:
        parser.print_help()
        exit()

    if args.output is None:
        print('No output path, only play mkv')
    elif os.path.isdir(args.output):
        print('Output path {} already existing, only play mkv'.format(
            args.output))
        args.output = None
    else:
        try:
            os.mkdir(args.output)
            os.mkdir('{}/color'.format(args.output))
            os.mkdir('{}/depth'.format(args.output))
        except (PermissionError, FileExistsError):
            print('Unable to mkdir {}, only play mkv'.format(args.output))
            args.output = None

    reader = ReaderWithCallback(args.input, args.output)
    reader.run()