#!/usr/bin/env python

# Software License Agreement (Apache License)
#
# Copyright 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import argparse
import em
import math
import os
import subprocess
import sys
import yaml

this_dir = os.path.abspath(os.path.dirname(__file__))
template_files = [
    os.path.join(this_dir, '..', '..', 'share', 'osrf_gear', 'worlds', 'gear.world.template'),
    os.path.join(this_dir, '..', '..', 'share', 'osrf_gear', 'launch', 'gear.launch.template'),
]
arm_configs = {
    'ur10': {
        'default_initial_joint_states': {
            'linear_arm_actuator_joint': 0,
            'shoulder_pan_joint': 0,
            'shoulder_lift_joint': -1.13,
            'elbow_joint': 1.51,
            'wrist_1_joint': 3.77,
            'wrist_2_joint': -1.51,
            'wrist_3_joint': 0,
        }
    },
}
sensor_configs = {
    'break_beam': None,
    'proximity_sensor': None,
    'logical_camera': None,
    'laser_profiler': None,
}
default_bin_origins = {
  'bin1': [-1.67, 0.66, 0.75],
  'bin2': [-1.67, -0.1, 0.75],
  'bin3': [-1.67, -0.86, 0.75],
  'bin4': [-1.67, -1.62, 0.75],
  'bin5': [-0.91, -1.62, 0.75],
  'bin6': [-0.91, -0.86, 0.75],
  'bin7': [-0.91, -0.1, 0.75],
  'bin8': [-0.91, 0.66, 0.75],
}


def prepare_arguments(parser):
    add = parser.add_argument
    add('-n', '--dry-run', action='store_true', default=False,
        help='print generated files to stdout, but do not write them to disk')
    add('-o', '--output', default=os.getcwd(),
        help='directory in which to output the generated files')
    mex_group = parser.add_mutually_exclusive_group(required=True)
    add = mex_group.add_argument
    add('config', nargs="?", metavar="CONFIG",
        help='yaml string that is the configuration')
    add('-f', '--file', nargs='+', help='list of paths to yaml files that contain the '
    'configuration (contents will be concatenated)')


eval_local_vars = {n: getattr(math, n) for n in dir(math) if not n.startswith('__')}


def expand_to_float(val):
    if isinstance(val, str):
        return float(eval(val, {}, eval_local_vars))
    return float(val)


def expand_yaml_substitutions(yaml_dict):
    for k, v in yaml_dict.items():
        if isinstance(v, dict):
            yaml_dict[k] = expand_yaml_substitutions(v)
        if k in ['xyz', 'rpy']:
            yaml_dict[k] = [expand_to_float(x) for x in v]
        if k in ['initial_joint_states']:
            yaml_dict[k] = {kp: expand_to_float(vp) for kp, vp in v.items()}
    return yaml_dict


class ArmInfo:
    def __init__(self, arm_type, initial_joint_states):
        self.type = arm_type
        self.initial_joint_states = initial_joint_states


class ModelToSpawnInfo:
    def __init__(self, name, model_type, pose, reference_frame):
        self.name = name
        self.type = model_type
        self.pose = pose
        self.reference_frame = reference_frame

class SensorInfo:
    def __init__(self, name, sensor_type, pose):
        self.name = name
        self.type = sensor_type
        self.pose = pose


class PoseInfo:
    def __init__(self, xyz, rpy):
        self.xyz = [str(f) for f in xyz]
        self.rpy = [str(f) for f in rpy]


class Options:
    def __init__(self, disable_shadows=False):
        self.disable_shadows = disable_shadows


def get_required_field(entry_name, data_dict, required_entry):
    if required_entry not in data_dict:
        print("Error: '{0}' entry does not contain a required '{1}' entry"
              .format(entry_name, required_entry),
              file=sys.stderr)
        sys.exit(1)
    return data_dict[required_entry]


def create_arm_info(arm_dict):
    arm_type = get_required_field('arm', arm_dict, 'type')
    initial_joint_states = get_required_field('arm', arm_dict, 'initial_joint_states')
    if arm_type not in arm_configs:
        print("Error: arm type '{0}' is not one of the valid arm entries: {1}"
              .format(arm_type, arm_configs), file=sys.stderr)
        sys.exit(1)
    default_joint_states = arm_configs[arm_type]['default_initial_joint_states']
    merged_initial_joint_states = dict(default_joint_states)
    for k, v in initial_joint_states.items():
        if k not in merged_initial_joint_states:
            print("Error: given joint name '{0}' is not one of the known joint "
                  "states for the '{1}' type arm: {2}".format(k, arm_type, default_joint_states),
                  file=sys.stderr)
            sys.exit(1)
        merged_initial_joint_states[k] = v
    return ArmInfo(arm_type, merged_initial_joint_states)


def create_pose_info(pose_dict):
    xyz = get_required_field('pose', pose_dict, 'xyz')
    rpy = get_required_field('pose', pose_dict, 'rpy')
    for key in pose_dict:
        if key not in ['xyz', 'rpy']:
            print("Warning: ignoring unknown entry in 'pose': " + key, file=sys.stderr)
    return PoseInfo(xyz, rpy)


def create_sensor_info(name, sensor_data):
    sensor_type = get_required_field(name, sensor_data, 'type')
    pose_dict = get_required_field(name, sensor_data, 'pose')
    for key in sensor_data:
        if key not in ['type', 'pose']:
            print("Warning: ignoring unknown entry in '{0}': {1}"
                  .format(name, key), file=sys.stderr)
    if sensor_type not in sensor_configs:
        print("Error: given sensor type '{0}' is not one of the known sensor types: {1}"
              .format(sensor_type, sensor_configs.keys()), file=sys.stderr)
    pose_info = create_pose_info(pose_dict)
    return SensorInfo(name, sensor_type, pose_info)


def create_sensor_infos(sensors_dict):
    sensor_infos = {}
    for name, sensor_data in sensors_dict.items():
        sensor_infos[name] = create_sensor_info(name, sensor_data)
    return sensor_infos


def create_model_to_spawn_info(model_name, model_to_spawn_data):
    model_type = get_required_field(model_name, model_to_spawn_data, 'type')
    pose_dict = get_required_field(model_name, model_to_spawn_data, 'pose')
    reference_frame = get_required_field(model_name, model_to_spawn_data, 'reference_frame')
    for key in model_to_spawn_data:
        if key not in ['type', 'pose', 'reference_frame']:
            print("Warning: ignoring unknown entry in '{0}': {1}"
                  .format(model_name, key), file=sys.stderr)
    pose_info = create_pose_info(pose_dict)
    return ModelToSpawnInfo(model_name, model_type, pose_info, reference_frame)


def create_models_to_spawn_infos(models_to_spawn_dict):
    models_to_spawn_infos = []
    for reference_frame, reference_frame_data in models_to_spawn_dict.items():
        models = get_required_field(reference_frame, reference_frame_data, 'models')
        model_count = 0
        for model_name, model_to_spawn_data in models.items():
            model_to_spawn_data['reference_frame'] = reference_frame
            # assign each model a unique name because gazebo can't do this
            # if the models all spawn at the same time
            model_name = reference_frame + '::' + model_name + '_' + str(model_count)
            models_to_spawn_infos.append(create_model_to_spawn_info(
                model_name, model_to_spawn_data))
            model_count += 1
    return models_to_spawn_infos


def create_models_over_bins_infos(models_over_bins_dict):
    models_to_spawn_infos = []
    for bin_name, bin_dict in models_over_bins_dict.items():
        if bin_name in default_bin_origins:
            offset_xyz = default_bin_origins[bin_name]
            # Allow the origin of the bin to be over-written
            if 'xyz' in bin_dict:
                offset_xyz = bin_dict['xyz']
        else:
            offset_xyz = get_required_field(bin_name, bin_dict, 'xyz')

        models = get_required_field(bin_name, bin_dict, 'models') or {}
        for model_type, model_to_spawn_dict in models.items():
            model_count = 0
            model_to_spawn_data = {}
            model_to_spawn_data['type'] = model_type
            model_to_spawn_data['reference_frame'] = 'world'
            xyz_start = get_required_field(
                model_type, model_to_spawn_dict, 'xyz_start')
            xyz_end = get_required_field(
                model_type, model_to_spawn_dict, 'xyz_end')
            rpy = get_required_field(model_type, model_to_spawn_dict, 'rpy')
            num_models_x = get_required_field(
                model_type, model_to_spawn_dict, 'num_models_x')
            num_models_y = get_required_field(
                model_type, model_to_spawn_dict, 'num_models_y')
            step_size = [
                (xyz_end[0] - xyz_start[0]) / max(1, num_models_x - 1),
                (xyz_end[1] - xyz_start[1]) / max(1, num_models_y - 1)]    

            # Create a grid of models
            for idx_x in range(num_models_x):
                for idx_y in range(num_models_y):
                    xyz = [
                        offset_xyz[0] + xyz_start[0] + idx_x * step_size[0],
                        offset_xyz[1] + xyz_start[1] + idx_y * step_size[1],
                        offset_xyz[2] + xyz_start[2]] 
                    model_to_spawn_data['pose'] = {'xyz': xyz, 'rpy': rpy}
                    # assign each model a unique name because gazebo can't do this
                    # if the models all spawn at the same time
                    model_name = bin_name + '::' + model_type + '_' + str(model_count)
                    models_to_spawn_infos.append(create_model_to_spawn_info(
                        model_name, model_to_spawn_data))
                    model_count += 1
    return models_to_spawn_infos


def create_options_object():
    return Options()


def prepare_template_data(config_dict):
    template_data = {
        'arm': None,
        'sensors': {},
        'models_to_insert': [],
        'models_to_spawn': []}
    for key, value in config_dict.items():
        if key == 'arm':
            template_data['arm'] = create_arm_info(value)
        elif key == 'sensors':
            template_data['sensors'].update(
                create_sensor_infos(value))
        elif key == 'models_over_bins':
            template_data['models_to_insert'].extend(
                create_models_over_bins_infos(value))
        elif key == 'models_to_spawn':
            template_data['models_to_spawn'].extend(
                create_models_to_spawn_infos(value))
        else:
            print("Error: unknown top level entry '{0}'".format(key), file=sys.stderr)
            sys.exit(1)
    template_data['options'] = create_options_object()
    return template_data


def generate_files(template_data):
    files = {}
    for template_file in template_files:
        with open(template_file, 'r') as f:
            data = f.read()
        files[template_file] = em.expand(data, template_data)
    return files


def main(sysargv=None):
    parser = argparse.ArgumentParser(
        description='Prepares and then executes a gazebo simulation based on configurations.')
    prepare_arguments(parser)
    args = parser.parse_args(sysargv)
    config_data = args.config or ''
    if args.file is not None:
        for file in args.file:
            with open(file, 'r') as f:
                comp_config_data = f.read()
                config_data += comp_config_data
    dict_config = yaml.load(config_data)
    expanded_dict_config = expand_yaml_substitutions(dict_config)
    print(yaml.dump({'Using configuration': expanded_dict_config}))
    template_data = prepare_template_data(expanded_dict_config)
    files = generate_files(template_data)
    if not args.dry_run and not os.path.isdir(args.output):
        if os.path.exists(args.output) and not os.path.isdir(args.output):
            print("Error, given output directory exists but is not a directory.", file=sys.stderr)
            sys.exit(1)
        print('creating directory: ' + args.output)
        os.makedirs(args.output)
    for name, content in files.items():
        if name.endswith('.template'):
            name = name[:-len('.template')]
        name = os.path.basename(name)
        if args.dry_run:
            print('# file: ' + name)
            print(content)
        else:
            file_path = os.path.join(args.output, name)
            print('writing file ' + file_path)
            with open(file_path, 'w+') as f:
                f.write(content)
    cmd = [
        'roslaunch',
        os.path.join(args.output, 'gear.launch'),
        'world_path:=' + os.path.join(args.output, 'gear.world')
    ]
    print("Running command: " + ' '.join(cmd))
    if not args.dry_run:
        try:
            p = subprocess.Popen(cmd)
            p.wait()
        except KeyboardInterrupt:
            pass
        finally:
            p.wait()
        return p.returncode

if __name__ == '__main__':
    sys.exit(main())
