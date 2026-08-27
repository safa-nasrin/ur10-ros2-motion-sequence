import yaml
import os

def test_poses_yaml_structure():
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'poses.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    poses = data['robot_task']['ros__parameters']['poses']
    for name in ('home', 'pose_1', 'pose_2'):
        assert name in poses
        assert len(poses[name]) == 6
