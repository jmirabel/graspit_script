import open3d as o3d
import pathlib
import yaml
import dataclasses
import numpy as np
import quaternion
import typing as T
import functools
import xml.etree.ElementTree as etree


def generate_ply(
    color_fn,
    depth_fn,
    ply_fn,
    width: int,
    height: int,
    intrinsic_matrix,
    depth_scale: float,
):
    color = o3d.io.read_image(color_fn)
    depth = o3d.io.read_image(depth_fn)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, intrinsic_matrix)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        convert_rgb_to_intensity=False,
        depth_trunc=float("inf"),
        depth_scale=1000 / depth_scale,
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # flip the orientation, so it looks upright, not upside-down
    # pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

    # o3d.visualization.draw_geometries([pcd])    # visualize the point cloud
    o3d.io.write_point_cloud(ply_fn, pcd)


def generate_ply_in_dir(dir: pathlib.Path, width: int, height: int):
    ply_dir = dir / "ply"
    ply_dir.mkdir(exist_ok=True)
    with open(dir / "info.yml", "r") as f:
        info = yaml.safe_load(f)
    for id in info:
        id_str = f"{id:04d}"
        print("handling", id_str)
        generate_ply(
            dir / "rgb" / (id_str + ".png"),
            dir / "depth" / (id_str + ".png"),
            dir / "ply" / (id_str + ".ply"),
            width,
            height,
            np.asarray(info[id]["cam_K"]).reshape((3, 3)),
            info[id]["depth_scale"],
        )


@dataclasses.dataclass
class Transform:
    t: np.ndarray = dataclasses.field(
        default_factory=functools.partial(np.zeros, shape=(3,))
    )
    R: np.ndarray = dataclasses.field(default_factory=functools.partial(np.eye, N=3))

    @staticmethod
    def from_gt_file(gt: T.Dict):
        t = np.array(gt["cam_t_m2c"])
        R = np.array(gt["cam_R_m2c"]).reshape((3,3))
        assert t.shape == (3,)
        assert R.shape == (3,3)
        return Transform(t=t, R=R)
    
    def inverse(self):
        assert False, "not tested"
        return Transform(t = - self.R * self.t, R = self.R.T)

    @property
    def quat(self):
        return quaternion.from_rotation_matrix(self.R)

    def graspit_format(self) -> str:
        q = self.quat
        t = self.t * 1000  # millimeters
        return f"({q.w:+f} {q.x:+f} {q.y:+f} {q.z:+f})[{t[0]:+f} {t[1]:+f} {t[2]:+f}]"


def generate_graspit_world(ground_truth: T.Dict[int, Transform], obs_xml: str, world_fn: pathlib.Path):
    world = etree.Element("world")
    # Obstacle
    obstacle = etree.SubElement(world, "obstacle")
    etree.SubElement(obstacle, "filename").text = obs_xml
    tf = etree.SubElement(obstacle, "transform")
    etree.SubElement(tf, "fullTransform").text = Transform().graspit_format()
    # Graspable bodies
    for id, M in ground_truth.items():
        gb = etree.SubElement(world, "graspableBody")
        etree.SubElement(gb, "filename").text = f"models/objects/tless/obj_{id:02d}.xml"
        tf = etree.SubElement(gb, "transform")
        etree.SubElement(tf, "fullTransform").text = M.graspit_format()

    # Robot
    robot = etree.SubElement(world, "robot")
    etree.SubElement(robot, "filename").text = "models/robots/pr2_gripper/pr2_gripper.xml"
    etree.SubElement(robot, "dofValues").text = "0"
    tf = etree.SubElement(robot, "transform")
    etree.SubElement(tf, "fullTransform").text = Transform().graspit_format()

    tree = etree.ElementTree(world) 
    with open (world_fn, "wb") as f:
        tree.write(f)


class Dataset:
    def __init__(self, tless_data_dir: pathlib.Path,
                 graspit_data_dir: pathlib.Path,
                 tless_type: str = "test",
                 tless_camera: str = "kinect",
                 tless_scene: int = 1,
                 ):
        self.tless_data_dir = tless_data_dir
        self.tless_scene_dir = tless_data_dir / f"{tless_type}_{tless_camera}" / f"{tless_scene:02d}"
        self.graspit_data_dir = graspit_data_dir
        self.tless_type = tless_type
        self.tless_camera = tless_camera
        self.tless_scene = tless_scene
    
    def tless_object(self, obj_id):
        return self.tless_data_dir / "models_cad" / f"obj_{obj_id:02d}.ply"
    
    def tless_scene_ply(self, id):
        return self.tless_scene_dir / "ply" / f"{id:04d}.ply"

    def graspit_object(self, obj_id, extension: str = "xml"):
        return self.graspit_data_dir / "models" / "objects" / "tless" / f"obj_{obj_id:02}.{extension}"

    def graspit_obstacle(self, extension: str = "xml"):
        return self.graspit_data_dir / "models" / "obstacles" / "tless" / f"{self.tless_type}_{self.tless_camera}" / f"{self.tless_scene:02d}.{extension}"
    
    def graspit_world(self):
        return self.graspit_data_dir / "worlds" / "tless" / f"{self.tless_type}_{self.tless_camera}" / f"{self.tless_scene:02d}.xml"

def generate_graspit_worlds(dataset: Dataset):
    with open(dataset.tless_scene_dir / "gt.yml") as f:
        gt = yaml.safe_load(f)
    
    for id, rgt in gt.items():
        ground_truth = { obj_id: Transform.from_gt_file(obj_gt_data) for obj_id, obj_gt_data in rgt.items() }
        # Copy the ply file of the object and of the scene.
        # Create the object xml file.


dir = pathlib.Path("t-less_v2/test_kinect/01")
w, h = 720, 540
# generate_ply_in_dir(dir, w, h)
