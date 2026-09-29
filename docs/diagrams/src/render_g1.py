"""Render the real Unitree G1 URDF meshes with the diagram camera into a transparent PNG.

Usage: PYOPENGL_PLATFORM=egl python3 render_g1.py /path/to/eut_g1/description_files/urdf/g1_29dof.urdf
Writes g1_render.png and g1_render.json (placement in SVG coordinates) next to this script.
Requires: numpy, trimesh, yourdfpy, pyrender, pillow.
"""
import json
import os
import sys
from pathlib import Path

import numpy as np
import pyrender
import trimesh
import yourdfpy
from PIL import Image

import scene_camera as cam

SUPERSAMPLE = 2
HERE = Path(__file__).resolve().parent

# Natural standing pose: the G1 zero configuration has the elbows bent 90 degrees forward.
POSE = {
    'left_elbow_joint': 1.3,
    'right_elbow_joint': 1.3,
    'left_shoulder_roll_joint': 0.12,
    'right_shoulder_roll_joint': -0.12,
}


def main(urdf_path: str) -> None:
    description_dir = Path(urdf_path).resolve().parents[1]

    def resolve(fname: str) -> str:
        return str(description_dir / fname.split('description_files/', 1)[1]) if 'description_files/' in fname else fname

    robot = yourdfpy.URDF.load(urdf_path, filename_handler=resolve, load_meshes=True, build_scene_graph=True)
    cfg = [POSE.get(name, 0.0) for name in robot.actuated_joint_names]
    robot.update_cfg(cfg)

    meshes = []
    for link in robot.robot.links:
        link_tf = robot.get_transform(link.name)
        for visual in link.visuals:
            if visual.geometry.mesh is None:
                continue
            mesh = trimesh.load(resolve(visual.geometry.mesh.filename), force='mesh')
            origin = visual.origin if visual.origin is not None else np.eye(4)
            rgba = (0.7, 0.7, 0.7, 1.0)
            if visual.material is not None and visual.material.color is not None:
                rgba = tuple(visual.material.color.rgba)
            meshes.append((mesh, link_tf @ origin, rgba))

    # Put the feet on the ground at the world origin.
    min_z = min(trimesh.transform_points(m.vertices, tf)[:, 2].min() for m, tf, _ in meshes)
    lift = np.eye(4)
    lift[2, 3] = -min_z

    scene = pyrender.Scene(bg_color=[0, 0, 0, 0], ambient_light=[0.30, 0.28, 0.34])
    for mesh, tf, rgba in meshes:
        light_part = rgba[0] > 0.5
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=[0.66, 0.67, 0.71, 1.0] if light_part else [0.07, 0.07, 0.08, 1.0],
            metallicFactor=0.15 if light_part else 0.35,
            roughnessFactor=0.45 if light_part else 0.55,
        )
        scene.add(pyrender.Mesh.from_trimesh(mesh, material=material, smooth=False), pose=lift @ tf)

    width, height = cam.WIDTH * SUPERSAMPLE, cam.HEIGHT * SUPERSAMPLE
    camera = pyrender.IntrinsicsCamera(
        fx=cam.FOCAL * SUPERSAMPLE, fy=cam.FOCAL * SUPERSAMPLE,
        cx=cam.CX * SUPERSAMPLE, cy=cam.CY * SUPERSAMPLE, znear=0.05, zfar=50.0,
    )
    camera_pose = np.eye(4)
    camera_pose[:3, 0] = cam.RIGHT
    camera_pose[:3, 1] = cam.UP
    camera_pose[:3, 2] = [-f for f in cam.FORWARD]
    camera_pose[:3, 3] = cam.CAMERA_POSITION
    scene.add(camera, pose=camera_pose)

    def light_pose(direction):
        z = -np.asarray(direction, dtype=float)
        z /= np.linalg.norm(z)
        x = np.cross([0, 0, 1], z)
        x /= np.linalg.norm(x)
        pose = np.eye(4)
        pose[:3, 0], pose[:3, 1], pose[:3, 2] = x, np.cross(z, x), z
        return pose

    scene.add(pyrender.DirectionalLight(color=[1.0, 0.98, 1.0], intensity=2.6), pose=light_pose([0.6, 0.8, -0.9]))
    scene.add(pyrender.DirectionalLight(color=[0.85, 0.80, 1.0], intensity=1.0), pose=light_pose([-0.7, 0.3, -0.4]))
    scene.add(pyrender.DirectionalLight(color=[1.0, 1.0, 1.0], intensity=0.8), pose=light_pose([0.1, -1.0, -0.2]))

    renderer = pyrender.OffscreenRenderer(width, height)
    color, _ = renderer.render(scene, flags=pyrender.RenderFlags.RGBA)
    renderer.delete()

    image = Image.fromarray(color, 'RGBA')
    bbox = image.getbbox()
    image = image.crop(bbox)
    image.save(HERE / 'g1_render.png', optimize=True)
    placement = {
        'x': bbox[0] / SUPERSAMPLE, 'y': bbox[1] / SUPERSAMPLE,
        'width': (bbox[2] - bbox[0]) / SUPERSAMPLE, 'height': (bbox[3] - bbox[1]) / SUPERSAMPLE,
    }
    (HERE / 'g1_render.json').write_text(json.dumps(placement, indent=2) + '\n')
    print('rendered', placement)


if __name__ == '__main__':
    os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
    main(sys.argv[1])
