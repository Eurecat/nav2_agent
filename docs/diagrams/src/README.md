Generators for the diagrams in `docs/diagrams/` (Eurecat-style palette in `common.py`).

`example.svg` embeds a render of the real Unitree G1 URDF meshes, taken with the same camera
(`scene_camera.py`) as the perspective floor, so robot and poses line up. Re-render it only when
the camera or robot pose changes (needs `numpy trimesh yourdfpy pyrender pillow` and EGL):

```bash
cd docs/diagrams/src
PYOPENGL_PLATFORM=egl python3 render_g1.py ~/workspaces/eut_g1/description_files/urdf/g1_29dof.urdf
```

Regenerate the SVGs and the 2x PNGs for slides:

```bash
python3 gen_flow.py ../architecture.svg
python3 gen_example.py ../example.svg
python3 gen_self_repair.py ../self_repair.svg
for n in architecture example self_repair; do
  google-chrome --headless=new --hide-scrollbars --force-device-scale-factor=2 \
    --window-size=1600,900 --screenshot=$PWD/../$n.png "file://$PWD/../$n.svg"
done
```
