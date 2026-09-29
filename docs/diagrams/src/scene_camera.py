"""Pinhole camera shared by the SVG floor drawing and the G1 mesh render, so both line up exactly."""
import math

WIDTH, HEIGHT = 1600, 900

CAMERA_POSITION = (1.1, -3.5, 2.1)
CAMERA_TARGET = (1.8, 0.6, 0.4)
# World region that must fit inside SCENE_BOX: the path, the poses and the robot's head.
FIT_POINTS = [(-0.3, -0.45, 0), (3.75, -0.45, 0), (3.75, 1.55, 0), (-0.3, 1.55, 0), (0, 0, 1.40)]
SCENE_BOX = (610, 170, 1570, 860)


def _sub(a, b):
    return tuple(i - j for i, j in zip(a, b))


def _dot(a, b):
    return sum(i * j for i, j in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    n = math.sqrt(_dot(a, a))
    return tuple(i / n for i in a)


FORWARD = _norm(_sub(CAMERA_TARGET, CAMERA_POSITION))
RIGHT = _norm(_cross(FORWARD, (0, 0, 1)))
UP = _cross(RIGHT, FORWARD)


def _normalized(p):
    d = _sub(p, CAMERA_POSITION)
    z = _dot(d, FORWARD)
    return _dot(d, RIGHT) / z, -_dot(d, UP) / z


_fit = [_normalized(p) for p in FIT_POINTS]
_minx, _maxx = min(u for u, _ in _fit), max(u for u, _ in _fit)
_miny, _maxy = min(v for _, v in _fit), max(v for _, v in _fit)
FOCAL = min((SCENE_BOX[2] - SCENE_BOX[0]) / (_maxx - _minx), (SCENE_BOX[3] - SCENE_BOX[1]) / (_maxy - _miny))
CX = SCENE_BOX[0] + ((SCENE_BOX[2] - SCENE_BOX[0]) - (_maxx - _minx) * FOCAL) / 2 - _minx * FOCAL
CY = SCENE_BOX[1] + ((SCENE_BOX[3] - SCENE_BOX[1]) - (_maxy - _miny) * FOCAL) / 2 - _miny * FOCAL


def project(x, y, z=0.0):
    """World point (x forward, y left, z up, meters) to SVG pixel coordinates."""
    u, v = _normalized((x, y, z))
    return CX + u * FOCAL, CY + v * FOCAL
