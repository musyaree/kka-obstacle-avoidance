"""Modul kontrol robot."""
from .follower import Controller, TangentPlane, FollowerController

try:
    from .cfmpc import CfMpcController
except ImportError:
    CfMpcController = None

__all__ = [
    "Controller",
    "TangentPlane",
    "FollowerController",
    "CfMpcController",
]