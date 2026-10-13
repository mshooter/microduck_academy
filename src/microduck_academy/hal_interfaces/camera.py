"""
Camera interface to extract BGR image from sim.frame

Follows the Academy's CameraNode. When porting to hardware, this is the only class 
to replace. It uses `sim.frame` which exists only in the simulator. Everything else
in `hal_interfaces` already uses the real duck's protocol. 

The real duck streams images from a separate video service. Its camera is mounted
90 degrees from upright, so frames arrive sideways. The hardware CameraDuck must
rotate them upright. See docs/api.md.
"""

import base64
import io

import numpy as np
from PIL import Image

from microduck_academy.hal_interfaces.client import DuckClient


class CameraDuck:
    def __init__(self, client: DuckClient):
        self._duck = client

    def getImage(self) -> np.ndarray:
        """Head camera, 360x640x3 BGR uint8."""
        frame = self._duck.call("sim.frame")

        # Check format is as expected
        if frame["format"] != "jpeg":
            raise ValueError(f"sim.frame: expected a jpeg, got {frame['format']!r}")

        rgb = np.asarray(
            Image.open(
                io.BytesIO(base64.b64decode(frame["data"])),
            ).convert("RGB"),
        )
        # we return BGR channel order like the other
        # Academy's getImage() functions
        return rgb[:, :, ::-1].copy()
