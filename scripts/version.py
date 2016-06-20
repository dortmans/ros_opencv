#!/usr/bin/env python

import cv2
from distutils.version import LooseVersion

if LooseVersion(cv2.__version__).version[0] == 2:
    # OpenCV2
    print("OpenCV2")
else:
    # OpenCV3
    print("OpenCV3")
