# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim

import pprint
import tempfile
import os
import time

import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from matplotlib.path import Path
from matplotlib.patches import PathPatch

pp = pprint.PrettyPrinter(indent=4)

client = airsim.VehicleClient()

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")

responses = client.simGetImages([airsim.ImageRequest("depthcam", airsim.ImageType.DepthPerspective, True)])
image = airsim.get_pfm_array(responses[0])

fig, ax = plt.subplots()
ax.imshow(image, vmin=1, vmax=10)
ax.axis('off')

plt.show()
