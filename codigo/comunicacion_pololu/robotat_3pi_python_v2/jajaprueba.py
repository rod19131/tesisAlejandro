import pandas as pd
from scipy.spatial.transform import Rotation

rot = Rotation.from_quat([0.7071, 0.0, 0.0, 0.7071])
rot_euler = rot.as_euler('xyz', degrees=True)
print(rot_euler)