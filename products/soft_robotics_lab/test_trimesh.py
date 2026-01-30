import trimesh
import numpy as np
cylinder = trimesh.creation.cylinder(radius=5, height=20)
cylinder.export('test_cylinder.stl')
print('Trimesh OK - exported test_cylinder.stl')
