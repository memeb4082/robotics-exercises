from assignment1_4 import * 
import numpy as np

pstar = np.array([150, 14, 150])
print(f"pstar: {pstar}")
print(f"ik: {np.degrees(inverse_kinematics(pstar))}")
print(f"ik geom: {np.degrees(inverse_kinematics_geom(pstar))}")
print(f"err: {np.linalg.norm(forward_kinematics(inverse_kinematics(pstar)) - pstar)}")
print(f"err geom: {np.linalg.norm(forward_kinematics(inverse_kinematics_geom(pstar)) - pstar)}")
