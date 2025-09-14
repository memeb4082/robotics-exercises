import numpy as np
from time import sleep
import scipy.optimize
import coppeliaRobot as cpr
# Robot dimensions (in mm)
# L0=138, L1=135, L2=147, L3=60, and L4=-80


def rot_mat(axis, angle):
  if axis == 'x':

    r = np.array([[1, 0, 0],

                  [0, np.cos(angle), -np.sin(angle)],

                  [0, np.sin(angle), np.cos(angle)]])

  elif axis == 'y':

    r = np.array([[np.cos(angle), 0, np.sin(angle)],

                  [0, 1, 0],

                  [-np.sin(angle), 0, np.cos(angle)]])

  elif axis == 'z':

    r = np.array([[np.cos(angle), -np.sin(angle), 0],

                  [np.sin(angle), np.cos(angle), 0],

                  [0, 0, 1]])

  rot_mat = np.eye(4)

  rot_mat[:3, :3] = r

  return rot_mat


def transform_mat(x, y, z):

  return np.array([[1, 0, 0, x],

                   [0, 1, 0, y],

                   [0, 0, 1, z],

                   [0, 0, 0, 1]])



def forward_kinematics_q(q):

  q1, q2, q3, q4 = q
  return (rot_mat('z', q1) @ transform_mat(0, 0, L0) @ rot_mat('y', q2) @ transform_mat(0, 0, L1) @ rot_mat('y', q3) @ transform_mat(L2, 0, 0) @ rot_mat('y', q4) @ transform_mat(L3, 0, L4))[:3, 3]


def joint_mapping(th):


  theta1, theta2, theta3 = th

  q1 = theta1

  q2 = theta2

  q3 = theta3 - theta2

  q4 = -theta3

  return np.array([q1, q2, q3, q4])


def forward_kinematics(theta):


  # First, map physical joint angles to kinematic joint angles

  q = joint_mapping(theta)



  # Now compute forward kinematics using kinematic joint angles

  return forward_kinematics_q(q)



def cost(theta, pstar):

  p = forward_kinematics(theta)

  c = np.linalg.norm(p - pstar)

  return c




def inverse_kinematics(pstar):


    # Initial guess for theta

  theta0 = np.array([0.0, 0.0, 0.0])

    # Optimize using scipy's minimize function

  result = scipy.optimize.minimize(cost, theta0, args=(pstar,), method='L-BFGS-B')

  return result.x





# --------- Question 3 ---------- #



def inverse_kinematics_geom(pstar):

  x, y, z = pstar
  theta1 = np.arctan2(y, x)
  r = y / np.sin(theta1)
  b = L0 + L4 - z
  # Calculate a
  a = r - L3
  sigma = np.arctan2(a, b)
  c = a / np.sin(sigma)
  alpha = np.arccos( (L1**2 + L2**2 - c**2) / (2 * L1 * L2) )
  beta = np.arcsin( (L2 * np.sin(alpha)) / c )
  eta = np.pi - (alpha + beta)
  theta2 = np.pi - (beta + sigma)
  theta3 = eta + (np.pi/2 - sigma)
  return np.array([theta1, theta2, theta3])
import numpy as np

def joint_interpolated_motion(start, end, steps):

    start = inverse_kinematics_geom(start)
    end = inverse_kinematics_geom(end)
    trajectory = np.linspace(start, end, steps)
    return trajectory


def cartesian_interpolated_motion(start, end, steps):

    # Interpolate in Cartesian space
    cartesian_points = np.linspace(start, end, steps)
    
    # Convert each Cartesian point to joint angles (IK required)
    joint_trajectory = np.array([inverse_kinematics_geom(p) for p in cartesian_points])
    return joint_trajectory


def _px_to_mm(x, y, z):
  # 200px = 120mm
  scale = 120 / 200
  return np.array([x * scale, y * scale, z * scale])



def type_letter(robot, c, wait_time=0.5):
  # Move to letter using cartesian interpolation
  current_pos = robot.get_joint_config()
  target_pos = letter_coords[c] + np.array([0, 0, 20])
  trajectory = cartesian_interpolated_motion(current_pos, target_pos, steps=10)
  for joint_angles in trajectory:
      robot.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])
  sleep(wait_time)
  # Move down to letter using joint interpolation
  current_pos = robot.get_joint_config()
  target_pos = letter_coords[c]
  trajectory = joint_interpolated_motion(current_pos, target_pos, steps=10)
  for joint_angles in trajectory:
      robot.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])
  sleep(wait_time)
  # Simulate key press
  robot.set_suction_cup(True)
  sleep(wait_time)
  robot.set_suction_cup(False)
  sleep(wait_time)
  # Move back up to above letter using joint interpolation
  current_pos = robot.get_joint_config()
  target_pos = letter_coords[c] + np.array([0, 0, 20])
  trajectory = joint_interpolated_motion(current_pos, target_pos, steps=10)
  for joint_angles in trajectory:
      robot.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])
  sleep(wait_time)

if __name__ == "__main__":
    coords = letter_coords
    angles = inverse_kinematics_geom(coords['z'])
    
    print(f"Joint angles for letter 'z': {angles * 180 / np.pi}")  # convert to degrees for readability 
    WORD = "place"
    robot = cpr.CoppeliaRobot()
    for c in WORD:
        type_letter(robot, c)
    type_letter(robot, 'return')
    
