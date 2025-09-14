import numpy as np
from time import sleep
import scipy.optimize
import coppeliaRobot as cpr

class WordTypingRobot:
  def __init__(self, robotObj = None):
    self.robotObj = robotObj
    if self.robotObj is None:
      self.robotObj = cpr.CoppeliaRobot()
    self.L0 = 138
    self.L1 = 135
    self.L2 = 147
    self.L3 = 60
    self.L4 = -80
    self.wait_time = 0.2
    _z_height = 0
    # Keyboard layout done row by row for qwerty layout
    __key_spacing = np.array([33.5, 0, 0])
    __q_pos = np.array([-170, 25, _z_height])
    __a_pos = np.array([-165, -10, _z_height])
    __z_pos = np.array([-150, -45, _z_height])

    __row_1 = ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p']
    __row_2 = ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l']
    __row_3 = ['z', 'x', 'c', 'v', 'b', 'n', 'm']

    self.letter_coords = {}
    for i, letter in enumerate(__row_1):
      self.letter_coords[letter] = __q_pos + i * __key_spacing
    for i, letter in enumerate(__row_2):
      self.letter_coords[letter] = __a_pos + i * __key_spacing
    for i, letter in enumerate(__row_3):
      self.letter_coords[letter] = __z_pos + i * __key_spacing
    self.letter_coords['HOME'] = np.array([0, 0, 100])
    self.letter_coords['return'] = self.letter_coords['l'] + 3 * __key_spacing
    self.letter_coords[' '] = self.letter_coords['b'] + [0, -35, 0]

    self.letter_coords = {k: self.__px_to_mm(*v) for k, v in self.letter_coords.items()}

    origin = np.array([0.175, -0.15, 0.002]) * 1000  # convert to mm
    R = self.__rot_mat('z', -135 * np.pi / 180)[:3, :3]
    t = origin 

    for k in self.letter_coords:
      self.letter_coords[k] = R @ self.letter_coords[k] + t
  def __px_to_mm(self, x, y, z):
    # 200px = 120mm
    scale = 120 / 200
    return np.array([x * scale, y * scale, z * scale])
  def __rot_mat(self, axis, angle):
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
  def __transform_mat(self, x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])
  def __forward_kinematics_q(self, q):
    q1, q2, q3, q4 = q
    return (self.__rot_mat('z', q1) @ self.__transform_mat(0, 0, self.L0) @ self.__rot_mat('y', q2) @ self.__transform_mat(0, 0, self.L1) @ self.__rot_mat('y', q3) @ self.__transform_mat(self.L2, 0, 0) @ self.__rot_mat('y', q4) @ self.__transform_mat(self.L3, 0, self.L4))[:3, 3]
    
  def __joint_mapping(self, th):
    theta1, theta2, theta3 = th
    q1 = theta1
    q2 = theta2
    q3 = theta3 - theta2
    q4 = -theta3
    return np.array([q1, q2, q3, q4])
  
  def __forward_kinematics(self, theta):
    q = self.__joint_mapping(theta)
    return self.__forward_kinematics_q(q)
  
  def __cost(self, theta, pstar):
    p = self.__forward_kinematics(theta)
    c = np.linalg.norm(p - pstar)
    return c
  
  def __inverse_kinematics(self, pstar):
    x, y, z = pstar

    # Calculate theta1

    theta1 = np.arctan2(y, x)

    # Calculate r

    r = y / np.sin(theta1)

    # Calculate b

    b = self.L0 + self.L4 - z

    # Calculate a

    a = r - self.L3

    # Calculate sigma

    sigma = np.arctan2(a, b)

    # Calcultate c

    c = a / np.sin(sigma)

    # Calculate alpha

    alpha = np.arccos( (self.L1**2 + self.L2**2 - c**2) / (2 * self.L1 * self.L2) )

    # Calculate beta

    beta = np.arcsin( (self.L2 * np.sin(alpha)) / c )

    # Calculate eta

    eta = np.pi - (alpha + beta)

    # Calculate theta2 and theta3

    theta2 = np.pi - (beta + sigma)

    theta3 = eta + (np.pi/2 - sigma)

    return np.array([theta1, theta2, theta3])
  
  def __joint_interpolated_motion(self, start, end, steps):
    start = self.__inverse_kinematics(end)
    end = self.__inverse_kinematics(end)
    trajectory = np.linspace(start, end, steps)
    return trajectory
  def __cartesian_interpolated_motion(self, start, end, steps):
    cartesian_points = np.linspace(start, end, steps)
    joint_trajectory = np.array([self.__inverse_kinematics(p) for p in cartesian_points])
    return joint_trajectory
  
  def __type_letter(self, c):
    # Move to letter using cartesian interpolation
    current_pos = self.robotObj.get_joint_config()
    target_pos = self.letter_coords[c] + np.array([0, 0, 20])
    trajectory = self.__cartesian_interpolated_motion(current_pos, target_pos, steps=10)
    for joint_angles in trajectory:
        self.robotObj.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])
    sleep(self.wait_time)
    # Move down to letter using joint interpolation
    current_pos = self.robotObj.get_joint_config()
    target_pos = self.letter_coords[c]
    trajectory = self.__joint_interpolated_motion(current_pos, target_pos, steps=10)
    for joint_angles in trajectory:
        self.robotObj.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])
    sleep(self.wait_time)
    # Simulate key press
    self.robotObj.set_suction_cup(True)
    sleep(self.wait_time)
    self.robotObj.set_suction_cup(False)
    sleep(self.wait_time)
    # Move back up to above letter using joint interpolation
    current_pos = self.robotObj.get_joint_config()
    target_pos = self.letter_coords[c] + np.array([0, 0, 20])
    trajectory = self.__joint_interpolated_motion(current_pos, target_pos, steps=10)
    for joint_angles in trajectory:
        self.robotObj.move_arm(joint_angles[0], joint_angles[1], joint_angles[2])
    sleep(self.wait_time)
    

  def typeWord(self, word: str):
    for c in word:
        self.__type_letter(c)
    self.__type_letter('return')


if __name__ == "__main__":
    WORD = "place"
    typer = WordTypingRobot()
    typer.typeWord(WORD)
