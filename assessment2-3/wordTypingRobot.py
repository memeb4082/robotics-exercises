import numpy as np
from time import sleep
import scipy.optimize

class WordTypingRobot:
  def __init__(self, robotObj = None):
    self.robotObj = robotObj
    self.L0 = 138
    self.L1 = 135
    self.L2 = 147
    self.L3 = 60
    self.L4 = -70
    _z_height = 0
    # Keyboard layout done row by row for qwerty layout
    __key_height = 11.5
    __key_width = 13.5
    __enter_key_width = 19
    __a_padding = 6.75
    __z_padding = 22.5
    
    __key_spacing = np.array([__key_width, 0, 0])
    
    __q_pos = np.array([6.75, 2.5 * __key_height, _z_height])
    __a_pos = np.array([6.75 + 0.5 * __key_width, 1.5 * __key_height, _z_height])
    __z_pos = np.array([22.5 + 0.5 * __key_width, 0.5 * __key_height, _z_height])
    
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
    self.letter_coords['return'] = self.letter_coords['l'] + np.array([__enter_key_width, 0, 0])
    self.letter_coords[' '] = self.letter_coords['b'] + [0, -35, 0]

    origin = np.array([247.5, -77.25, 0])
    R = self.__rot_mat('z', np.pi / 2)[:3, :3]
    t = origin 

    for k in self.letter_coords:
      self.letter_coords[k] = R @ self.letter_coords[k] + t
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
    theta1 = np.arctan2(y, x)
    r = y / np.sin(theta1)
    b = self.L0 + self.L4 - z
    a = r - self.L3
    sigma = np.arctan2(a, b)
    c = a / np.sin(sigma)
    alpha = np.arccos( (self.L1**2 + self.L2**2 - c**2) / (2 * self.L1 * self.L2) )
    beta = np.arcsin( (self.L2 * np.sin(alpha)) / c )
    eta = np.pi - (alpha + beta)
    theta2 = np.pi - (beta + sigma)
    theta3 = eta + (np.pi/2 - sigma)
    return np.array([theta1, theta2, theta3])
  
  def __type_letter(self, c):
    #   dont use interpolation for now, just move directly
    if c not in self.letter_coords:
      print(f"Letter {c} not in known coordinates, skipping")
      return
    target_pos = self.letter_coords[c] + np.array([0, 0, 20])
    # Move to above letter
    self.robotObj.move_arm(*self.__inverse_kinematics(target_pos))
    target_pos = self.letter_coords[c]
    # Move down to letter
    self.robotObj.move_arm(*self.__inverse_kinematics(target_pos))
    # Move back up to above letter
    target_pos = self.letter_coords[c] + np.array([0, 0, 20])
    self.robotObj.move_arm(*self.__inverse_kinematics(target_pos))
    

  def typeWord(self, word: str):
    word = word.lower()
    for c in word:
        self.__type_letter(c)
    self.__type_letter('return')

def wordTypingRobot(robotObj, word):
    robject = WordTypingRobot(robotObj)
    robject.typeWord(word)

if __name__ == "__main__":
    WORD = "ijn"
    typer = WordTypingRobot()
    typer.typeWord(WORD)
