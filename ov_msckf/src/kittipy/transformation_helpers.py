"""
Helper functions for converting euler angles into quarternions.
"""
import math
import numpy as np

def hat_operator(theta):
  """
  Take the hat map of theta, a 3 x 1 vector.
  """
  mat = np.zeros((3, 3))
  mat[0, 1] = -theta[2, 0]
  mat[0, 2] = theta[1, 0]
  mat[1, 0] = theta[2, 0]
  mat[1, 2] = -theta[0, 0]
  mat[2, 0] = -theta[1, 0]
  mat[2, 1] = theta[0, 0]
  return mat

def vee_operator(u_hat):
  """
  Inverse of the hat operator. Extract a vector from its skew-symmetric matrix.
  """
  u_3 = -u_hat[0, 1]
  u_2 = u_hat[0, 2]
  u_1 = -u_hat[1, 2]
  u = np.array([u_1, u_2,u_3])
  return u

def euler_to_quat(R):
  """
  :param R: a 3x3 rotation matrix, in euler angles
  :output q: a (4,) quaternion representation of the rotation
  """
  eps = 0.0001
  assert (np.trace(R) - 1) / 2.0 < 1 + eps and (np.trace(R) - 1) / 2.0 > -1 - eps
  theta_norm = math.acos(np.clip((np.trace(R) - 1) / 2.0, -1, 1))
  if theta_norm == 0:
    # unity quaternion, no rotation
    return np.array([1, 0, 0, 0])
  else:
    theta_hat_unit = (R - R.T) / (2 * math.sin(theta_norm))
    si = vee_operator(theta_hat_unit)
    return np.array([math.cos(theta_norm / 2.0)] + list(math.sin(theta_norm / 2.0) * si))

def test_euler_to_quat():
  R = np.array([[1.000000e+00, 1.197625e-11, 1.704638e-10],
                [1.197625e-11, 1.000000e+00, 3.562503e-10],
                [1.704638e-10, 3.562503e-10, 1.000000e+00]])
  q = euler_to_quat(R)
  print("Rotation matrix: " + str(R))
  print("quarternion: " + str(q))


def test_euler_to_quat_x_90():
  """
  rotate x axis 90 degrees. 
  """ 
  R = np.array([[1, 0, 0],
                [0, 0, -1],
                [0, 1, 0]])
  q = euler_to_quat(R)
  print("Rotation matrix: " + str(R))
  print("quarternion: " + str(q))

def test_euler_to_quat_y_90():
  """
  rotate y axis 90 degrees.
  """
  R = np.array([[0, 0, 1],
                [0, 1, 0],
                [-1, 0, 0]])
  q = euler_to_quat(R)
  print("Rotation matrix: " + str(R))
  print("quarternion: " + str(q))


def test_euler_to_quat_z_90():
  """
  rotate z axis 90 degrees.
  """
  R = np.array([[0, -1, 0],
                [1, 0, 0],
                [0, 0, 1]])
  q = euler_to_quat(R)
  print("Rotation matrix: " + str(R))
  print("quarternion: " + str(q))

test_euler_to_quat_x_90()
test_euler_to_quat_y_90()
test_euler_to_quat_z_90()