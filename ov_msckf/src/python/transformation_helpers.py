"""
Helper functions for converting euler angles into quarternions.
"""
from tf.transformations import quaternion_from_matrix
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

def compute_angle(R):
    """
    Compute the rotation angle from a 4x4 homogeneous matrix.
    """
    # an invitation to 3-d vision, p 27
    return numpy.arccos(min(1,max(-1, (numpy.trace(R) - 1)/2)))


def matrix_to_quaternion_hamiltonian(R):
    """
    Convert a rotation matrix to quaternion in Hamiltonian format, with scalar at the last position.
    :param R: a 3x3 rotation matrix, in euler angles
    :output q: a (4,) quaternion representation of the rotation, [qx, qy, qz, qw]
    """
    r = np.eye(4)
    r[0:3, 0:3] = R
    q = quaternion_from_matrix(r)
    return q #[qx, qy, qz, qw]

def matrix_to_quaternion_JPL(R):
    """
    Convert a rotation matrix to quaternion in JPL format, with scalar at last.
    Equation 74. in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
    :param R: a 3x3 rotation matrix, in euler angles
    :output q: a (4,) quaternion representation of the rotation
    """
    q = np.zeros((4,), dtype=np.float64)
    T = np.trace(R, dtype=np.float64)
    # print("Trace: %s" % T)
    if R[0,0] >= T and R[0,0] >= R[1,1] and R[0,0] >= R[2,2]:
      q[0] = np.sqrt((1 + (2 * R[0,0]) - T) / 4)
      q[1] = (1/ (4 * q[0])) * (R[0,1] + R[1,0])
      q[2] = (1/ (4 * q[0])) * (R[0,2] + R[2,0])
      q[3] = (1/ (4 * q[0])) * (R[1,2] - R[2,1])
    elif R[1,1] >= T and R[1,1] >= R[0,0] and R[1,1] >= R[2,2]:
      q[1] = np.sqrt((1 + (2 * R[1,1]) - T) / 4)
      q[0] = (1/ (4 * q[1])) * (R[0,1] + R[1,0])
      q[2] = (1/ (4 * q[1])) * (R[1,2] + R[2,1])
      q[3] = (1/ (4 * q[1])) * (R[2,0] - R[0,2])
    elif R[2,2] >= T and R[2,2] >= R[0,0] and R[2,2] >= R[1,1]:
      q[2] = np.sqrt((1 + (2 * R[2,2]) - T) / 4)
      # print("q[2] is: %s" % q[2])
      q[0] = (1/ (4 * q[2])) * (R[0,2] + R[2,0])
      q[1] = (1/ (4 * q[2])) * (R[1,2] + R[2,1])
      q[3] = (1/ (4 * q[2])) * (R[0,1] - R[1,0])
    else:
      q[3] = np.sqrt((1 + T) / 4)
      q[0] = (1/ (4 * q[3])) * (R[1,2] - R[2,1])
      q[1] = (1/ (4 * q[3])) * (R[2,0] - R[0,2])
      q[2] = (1/ (4 * q[3])) * (R[0,1] - R[1,0])

    if q[3] < 0:
      q = -q

    q = q / np.linalg.norm(q)
    return q

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

def T_inv(T):
    """
    Invert a transformation matrix
    :param T: a 4x4 transformation matrix;
    :output T_inv, the inverse of the transformation
    """
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    T_inv = np.zeros((4,4), dtype=np.float64)
    T_inv[0:3, 0:3] = R.T
    T_inv[0:3, 3] = - R.T.dot(p)
    T_inv[3, 3] = 1
    return T_inv 


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

# test_euler_to_quat_x_90()
# test_euler_to_quat_y_90()
# test_euler_to_quat_z_90()