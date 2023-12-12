#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np


class SO3:
    """This class represents an SO3 rotations internally represented by rotation
    matrix."""

    def __init__(self, rotation_matrix):
        """Creates a rotation transformation from rot_vector."""
        super().__init__()
        self.rot: np.ndarray = (
            np.asarray(rotation_matrix) if rotation_matrix is not None else np.eye(3)
        )

    @staticmethod
    def exp(rot_vector):
        """Compute SO3 transformation from a given rotation vector, i.e. exponential
        representation of the rotation."""
        v = np.asarray(rot_vector)
        assert v.shape == (3,)

        # HW01: implement Rodrigues' formula, t.rot = ...
        theta = np.linalg.norm(v)  # Compute the rotation angle from the vector's magnitude

        if theta < 1e-6:
            # When the angle is close to zero, use the identity matrix
            t = SO3(np.eye(3))
        else:
            # Normalize the rotation vector
            axis = v / theta

            # Create the skew-symmetric matrix for the normalized axis
            K = np.array([[0, -axis[2], axis[1]],
                          [axis[2], 0, -axis[0]],
                          [-axis[1], axis[0], 0]])

            # Rodrigues' formula for rotation matrix
            t = SO3(np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K))

        return t

    def log(self) -> np.ndarray:
        """Compute rotation vector from this SO3"""
        # HW01: implement computation of rotation vector from this SO3
        # Get the rotation matrix
        R = self.rot

        # Calculate the trace of the rotation matrix
        trace = np.trace(R)

        # Ensure the trace is within the valid range [-1, 3]
        trace = np.clip(trace, -1, 3)

        # Calculate the rotation angle (theta)
        theta = np.arccos((trace - 1) / 2)

        if np.isclose(theta, 0.0):
            # When theta is close to zero, the rotation vector is [0, 0, 0]
            return np.zeros(3)

        # Calculate the skew-symmetric matrix for logarithm calculation
        K = (theta / (2 * np.sin(theta))) * np.array([
            [R[2, 1] - R[1, 2]],
            [R[0, 2] - R[2, 0]],
            [R[1, 0] - R[0, 1]]
        ])

        # Extract the rotation vector from the skew-symmetric matrix
        rotation_vector = K.flatten()

        return rotation_vector

    def __mul__(self, other):
        """Compose two rotations, i.e., self * other"""
        # HW01: implement composition of two rotation.
        if self.rot.shape != (3, 3) or other.rot.shape != (3, 3):
            raise ValueError("Rotation matrices must be 3x3")

            # Multiply the rotation matrices to compose the rotations
        result_rotation = np.dot(self.rot, other.rot)

        # Return the result as a new SO3 instance
        return SO3(result_rotation)

    def inverse(self):
        """Return inverse of the transformation."""
        # HW01: implement inverse, do not use np.linalg.inverse()
        R = self.rot
        # Compute the transpose of the rotation matrix to get the inverse
        inverse_rotation = np.transpose(R)
        return SO3(inverse_rotation)

    def act(self, vector):
        """Rotate given vector by this transformation."""
        v = np.asarray(vector)
        assert v.shape == (3,)
        return self.rot @ v

    def __eq__(self, other) -> bool:
        """Returns true if two transformations are almost equal."""
        return np.allclose(self.rot, other.rot)

    @staticmethod
    def rx(angle: float):
        """Return rotation matrix around x axis."""
        # HW1opt: implement rx
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, cos_angle, -sin_angle],
                                    [0, sin_angle, cos_angle]])
        return SO3(rotation_matrix)

    @staticmethod
    def ry(angle: float):
        """Return rotation matrix around y axis."""
        # HW1opt: implement ry
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        rotation_matrix = np.array([[cos_angle, 0, sin_angle],
                                    [0, 1, 0],
                                    [-sin_angle, 0, cos_angle]])
        return SO3(rotation_matrix)

    @staticmethod
    def rz(angle: float):
        """Return rotation matrix around z axis."""
        # HW1opt: implement rz
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        rotation_matrix = np.array([[cos_angle, -sin_angle, 0],
                                    [sin_angle, cos_angle, 0],
                                    [0, 0, 1]])
        return SO3(rotation_matrix)

    @staticmethod
    def from_quaternion(q):
        """Compute rotation from quaternion in a form [qx, qy, qz, qw]."""
        # HW1opt: implement from quaternion
        q = np.asarray(q)
        if q.shape != (4,):
            raise ValueError("Quaternion must be a 4-element array [qx, qy, qz, qw].")

        qx, qy, qz, qw = q
        # Create a copy of the quaternion and then normalize it
        norm = np.linalg.norm(q)
        if norm == 0:
            raise ValueError("Quaternion has zero norm.")
        q = q / norm

        rotation_matrix = np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)]
        ])

        return SO3(rotation_matrix)

    def to_quaternion(self) -> np.ndarray:
        """Compute quaternion from self."""
        # HW1opt: implement to quaternion
        rot_matrix = self.rot
        qw = np.sqrt(1 + rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]) / 2
        qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) / (4 * qw)
        qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) / (4 * qw)
        qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) / (4 * qw)

        return np.array([qx, qy, qz, qw])

    @staticmethod
    def skew(vector):
        """Compute the skew-symmetric matrix of a 3D vector."""
        return np.array([[0, -vector[2], vector[1]],
                         [vector[2], 0, -vector[0]],
                         [-vector[1], vector[0], 0]])

    @staticmethod
    def from_angle_axis(angle: float, axis):
        """Compute rotation from angle axis representation."""
        # HW1opt: implement from angle axis
        # Normalize the axis vector
        axis = np.asarray(axis)
        axis /= np.linalg.norm(axis)

        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        rotation_matrix = (
                cos_angle * np.eye(3) +
                (1 - cos_angle) * np.outer(axis, axis) +
                sin_angle * SO3.skew(axis)
        )
        return SO3(rotation_matrix)

    def to_angle_axis(self):
        """Compute angle axis representation from self."""
        # HW1opt: implement to angle axis
        trace = np.trace(self.rot)
        if trace > 3.0:
            trace = 3.0
        elif trace < -1.0:
            trace = -1.0

        # Calculate the rotation angle (θ)
        theta = np.arccos((trace - 1) / 2.0)

        # Calculate the rotation axis (a)
        if np.isclose(theta, 0.0):
            # When theta is close to 0, the rotation axis is undefined.
            axis = np.array([1.0, 0.0, 0.0])  # Default axis (arbitrary)
        else:
            axis = 1.0 / (2.0 * np.sin(theta)) * np.array([
                self.rot[2, 1] - self.rot[1, 2],
                self.rot[0, 2] - self.rot[2, 0],
                self.rot[1, 0] - self.rot[0, 1]
            ])

        return theta, axis

    @staticmethod
    def from_euler_angles(angles, seq):
        """Compute rotation from euler angles defined by a given sequence.
        angles: is a three-dimensional array of angles
        seq: is a list of axis around which angles rotate, e.g. 'xyz', 'xzx', etc.
        """
        # HW1opt: implement from euler angles
        # Initialize the rotation matrix as the identity matrix
        rotation_matrix = np.eye(3)
        seq = seq[::-1]
        angles = angles[::-1]
        for axis, angle in zip(seq, angles):
            if axis == 'x':
                rotation_matrix = np.dot(SO3.rx(angle).rot, rotation_matrix)
            elif axis == 'y':
                rotation_matrix = np.dot(SO3.ry(angle).rot, rotation_matrix)
            elif axis == 'z':
                rotation_matrix = np.dot(SO3.rz(angle).rot, rotation_matrix)
            else:
                raise ValueError("Invalid axis in the sequence. Use 'x', 'y', or 'z'.")

        return SO3(rotation_matrix)

    def __hash__(self):
        return id(self)
