#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np


class SO2:
    """This class represents an SO2 rotations internally represented by rotation
    matrix."""

    def __init__(self, angle: float = 0.0) -> None:
        """Creates a rotation transformation that rotates vector by a given angle, that
        is expressed in radians. Rotation matrix .rot is used internally, no other
        variables can be stored inside the class."""
        super().__init__()
        #  HW01: implement computation of rotation matrix from the given angle
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        self.rot: np.ndarray = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

    def __mul__(self, other):
        """Compose two rotations, i.e., self * other"""
        #  HW01: implement composition of two rotation.
        result_matrix = np.dot(self.rot, other.rot)
        # Extract the composed angle from the resulting matrix
        composed_angle = np.arctan2(result_matrix[1, 0], result_matrix[0, 0])
        return SO2(composed_angle)

    @property
    def angle(self) -> float:
        """Return angle [rad] from the internal rotation matrix representation."""
        #  HW01: implement computation of rotation matrix from angles.
        angle = np.arctan2(self.rot[1, 0], self.rot[0, 0])
        return angle

    def inverse(self):
        """Return inverse of the transformation."""
        inverse_matrix = self.rot.T
        # Calculate the angle of the inverse rotation
        inverse_angle = np.arctan2(inverse_matrix[1, 0], inverse_matrix[0, 0])
        return SO2(inverse_angle)

    def act(self, vector):
        """Rotate given vector by this transformation."""
        v = np.asarray(vector)
        assert v.shape == (2,)
        return self.rot @ v

    def __eq__(self, other):
        """Returns true if two transformations are almost equal."""
        return np.allclose(self.rot, other.rot)

    def __hash__(self):
        return id(self)
