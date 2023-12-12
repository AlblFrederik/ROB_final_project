#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import numpy as np


from robotics_toolbox.core import SO2


class SE2:
    """Transformation in 2D that is composed of rotation and translation."""

    def __init__(
            self, translation, rotation):
        """Crete an SE2 transformation. Identity is the default."""
        super().__init__()
        self.translation = (
            np.asarray(translation) if translation is not None else np.zeros(2)
        )
        if isinstance(rotation, SO2):
            self.rotation = rotation
        elif isinstance(rotation, float):
            self.rotation = SO2(rotation)
        else:
            self.rotation = SO2()
        assert self.translation.shape == (2,)

    def __mul__(self, other):
        """Compose two transformation, i.e., self * other"""
        # HW01: implement composition of two transformation.
        # Calculate the new translation
        se2_matrix = np.eye(3)
        # Copy the rotation part into the top-left 2x2 submatrix
        se2_matrix[0:2, 0:2] = self.rotation.rot
        # Copy the translation part into the top-right 2x1 submatrix
        se2_matrix[0:2, 2] = self.translation

        mat2 = np.eye(3)
        mat2[0:2, 0:2] = other.rotation.rot
        # Copy the translation part into the top-right 2x1 submatrix
        mat2[0:2, 2] = other.translation

        combined = np.dot(se2_matrix, mat2)
        rot = combined[0:2, 0:2]
        trans = combined[0:2, 2]
        composed_transform = SE2(trans, SO2(np.arctan2(rot[1, 0], rot[0, 0])))
        return composed_transform

    def inverse(self):
        """Compute inverse of the transformation"""
        # HW1 implement inverse
        # Calculate the inverse translation by applying the inverse rotation
        # 1. Invert the rotation matrix
        # In 2D, the inverse of a rotation matrix is its transpose
        inverse_rotation = np.transpose(self.rotation.rot)

        # 2. Compute the inverse translation and reshape it to be a column vector
        inverse_translation = -np.dot(inverse_rotation, self.translation).reshape((2, 1))

        # 3. Combine the inverted rotation and translation to get the final inverse transformation
        combined = np.vstack((np.hstack((inverse_rotation, inverse_translation)), [0, 0, 1]))
        rot = combined[0:2, 0:2]
        trans = combined[0:2, 2]
        composed_transform = SE2(trans, SO2(np.arctan2(rot[1, 0], rot[0, 0])))
        return composed_transform

    def act(self, vector):
        """Transform given 2D vector by this SE2 transformation."""
        v = np.asarray(vector)
        assert v.shape == (2,)
        # HW1 implement transformation of a given vector
        # Apply the translation

        # 1. Apply the rotation matrix to the vector
        rotated_vector = np.dot(self.rotation.rot, v)

        # 2. Add the translation vector to the result of the rotation
        transformed_vector = rotated_vector + self.translation
        return transformed_vector

    def set_from(self, other):
        """Copy the properties into current instance."""
        self.translation = other.translation
        self.rotation = other.rotation

    def __eq__(self, other) -> bool:
        """Returns true if two transformations are almost equal."""
        return (
                np.allclose(self.translation, other.translation)
                and self.rotation == other.rotation
        )

    def __hash__(self):
        return id(self)

    def __repr__(self):
        return (
            f"SE2(translation={self.translation}, rotation=SO2({self.rotation.angle}))"
        )
