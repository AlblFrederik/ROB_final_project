#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#


import numpy as np


from robotics_toolbox.core import SO3


class SE3:
    """Transformation in 2D that is composed of rotation and translation."""

    def __init__(
            self, translation, rotation):
        """Crete an SE3 transformation. Identity is the default."""
        super().__init__()
        self.translation = (
            np.asarray(translation) if translation is not None else np.zeros(3)
        )
        self.rotation = rotation if rotation is not None else SO3()
        assert self.translation.shape == (3,)

    def __mul__(self, other):
        """Compose two transformation, i.e., self * other"""
        # HW01: implement composition of two transformation.
        se3_matrix = np.eye(4)
        # Copy the rotation part into the top-left 2x2 submatrix
        se3_matrix[0:3, 0:3] = self.rotation.rot
        # Copy the translation part into the top-right 2x1 submatrix
        se3_matrix[0:3, 3] = self.translation

        mat3 = np.eye(4)
        mat3[0:3, 0:3] = other.rotation.rot
        # Copy the translation part into the top-right 2x1 submatrix
        mat3[0:3, 3] = other.translation

        combined = np.dot(se3_matrix, mat3)

        rot = combined[0:3, 0:3]
        trans = combined[0:3, 3]
        composed_transform = SE3(trans, SO3(rot))
        return composed_transform

    def inverse(self):
        """Compute inverse of the transformation"""
        # HW1 implement inverse
        # Compute the inverse of the rotation matrix (transpose)
        # In 2D, the inverse of a rotation matrix is its transpose
        inverse_rotation = np.transpose(self.rotation.rot)

        # 2. Compute the inverse translation and reshape it to be a column vector
        inverse_translation = -np.dot(inverse_rotation, self.translation).reshape((3, 1))

        # 3. Combine the inverted rotation and translation to get the final inverse transformation
        combined = np.vstack((np.hstack((inverse_rotation, inverse_translation)), [0, 0, 0, 1]))
        # print("combined")
        # print(combined)
        # print("------------")
        rot = combined[0:3, 0:3]
        trans = combined[0:3, 3]

        return SE3(trans, SO3(rot))

    def act(self, vector):
        """Rotate given 3D vector by this transformation."""
        v = np.asarray(vector)
        assert v.shape == (3,)
        # HW1 implement transformation of a given vector
        rotated_vector = np.dot(self.rotation.rot, v)
        transformed_vector = rotated_vector + self.translation
        return transformed_vector

    def set_from(self, other):
        """Copy the properties into current instance."""
        self.translation = other.translation
        self.rotation = other.rotation

    def __eq__(self, other):
        """Returns true if two transformations are almost equal."""
        return (
                np.allclose(self.translation, other.translation)
                and self.rotation == other.rotation
        )

    def __hash__(self):
        return id(self)

    def __repr__(self):
        return f"(translation={self.translation}, log_rotation={self.rotation.log()})"
