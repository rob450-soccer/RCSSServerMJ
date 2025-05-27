from __future__ import annotations

from typing import Final


class AABB2D:
    """2-dimensional axis-aligned bounding-box."""

    def __init__(
        self,
        min_x: float = -1,
        max_x: float = 1,
        min_y: float = -1,
        max_y: float = 1,
    ) -> None:
        """Construct a new 2D axis-aligned bounding-box.

        Parameter
        ---------
        min_x: float = -1
            The lower bound along the x-axis.

        max_x: float = 1
            The upper bound along the x-axis.

        min_y: float = -1
            The lower bound along the y-axis.

        max_y: float = 1
            The upper bound along the y-axis.
        """

        if min_x > max_x:
            min_x, max_x = max_x, min_x

        if min_y > max_y:
            min_y, max_y = max_y, min_y

        self.min_x: Final[float] = min_x
        self.max_x: Final[float] = max_x
        self.min_y: Final[float] = min_y
        self.max_y: Final[float] = max_y

    def center(self) -> tuple[float, float]:
        """Return the center of the bounding-box."""

        return (self.min_x + self.max_x) / 2, (self.min_y + self.max_y) / 2

    def contains_x(self, x: float) -> bool:
        """Check if the given x-coordinate is within the bounding box."""

        return self.min_x <= x and x <= self.max_x

    def contains_y(self, y: float) -> bool:
        """Check if the given y-coordinate is within the bounding box."""

        return self.min_y <= y and y <= self.max_y

    def contains(self, x: float, y: float) -> bool:
        """Check if the given x- and y-coordinate is within the bounding box."""

        return self.min_x <= x and x <= self.max_x and self.min_y <= y and y <= self.max_y


class AABB3D:
    """3-dimensional axis-aligned bounding-box."""

    def __init__(
        self,
        min_x: float = -1,
        max_x: float = 1,
        min_y: float = -1,
        max_y: float = 1,
        min_z: float = -1,
        max_z: float = 1,
    ) -> None:
        """Construct a new 3D axis-aligned bounding-box.

        Parameter
        ---------
        min_x: float = -1
            The lower bound along the x-axis.

        max_x: float = 1
            The upper bound along the x-axis.

        min_y: float = -1
            The lower bound along the y-axis.

        max_y: float = 1
            The upper bound along the y-axis.

        min_z: float = -1
            The lower bound along the z-axis.

        max_z: float = 1
            The upper bound along the z-axis.
        """

        if min_x > max_x:
            min_x, max_x = max_x, min_x

        if min_y > max_y:
            min_y, max_y = max_y, min_y

        if min_z > max_z:
            min_z, max_z = max_z, min_z

        self.min_x: Final[float] = min_x
        self.max_x: Final[float] = max_x
        self.min_y: Final[float] = min_y
        self.max_y: Final[float] = max_y
        self.min_z: Final[float] = min_z
        self.max_z: Final[float] = max_z

    def center(self) -> tuple[float, float, float]:
        """Return the center of the bounding-box."""

        return (self.min_x + self.max_x) / 2, (self.min_y + self.max_y) / 2, (self.min_z + self.max_z) / 2

    def contains_x(self, x: float) -> bool:
        """Check if the given x-coordinate is within the bounding box."""

        return self.min_x <= x and x <= self.max_x

    def contains_y(self, y: float) -> bool:
        """Check if the given y-coordinate is within the bounding box."""

        return self.min_y <= y and y <= self.max_y

    def contains_z(self, z: float) -> bool:
        """Check if the given z-coordinate is within the bounding box."""

        return self.min_z <= z and z <= self.max_z

    def contains_xy(self, x: float, y: float) -> bool:
        """Check if the given x- and y-coordinates are within the bounding box."""

        return self.min_x <= x and x <= self.max_x and self.min_y <= y and y <= self.max_y

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if the given x-, y- and z-coordinates are within the bounding box."""

        return self.min_x <= x and x <= self.max_x and self.min_y <= y and y <= self.max_y and self.min_z <= z and z <= self.max_z
