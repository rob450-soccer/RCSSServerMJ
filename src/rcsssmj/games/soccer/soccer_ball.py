from typing import Any

import numpy as np

from rcsssmj.agents import AgentID
from rcsssmj.mjutils import filter_agent_contacts_with
from rcsssmj.sim_object import SimObject


class SoccerBall(SimObject):
    """The soccer ball object in simulation."""

    def __init__(self) -> None:
        """Construct a new soccer ball."""

        super().__init__('ball')

        self._radius: float = 0.117
        """The radius of the ball."""

        self._active_contact: AgentID | None = None
        """The currently active agent contact (can last more than one simulation cycle)."""

        self._last_contact: AgentID | None = None
        """The previous agent contact (updated after the active contact has changed / has been lost)."""

        self.place_pos: tuple[float, float] | None = None
        """The target position to place the ball (if a ball placement is requested by some referee command)."""

    @property
    def radius(self) -> float:
        """The radius of the ball."""

        return self._radius

    @property
    def active_contact(self) -> AgentID | None:
        """The currently active agent contact (can last more than one simulation cycle)."""

        return self._active_contact

    @property
    def last_contact(self) -> AgentID | None:
        """The previous agent contact (updated after the active contact has changed / has been lost)."""

        return self._last_contact

    def init(self, mj_spec: Any, mj_model: Any, mj_data: Any, radius: float = 0.117) -> None:
        """(Re-)Initialize the ball instance."""

        self.bind(mj_model, mj_data)

        self._prev_xpos = self.xpos.astype(np.float64)
        self._radius = radius
        self._active_contact = None
        self._last_contact = None
        self.place_pos = None

    def _set_active_contact(self, contact: AgentID | None) -> None:
        """Set the currently active agent contact.

        Parameter
        ---------
        contact: AgentID | None
            The agent currently in contact with the ball or None if there is no such contact at the moment.
        """

        if self._active_contact is not None:
            self._last_contact = self._active_contact

        self._active_contact = contact

    def get_most_recent_contact(self) -> AgentID | None:
        """Return the most recent contact (either the active or the last contact)."""

        return self._last_contact if self._active_contact is None else self._active_contact

    def reset_contacts(self) -> None:
        """Reset agent contact information."""

        self._active_contact = None
        self._last_contact = None

    def post_step(self, mj_model: Any, mj_data: Any) -> None:
        super().post_step(mj_model, mj_data)

        # update agent contacts
        ball_contacts = filter_agent_contacts_with('ball', mj_model, mj_data)
        if ball_contacts:
            if self._active_contact not in ball_contacts:
                self._set_active_contact(next(iter(ball_contacts)))
        else:
            self._set_active_contact(None)

    def relocate(self) -> None:
        """Place the object at the buffered relocation position (if existing)."""

        if self.place_pos is not None:
            self.place_at((self.place_pos[0], self.place_pos[1], self._radius))

            self.place_pos = None
