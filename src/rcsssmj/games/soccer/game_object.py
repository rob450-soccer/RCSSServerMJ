from typing import Any, Final

import numpy as np

from rcsssmj.agents import AgentID
from rcsssmj.mjutils import filter_agent_contacts_with


class SoccerBall:
    """The soccer ball object in simulation."""

    def __init__(self) -> None:
        """Construct a new soccer ball."""

        self.spec: Any | None = None
        """The soccer ball object specification."""

        self.last_position: tuple[float, float, float] = (0, 0, 0)
        """The previous position of the ball."""

        self.position: tuple[float, float, float] = (0, 0, 0)
        """The current position of the ball."""

        self.active_contact: AgentID | None = None
        """The currently active agent contact (can last more than one simulation cycle)."""

        self.last_contact: AgentID | None = None
        """The previous agent contact (updated after the active contact has changed / has been lost)."""

        self.place_pos: tuple[float, float] | None = None
        """The target position to place the ball (if a ball placement is requested by some referee command)."""

    def set_active_contact(self, contact: AgentID | None) -> None:
        """Set the currently active agent contact.

        Parameter
        ---------
        contact: AgentID | None
            The agent currently in contact with the ball or None if there is no such contact at the moment.
        """

        if self.active_contact is not None:
            self.last_contact = self.active_contact

        self.active_contact = contact

    def get_most_recent_contact(self) -> AgentID | None:
        """Return the most recent contact (either the active or the last contact)."""

        return self.last_contact if self.active_contact is None else self.active_contact

    def reset_contacts(self) -> None:
        """Reset agent contact information."""

        self.active_contact = None
        self.last_contact = None

    def activate(self, body_spec: Any) -> None:
        """(Re-)Activate the ball instance.

        Parameter
        ---------
        body_spec: Any
            The soccer ball body specification.
        """

        self.spec = body_spec
        self.last_position = body_spec.pos[0:3].astype(np.float64)
        self.position = self.last_position
        self.active_contact = None
        self.last_contact = None
        self.place_pos = None

    def update(self, mj_model: Any, mj_data: Any) -> None:
        """Update ball state.

        Parameter
        ---------
        mj_model: MjModel
            The mujoco simulation model.

        mj_data: MjData
            The mujoco simulation data array.
        """

        # update position
        self.last_position = self.position
        self.position = mj_data.body('ball').xpos[0:3].astype(np.float64)

        # update agent contacts
        ball_contacts = filter_agent_contacts_with('ball', mj_model, mj_data)
        if ball_contacts:
            if self.active_contact not in ball_contacts:
                self.set_active_contact(next(iter(ball_contacts)))
        else:
            self.set_active_contact(None)


class SoccerPlayer:
    """A soccer player object in simulation."""

    def __init__(self, agent_id: AgentID, robot_spec: Any) -> None:
        """Construct a new soccer player."""

        self.agent_id: Final[AgentID] = agent_id
        """The agent id associated with the soccer player."""

        self.spec: Final[Any] = robot_spec
        """The robot model specification."""

        self.position: tuple[float, float, float] = robot_spec.body('torso').pos[0:3].astype(np.float64)
        """The current position of the player."""

        self.place_pos: tuple[float, float, float] | None = None
        """The target position to place the player (if a player placement is requested by some referee command)."""

        self.place_quat: tuple[float, float, float, float] | None = None
        """The target rotation quaternion to place the player (if a player placement is requested)."""

    def update(self, mj_model: Any, mj_data: Any) -> None:
        """Update ball state.

        Parameter
        ---------
        mj_model: MjModel
            The mujoco simulation model.

        mj_data: MjData
            The mujoco simulation data array.
        """

        # update position
        self.position = mj_data.body(self.agent_id.prefix + 'torso').xpos[0:3].astype(np.float64)
