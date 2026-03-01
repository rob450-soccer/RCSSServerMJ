from typing import Any

import numpy as np

from rcsssmj.sim.actuators import Speaker
from rcsssmj.sim.audio_data import AudioData
from rcsssmj.sim.sensors import Microphone


def a_compile(mj_spec: Any) -> AudioData:
    """Compile specification.

    Parameter
    ---------
    mj_spec: Any
        The world specification.
    """

    # extract audio sensors
    sensors = {site.name[:-8]: Microphone(site.name[:-8], site.name) for site in mj_spec.sites if site.name.endswith('_a-micro')}

    # extract audio actuators
    actuators = {site.name[:-10]: Speaker(site.name[:-10], site.name) for site in mj_spec.sites if site.name.endswith('_a-speaker')}

    return AudioData(sensors, actuators)


def a_recompile(mj_spec: Any, old_data: AudioData) -> AudioData:
    """Recompile specification.

    Parameter
    ---------
    mj_spec: Any
        The world specification.

    old_data: AudioData
        The previous audio state data.
    """

    new_data = a_compile(mj_spec)

    # update new actuator states with old actuator states...
    for actuator in new_data.actuators.values():
        old_actuator = old_data.actuators.get(actuator.name, None)
        if old_actuator is not None:
            actuator.ctrl = old_actuator.ctrl
            actuator.gainprm[:] = old_actuator.gainprm[:]

    # copy all messages from actuators that are still available
    indices = []
    new_messages = []
    new_sources = []

    for idx, source in enumerate(old_data.sources):
        if new_data.actuators.get(source, None) is not None:
            indices.append(idx)
            new_messages.append(old_data.messages[idx])
            new_sources.append(source)

    new_data.messages = new_messages
    new_data.sources = new_sources
    new_data.volumes = old_data.volumes[indices].astype(np.float64)
    new_data.origins = old_data.origins[:, indices].astype(np.float64)

    return new_data


def a_step(a_data: AudioData, mj_data: Any) -> None:
    """Progress the audio simulation.

    Parameter
    ---------
    a_data: AudioData
        The audio state data.

    mj_data: Any
        The mujoco state data.
    """

    messages = []
    sources = []
    volumes_arr = []

    # collect all messages broadcasted by speaker actuators
    for speaker in a_data.actuators.values():
        if speaker.ctrl:
            messages.append(speaker.ctrl)
            sources.append(speaker.name)
            volumes_arr.append(speaker.gainprm[0].astype(np.float64))

    # update state data
    a_data.messages = messages
    a_data.sources = sources
    a_data.volumes = np.array(volumes_arr, dtype=np.float64)
    a_data.origins = np.zeros((3, len(messages)), dtype=np.float64)

    # update actuator origins and calculate sensor states
    a_forward(a_data, mj_data)

    # reset actuators
    for speaker in a_data.actuators.values():
        speaker.ctrl = None


def a_forward(a_data: AudioData, mj_data: Any) -> None:
    """Calculate audio state.

    Parameter
    ---------
    a_data: AudioData
        The audio state data.

    mj_data: Any
        The mujoco state data.
    """

    # re-fetch actuator site positions
    for idx, source in enumerate(a_data.sources):
        a_data.origins[:, idx] = mj_data.site(a_data.actuators[source].site).xpos.astype(np.float64)

    # generate senor information
    for mic in a_data.sensors.values():
        # fetch sensor pose
        s_site = mj_data.site(mic.site)
        s_pos = s_site.xpos.astype(np.float64)
        s_rot = s_site.xmat.astype(np.float64).reshape((3, 3))

        # transform detectable audio signals to microphone frame
        local_origins = np.matmul(s_rot.T, a_data.origins - s_pos[:, np.newaxis])

        # calculate volumes based on origin distances
        distances = np.linalg.norm(local_origins, axis=0)
        s_volumes = a_data.volumes * np.pow(10, np.log2(np.maximum(distances, 1)) * -6 / 20)

        # set sensor information
        mic.messages = a_data.messages
        mic.sources = a_data.sources
        mic.volumes = s_volumes
        mic.origins = local_origins
        mic.distances = distances
