import os
from importlib import resources
from typing import TYPE_CHECKING, Any

import mujoco

if TYPE_CHECKING:
    from importlib.abc import Traversable


class ModelSpecProvider:
    """
    Simple class for loading robot and environment specs.
    """

    def load_robot(self, name: str) -> Any | None:
        """
        Load the robot model with the given name.
        """

        # load model
        return self.load_model_file('robots', name, 'robot.xml')

    def load_environment(self, name: str) -> Any | None:
        """
        Load the environment model with the given name.
        """

        return self.load_model_file('environments', name, 'world.xml')

    def load_model_file(self, section: str, model_name: str, file_name: str) -> Any | None:
        """
        Load the model with the given name from the resource section.
        """

        # fetch robot model dir
        model_dir = os.path.join(os.path.dirname(__file__), section, model_name)
        if not os.path.isdir(model_dir):
            return None

        # fetch model file path
        model_file = os.path.join(model_dir, file_name)
        if not os.path.isfile(model_file):
            return None

        # create new mujoco spec
        return mujoco.MjSpec.from_file(model_file)

    def load_model_resource(self, section: str, model_name: str, file_name: str) -> Any | None:
        """
        Load the model with the given name from the resource section.
        """

        # fetch package traversable
        package_dir: Traversable = resources.files('rcsssmj')

        # fetch robot model dir
        model_dir: Traversable = package_dir.joinpath('resources').joinpath(section).joinpath(model_name)
        if not model_dir.is_dir():
            return None

        # fetch model file
        model_file: Traversable = model_dir.joinpath(file_name)
        if not model_file.is_file():
            return None

        # load model xml
        model_xml = model_file.read_text('UTF-8')

        # load model assets
        assets: dict[str, bytes] = {}

        for res in model_dir.iterdir():
            if res.is_file() and res.name != file_name:
                assets[res.name] = res.read_bytes()

        # create new mujoco spec
        return mujoco.MjSpec.from_string(model_xml, assets=assets)
