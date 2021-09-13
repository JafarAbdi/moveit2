from pathlib import Path
from typing import Optional, List
import logging
from ament_index_python.packages import get_package_share_directory

from parameter_builder import ParameterBuilder, load_yaml, load_xacro


class MoveItConfigs(object):
    __slots__ = [
        "__robot_description",
        "__robot_description_semantic",
        "__robot_description_planning",
        "__robot_description_kinematics",
        "__planning_pipelines",
        "__trajectory_execution",
        "__planning_scene_monitor",
        "__move_group_capabilities",
        "__move_group",
        "__joint_limits",
        "__moveit_cpp",
    ]

    def __init__(self, **kwargs):
        assert all(
            "__" + key in self.__slots__ for key in kwargs.keys()
        ), "Invalid arguments passed to constructor: %s" % ", ".join(
            sorted(k for k in kwargs.keys() if "__" + k not in self.__slots__)
        )
        self.__robot_description = kwargs.get("robot_description", None)
        self.__robot_description_semantic = kwargs.get(
            "robot_description_semantic", None
        )
        self.__robot_description_planning = kwargs.get(
            "robot_description_planning", None
        )
        self.__robot_description_kinematics = kwargs.get(
            "robot_description_kinematics", None
        )
        self.__planning_pipelines = kwargs.get("planning_pipelines", None)
        self.__trajectory_execution = kwargs.get("trajectory_execution", None)
        self.__planning_scene_monitor = kwargs.get("planning_scene_monitor", None)
        self.__move_group_capabilities = kwargs.get("move_group_capabilities", None)
        self.__move_group = kwargs.get("move_group", None)
        self.__joint_limits = kwargs.get("joint_limits", None)
        self.__moveit_cpp = kwargs.get("moveit_cpp", None)

    @property
    def robot_description(self):
        return self.__robot_description

    @robot_description.setter
    def robot_description(self, value):
        self.__robot_description = value

    @property
    def robot_description_semantic(self):
        return self.__robot_description_semantic

    @robot_description_semantic.setter
    def robot_description_semantic(self, value):
        self.__robot_description_semantic = value

    @property
    def robot_description_planning(self):
        return self.__robot_description_planning

    @robot_description_planning.setter
    def robot_description_planning(self, value):
        self.__robot_description_planning = value

    @property
    def robot_description_kinematics(self):
        return self.__robot_description_kinematics

    @robot_description_kinematics.setter
    def robot_description_kinematics(self, value):
        self.__robot_description_kinematics = value

    @property
    def planning_pipelines(self):
        return self.__planning_pipelines

    @planning_pipelines.setter
    def planning_pipelines(self, value):
        self.__planning_pipelines = value

    @property
    def trajectory_execution(self):
        return self.__trajectory_execution

    @trajectory_execution.setter
    def trajectory_execution(self, value):
        self.__trajectory_execution = value

    @property
    def planning_scene_monitor(self):
        return self.__planning_scene_monitor

    @planning_scene_monitor.setter
    def planning_scene_monitor(self, value):
        self.__planning_scene_monitor = value

    @property
    def move_group_capabilities(self):
        return self.__move_group_capabilities

    @move_group_capabilities.setter
    def move_group_capabilities(self, value):
        self.__move_group_capabilities = value

    @property
    def move_group(self):
        return self.__move_group

    @move_group.setter
    def move_group(self, value):
        self.__move_group = value

    @property
    def joint_limits(self):
        return self.__joint_limits

    @joint_limits.setter
    def joint_limits(self, value):
        self.__joint_limits = value

    @property
    def moveit_cpp(self):
        return self.__moveit_cpp

    @moveit_cpp.setter
    def moveit_cpp(self, value):
        self.__moveit_cpp = value


class MoveItConfigsBuilder(ParameterBuilder):
    __moveit_configs = MoveItConfigs()
    __robot_name: str
    __urdf_package: Path
    __urdf_filename: str
    __srdf_filename: str
    __robot_description: str
    # Look-up for robot_name_moveit_config package
    def __init__(self, robot_name: str, robot_description="robot_description"):
        super().__init__(robot_name + "_moveit_config")
        self.__robot_name = robot_name
        setup_assistant_file = self._package_path / ".setup_assistant"
        if not setup_assistant_file.exists():
            logging.warning(
                f"\x1b[33;21mPackage `{self._package_path}` doesn't have `.setup_assistant` file "
                f"-- using config/{robot_name}.urdf and config/{robot_name}.srdf\x1b[0m"
            )
            self.__urdf_package = self._package_path
            self.__urdf_filename = "config/" + self.__robot_name + ".urdf"
            self.__srdf_filename = "config/" + self.__robot_name + ".srdf"
        else:
            setup_assistant_yaml = load_yaml(setup_assistant_file)
            self.__urdf_package = Path(
                get_package_share_directory(
                    setup_assistant_yaml["moveit_setup_assistant_config"]["URDF"][
                        "package"
                    ]
                )
            )
            self.__urdf_filename = setup_assistant_yaml[
                "moveit_setup_assistant_config"
            ]["URDF"]["relative_path"]
            self.__srdf_filename = setup_assistant_yaml[
                "moveit_setup_assistant_config"
            ]["SRDF"]["relative_path"]
        self.__robot_description = robot_description

    def robot_description(self, file_path: Optional[str] = None, mappings: dict = None):
        if file_path is None:
            robot_description_file_path = self.__urdf_package / self.__urdf_filename
        else:
            robot_description_file_path = self._package_path / file_path
        self.__moveit_configs.robot_description = {
            self.__robot_description: load_xacro(
                robot_description_file_path, mappings=mappings
            )
        }
        return self

    def robot_description_semantic(
        self, file_path: Optional[str] = None, mappings: dict = None
    ):
        self.__moveit_configs.robot_description_semantic = {
            self.__robot_description
            + "_semantic": load_xacro(
                self._package_path / (file_path or self.__srdf_filename),
                mappings=mappings,
            )
        }
        return self

    def robot_description_kinematics(self, file_path: Optional[str] = None):
        self.__moveit_configs.robot_description_kinematics = {
            self.__robot_description
            + "_kinematics": load_yaml(
                self._package_path / (file_path or "config/kinematics.yaml")
            )
        }
        return self

    def joint_limits(self, file_path: Optional[str] = None):
        self.__moveit_configs.joint_limits = {
            self.__robot_description
            + "_planning": load_yaml(
                self._package_path / (file_path or "config/joint_limits.yaml")
            )
        }
        return self

    def moveit_cpp(self, file_path: Optional[str] = None):
        self.__moveit_configs.moveit_cpp = load_yaml(
            self._package_path / (file_path or "config/moveit_cpp.yaml")
        )
        return self

    def trajectory_execution(
        self,
        file_path: Optional[str] = None,
        moveit_manage_controllers: bool = True,
    ):
        self.__moveit_configs.trajectory_execution = {
            "moveit_manage_controllers": moveit_manage_controllers,
        }
        self.__moveit_configs.trajectory_execution.update(
            load_yaml(
                self._package_path
                / (file_path or f"config/{self.__robot_name}_controllers.yaml")
            )
        )
        return self

    def planning_scene_monitor(
        self,
        publish_planning_scene: bool = True,
        publish_geometry_updates: bool = True,
        publish_state_updates: bool = True,
        publish_transforms_updates: bool = True,
    ):
        self.__moveit_configs.planning_scene_monitor = {
            # TODO: Fix parameter namespace upstream -- see planning_scene_monitor.cpp:262
            # "planning_scene_monitor": {
            "publish_planning_scene": publish_planning_scene,
            "publish_geometry_updates": publish_geometry_updates,
            "publish_state_updates": publish_state_updates,
            "publish_transforms_updates": publish_transforms_updates,
            # }
        }
        return self

    def planning_pipelines(
        self, default_planning_pipeline: str = None, pipelines: List[str] = None
    ):
        if pipelines is None:
            pipelines = ["ompl"]
            default_planning_pipeline = pipelines[0]
        if default_planning_pipeline not in pipelines:
            raise RuntimeError(
                f"default_planning_pipeline: `{default_planning_pipeline}` doesn't name any of the input pipelines "
                f"`{','.join(pipelines)}`"
            )
        self.__moveit_configs.planning_pipelines = {
            "planning_pipelines": pipelines,
            "default_planning_pipeline": default_planning_pipeline,
        }
        for pipeline in pipelines:
            self.__moveit_configs.planning_pipelines[pipeline] = load_yaml(
                self._package_path / "config" / (pipeline + "_planning.yaml")
            )
        return self

    def to_moveit_configs(self):
        return self.__moveit_configs

    def to_dict(self, include_moveit_configs: bool = True):
        parameters = self._parameters
        if include_moveit_configs:
            if self.__moveit_configs.robot_description:
                parameters.update(self.__moveit_configs.robot_description)
            if self.__moveit_configs.robot_description_semantic:
                parameters.update(self.__moveit_configs.robot_description_semantic)
            if self.__moveit_configs.robot_description_kinematics:
                parameters.update(self.__moveit_configs.robot_description_kinematics)
            if self.__moveit_configs.planning_pipelines:
                parameters.update(self.__moveit_configs.planning_pipelines)
            if self.__moveit_configs.trajectory_execution:
                parameters.update(self.__moveit_configs.trajectory_execution)
            if self.__moveit_configs.planning_scene_monitor:
                parameters.update(self.__moveit_configs.planning_scene_monitor)
            if self.__moveit_configs.joint_limits:
                parameters.update(self.__moveit_configs.joint_limits)
            if self.__moveit_configs.moveit_cpp:
                parameters.update(self.__moveit_configs.moveit_cpp)
        return parameters


# USAGE
# moveit_config = (
#     MoveItConfigsBuilder("ROBOT_NAME")
#         .robot_description()
#         .robot_description_semantic()
#         .robot_description_kinematics()
#         .joint_limits()
#         .trajectory_execution(file_path="panda_gripper_controllers.yaml")
#         .planning_scene_monitor()
#         .planning_pipelines()
#         .to_moveit_configs()
# )
