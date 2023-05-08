from moveit_configs_utils import MoveItConfigsBuilder
import re
import pytest
from launch_param_builder.utils import ParameterBuilderFileNotFoundError

# Get robot name from moveti_resources_ROBOT_moveit_config package name
ROBOT_NAME = re.compile(r"moveit_resources_(.*)_moveit_config")


def test_moveit_resources_configs():
    for pkg_name in [
        "moveit_resources_fanuc_moveit_config",
        "moveit_resources_panda_moveit_config",
    ]:
        try:
            robot_name = re.match(ROBOT_NAME, pkg_name).group(1)
            builder = MoveItConfigsBuilder(pkg_name)
            assert builder._robot_description_config is None
            assert builder._robot_description_semantic_config is None
            assert builder._robot_description_kinematics_config is None
            assert builder._planning_pipelines_config is None
            assert builder._trajectory_execution_config is None
            assert builder._sensors_config is None
            assert builder._joint_limits_config is None
            assert builder._moveit_cpp_config is None
            builder.robot_description(file_path=f"config/{robot_name}.urdf.xacro")
            builder.robot_description_semantic(file_path=f"config/{robot_name}.srdf")
            builder.robot_description_kinematics(file_path=f"config/kinematics.yaml")
            builder.planning_pipelines(
                pipelines=[
                    "ompl",
                    "chomp",
                ]
            )
            builder.trajectory_execution(file_path="config/moveit_controllers.yaml")
            if robot_name == "panda":
                builder.sensors(file_path="config/sensors_kinect_pointcloud.yaml")
            else:
                builder.sensors(file_path="config/sensors_3d.yaml")
            builder.joint_limits(file_path="config/joint_limits.yaml")

            with pytest.raises(ParameterBuilderFileNotFoundError):
                builder.moveit_cpp(file_path="config/moveit_cpp.yaml")

            assert builder._robot_description_config is not None
            assert builder._robot_description_semantic_config is not None
            assert builder._robot_description_kinematics_config is not None
            assert len(builder._planning_pipelines_config.pipelines) == 2
            assert len(builder._planning_pipelines_config.configs) == 2
            assert builder._trajectory_execution_config is not None
            assert builder._sensors_config is not None
            assert builder._joint_limits_config is not None
            assert builder._moveit_cpp_config is None

            assert builder.to_moveit_configs()

        except RuntimeError as e:
            assert False, f"Default {pkg_name} configuration failed to build: {e}"
