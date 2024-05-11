from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("asabe_robot_description", package_name="sam_bot_nav2_gz_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
