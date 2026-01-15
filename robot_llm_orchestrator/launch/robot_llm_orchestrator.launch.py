from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    problem_pddl_topic = LaunchConfiguration("problem_pddl_topic")
    domain_file = LaunchConfiguration("domain_file")
    publish_partials = LaunchConfiguration("publish_partials")
    max_pddl_chars = LaunchConfiguration("max_pddl_chars")
    max_new_tokens = LaunchConfiguration("max_new_tokens")
    max_context_len = LaunchConfiguration("max_context_len")
    top_p = LaunchConfiguration("top_p")
    temperature = LaunchConfiguration("temperature")
    repeat_penalty = LaunchConfiguration("repeat_penalty")
    keep_history = LaunchConfiguration("keep_history")
    skip_special_token = LaunchConfiguration("skip_special_token")
    lora_model_path = LaunchConfiguration("lora_model_path")
    lora_adapter_name = LaunchConfiguration("lora_adapter_name")
    lora_scale = LaunchConfiguration("lora_scale")
    prompt_cache_path = LaunchConfiguration("prompt_cache_path")
    save_prompt_cache = LaunchConfiguration("save_prompt_cache")
    system_prompt_file = LaunchConfiguration("system_prompt_file")
    busy_mode = LaunchConfiguration("busy_mode")
    publish_planner_feedback = LaunchConfiguration("publish_planner_feedback")
    planner_timeout_sec = LaunchConfiguration("planner_timeout_sec")
    planner_goal_topic = LaunchConfiguration("planner_goal_topic")
    planner_status_topic = LaunchConfiguration("planner_status_topic")
    enable_robot_planner = LaunchConfiguration("enable_robot_planner")
    robot_planner_launch_file = LaunchConfiguration("robot_planner_launch_file")
    enable_plansys2 = LaunchConfiguration("enable_plansys2")
    plansys2_launch_package = LaunchConfiguration("plansys2_launch_package")
    plansys2_launch_file = LaunchConfiguration("plansys2_launch_file")
    plansys2_domain_file = LaunchConfiguration("plansys2_domain_file")
    plansys2_problem_file = LaunchConfiguration("plansys2_problem_file")
    plansys2_domain_arg = LaunchConfiguration("plansys2_domain_arg")
    plansys2_problem_arg = LaunchConfiguration("plansys2_problem_arg")

    robot_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_planner"), "launch", robot_planner_launch_file]
            )
        ),
        condition=IfCondition(enable_robot_planner),
        launch_arguments={
            "enable_plansys2": enable_plansys2,
            "plansys2_launch_package": plansys2_launch_package,
            "plansys2_launch_file": plansys2_launch_file,
            "plansys2_domain_arg": plansys2_domain_arg,
            "plansys2_problem_arg": plansys2_problem_arg,
            "domain_file": plansys2_domain_file,
            "problem_file": plansys2_problem_file,
        }.items(),
    )

    default_domain = PathJoinSubstitution(
        [FindPackageShare("robot_planner"), "pddl", "domain.pddl"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=EnvironmentVariable("LLM_MODEL", default_value="qwen3-vl-4b-instruct_w8a8_rk3588.rkllm"),
        ),
        DeclareLaunchArgument("problem_pddl_topic", default_value="/pddl/problem"),
        DeclareLaunchArgument("domain_file", default_value=default_domain),
        DeclareLaunchArgument("publish_partials", default_value="true"),
        DeclareLaunchArgument("max_pddl_chars", default_value="12000"),
        DeclareLaunchArgument("max_new_tokens", default_value="256"),
        DeclareLaunchArgument("max_context_len", default_value="2048"),
        DeclareLaunchArgument("top_p", default_value="0.95"),
        DeclareLaunchArgument("temperature", default_value="0.8"),
        DeclareLaunchArgument("repeat_penalty", default_value="1.1"),
        DeclareLaunchArgument("keep_history", default_value="0"),
        DeclareLaunchArgument("skip_special_token", default_value="true"),
        DeclareLaunchArgument("lora_model_path", default_value=""),
        DeclareLaunchArgument("lora_adapter_name", default_value="default"),
        DeclareLaunchArgument("lora_scale", default_value="1.0"),
        DeclareLaunchArgument("prompt_cache_path", default_value=""),
        DeclareLaunchArgument("save_prompt_cache", default_value="false"),
        DeclareLaunchArgument("system_prompt_file", default_value="config/system_prompt.json"),
        DeclareLaunchArgument("busy_mode", default_value="reject"),
        DeclareLaunchArgument("publish_planner_feedback", default_value="false"),
        DeclareLaunchArgument("planner_timeout_sec", default_value="0.0"),
        DeclareLaunchArgument("planner_goal_topic", default_value="/robot_planner/goal"),
        DeclareLaunchArgument("planner_status_topic", default_value="/robot_planner/goal_status"),
        DeclareLaunchArgument("enable_robot_planner", default_value="true"),
        DeclareLaunchArgument("robot_planner_launch_file", default_value="planner.launch.py"),
        DeclareLaunchArgument("enable_plansys2", default_value="true"),
        DeclareLaunchArgument("plansys2_launch_package", default_value="plansys2_bringup"),
        DeclareLaunchArgument("plansys2_launch_file", default_value="plansys2_bringup_launch_monolithic.py"),
        DeclareLaunchArgument("plansys2_domain_arg", default_value="model_file"),
        DeclareLaunchArgument("plansys2_problem_arg", default_value="problem_file"),
        DeclareLaunchArgument("plansys2_domain_file", default_value=domain_file),
        DeclareLaunchArgument("plansys2_problem_file", default_value="/tmp/problem.pddl"),
        robot_planner_launch,
        Node(
            package="ros2_rknn_llm",
            executable="rkllm_node",
            name="rkllm_node",
            output="screen",
            parameters=[
                {
                    "model_path": model_path,
                    "problem_pddl_topic": problem_pddl_topic,
                    "domain_file": domain_file,
                    "publish_partials": publish_partials,
                    "max_pddl_chars": max_pddl_chars,
                    "max_new_tokens": max_new_tokens,
                    "max_context_len": max_context_len,
                    "top_p": top_p,
                    "temperature": temperature,
                    "repeat_penalty": repeat_penalty,
                    "keep_history": keep_history,
                    "skip_special_token": skip_special_token,
                    "lora_model_path": lora_model_path,
                    "lora_adapter_name": lora_adapter_name,
                    "lora_scale": lora_scale,
                    "prompt_cache_path": prompt_cache_path,
                    "save_prompt_cache": save_prompt_cache,
                    "system_prompt_file": system_prompt_file,
                    "busy_mode": busy_mode,
                    "publish_planner_feedback": publish_planner_feedback,
                    "planner_timeout_sec": planner_timeout_sec,
                    "planner_goal_topic": planner_goal_topic,
                    "planner_status_topic": planner_status_topic,
                }
            ],
        ),
    ])
