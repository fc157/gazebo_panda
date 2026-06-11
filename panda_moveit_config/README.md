# panda_moveit_config

MoveIt 2 configuration for the Franka Emika Panda arm on **ROS 2 Foxy + Gazebo 11**.

This package contains the SRDF, kinematics, joint limits, OMPL planning,
controller bridging and launch files needed to plan and execute trajectories
on the Panda simulated in Gazebo with MoveIt 2.

---

## Workspace layout

```
panda/                       # original Panda description + Gazebo launch
  urdf/   meshes/   config/   launch/   src/

panda_moveit_config/         # this package
  config/
    panda.srdf                   Semantic description (groups, states, collisions)
    kinematics.yaml              Kinematics solver (KDL)
    joint_limits.yaml            Velocity / acceleration limits per joint
    initial_positions.yaml       Initial configuration
    ompl_planning.yaml           OMPL planner configurations
    panda_controllers.yaml       (informational) controller joint lists
    moveit_controllers.yaml      MoveItSimpleControllerManager bridge
    sensors_3d.yaml              3D sensors (empty by default)
    panda_moveit_controller_manager.launch.xml
  launch/
    demo.launch.py               # Full demo: Gazebo + MoveIt + RViz
    demo_fake.launch.py          # MoveIt only (no Gazebo) - uses fake controllers
    planning_context.launch.py   # move_group node + all MoveIt params
    moveit_rviz.launch.py        # RViz with the MotionPlanning plugin
    spawn_controllers.launch.py  # Optional spawner for ros2_control controllers
    ros2_controllers.launch.py   # Standalone controller_manager
  src/
    moveit_test.cpp              # C++ example: joint / Cartesian planning
```

---

## Build

The package is part of the same colcon workspace as `panda`:

```bash
cd /home/test/zfc/code/gazebo/panda
source /opt/ros/foxy/setup.bash
colcon build --packages-select panda panda_moveit_config
source install/setup.bash
```

The C++ test node `moveit_test` is built into
`install/panda_moveit_config/lib/panda_moveit_config/moveit_test`.

---

## Run

### Full demo (Gazebo + MoveIt + RViz)

```bash
ros2 launch panda_moveit_config demo.launch.py
```

Sequence:
| t (s) | What happens |
|-------|---------------|
| 0     | Gazebo server, gzclient, robot_state_publisher, spawn robot |
| 5     | `joint_state_broadcaster` started |
| 7     | `panda_arm_controller` started |
| 9     | `panda_gripper_controller` started |
| 10    | `move_group` (MoveIt) starts |
| 12    | RViz (with MotionPlanning plugin) starts |

In RViz, add the `MotionPlanning` display (or use the default config),
select `panda_arm` as the planning group and click **Plan & Execute**.

### Offline demo (no Gazebo)

```bash
ros2 launch panda_moveit_config demo_fake.launch.py
```

This uses `MoveItFakeControllerManager` and `joint_state_publisher` to
simulate joint updates, so MoveIt can plan and "execute" without any
simulator.

### Programmatic C++ test

In a separate shell (with the demo already running):

```bash
ros2 run panda_moveit_config moveit_test
```

The node will:
1. Move to the named `ready` state.
2. Move to a custom joint configuration.
3. Compute and execute a short Cartesian path (down 10 cm, right 10 cm, up 10 cm).
4. Move back to `ready`.

---

## MoveIt groups and states

The SRDF defines three MoveIt groups:

* **`panda_arm`** – kinematic chain `panda_link0 -> panda_link8` (7-DOF).
* **`hand`** – gripper (single `panda_finger_joint1` + mimic'd `panda_finger_joint2`).
* **`panda_arm_hand`** – union of the two above.

Named states:
* `panda_arm`: `ready`, `extended`, `transport`.
* `hand`: `open`, `close`.

End-effector: `hand` is the end-effector of `panda_arm` on `panda_link8`.

---

## Controllers

The MoveIt controller manager is `MoveItSimpleControllerManager`, configured
in `config/moveit_controllers.yaml`:

| MoveIt name              | Action type            | Action namespace            |
|--------------------------|------------------------|-----------------------------|
| `panda_arm_controller`   | `FollowJointTrajectory`| `panda_arm_controller/follow_joint_trajectory` |
| `panda_gripper_controller`| `FollowJointTrajectory`| `panda_gripper_controller/follow_joint_trajectory` |

These match the controllers started by `panda/launch/gazebo.launch.py` (which
loads `panda_arm_controller` and `panda_gripper_controller` as
`joint_trajectory_controller`s in `panda/config/ros2_controllers.yaml`).

If you prefer `GripperCommand` for the gripper, replace the
`panda_gripper_controller` block in `moveit_controllers.yaml` with
`type: GripperCommand` and `action_ns: gripper_cmd`, and switch the
underlying controller to `gripper_controllers/GripperActionController`.

---

## Tips

* If `move_group` complains about the robot model, make sure the URDF on
  `robot_description` is being published before `move_group` starts. The
  launch file uses a `RegisterEventHandler` on the gripper controller to
  start `move_group` only after all controllers are loaded.
* For a headless Gazebo run, comment out the `gazebo_client` `ExecuteProcess`
  in `launch/demo.launch.py`.
* To use a different kinematics solver, edit `config/kinematics.yaml`
  (e.g. `kdl_kinematics_plugin/KDLKinematicsPlugin` or
  `pick_ik/PickIkPlugin`).
