# yaro-ros2-tests

C++17 / ROS 2 Jazzy package that loads the public URDF descriptions of the YARO cobots
(YardStick Robotics, MIT licence) and tests them: kinematics, a model audit, a comparison with
the published data sheet, and a joint limit monitor node. Everything builds and runs in Docker,
and every number in [docs/RESULTS.md](docs/RESULTS.md) is generated from `results/*.json`.

**Live page:** https://huggingface.co/spaces/pavanyadava07/yaro-ros2-tests (try the limit monitor in the browser; its JavaScript
kinematics is checked against the C++ library in CI, worst difference 5e-12 m over 250 configurations).

This is an independent project. It is not affiliated with or endorsed by YardStick Robotics or Rheinmetall.
No real robot was used; all results come from the URDF files and the public data sheet.

## What is in it

| part | what it does |
|---|---|
| `yaro_check_core` (C++ library) | Serial-chain forward kinematics and geometric Jacobian from a URDF (Eigen, no KDL), joint position / velocity limit checks, model audit rules, data sheet comparison |
| `joint_limit_monitor` (ROS 2 node, rclcpp) | Subscribes to `/joint_states`, publishes `/diagnostics` per joint (OK / WARN / ERROR), `~/limits_ok` and `~/tip_pose`; finite-difference velocities when a message has none |
| `yaro_audit` | Audit + data sheet comparison + sampled reach for all models, as JSON |
| `yaro_bench` | FK and Jacobian timing against KDL |
| `yaro_fk` | Tip position for one joint configuration (used to check the web page's kinematics) |
| tests | GoogleTest suites (kinematics, limits, audit, data sheet, KDL cross-check) and a launch_testing integration test that talks to the running node over DDS |

## Results (from docs/RESULTS.md)

- **50 test cases, 0 failures**: 44 GoogleTest cases and 6 launch_testing cases, built with `-Wall -Wextra -Wpedantic -Wshadow -Werror`.
- FK and Jacobian agree with KDL to **below 1e-15** on 1000 random configurations per model, on all five public models.
- The URDFs are physically consistent: no audit errors, sampled reach within 25 mm of the data sheet reach on all five
  models, URDF mass within 1.2 kg of the data sheet weight.
- Two warnings on every model: 14 mesh paths are relative (`../meshes/stl/...`) instead of `package://` or `file://`
  URIs, and ROS `resource_retriever` rejects them (`ValueError: unknown url type`); and the root link carries an inertia
  that KDL ignores (kdl_parser prints a warning).
- **46 of 60 joint range and speed values in the URDFs differ from the data sheet**, e.g. yaro_1105 A3 is 480 deg/s in
  the URDF and 270 deg/s on the data sheet. The table in RESULTS.md lists all of them. Which source a controller should
  enforce is for the robot's owners to decide; the monitor has a `velocity_scale` parameter for a reduced limit.
- Own FK 1.4x and Jacobian 2.1x faster than KDL on one CPU core (AMD EPYC 7R13).

## Run it

```bash
make all          # docker build (colcon), colcon test, audit, bench, docs/RESULTS.md
make test         # tests only
```

Live monitor with a model of your choice:

```bash
docker run --rm -it yaro-ros2-tests:jazzy bash -c \
  '. /ws/ros2_ws/install/setup.sh && ros2 launch yaro_check monitor.launch.py model:=yaro_1105 velocity_scale:=0.25'
```

## Layout

```
models/            vendored URDFs + LICENSE per model, PINNED_COMMITS.txt, datasheet.txt
ros2_ws/src/yaro_check/
  include/ src/    library, node, CLIs
  test/            GoogleTest suites, launch_testing test
  launch/          monitor.launch.py
docker/Dockerfile  ros:jazzy-ros-base, nothing extra installed
scripts/           test result collector, report generator, Space builder and its FK check
site/              static Hugging Face Space (index.html, fk.js, data.json)
results/           raw JSON behind docs/RESULTS.md
```

## Licence

Code: MIT (see LICENSE). The files under `models/yaro_*/robot.urdf` are copied unchanged from YardStick Robotics'
public repositories and remain under their MIT licence (copyright 2025 Yardstick Robotics); see THIRD_PARTY.md.
