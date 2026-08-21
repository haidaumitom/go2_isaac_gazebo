# Go2 IsaacLab Deployment Workspaces

This branch separates simulation verification from physical deployment:

- `go2_isaac_gazebo/`: Gazebo and ROS 2 policy testing.
- `go2_real_deployment/rl_sar/`: standalone onboard Go2 deployment runtime.

Start with each workspace README. Train in IsaacLab, validate in Gazebo, then
copy only approved policy bundles to the real-robot deployment workspace.

The branch is `update-deployment-ws` because Git branch names cannot contain spaces.
