# Background
Use MoveIt to execute a pick & place operation that stacks two blocks on top of each other. Leverage the Panda arm environment provided in the MoveIt install.

## Milestone #1

Launch panda MoveIt environment

Run `setup` task
Run `install dependencies` task
Run `build` task
`roslaunch panda_moveit_config demo.launch rviz_tutorial:=true`

https://ros-planning.github.io/moveit_tutorials/

## Milestone #2

Create a node that exposes a service to randomly add two boxes to the planning scene

## Milestone #3

Write a service that when called, plans and executes a motion that stacks on box on the other

### Bonus: Use an action instead of a service

### Acknowledgements

VSCode devcontainer environment setup inspired Allison Thackston's work found here:
- https://github.com/athackst/vscode_ros2_workspace
- https://www.allisonthackston.com/articles/vscode-docker-ros2.html
