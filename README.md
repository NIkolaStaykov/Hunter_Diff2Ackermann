# Definitions
Let us define the following variables:

1. $v_b$ is the base linear velocity.
2. $\omega_b$ is the rotational velocity of the base
3. $\theta_b$ is the steering angle of the base relative to the body-frame x direction (also forward velocity vector)

# Assumptions and Limitations
## Physics
1. Following the problem description we need to convert the tuple ($v_b$, $\omega_b$) in differential drive to ($v_b$, $\theta_b$) in Ackerman drive.
2. **Dimensions**: We consider exclusively rotations of the base relative to the the z axis and motion in the 2D XY space.
3. We assume a kinematic-level control, ignoring lateral slip and dynamic effects.
4. The differential drive controls are defined for the geometric center of the robot and include linear and rotational velocity. We do not deal with individual tire rotation speeds.
5. Using the ackerman steering we model the center of the rear axis and it always moves only in local x direction. For differential drive we have a similar situation, but for the geometric center of the base. In the conversion between differential and ackerman drive, we will assume those two points are the same. This is a modeling **limitation**, but also highlights the physical infeasibility of a completely equivalent transformation between the two control modes.

## Geometry File
1. We assume in the URDF the robot local frame is aligned with the global frame. We can then extract the length base and track width from the joint positions in the URDF.

## Implementation
1. To enforce physical limitations, we will consider the joint rotation limits of the individual tires and calculate a steering angle limit from that.
2. The joint-level Ackerman control is performed by the official ackermann_steering_controller package.
3. The Ackerman controller already enforces limitations on the maximum steering angles. We will consider the case when the global steering angle goes out of bounds and not differentiate between the individual tire angles. This limitation is acceptable since correct control is ensured and angle deviations between the induvidual tires are small.
4. We have the following topic structure:
   1. **/diff_drive/vel_cmd:** differential drive control message, *base velocities*
   2. **/cmd_vel:** ackerman control message, *linear velocity and steering angle*

# Challenges

## 1. Using Docker with GPU acceleration with Windows host and WSL2, ended up installing Ubuntu
## 2. Extracting robot data from the URDF
## 3. Considering all edge cases and limits
**Included Limit Checks for:**
1. Tire joint rotation limit -> steering angle limit
2. Linear velocity limit

# Approach
1. Extract model parameters from the URDF robot file, read from the ros parameters
2. Generate extra parameters such as **max steering angle** from the maximum joint rotation
3. Launch a subscriber and a publisher.
4. On every differential drive control message, calculate the steering angle from the base velocities and check for constraint violations.
5. Publish the ackermann drive message to topic ```\cmd_vel ```. The ros2 ackerman controller takes it from here. 

# Edge Cases and Evaluation
## No linear motion
The Ackerman controller cannot directly achieve isolated rotation. Withouth linear velocity there will be no movement, the node sets the steering angle to its maximum to avoid division by 0 when calculating the steering angle.

## Circular motion
When using ```"{linear: {x: 1}, angular: {z: 0.5}}"``` as a differential drive command, we observe the steering angle being approximately 0.26 and the robot making circles with radius 2, exactly as expected.

# Usage
## Docker Setup
1. Open the root repo folder in vscode, build and connect to a devcontainer.
2. Extension **Dev Containers** needed.
3. Install and launch the Hunter_SL gazebo simulation as instructed [here.](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/humble/hunter_se)
## ROS Setup
1. To setup the ros package
   ``` bash
   cd ackermann_ws  
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   ```
3. To setup the systemd service
   ``` bash
   sudo cp install/ackermann_bridge/share/ackermann_bridge/config/ackermann_bridge.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable ackermann_bridge.service
   sudo systemctl start ackermann_bridge.service
   ```

## Testing
1. **Direct Usage**
   ``` bash
   cd ackermann_ws
   source install/setup.bash
   ros2 launch ackermann_bridge bridge.launch.py
   ```
   In another terminal:
   ``` bash
   ros2 topic pub --once /diff_drive/vel_cmd geometry_msgs/msg/Twist "{linear: {x: INPUT}, angular: {z: INPUT}}"
   ```
   to publish a differential drive command. Fill in the respective speeds.

2. **Systemd Verification:**
   ```
   sudo systemctl start ackermann_bridge.service
   journalctl -u ackermann_bridge.service -f
   ros2 node list  # Should show 'control_transformer'
   ```
   **Note:** The systemd verification was done directly on the host machine outside the container.