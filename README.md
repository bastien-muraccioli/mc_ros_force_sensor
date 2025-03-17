# mc_ros_force_sensor plugin

This project is an mc_rtc plugin designed to retrieve force/torque data from the Bota SensONE F/T sensor's ROS topic.

It can be used to access any ROS message related to force/torque sensor data, including wrench messages.

## Quick start

1. **Build and install the project.**

   > Note: If you are using [mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild), create `mc_ros_force_sensor.cmake`, inside `mc-rtc-superbuild/extensions/plugins/`.

   With the following content:    
   
   ```cmake
   AddProject( mc_ros_force_sensor
     GITHUB bastien-muraccioli/mc_ros_force_sensor
     GIT_TAG origin/main
     DEPENDS mc_rtc
   )
   ```

   Then, add it in the `extensions/local.cmake` file and rebuild the superbuild.

2. **Create `RosForceSensor.yaml` plugin config file** inside `~/.config/mc_rtc/plugins/`.

   In the configuration file, specify the reference frame of the force/torque sensor used in your `xacro`/`urdf`, as well as the name of the ROS topic.

   Example configuration:

   ```yaml
   reference_frame: FT_sensor_wrench
   ros_force_sensor: true
   ros_topic_sensor: /bus0/ft_sensor0/ft_sensor_readings/wrench
   ft_freq: 1000
   ```

3. **Run using your [mc_rtc] interface of choice.** Add `RosForceSensor` to the `Plugins` configuration entry or enable the autoload option.

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/