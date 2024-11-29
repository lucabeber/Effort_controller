# Catesian Impedance Controller

This controller implements a catesian impedance controller which take a desired pose and a desired force in the cartesian frame.





## Example Configuration
Below is an example `controller_manager.yaml` for a controller specific configuration.
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    cartesian_impedance_controller:
      type: cartesian_impedance_controller/CartesianImpedanceController

    # More controller instances here
    # ...

cartesian_impedance_controller:
  ros__parameters:
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    ft_sensor_ref_link: "sensor_link"
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6

    solver:
        error_scale: 0.5

    postural_task: true  # default value: true
    nullspace_stiffness: 10.0  # default value, used only when postural task is enabled

    stiffness:
        trans_x: 500
        trans_y: 500
        trans_z: 500
        rot_x: 50
        rot_y: 50
        rot_z: 50


# More controller specifications here
# ...

```

