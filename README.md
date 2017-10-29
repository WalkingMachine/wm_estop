# wm_estop

The supervisor node is here for manage the e-stop security button of [**S.A.R.A.**](http://walkingmachine.ca/).

It uses an Arduino linked with a `serial connection` to the computer and a `ROS node`.

### How it work :

When the `estop` is pressed, the system go in **stop** mode. To go back in **run** mode you need to unlock `estop` AND press on the **restart** button.

### Control :

It is two-way controlled thanks to watchdogs :

- If the **arduino** lose the control with the **computer**, the power of the robot will be turned off.

- If the **computer** lose the controle with the **arduino**, the reaction as if the estop was pressed.

### LED signification :

TODO

## The node implementation:

### Publisher :

The ros node `wm_estop` publish every time the state of the **estop**  on the `supervisor/estop_state` topic.

*It uses [`std_msgs::Bool`](http://docs.ros.org/api/std_msgs/html/msg/Bool.html) messages.*

### Service :

It also calls a service for the communicating with [**Ros Control**](https://github.com/WalkingMachine/sara_control).

The service name is `estop`, it's a [`std_srvs::SetBool`](http://docs.ros.org/jade/api/std_srvs/html/srv/SetBool.html).

### Usage :

For both **publisher** and **service** data :

|**State**|**Message**|
|:-------:|:---------:|
|`RUN`    |`TRUE`     |
|`STOP`   |`FALSE`    |

## Install :

TODO

### Wiring :

TODO

## Data transmission :

TODO

## Credits

- [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

## License

MIT
