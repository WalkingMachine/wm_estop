# wm_supervisor

The supervisor node is here for manage the e-stop security button of [**S.A.R.A.**](http://walkingmachine.ca/).

It use an Arduino linked with a `serial connection` with the computer, and a `ROS node`.

### How it work :

When the `estop` is pressed, the system go in **stop** mode. For go back in **run** mode you need to unlock `estop` AND press on **restart** button.

### Control :

It is two ways controlled thank to watchdogs :

- If the **arduino** lost the controle with the **computer**, the power of the robot will be turned off.

- If the **computer** lost the controle with the **arduino**, the reaction will be the same as if the estop was pressed.

### LED signification :

TODO

## The node implementation:

### Publisher :

The ros node `wm_supervisor` publish every time the state of the **estop** on t`supervisor/estop_state` topic.

*It use [`std_msgs::Bool`](http://docs.ros.org/api/std_msgs/html/msg/Bool.html) messages.*

### Service :

It also call a service for the communication with [**Ros Control**](https://github.com/WalkingMachine/sara_control).

Service name is `estop`, it's a [`std_srvs::SetBool`](http://docs.ros.org/jade/api/std_srvs/html/srv/SetBool.html).

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