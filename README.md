# WLED_ros

Control of WLED WiFi LED controllers from ROS, using the [Python-WLED](https://github.com/frenck/python-wled) library.

## Start the nodes

``` bash
roslaunch wled_ros wled_ros.launch
```

## Usage example

* To play the "Rainbow" effect on LEDs 100-125, with intensity 220, speed 255, and duration 2.0 seconds:

``` bash
rosrun wled_ros play_on_segment_client.py --effect-name "Rainbow" --effect-intensity 220 --effect-speed 255 --start 100 --stop 125 --duration 2.0
```

* To light up all LEDs (0-300) to brightness 127 with colour (255, 0, 0) (red) until next instructed:

``` bash
rosrun wled_ros play_on_segment_client.py --effect-name "Solid" --start 0 --stop 300 --brightness 127 --primary-colour 255 0 0
```
