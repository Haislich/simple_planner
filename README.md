# Introduction

In this context a map is a PGM file.
The choice of PGM, is due to it's inherent [simplicity](https://raytracing.github.io/books/RayTracingInOneWeekend.html#outputanimage/theppmimageformat:~:text=Whenever%20you%20start%20a%20renderer%2C%20you%20need%20a%20way%20to%20see%20an%20image.%20The%20most%20straightforward%20way%20is%20to%20write%20it%20to%20a%20file.%20The%20catch%20is%2C%20there%20are%20so%20many%20formats.%20Many%20of%20those%20are%20complex.%20I%20always%20start%20with%20a%20plain%20text%20ppm%20file.%20Here%E2%80%99s%20a%20nice%20description%20from%20Wikipedia%3A).
A natural choiche would've been PBM, in which the encoding would've been white for

## Setup

Assuming you are using `ros-noetic`

```bash
sudo apt-get install ros-noetic-map-server
```

## How to run

```bash
git clone ...
cd simple_planner_ws
source devel/setup.bash`
rosrun simple_planner simple_planner_node
```

## Additional Resources

- [PGM format](https://netpbm.sourceforge.net/doc/pgm.html)
- [MAP server](https://wiki.ros.org/map_server)
- [Marker Example for rviz](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)
