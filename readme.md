# iXblue INS driver - Protocol iXblue stdbin

![ROS CI Badge](https://github.com/ixblue/ixblue_ins_stdbin_driver/workflows/ROS%20CI/badge.svg)
![CI Badge](https://github.com/ixblue/ixblue_ins_stdbin_driver/workflows/CI/badge.svg)

iXblue is a global leader in the design and manufacturing of innovative
solutions devoted to navigation, positioning and underwater imaging,
shipbuilding, as well as photonics. Using its unique in-house technology,
the company offers turnkey solutions to its Civil and Defense customers to
carry out their sea, land and space operations with optimum efficiency and
reliability. Employing a workforce of 600 people worldwide, iXblue conducts
its business with over 60 countries.


The aim of an iXblue driver is to give access to the ROS community to iXblue inertial system data.
To do so, we have created a driver which only works in UDP and calls the [library](https://github.com/ixblue/ixblue_stdbin_decoder) which parses iXblue's stdbin protocol.
You can find more details on this protocol in the implementation section.

![Alt text](images/logos.png)

---
## Table of Contents


1. [Installation](#installation)
2. [Parameters](#parameters)
3. [Implementation](#implementation)
4. [Roadmap](#roadmap)
5. [License](#license)

---
## Installation

### Binary Installation

On Ubuntu Bionic, the package is released so the installation is as simple as:

```sh
sudo apt install ros-melodic-ixblue-ins
```

### Installation from source

Notice that, because of ROS requirement, the driver can only be used in a Linux environnement.

#### Dependencies

* [Boost](https://www.boost.org/) - Useful C++ library
* [Libpcap](https://www.tcpdump.org/) - (optional) Used to read pcap files in order to test the code
* [Gtest](https://github.com/google/googletest) - Used to create the unit tests
* [ixblue_stdbin_decoder](https://github.com/ixblue/ixblue_stdbin_decoder) - iXblue standard binary protocol parsing library

#### Build the driver

First, clone the [ixblue_stdbin_decoder](https://github.com/ixblue/ixblue_stdbin_decoder) library and the
[driver](https://github.com/ixblue/ixblue_ins_stdbin_driver) into the `src` directory of your catkin workspace
thanks to the commands :

```sh
git clone https://github.com/ixblue/ixblue_stdbin_decoder.git
git clone https://github.com/ixblue/ixblue_ins_stdbin_driver.git
```

Then you can build the code in your catkin workspace:

```sh
cd ..
catkin_make
```

### Use the driver

There are two ways to launch the ROS driver node:
* Thanks to the [launchfile](ixblue_ins_driver/launch/ixblue_ins_driver.launch):
```sh
roslaunch ixblue_ins_driver ixblue_ins_driver.launch
```
* Or directly with the ROS node:

|       First Terminal         |          Second Terminal       |
| ---------------------------- | ------------------------------ |
|          `roscore`           | `rosrun ixblue_ins_driver node`|

## Parameters

Depending on, your requirements and your configuration, you can also modify some arguments.

* **frame_id** (*string*, default: `imu_link_ned`): frame of the sensor, used in the headers of the messages.
* **udp_port** (*int*, default: `8200`): your system will receive the data from the INS by this port.
* **ip** (*string*, default: `0.0.0.0`):  the address of the network interface to listen to
* **time_source** (*string*, default: `ins`): determine the source of the timestamp data. "ins" for ins timestamp. "ros" for ROS timestamp.
* **time_origin** (*string*, default: `unix`): determine the time origin of the timestamp. "sensor_default" for ins base time. "unix" for UNIX base time (since 1st of january 1970).
* **expected_frequency** (*double*, default: `10.0`): expected INS output frequency in Hz, used for diagnostics. Must match the setting on the INS configuration webpage.
* **max_latency** (*double*, default: `1.0`): maximum acceptable timestamp delay in seconds.
* **connection_lost_timeout** (*double*, default: `10.0`): time without receiving data before switching to error diagnostic.
* **use_compensated_acceleration** (*bool*, default: `false`): use acceleration compensated from gravity and Coriolis.

**[Back to top](#table-of-contents)**

---
## Implementation

### iXblue stdbin protocol

The protocol iXblue stdbin allows to obtain as much information as possible from inertial system data thanks to its modularity.
Here is a non exhaustive list of data that are proposed in the protocol:
* Attitude
* Positions
* Status
* Accelerations
* ...
This information can be given in vessel or geographic frame, compensated or not from gravity and earth rotation.
You can find the details of this protocol in the interface library document of your product user manual.

### ROS driver

#### Structure
The iXblue ROS driver is divided into three parts:
* `ixblue_ins`: ROS metapackage for iXblue INS
* `ixblue_ins_driver`: ROS driver which receives the data and sends the messages
* `ixblue_ins_msgs`: iXblue messages definition

#### Published topics

The messages are filled into the `ros_publisher.cpp` file.

To follow the [REP-145](https://www.ros.org/reps/rep-0145.html), the sensor data are not transformed within the driver, only the units are modified in order to respect [REP-103](https://www.ros.org/reps/rep-0103.html).

To change the orientation of the INS within your system, you can use the [`imu_transformer`](http://wiki.ros.org/imu_transformer) package.

*ROS standard messages:*

* **~/standard/navsatfix** [sensors_msgs/NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)
* **~/standard/imu** [sensors_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)
* **~/standard/timereference** [sensors_msgs/TimeReference](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html)

*iXblue message :*

* **iX/ins** [ixblue_ins_msgs/ins](ixblue_ins_msgs/msg/Ins.msg)

[ixblue_ins_msgs/ins](ixblue_ins_msgs/msg/Ins.msg) is defined as:

```
Header header # Standard ROS message header

int8 ALT_REF_GEOID=0
int8 ALT_REF_ELLIPSOID=1

# Position
float64     latitude                             # In degrees, positive north, [-90,90]
float64     longitude                            # In degrees, increasgin toward east, [0,360]
int8        altitude_ref                         # 0: Geoid / 1: Ellipsoid, see constant above.
float32     altitude                             # In meters, positive up

# Position Deviation
float64[9] position_covariance                   # In square meters, ENU in row-major order
                                                 # Null matrix if unknown

# Attitude
float32 heading                                  # In degrees, [0,360]
float32 roll                                     # In degrees, positive port up, [-180,180]
float32 pitch                                    # In degrees, positive bow down, [-90,90]

# Attitude Deviation
float64[9] attitude_covariance                   # In rad2/sec2
                                                 # Null matrix if unknown

# Speed Vessel Frame
geometry_msgs/Vector3 speed_vessel_frame         # In m/s, x: forward, y: left, z: up

# Speed Vessel Frame Deviation
float64[9] speed_vessel_frame_covariance         # In m2/s4, ENU in row-major order
                                                 # Null matrix if unknown

```

## Contributing:

If the above messages do not correspond to your application feel free to
* add your own messages
* make a request for iXblue to add a message with the missing information, if they are available in the protocol.

Feel free to open an issue on this github repository if you encounter any problem.

**[Back to top](#table-of-contents)**

---
## Roadmap

* Decode INS status: user, algorithm, sensors, temperatures
* Allow to use the driver with TCP protocol
* Create a ROS 2 version of this driver.

**[Back to top](#table-of-contents)**

---
## License

This project is licensed under the MIT License

#### (The MIT License)

Copyright (c) 2019 iXblue SAS

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
'Software'), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


**[Back to top](#table-of-contents)**
