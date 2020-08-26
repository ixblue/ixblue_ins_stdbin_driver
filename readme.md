# iXblue INS driver - Protocol iXblue stdbin

iXblue is a global leader in the design and manufacturing of innovative 
solutions devoted to navigation, positioning and underwater imaging, 
shipbuilding, as well as photonics. Using its unique in-house technology, 
the company offers turnkey solutions to its Civil and Defense customers to 
carry out their sea, land and space operations with optimum efficiency and 
reliability. Employing a workforce of 600 people worldwide, iXblue conducts 
its business with over 60 countries.


The aim of an iXblue driver is to give access to ROS community to iXblue inertial system data. 
To do so, we create a driver which only works in UDP and call the library which parses iXblue's stdbin protocol. 
You can find more details on this protocol in the implementation section.

![Alt text](images/logos.png)

---
## Table of Contents


1. [Installation](#installation)
2. [Implementation](#implementation)
3. [Roadmap](#roadmap)
4. [License](#license)

---
## Installation

Notice that, because of ROS requirement, the driver can only be used in linux environnement. 

### Dependencies
* [Boost](https://www.boost.org/) - Useful C++ library
* [Libpcap](https://www.tcpdump.org/) - Used to read pcap files for in order to test the code
* [Gtest](https://github.com/google/googletest) - Used to create the unit tests
* [ixblue_stdbin_decoder](https://github.com/ixblue/ixblue_stdbin_decoder) - iXblue standard bin protocol parsing library

### Build the code

You must first install the ixblue_stdbin_decoder library in your system by following instructions [here](https://github.com/ixblue/ixblue_stdbin_decoder).

If you have not already done so, you can first download the source code into your src directory of your catkin workspace thanks to the command : 

```sh
git clone git@github.com:ixblue/ixblue_ins_stdbin_driver.git
```

Then you can build the code in your catkin workspace : 

```sh
cd ..
catkin_make
```

### Use the code

There are two ways to launch the ROS driver node : 
* Thanks to launchfile : 
```sh
roslaunch ixblue_ins_driver ixblue_ins_driver.launch 
```
* Or directly with the ROS node :                                
              
|       First Terminal         |          Second Terminal       |  
| ---------------------------- | ------------------------------ |
|          `roscore`           | `rosrun ixblue_ins_driver node`|



Depending on, your requirements and your configuration, you can also modify some arguments.
* **udp_port** : your system will receive the data from the INS by this port. 
* **ip** :  the address of the network interface to listen to
* **time_source** : determine the source of the timestamp data. "ins" for ins timestamp. "ros" for ROS timestamp.
* **time_origin** : determine the time origin of the timestamp. "sensor_default" for ins base time. "unix" for UNIX base time (since 1st of january 1970).

**[Back to top](#table-of-contents)**  

---
## Implementation

### iXblue stdbin protocol
   
The protocol iXblue stdbin allows to obtain as much information as possible from inertial system data thanks to its modularity. 
Here is a non exhaustive list of data that are proposed in the protocol : 
* Attitude
* Positions
* Status
* Accelerations
* ... 
This information can be given in vessel or geographic frame, compensated or not from gravity and earth rotation.
You can find the details of this protocol in the interface library document of your product user manual.

### ROS driver

#### Structure 
The iXblue ROS driver is divided into three parts : 
* `ixblue_ins` : ROS metapackage for iXblue INS
* `ixblue_ins_driver` : ROS driver which receive the data and send the messages
* `ixblue_ins_msgs` : iXblue messages definition

#### Output messages

The messages are filled into the `ros_publisher.cpp` file. 

*ROS standard messages :*

* [sensors_msgs/NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)
* [sensors_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)
* [sensors_msgs/TimeReference](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html)

*iXblue message :*

We defined one iXblue INS message whose format is : 

```
Header header # Standard ROS message header

int8 ALT_REF_GEOID=0
int8 ALT_REF_ELLIPSOID=1

# Position 
float64     latitude                             # In degres, positive north, [-90,90]
float64     longitude                            # In degres, increasgin toward east, [0,360]
int8      	altitude_ref                         # 0 : Geoid / 1 : Ellipsoid, see constant above.
float32     altitude                             # In meters, positive up

# Position Deviation
float64[9] position_covariance                   # In square meters, ENU in row-major order
                                                 # Null matrix if unknown

# Attitude
float32 heading                                  # In degres, [0,360]
float32 roll                                     # In degres, positive port up, [-180,180]
float32 pitch                                    # In degres, positive bow down, [-90,90]

# Attitude Deviation
float64[9] attitude_covariance                   # In rad2/sec2
                                                 # Null matrix if unknown

# Speed Vessel Frame
geometry_msgs/Vector3 speed_vessel_frame         # In m/s, x : forward, y : left, z : up 

# Speed Vessel Frame Deviation
float64[9] speed_vessel_frame_covariance         # In m2/s4, ENU in row-major order
                                                 # Null matrix if unknown
  
```

*Contributing :* 

If the above messages do not correspond to your application feel free to 
* add your own messages
* make a request for iXblue to add a message with the missing information, if they are available in the protocol. 

Feel free to open an issue on this github repository if you encounter any problem. 

**[Back to top](#table-of-contents)**

---
## Roadmap 
* Decode INS status : user, algorithm, sensors, temperatures
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


