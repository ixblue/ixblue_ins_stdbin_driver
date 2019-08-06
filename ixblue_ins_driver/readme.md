# Tools
This tools is only here to help development of the INS driver.
The packets_replayer allow to send on an UDP Socket the content of a pcap file. This file can be used to replay wirechark capture. In a ros environement this is useless because rosbag exists. But to develop the driver this tools is usefull.
We can launch the packet replayer with the following command line : 
````
./packets_replayer --ip 127.0.0.1 --port 8000 --period 10 --repeat --file <my pcap file>
````
All parameters are optional except the pcap file.
It is assumed that only the INS frame in UDP protocol is present in the PCAP file.

