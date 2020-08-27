^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ixblue_ins_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2020-08-27)
------------------
* Prepare ROS release
* Add unit test on ROSPublisher::toNavSatFix()
* Prepare files catkin for release
* Switch stdbin data received log to debug level
* Default packets_replayer port is now the same than driver
  And add missing newline at the end of error message
* ixblue_stdbin_decoder can now be a package within the ROS workspace and
  not only a system library anymore
* Apply ixblue_stdbin_decoder changes on namespace and include dir name
* Build packets_replayer only if PCAP dev package is found
* Initial Commit.
* Contributors: Adrien BARRAL, Romain Reignier
