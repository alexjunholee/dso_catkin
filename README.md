# ROS catkin Wrapper around DSO: Direct Sparse Odometry

For more information see
[https://vision.in.tum.de/dso](https://vision.in.tum.de/dso)

This is meant as simple, minimal example of how to integrate DSO from a different project, and run it on real-time input data.
It does not provide a full ROS interface (no reconfigure / pointcloud output / pose output).
To access computed information in real-time, I recommend to implement your own Output3DWrapper; see the DSO code.


### Related Papers

* **Direct Sparse Odometry**, *J. Engel, V. Koltun, D. Cremers*, In arXiv:1607.02565, 2016

* **A Photometrically Calibrated Benchmark For Monocular Visual Odometry**, *J. Engel, V. Usenko, D. Cremers*, In arXiv:1607.02555, 2016



# 1. Installation

1. run 

		catkin_make
	

# 2. Usage
change the project folder name into dso_ros.
modify the bagfilename from src/main.cpp. you can modify this to roslaunch later.


		rosrun dso_ros dso_ros


## 3. Accessing Data.
see the DSO Readme. As of now, there is no default ROS-based `Output3DWrapper` - you will have to write your own.




# 4 License
This ROS wrapper around DSO is licensed under the GNU General Public License
Version 3 (GPLv3).
For commercial purposes, we also offer a professional version, see
[http://vision.in.tum.de/dso](http://vision.in.tum.de/dso) for details.
