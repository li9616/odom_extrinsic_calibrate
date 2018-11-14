odom_extrinsic_calibrate
=========
 
Introduction
------------

This C++ library supports the following tasks:

1. Intrinsic calibration of Loomo GX odometer(radius_left, radius_right).
2. Extrinsic self-calibration of sensor-odometer(3D, delta_x, delta_y, delta_theta).


The workings of the library are described in the this paper:

         Brookshire J, Teller S. Automatic calibration of multiple coplanar sensors[J]. Robotics: Science and Systems VII, 2012, 33.
         
        
        Di Cicco, M., Della Corte, B., & Grisetti, G. (2016, May). Unsupervised calibration of wheeled mobile platforms. In Robotics and Automation (ICRA), 2016 IEEE International Conference on (pp. 4328-4334). IEEE.


![image](https://github.com/pangfumin/odom_extrinsic_calibrate/blob/master/sensors.png)

Build Instructions for Ubuntu
-----------------------------

*Required dependencies*
* Eigen3.3 (included)
* Sophus (included)
* ceres-solver

1. Build the code.

        mkdir build
        cd build
        cmake -DCMAKE_BUILD_TYPE=Release ..


Examples
--------

Go to the build folder where the executables corresponding to the examples are located in. To see all allowed options for each executable, use the --help option which shows a description of all available options.

1.  calibration 

        ./build/calib data_path  [calibrate_result_path]

There are some samples in calibrate_dataset folder.



