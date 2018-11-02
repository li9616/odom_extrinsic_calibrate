odom_extrinsic_calibrate
=========
 
Introduction
------------

This C++ library supports the following tasks:

1. Intrinsic calibration of Loomo GX odometer(r_left, r_right).
2. Extrinsic self-calibration of sensor-odometer(3D, delta_x, delta_y, delta_theta).


The workings of the library are described in the this paper:

        Di Cicco, M., Della Corte, B., & Grisetti, G. (2016, May). Unsupervised calibration of wheeled mobile platforms. In Robotics and Automation (ICRA), 2016 IEEE International Conference on (pp. 4328-4334). IEEE.





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


Issues
--------
If meeting some trouble, please contact Pang Fumin, fumin.pang@ninebot.com 

