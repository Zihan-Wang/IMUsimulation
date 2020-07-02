# IMUsimulation
+ This is a sim platform based on OpenVINS. For more information, please see https://github.com/rpng/open_vins.

## Dependency
- openCV

You should follow:

    git clone --branch 3.4.6 https://github.com/opencv/opencv/
    git clone --branch 3.4.6 https://github.com/opencv/opencv_contrib/
    mkdir opencv/build/
    cd opencv/build/
    cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
    make -j8
    sudo make install
    
## Compilation

The package can be built with Catkin. The build has been test on Ubuntu 18.04, Melodic version

## Catkin Build Instructions

    mkdir -p ~/ws/catkin_imu/src/
    cd ~/ws/catkin_imu/src/
    git clone https://github.com/Zihan-Wang/IMUsimulation.git
    cd ..
    catkin build

## Launch Simulation

    roslaunch ov_msckf simulation.launch

## Sample Dataset
https://github.com/Zihan-Wang/IMUsimulation/tree/master/ov_data/sim


## Custmize Launch File
    <!-- set groundtruth dataset file-->
    <arg name="dataset"    default="udel_gore.txt" /> <!-- euroc_V1_01_easy, udel_gore, tum_corridor1_512_16_okvis, udel_arl -->
    
    <!-- sensor noise values / update -->
    <arg name="up_msckf_sigma_px"              default="1" />
    <arg name="up_msckf_chi2_multipler"        default="1" />
    <arg name="up_slam_sigma_px"               default="1" />
    <arg name="up_slam_chi2_multipler"         default="1" />
    <arg name="up_aruco_sigma_px"              default="1" />
    <arg name="up_aruco_chi2_multipler"        default="1" />
    <arg name="gyroscope_noise_density"        default="1.6968e-04" />
    <arg name="gyroscope_random_walk"          default="1.9393e-05" />
    <arg name="accelerometer_noise_density"    default="2.0000e-3" />
