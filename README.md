# IMUsimulation
+This is a wrapped sim platform based on openvins. For more information, please see https://github.com/rpng/open_vins.

## dependency
-openCV

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

