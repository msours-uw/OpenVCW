## Open Virtual Camera World
Open Virtual Camera World allows any number of "virtual cameras" to be placed in a "virtual world", where each camera has it's own set of intrinsic parameters, as well as an orientation with reference to a single point in the world. A 2D "virtual target" (specified as an image) of some width and height in meters, and with some orientation with reference to the same point in the world, can then be observed by the cameras (projected using linear algebra based on the pin-hole camera model and bilinear interpolation). The result is an image of each cameras perspective of the scene containing the target. 

In addition, shot noise and gaussian noise can optionally be introduced into each cameras perspective. This can be done by initializing the camera model with parameters usually provided by the manufacturer, or by tweaking the models default values until the desired effect is observed.

### Use Cases Overview

The intent is for this library to be used as is, modified, or with components pulled out and used on their own; basically in any way that is helpful. With that being said, below are some of the intended use cases.

#### Validating intrinsic and extrinsic parameter estimation algorithms 
A well known drawback of using real data to validate an intrinsic or extrinsic estimation algorithm is that it is often done without any "ground truth" component. This forces the focus to shift to relative variation, with only a ballpark estimate of accuracy. 

Using simulated perspectives of a target, it can be verified that a particular algorithm arrives at the correct values in somewhat ideal scenarios, and is at least robust to simulated camera noise, and poor contrast environments.

Below is an example, placing four cameras each at different orientations with reference to a single point, observing a charuco board placed at a series of orientations with reference to the same point.
![Alt text](/Vcw.Sandbox/resources/CameraPerspectives.gif?raw=true "Charuco poses over time")

TODO: 
Provide a complete example for validating the "bundle adjustment" algorithm for estimating camera extrinsics, described here http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment

#### Validating Deep Learning models for image processing applications
For image segmentation and object classification type problems, it can be heplful to have an unlimited source of target data when down selecting a particular convolutional network.

#### Simulating Model Specific Camera Noise
Shot Noise (poisson distribution, intensity dependant) and Gaussian Noise (normal distribution, intensity independant) can be simulated in perspectives of a target by initializing the camera model with parameters usually provided by the manufacturer:
* Bit Depth
* Photons Per Pixel
* Quantum Efficienct
* Temporal Dark Noise
* Photon Sensitivity
* Intensity Baseline.


Below is an example, adding quite an exaggerated amount of shot noise and gaussian noise to a series of camera perspectives.
![Alt text](/Vcw.Sandbox/resources/WhistlersMother.gif?raw=true "Whistlers Mother")

### Building and Installing
Requirements:
* OpenCV (tested with 3.4.6)
* Full C++17 support
#### Linux
GCC is the only compiler that has been tested on Linux. GCC 10 is the first compiler to fully support C++17, which should already exist on newer Linux distributions, or at least be easily obtained. With older distributions such as Ubuntu 16.04, it may be neccesary to build GCC 10 from source: https://solarianprogrammer.com/2016/10/07/building-gcc-ubuntu-linux/

After cloning the repo,
* `mkdir build`
* `cd build`
* `cmake .. -D CMAKE_CXX_COMPILER=g++-10 -D CMAKE_C_COMPILER=gcc-10`
* `cmake --build . --config release --target install`

#### Windows
Visual Studio 15 (2017) and newer should fully support C++17. Building with Windows 64 bit:

After cloning the repo,
* `mkdir build`
* `cd build`
* `cmake .. -G "Visual Studio 15 2017 Win64" -D OpenCV_DIR=<Path to opencv build dir>`
* `cmake --build . --config release --target install`
