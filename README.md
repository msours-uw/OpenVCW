# OpenVCW - Open Source Library for Simulating Camera Perspectives of Real World Targets
Open Virtual Camera World allows any number of "virtual cameras" to be placed in a "virtual world", where each camera has it's own set of intrinsic parameters, as well as an orientation with reference to a single point in the world. A 2D "virtual target" (specified as an image) of some width and height in meters, and with some orientation with reference to the same point in the world, can then be observed by the cameras (projected using linear algebra based on the pin-hole camera model and bilinear interpolation). The result is an image of each cameras perspective of the scene containing the target. 

In addition, shot noise and gaussian noise can optionally be introduced into each cameras perspective. This can be done by initializing the camera model with parameters usually provided by the manufacturer, or by tweaking the models default values until the desired effect is observed.

# Use Cases Overview

The intent is for this library to be used as is, modified, or with components pulled out and used on their own; basically in any way that is helpful. I do request however that if you pull out a component, to please consider including a link to the original source.

With that being said, below are some of the use cases that I pictured.

# Validating intrinsic and extrinsic parameter estimation algorithms 
A well known drawback of using real data to validate an intrinsic or extrinsic estimation algorithm is that it is often done without any "ground truth" component. This forces the focus to shift to relative variation, with only a ballpark estimate of accuracy. 

Using simulated perspectives of a target, it can be verified that a particular algorithm arrives at the correct values in somewhat ideal scenarios, and is at least robust to simulated camera noise, and poor contrast environments.

Below is an example, placing four cameras each at different orientations with reference to a single point, observing a charuco board placed at a series of orientations with reference to the same point.
![Alt text](/Vcw.Sandbox/resources/CameraPerspectives.gif?raw=true "Charuco poses over time")

TODO: 
Provide a complete example for validating the "bundle adjustment" algorithm for estimating camera extrinsics, described here http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment

# Validating Deep Learning models for image processing applications
For image segmentation and object classification type problems, it can be heplful to have an unlimited source of target data when down selecting a particular convolutional network.

# Simulating Model Specific Camera Noise
shot noise (poisson distribution, intensity dependant) and gaussian noise (normal distribution, intensity independant) can be simulated in perspectives of a target by nitializing the camera model with parameters usually provided by the manufacturer:

* Bit Depth
* Photons Per Pixel
* Quantum Efficienct
* Temporal Dark Noise
* Photon Sensitivity
* Intensity Baseline.


Below is an example, adding quite an exaggerated amount of shot noise and gaussian noise to a series of camera perspectives.
![Alt text](/Vcw.Sandbox/resources/WhistlersMother.gif?raw=true "Whistlers Mother")

TODO: Add build instructions section
