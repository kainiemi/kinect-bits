Simple background removal and ROI estimation 
============================================

Contributors: Petri Kainiemi, Ilkka Salento

While developing with Kinect and especially when Kinect's low-level depth stream
is involved it is often required to get environment seen in depth frame layered
using depth values and unneeded data removed. That unneeded data is often considered 
as a background data. In addition bounds of the object which later enters the scene may
need to be identified. 

This repository represents a simple method to remove the background and find the bounds 
of the object from the depth frame. Visual Studio example project is included with full 
source code (modified copy of Kinect SDKs DepthSample) which includes also the source code of 
the algorithm. Algorithm implementation can be found from DepthBasics.cpp starting on line 424.

* [DepthBasics.cpp](https://github.com/kainiemi/kinect-bits/blob/master/BackgroundRemovalSample/DepthBasics.cpp#L424)

**Background removal algorithm**

Background removal algorithm calculates average depth values of first 10 sequential depth 
frames of the environment without the object that needs to be separated from the background. 
Some of the pixels in the depth frames may have zero value due to how distance is calculated with 
depth cameras. As the averaged depth frame will be subtracted from input depth frame all the pixel 
values needs have meaningful values. Zero values are fixed by filling them with row’s average value. 
This requires that Kinect camera is aligned horizontally and perpendicular to wall (if any), 
otherwise depth values on the same pixel row could have totally different values.  
 
Resulting averaged depth frame is subtracted pixel by pixel from the input depth frame and if 
difference is greater than threshold value pixel is considered as foreground pixel. Threshold value
needs to be specifically set case by case depending on object’s distance from background and noise. 
 
**Region Of Interest (ROI) estimation**
 
Boundaries of the object can be identified by finding out the minimum and maximum coordinates from 
the foreground pixels. As the object enters the scene it may change the values of some pixels which 
will turn to stray pixels in the image and therefore needs to be ignored. To ignore these stray pixels, 
image is traversed through by 8x8 kernel and count of the foreground pixels is calculated in each kernel. 
Amount of pixels in each kernel determines whether kernel is on top of the object and is considered as 
foreground kernel. Finally ROI is determined by calculating minimum and maximum top-left and bottom-right
coordinates from foreground kernels. 
