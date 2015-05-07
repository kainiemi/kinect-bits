Floor determination and removal
============================================

Contributors: Petri Kainiemi, Ilkka Salento

**Floor plane determination**

Kinect offers an API to get floor clip plane. Floor clip plane can be obtained from `IBodyFrame` by calling [`get_FloorClipPlane()`](https://github.com/kainiemi/kinect-bits/blob/master/FloorRemovalSample/DepthBasics.cpp#L264) (Kinect for Windows 2.0 SDK). 
Floor clip plane is a vector containing coefficients A, B, C and D of an estimated floor plane equation Ax + By + Cz +D = 0. 

```C++
Vector4 fcp;
pBodyFrame->get_FloorClipPlane(&fcp);
//A = fcp.x
//B = fcp.y
//C = fcp.z
//D = fcp.w
```  
**Mapping from depth space to camera space**

Floor clip plane is defined in camera space, thus depth frame needs to be mapped to camera space before calculation takes place.
Depth frame can be mapped to camera space using `CoordinateMapper` class. It provides [`MapDepthFrameToCameraSpace()`](https://github.com/kainiemi/kinect-bits/blob/master/FloorRemovalSample/DepthBasics.cpp#L486) method, which maps entire depth frame to corresponding CameraSpacePoints. 
`CameraSpacePoint` is x, y and z coordinates of point in meters. 

```C++
m_pCoordinateMapper->MapDepthFrameToCameraSpace(size, (UINT16*)srcDepthFrame, size, m_pCameraSpacePoints);
```

**Calculating distance to the floor plane**

Now that depth pixel's position in camera space is known, distance to the floor can be calculated using basic [point-plane distance equation](https://github.com/kainiemi/kinect-bits/blob/master/FloorRemovalSample/DepthBasics.cpp#L490).

```C++
CameraSpacePoint s;
s = m_pCameraSpacePoints[i];
float divisor = sqrtf(fcp.x * fcp.x + fcp.y * fcp.y + fcp.z * fcp.z);             
float dist = (fcp.x * s.X + fcp.y * s.Y + fcp.z * s.Z + fcp.w) / divisor;
```

Note that distance is in meters. Floor removal can be done by simply comparing distance to predefined threshold value and discarding depth pixels that are close or opposite side of the floor plane. 
