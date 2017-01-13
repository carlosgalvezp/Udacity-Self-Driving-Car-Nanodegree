# Advanced Lane Finding

<img src="./output_images/test1.jpg" height="400"/>

---

## General overview

## Camera calibration
The process of camera calibration consists on extracting the distortion
coefficients of the camera from a series of calibration images, that contain
a well-known pattern, like a checkerboard.

The first step is to extract the pixel coordinates of the corners of the
squares of the checkerboard, using the OpenCV function `cv2.findChessboardCorners`,
as shown in `checkerboard_corners.jpg`:

<img src="./res/checkerboard_corners.jpg" height="300"/>

The procedure is applied to multiple images taken from different perspectives.

Once the corners have been collected, we pass them to the OpenCV function
`cv2.calibrateCamera`, which returns the distorsion coefficients necessary
to undistort an image:

```python
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, gray.shape[::-1],None,None)
```


## Image undistortion
Once we have obtained the distortion coefficients, we can undistort any image
using the OpenCV function `cv2.undistort`. As an example, we apply it
to the image `img_distorted.jpg`, obtaining `img_undistorted.jpg`:

<img src="./res/img_distorted.jpg" height="200"/>
<img src="./res/img_undistorted.jpg" height="200"/>


## Masking

### Region of Interest

### Color masks

### Gradient masks

### Final mask

<img src="./res/final_mask.jpg" height="200"/>

## Perspective transformation

One of the goals of the project is to compute the road curvature and
vehicle position within the lane. In order to do that, we must obtain
a _birds-eye view_ of the image, which allows us to obtain real measurements,
not affected by the perspective of the camera.

This operation is called _perspective transformation_. We use the OpenCV functions
`cv2.getPerspectiveTransform` and `cv2.warpPerspective` to this extent.
We need to manually select 4 points in the source image that lie on the same
plane. The chosen points can be seen in the red rectangle in
`persp_transform_original.jpg`:

<img src="./res/persp_transform_original.jpg" height="200"/>

These 4 points are mapped into a real rectangle,
with parallel lines, as shown in `persp_transform_warped.jpg`:

<img src="./res/persp_transform_warped.jpg" height="200"/>

As can be seen, the road lines don't appear parallel in the original image
due to the camera perspective, but after the perspective transform they do
appear parallel, since we chose the 4 points carefully to do so.

After this, we can start searching for lines and computing the road curvature.

## Line search



### Starting point
<img src="./res/before_line_search.jpg" height="200"/>
<img src="./res/line_search_initial_point.jpg" height="200"/>

### Sliding window

<img src="./res/sliding_window_search_left.jpg" height="200"/>
<img src="./res/sliding_window_search_right.jpg" height="200"/>

## Line fitting

<img src="./res/line_fit_left.jpg" height="200"/>
<img src="./res/line_fit_right.jpg" height="200"/>

## Line tracking
<img src="./res/tracking_search_left.jpg" height="200"/>
<img src="./res/tracking_search_right.jpg" height="200"/>


## Road curvature estimation

## Vehicle position estimation

## Visualization

<img src="./res/free_space.jpg" height="200"/>

<img src="./res/main_visualization.jpg" height="200"/>


<img src="./output_images/test1.jpg" height="400"/>

## Complete pipeline

## Results

### Test images

<img src="./output_images/test1.jpg" height="400"/>

<img src="./output_images/test2.jpg" height="400"/>

<img src="./output_images/test3.jpg" height="400"/>

<img src="./output_images/test4.jpg" height="400"/>

<img src="./output_images/test5.jpg" height="400"/>

<img src="./output_images/test6.jpg" height="400"/>

### Test video

### Challenge videos

## Conclusions
