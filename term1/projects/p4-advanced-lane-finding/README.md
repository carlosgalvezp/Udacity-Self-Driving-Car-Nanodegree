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

<img src="./res/persp_transform_original.jpg" height="200"/>
<img src="./res/persp_transform_warped.jpg" height="200"/>

## Line search

<img src="./res/before_line_search.jpg" height="200"/>
<img src="./res/line_search_initial_point.jpg" height="200"/>

## Line tracking

## Line fitting

<img src="./res/line_fit_left.jpg" height="200"/>
<img src="./res/line_fit_right.jpg" height="200"/>

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
