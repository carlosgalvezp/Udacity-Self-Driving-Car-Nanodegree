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

When we receive the first video frame, we have no information about the lane
lines in the image. Therefore we must perform a search without prior assumptions.
We will work with the birds-eye view image (warped image), passed through the
`final_mask()` function, as explained before.

The implemented approach is as follows:

1. Discover the starting point of the line, in the bottom of the image.
2. Follow the line all the way up to the top of the image, using a sliding
window technique.

### Starting point

To search for the starting point of the lines, we compute the **histogram**
over the number of pixels for each `x` position in the image. To simplify
the computations and make it more robust, we only perform this operation
in the bottom half of the image.

The image we start with is shown in the figure `before_line_search.jpg`. It can
be seen it's a binary image (since it went through the mask) and it's warped
to be a birds-eye view.

<img src="./res/before_line_search.jpg" height="200"/>

The histogram over non-zero pixels values is shown in
`line_search_initial_point.jpg`:

<img src="./res/line_search_initial_point.jpg" height="200"/>

It can clearly be seen that there are 2 main peaks, which correspond to the starting
position of the lines.

### Sliding window

We now know where to start searching. The next step is to place a box around
this starting point, extract the non-zero pixels inside it, and then move it
upwards following the line, all the way up to the top of the image.

The sliding window is moved as follows:

 - It moves the amount `size_y` in the vertical direction, where `size_y` is the
size of the window.
 - If the window contained pixels, it moves towards the mean `x` position of those
pixes. Otherwise it moves the same amount as in the previous step, assuming
that the line has the same curvature in the image.

An example is shown for both the left and right images, respectively, in
`sliding_window_search_left.jpg` and `sliding_window_search_right.jpg`:

<img src="./res/sliding_window_search_left.jpg" height="200"/>
<img src="./res/sliding_window_search_right.jpg" height="200"/>

The final result is that each line contains a list of the `x` and `y` coordinates
of the pixels that it contains.

## Line fitting

Once we have the pixels for each line, we can perform **line fitting**, where
we simply fit a second-order polynomial to the stored `x` and `y` datapoints.
This is performed using the function `np.polyfit(y, x, 2)`.

NOTE: we fit the polynomial using `y` as `x` and viceversa for a more stable
result (since the lines are almost vertical) and since it will be useful later
on for drawing purposes.

Finally, we plot the polynomial on top of the original image by just computing
the corresponding `x` value for every `y` in the image. We use the function
`cv2.line` to plot the lines, as seen in pictures `line_fit_left.jpg` and
`line_fit_right.jpg`:

<img src="./res/line_fit_left.jpg" height="200"/>
<img src="./res/line_fit_right.jpg" height="200"/>


## Line tracking

Once we have a first estimate of the line, we don't need to search in the whole
image. Instead, we create specific search regions for each line, assuming
that they will remain more or less similar from one frame to the next one.

We do this by simply adding an offset to the last coefficient of the polynomial,
which controls the `x` position of the line in the image. This offset is
applied to the left and right of the line, as can be shown in the images
`tracking_search_left.jpg` and `tracking_search_right.jpg`:

<img src="./res/tracking_search_left.jpg" height="200"/>
<img src="./res/tracking_search_right.jpg" height="200"/>

Then we simply extract the non-zero pixels in these regions and fit the line
polynomial as before. The process is faster and simpler since we only had to search
in a small region of the image.

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
