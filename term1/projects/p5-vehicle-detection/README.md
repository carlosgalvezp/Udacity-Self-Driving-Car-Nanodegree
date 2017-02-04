# Vehicle Detection and Tracking
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

##Overview

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

## Rubric Points
In this write-up, I will address the [rubric](https://review.udacity.com/#!/rubrics/513/view) points. Each question will be <mark>highlighted</mark> for the reader's convenience.
All the references to code and cells are relative to the iPython Notebook
`p5-vehicle-detection.ipynb`. In addition, there will be references to images, which
will be stored in the `output_images` folder.

---

### Writeup / README
<mark>Provide a Writeup / README that includes all the rubric points and how you addressed each one.<mark>

This `README.md` file is the required write-up document that explains the work for this project.

---

### Histogram of Oriented Gradients (HOG)
<mark>Explain how (and identify where in your code) you extracted HOG features from the training images. Explain how you settled on your final choice of HOG parameters.</mark>

The first step in this project was to take a look at the input data. We used the `vehicle`
and `non-vehicle` datasets provided by Udacity, containing 8792 and 9666 64x64 RGB images,
respectively. One example of these images can be seen in `vehicle_non_vehicle.jpg`:

<img src="./output_images/vehicle_non_vehicle.jpg" height="400"/>

Next, we performed a **preprocessing** step, consisting on color space conversion from
RGB to YCrCb, given the better results shown in the literature with the latter color space.
This operation is performed in `cell #10`, using the function `cv2.cvtColor`.

Finally, we extract HOG features from the YCrCb image in `cell #11`, using the
`hog` function from the `skimage.feature` package. The input image was first resized
to 64x64 pixels. We then extract HOG features for each of the 3 channels, stacking
them together to create a single vector.

We used the following HOG parameters:

 - Pixels per cell: 8
 - Cells per block: 2
 - Number of orientation bins: 9

The reason for choosing these parameters is XXXXXXXXXXXXX

An example result of computing HOG features to an image of class `vehicle` is shown
in `hog_img_vehicle.jpg`:

<img src="./output_images/hog_img_vehicle.jpg" height="400"/>

Similarly, for `non-vehicle` we have `hog_img_non_vehicle.jpg`:

<img src="./output_images/hog_img_non_vehicle.jpg" height="400"/>

It can be observed that in the `vehicle` class there is a more defined structure,
with multiple horizontal and vertical lines all around the image, mostly for the
`Y` channel both quite a few for the `Cr` and `Cb` channels as well. The gradients
for the `non-vehicle` class seem more unstructured, which will allow us to effectively
classify vehicles in images.

The output of the HOG features is a 5292-dimensional vector, which we then
use as input to the classifier.

<mark>Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).</mark>

We decided to use an SVM (Support Vector Machine) as a classifier for this project,
given it's powerfulness with non-linearly separable datasets and easy to use API
from the `sklearn` kit.

Before using the classifier, we perform **feature normalization** on the HOG
features that we extracted previously. This is implemented in the function
`normalize_features`, `cell #14`. To this extent, we used the `StandardScaler`
object , part of the `sklearn.preprocessing` package, as suggested by Udacity.
According to the documentation, the normalization consists on removing the mean
and scaling to unit variance, which is desirable to make the training procedure
more stable.

Next, we **splitted** the data into training and validation sets, with a ratio
of 0.2 for the validation data, using the function `train_test_split`, in
`cell #15`. This function already takes care of shuffling the data as well.

The classifier is implemented and trained in `cell #16-17`. The API
is very simple to use: simply the `fit` function will train the classifier
given the training data. We started with the `LinearSVC` classifier, and obtained
98.6% validation accuracy. Then we moved into the non-linear `SVC`, with default
`rbf` kernel, which made it more powerful obtaining 99.1% validation accuracy.

Finally, we verify the classifier on some test image using the function
`classify_img`, in `cell 21`. The result can be observed in `svm_test.jpg`:

<img src="./output_images/svm_test.jpg" height="400"/>

It can be observed that the classifier is able to correctly classify the images.

---
### Sliding Window Search
<mark>Describe how (and identify where in your code) you implemented a sliding window search. How did you decide what scales to search and how much to overlap windows?</mark>

In order to detect vehicles in the complete image, we implement a sliding window
approach. First, we implement a `SearchWindow` class, (see `cell #30`), that
helps us extract the contents of an image, the corresponding HOG features as
well as determining whether there's a vehicle or not.

Second, we implement a function that takes an image and returns a list of
windows in which we should search for objects: function `get_search_windows`,
in `cell #33`, with the following properties:

 - Window size: 64x64, to match the HOG implementation and training data.
 - Overlap: 0.75.
 - Region of interest: bottom half of the image, so we don't search above
 the horizon.


**Scaling**. In order to search on different scales, we use the same function
`get_search_windows`, but we **resize the image first**. In particular, we
search at scales 1.0, 0.75 and 0.5, which is equivalent to window sizes
of size 64x64, 96x96 and 128x128, respectively. We also tried windows
of size 256x256 but turned out to be too big and returned quite many false positives.

**Motivation for parameters**

The choice of overlapping and number of scales was always a trade-off between
number of windows and accuracy. The larger number of windows, the more computational
time. However a small overlapping, like 0.5, would case windows to never have a vehicle
somewhat centered, so the number of false negatives would increase.

The final choice is essentially the result of trial and error, using reasonable
values to obtain approximately 2000 search windows.

<mark>Show some examples of test images to demonstrate how your pipeline is working. How did you optimize the performance of your classifier?</mark>

The complete pipeline is applied to the given test images, `test1.jpg` through
`test6.jpg`, as can be seen in the folder `output_images`, shown below:

<img src="./output_images/test1.jpg" height="400"/>
<img src="./output_images/test2.jpg" height="400"/>
<img src="./output_images/test3.jpg" height="400"/>
<img src="./output_images/test4.jpg" height="400"/>
<img src="./output_images/test5.jpg" height="400"/>
<img src="./output_images/test6.jpg" height="400"/>


It can be seen that the vehicles are detected reliably with many windows at different
scales, which will provide good results in the video afterwards. In addition,
we observe that the number of false positives is minimum, only one in the image
`test2.jpg`, which contains no cars. There are no more false positives in the
rest of test images.

**Optimizing the performance of the classifier** was a really tough work. The following
was considered during the process of tuning the classifier:

 - **Choice of the feature vector**. This is very related to the question regarding
 the parameters of the HOG classifier. We noticed that having a smaller feature vector
 improved both the robustness of the classifier and also the computational speed.
 For this reason, the feature vector is 972-dimensional instead of 5292-dimensional
 (16 pixels per cell vs 8 pixels per cell).

 - **Thresholding on the decision function**. This method was very useful when we first
 tried the `LinearSVM` classifier. Instead of using the `predict` function of the classifier,
 we used the `decision_function` function, which provides the signed distance
 to the hyperplane for each prediction. Then we would do classification ourselves
 by imposing some threshold on the score returned by this function. However, we noticed that:

   1. The scores vary quite a lot, so it's really hard to know in which region we should
   tune the threshold, and if it would be valid for other test images. We consider creating
   a ROC curve to find the optimal threshold, but had to give up on it given
   the tight time constraints for this project.

   2. We could reduce the number of false positives by increasing the threshold; however it
   also reduced the number of true positives. Sometimes we would end up still having
   false positives while all the true positives were removed.


 For this reason, we quickly discarded the `LinearSVM` option, despite it's computational
 efficiency.

 - **Use data augmentation**. We implemented data augmentation in `cell #9`, by applying
 random shifts to the images. This improved the cross-validation accuracy and mostly
 helped detecting vehicles more reliably, while removing some of the false positives
 we used to have.

---
### Video Implementation
<mark>Provide a link to your final video output. Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)</mark>

The video output can be found [here](./output_images/project_video.mp4).

It can be observed that vehicles are reliably detected throughout the entire video,
with very few and with low duration false positives.

<mark>Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.</mark>

TODO

---
### Discussion
<mark>Briefly discuss any problems / issues you faced in your implementation of this project. Where will your pipeline likely fail? What could you do to make it more robust?</mark>

This project was one of the toughest CV ones in this course, mainly due to the following
issues:

 - **Computational time**, main limiting factor in this project. It was very hard
 to trade off frame rate and accuracy. This mainly due to the **sliding window**
 approach: we have to search in too many images, which raises the computational
 time. A much better approach for this project would have been Deep Learning,
 with networks like YOLO (You Only Look Once) or SSD (Single Shot Detector).

 - **Parameter tuning**. As all traditional Computer Vision approaches, there were
 just too many paramters to tune: HOG parameters, number of search windows, overlap,
 number of scales, heatmap update and cooldown, etc.

 - **Time constraints**. We only had 2 weeks to work on this project, which in my
 opinion was way too little time. I would have liked to have more time to try out
 Deep Learning approaches to this problem.


Regarding the pipeline, we can see some points where it could be improved:

 - **Framerate**. This pipeline can obviously not run in real time, so it's a bit
 weak point that should be improved. In my opinion, sliding window approaches
 cannot be applied to real-time systems because they just take too much time,
 no matter which classifier is used. As said before, a YOLO or SSD network
 would provide a much better performance.


