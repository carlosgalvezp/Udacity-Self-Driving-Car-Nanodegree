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

Next, we performed some **preprocessing** step, consisting on color space conversion from
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

TODO

<mark>Show some examples of test images to demonstrate how your pipeline is working. What did you do to try to minimize false positives and reliably detect cars?</mark>

TODO

---
### Video Implementation
<mark>Provide a link to your final video output. Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)</mark>

TODO

<mark>Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.</mark>

TODO

---
### Discussion
<mark>Briefly discuss any problems / issues you faced in your implementation of this project. Where will your pipeline likely fail? What could you do to make it more robust?</mark>

TODO
