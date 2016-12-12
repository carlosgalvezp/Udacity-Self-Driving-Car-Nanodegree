Project 3 - Behavioral Cloning
==============================

Introduction
------------
This project consists on applying Deep Learning to the task of end-to-end
learning for a self-driving car: given an input image from a camera, the
network should predict the required steering angle to properly drive.

Input Data
----------

Output Data
-----------

Model Architecture
------------------
The approach taken in this project is similar to the one taken by Nvidia
in their [paper](http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf).
It is a relatively simple model (only 9 layers) with a moderate number of
parameters (around 250.000), so we can expect reasonable training times.
It was also proven successful, so why not use it?

The use of pre-trained networks like AlexNet or VGG (i.e. transfer learning)
was not considered from the beginning, since they were training for different
purposes. Re-training these networks from scratch would take a much larger
effort, and we believe that to have a short loop between training and testing
is crucial. This was recommended also by many other students in Confluence.

The implemented network consists of the following layers:

-**Convolutional 1**. 24 filters of size 5x5x3 (since the input has 3 channels).
The filter is applied with strides of (2, 2) instead of using MaxPooling.
This can be done because the input image is relatively high resolution.
The used padding was 'valid', as proposed by Nvidia.

-**Convolutional 2**. 36 filters of size 5x5x24. Strides of (2, 2).
-**Convolutional 3**. 48 filters of size 5x5x36. Strides of (2, 2).
-**Convolutional 4**. 64 filters of size 3x3x48. Strides of (1, 1). As can be
observed, the filter size and strides are now reduced, given that the input
images are much smaller.
-**Convolutional 5**. 64 filters of size 3x3x64. Strides of (1, 1).

-**Dropout** (p = 0.5 during training) to mitigate the effects of overfitting.
-**Flatten**.

-**Fully Connected 1**, with 100 neurons + Dropout(0.5)
-**Fully Connected 2**, with 50 neurons + Dropout(0.5)
-**Fully Connected 3**, with 10 neurons + Dropout(0.5)
-**Fully Connected 4**, with 1 neuron, being the output.

All the layers, except for the output layer, have a ReLU activation function.
In addition, all the layers are initialized with the 'normal' function,
available in Keras, which by default uses a standard deviation of 0.05,
similar to what was done in the TensorFlow labs and exercises. A truncated
normal function would have been desirable, but could not be found in Keras.

The main improvement over Nvidia's implementation is to add the Dropout
layers in order to fight against overfitting.

In total the network has 252219 parameters, including weights and biases.

Data Preprocessing
------------------

Dataset Extension
-----------------


Data Collection
---------------


Training Strategy
-----------------


Simulation Results
------------------
