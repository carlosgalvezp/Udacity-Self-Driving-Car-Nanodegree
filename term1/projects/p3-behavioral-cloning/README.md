Project 3 - Behavioral Cloning
==============================

Introduction
------------
This project consists on applying Deep Learning to the task of end-to-end
learning for a self-driving car: given an input image from a camera, the
network should predict the required steering angle to properly drive.

Input Data
----------
The input data comes from a set of forward-looking cameras installed inside the
vehicle: one in the center, and two to the sides (left, right).

The simulator only uses the center camera for predicting the steering angle,
so we only use this one for training as well. However the results could be
improved by using the other two cameras, as reported by other students.

The images from the cameras are RGB, with resolution (160, 320)

In addition, we also have access to other vehicle signals, like throttle
and break commands, but are not relevant for the task of steering angle
prediction from images.

Output Data
-----------
The output data is simply the steering angle in degrees. The log files have
normalized this value to a range between [-1, 1] to better suit the
Neural Network framework. This corresponds to roughly [-25º, 25º].


Model Architecture
------------------
The approach taken in this project is similar to the one taken by Nvidia
in their [paper](http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf).
It is a relatively simple model (only 9 layers) with a moderate number of
parameters (around 250.000), so we can expect reasonable training times.
Clearly the desired approach is a combination of Convolutional layers
followed by Fully-Connected layers, since the input data are images.
This time, the architecture is applied to a regression problem (predicting
steering angle) instead of classification, so no activation function
or softmax must be applied at the last layer, which will have only one neuron.

The use of pre-trained networks like AlexNet or VGG (i.e. transfer learning)
was not considered from the beginning, since they were training for different
purposes. Re-training these networks from scratch would take a much larger
effort, and we believe that to have a short loop between training and testing
is crucial. This was recommended also by many other students in Confluence.

The implemented network consists of the following layers:

-**Input**. Image of size (66, 200, 3).
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
-**Flatten**. The input for the next layer will have size 1152.

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

Conclusion
----------
