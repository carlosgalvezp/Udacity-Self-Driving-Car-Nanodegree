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

- **Input**. Image of size (66, 200, 3).
- **Convolutional 1**. 24 filters of size 5x5x3 (since the input has 3 channels).
The filter is applied with strides of (2, 2) instead of using MaxPooling.
This can be done because the input image is relatively high resolution.
The used padding was 'valid', as proposed by Nvidia.

- **Convolutional 2**. 36 filters of size 5x5x24. Strides of (2, 2).
- **Convolutional 3**. 48 filters of size 5x5x36. Strides of (2, 2).
- **Convolutional 4**. 64 filters of size 3x3x48. Strides of (1, 1). As can be
observed, the filter size and strides are now reduced, given that the input
images are much smaller.
- **Convolutional 5**. 64 filters of size 3x3x64. Strides of (1, 1).

- **Dropout** (p = 0.5 during training) to mitigate the effects of overfitting.
- **Flatten**. The input for the next layer will have size 1152.

- **Fully Connected 1**, with 100 neurons + Dropout(0.5).
- **Fully Connected 2**, with 50 neurons + Dropout(0.5).
- **Fully Connected 3**, with 10 neurons + Dropout(0.5).
- **Fully Connected 4**, with 1 neuron, being the output.

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
Before sending the input images to the neural network, we perform the following
preprocessing steps:

1. Image resize to have a width of 200, keeping the aspect ratio.
2. Image crop to have a height of 66. We remove the pixels from the top of the
   the image, since they belong to the sky, which contains no relevant information.
3. Conversion from RGB to YUV, in order to make it more robust especially
   to different color and illumination conditions.
4. Normalization from [0, 255] to [-0.5, 0.5] to make the training process
   more stable.

This pipeline is implemented in a separate file, `preprocess_input.py`,
so it can be use both by `model.py` and `drive.py`.

Dataset Extension
-----------------
The dataset is extended in order to reduce the time for data collection,
improving the overall collect data-train-test loop.

In particular, we **flip every image horizontally**, and assign the
opposite steering angle. This solution is very easy to implement and
doubles the dataset size. Besides, we don't need to drive in opposite direction
when gathering training data.


Data Collection
---------------
### Environment
Thanks to the recommendations from the Slack community and the Confluence
forums, we performed data collection using:

- The 50Hz version of the simulator.
- A gamepad with analog sticks for steering.

The main reason is that otherwise (using the 10Hz version and keyboard input)
most of the dataset contains images associated with a input command of 0º,
and a few images with a high steering angle, since the keyboard produces
a binary output. A lot of work in filtering the keyboard signal would
have to be done to smooth out the signal.

### Strategy
We recorded data in the following way, keeping a constant speed of 30 mph:

- **Normal driving**, with the vehicle kept in the center of the road.
Approximately 2 laps of driving.

- **Recovery**. This was the crucial part to manage to get the car driving
the whole lap. First, we drove toward the left or right edge of the road,
without recording. Then we turned on recording, and used the joystick
to steer the vehicle back on track. This was performed at different
distances from the center of the lane. We took 2 laps of recording
the vehicle recovering from left to center, and another 2 laps of
recovery from right to center.

It was not necessary to drive in the opposite direction, since we extend
the dataset by flipping the image, as mentioned before.

Data Generator
--------------
The amount of collected data is very large: thousands of medium-resolution
images. During training, we realized it's impossible to read them all from
the hard disk and keep them in RAM, since it was just too much memory.

The solution was found thanks to Slack and Confluence forums: use a
**Python generator**, which reads a batch at a time and performs preprocessing
and dataset extension on-the-fly. The main advantage is that it can be
applied to any input dataset, no matter how big it is.

Dataset Splitting
-----------------
To properly train and evaluate the performance of the network, we split
the training dataset into the following sets:

- Training set: 80% of the data.
- Validation set: 10% of the data.
- Test set: 10% of the data.

Training Strategy
-----------------
The training process was performed using the following configuration:

- **Optimization parameter**: Mean Square Error (mse), since this is a regression
problem.

- **Optimizer**: Adam, given the great performance on the Traffic Signs Lab.
We use a learning of 0.0001 (1/10th of the default) for a more stable
learning.

- **Metrics**: none, just the loss. We observe that the `accuracy` metric
was quite useless (stayed at around all the time 33%), since it's more
relevant in classification problems. Here the best available indicator of the
performance of the network is the train/validation loss. 
However we realized soon that to really evaluate the performance we must
run the model on the simulator, since the loss is not 100% reliable either.

- **Batch size**: 128, to fit in GPU memory.
- **Number of epochs**: XXXX

The function `fit_generator` was used, in order to take advantage of the
Python generator and be able to process the complete dataset in every epoch.
The train and validation datasets are always shuffled at the
beginning of every epoch.


Simulation Results
------------------

Conclusion
----------
