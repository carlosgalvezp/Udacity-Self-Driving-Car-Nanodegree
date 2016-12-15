import os
import cv2
import math
import numpy as np
import pandas as pd
import csv
import time
import argparse
import json
from sklearn.model_selection import train_test_split
from keras.models import Sequential
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers import Dense, Dropout, Flatten, Lambda, Activation, ELU
from keras.optimizers import Adam

import preprocess_input

# Angle offset for the left and right cameras. It's and estimation of the
# additional steering angle (normalized to [-1,1]) that we would have to steer
# if the center camera was in the position of the left or right one
ANGLE_OFFSET = 0.25

# Angle offsets applied to center, left and right image
ANGLE_OFFSETS = [0.0, ANGLE_OFFSET, -ANGLE_OFFSET]

# Batch size
BATCH_SIZE = 64

# Additional images are generated randomly. This number controls how much
# data is generated. len(X_train) = EXTENDED_DATA_FACTOR * len(X_train_initial)
EXTENDED_DATA_FACTOR = 4

def random_horizontal_flip(x, y):
    flip = np.random.randint(2)

    if flip:
        x = cv2.flip(x, 1)
        y = -y

    return x, y

def data_augmentation(x, y):
    # Random flip
    x, y = random_horizontal_flip(x, y)

    return x, y

def image_generator(X, y, batch_size,
                    img_shape = preprocess_input.FINAL_IMG_SHAPE):
    """ Provides a batch of images from a log file. The main advantage
        of using a generator is that we do not need to read the whole log file,
        only one batch at a time, so it will fit in RAM.
        This function also generates extended data on the fly. """
    # Pre-allocate batch data
    x_out = np.ndarray(shape=(batch_size,) + img_shape)
    y_out = np.ndarray(shape=(batch_size,))

    # Supply images indefinitely
    while 1:
        # Fill batch
        for i in range(0, batch_size):
            # Get random index to an element in the dataset. Also select
            # randomly which of the 3 images (center, left, right) to use
            idx = np.random.randint(len(y))
            idx_img = np.random.randint(len(ANGLE_OFFSETS))

            # Read image and steering angle (with added offset)
            x_i = cv2.imread(X[idx][idx_img])
            y_i = y[idx] + ANGLE_OFFSETS[idx_img]

            # Preprocess image
            x_i = np.squeeze(preprocess_input.main(np.reshape(x_i, (1,) + x_i.shape)))

            # Augment data
            x_i, y_i = data_augmentation(x_i, y_i)

            # Add to batch
            x_out[i] = x_i
            y_out[i] = y_i

        yield (x_out, y_out)


def make_multiple(x, number):
    """ Increases x to be the smallest multiple of number """
    return int(math.ceil(float(x) / float(number)) * number)


def normalize(X):
    """ Normalizes the input between -0.5 and 0.5 """
    return X / 255. - 0.5


def define_model():
    """ Defines the network architecture, following Nvidia's example on:
        http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf """

    # Parameters
    input_shape = preprocess_input.FINAL_IMG_SHAPE

    weight_init='glorot_uniform'
    padding = 'valid'
    dropout_prob = 0.5

    # Define model
    model = Sequential()

    model.add(Lambda(normalize, input_shape=input_shape, output_shape=input_shape))

    model.add(Convolution2D(24, 5, 5,
                            border_mode=padding,
                            init = weight_init, subsample = (2, 2)))
    model.add(ELU())
    model.add(Convolution2D(36, 5, 5,
                            border_mode=padding,
                            init = weight_init, subsample = (2, 2)))
    model.add(ELU())
    model.add(Convolution2D(48, 5, 5,
                            border_mode=padding,
                            init = weight_init, subsample = (2, 2)))
    model.add(ELU())
    model.add(Convolution2D(64, 3, 3,
                            border_mode=padding,
                            init = weight_init, subsample = (1, 1)))
    model.add(ELU())
    model.add(Convolution2D(64, 3, 3,
                            border_mode=padding,
                            init = weight_init, subsample = (1, 1)))

    model.add(Flatten())
    model.add(Dropout(dropout_prob))
    model.add(ELU())

    model.add(Dense(100, init = weight_init))
    model.add(Dropout(dropout_prob))
    model.add(ELU())

    model.add(Dense(50, init = weight_init))
    model.add(Dropout(dropout_prob))
    model.add(ELU())

    model.add(Dense(10, init = weight_init))
    model.add(Dropout(dropout_prob))
    model.add(ELU())

    model.add(Dense(1, init = weight_init, name = 'output'))

    model.summary()

    # Compile it
    model.compile(loss = 'mse', optimizer = Adam(lr = 0.0001))

    return model


def train_model(model, n_epochs, X_train, y_train, X_val, y_val):
    """ Trains model """
    print('Training model...')

    batch_size = BATCH_SIZE

    n_train_samples = EXTENDED_DATA_FACTOR * len(y_train)
    n_val_samples = EXTENDED_DATA_FACTOR * len(y_val)

    gen_train = image_generator(X_train, y_train, batch_size)
    gen_val = image_generator(X_val, y_val, batch_size)

    model.fit_generator(generator = gen_train,
                        samples_per_epoch = make_multiple(n_train_samples, batch_size),
                        validation_data = gen_val,
                        nb_val_samples = make_multiple(n_val_samples, batch_size),
                        nb_epoch = n_epochs,
                        verbose = 1)


def save_model(out_dir, model):
    """ Saves model (json) and weights (h5) to disk """
    print('Saving model in %s...' % out_dir)

    # Create directory if needed
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # Save model
    model_json = model.to_json()
    with open(os.path.join(out_dir, 'model.json'), 'w+') as f:
        json.dump(model_json, f)

    # Save weights
    model.save_weights(os.path.join(out_dir, 'model.h5'))


def get_training_data(log_file_path):
    """ Reads the CSV file and splits it into training and validation sets """
    # Read CSV file with pandas
    data = pd.read_csv(log_file_path, sep=', ')

    # Get image paths and steering angles
    X = np.column_stack((np.copy(data.iloc[:, 0]), np.copy(data.iloc[:, 1]), np.copy(data.iloc[:, 2])))
    y = np.copy(data.iloc[:, 3])

    # Split into train and validation set
    X_train, X_val, y_train, y_val = train_test_split(X, y, random_state = 192837465)

    return X_train, y_train, X_val, y_val

def build_model(log_file_path, n_epochs):
    """ Builds and trains the network given the input data in train_dir """

    # Get training and validation data
    X_train, y_train, X_val, y_val = get_training_data(log_file_path)
    print ('Train set: %d, validation set: %d' % (len(y_train), len(y_val)))

    # Build and train the network
    model = define_model()
    train_model(model, n_epochs, X_train, y_train, X_val, y_val)

    return model


def parse_input():
    """ Sets up the required input arguments and parses them """
    parser = argparse.ArgumentParser()

    parser.add_argument('log_file', help='CSV file of log data')
    parser.add_argument('-e, --n_epochs', dest='n_epochs',
                        help='number of training epochs', metavar='',
                        type=int, default=5)
    parser.add_argument('-o, --out_dir', dest='out_dir', metavar='',
                        default=time.strftime("%Y%m%d_%H%M%S"),
                        help='directory where the model is stored')

    return parser.parse_args()


def main():
    """ Main function """
    # Get input
    args = parse_input()

    # Build a model
    model = build_model(args.log_file, args.n_epochs)

    # Save model
    save_model(args.out_dir, model)

    print('Finished!')

if __name__ == '__main__':
    main()
