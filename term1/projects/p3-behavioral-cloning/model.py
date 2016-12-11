import os
import cv2
import math
import numpy as np
import pandas as pd
import csv
import time
import argparse
import json
from keras.models import Sequential
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.core import Dense, Dropout, Flatten
from keras.optimizers import Adam

import preprocess_input

def image_generator(log_file_csv, batch_size,
                    img_shape = preprocess_input.FINAL_IMG_SHAPE):
    """ Provides a batch of images from a log file. The main advantage
        of using a generator is that we do not need to read the whole log file,
        only one batch at a time, so it will fit in RAM.
        This function also generates extended data on the fly. """
    log_idx = 0
    batch_size_half = int(batch_size/2)

    n_log_img = len(log_file_csv)

    # Pre-allocate data
    x = np.ndarray(shape=(batch_size, img_shape[0], img_shape[1], img_shape[2]))
    y = np.ndarray(shape=(batch_size,))

    shuffle_idx = np.arange(0, n_log_img)

    while 1:
        if log_idx  < batch_size_half:
            # shuffle
            np.random.shuffle(shuffle_idx)

        for j in range(0, batch_size_half):
            log_idx  = (log_idx  + 1) % n_log_img

            # Compute shuffled idx
            i_shuffle = shuffle_idx[log_idx]

            # Read image from log file
            x_i = cv2.imread(log_file_csv.iloc[i_shuffle][0])
            y_i = float(log_file_csv.iloc[i_shuffle][3])

            # Preprocess image
            x_i = np.squeeze(preprocess_input.main(np.reshape(x_i, (1,) + x_i.shape)))

            # Add to batch
            i = 2*j
            x[i] = x_i
            y[i] = y_i

            # Add a horizontally-flipped copy
            x[i + 1] = cv2.flip(x_i, 1)
            y[i + 1] = -y_i

        yield (x, y)


def define_model():
    # Based on http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf
    # Parameters
    input_shape = preprocess_input.FINAL_IMG_SHAPE
    conv1_filter_size = 5
    conv2_filter_size = 5
    conv3_filter_size = 5
    conv4_filter_size = 3
    conv5_filter_size = 3

    padding = 'valid'
    pool_size = (2,2)
    dropout_prob = 0.5

    n_fc1 = 100
    n_fc2 = 50
    n_fc3 = 10
    n_fc4 = 1

    # Define model
    model = Sequential()

    model.add(Convolution2D(24, conv1_filter_size, conv1_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(2, 2), input_shape=input_shape))
    model.add(Convolution2D(36, conv2_filter_size, conv2_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(2, 2)))
    model.add(Convolution2D(48, conv3_filter_size, conv3_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(2, 2)))
    model.add(Convolution2D(64, conv4_filter_size, conv4_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(1, 1)))
    model.add(Convolution2D(64, conv5_filter_size, conv5_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(1, 1)))

    model.add(Flatten())

    model.add(Dense(n_fc1, activation = 'relu'))
    model.add(Dense(n_fc2, activation = 'relu'))
    model.add(Dense(n_fc3, activation = 'relu'))
    model.add(Dense(n_fc4, name = 'output'))

    model.summary()

    # Compile it
    model.compile(loss='mse', optimizer=Adam(lr=0.0001),
                  metrics=['accuracy'])

    return model


def train_model(model, log_data_csv):
    print('Training model...')

    batch_size = 128
    n_epochs = 10
    samples_per_epoch = math.ceil(2 * len(log_data_csv)/batch_size) * batch_size

    gen = image_generator(log_data_csv, batch_size)

    model.fit_generator(image_generator(log_data_csv, batch_size),
                        samples_per_epoch = samples_per_epoch,
                        nb_epoch = n_epochs,
                        verbose = 1)


def save_model(out_dir, model):
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

def build_model(log_file_path):
    """ Builds and trains the network given the input data in train_dir """
    # Read CSV file with pandas
    data = pd.read_csv(log_file_path)

    # Build and train the network
    model = define_model()
    train_model(model, data)
    return model


def parse_input():
    """ Sets up the required input arguments and parses them """
    parser = argparse.ArgumentParser()

    parser.add_argument('log_file', help='CSV file of log data')
    parser.add_argument('-o, --out_dir', dest='out_dir', metavar='',
                        default=time.strftime("%Y%m%d_%H%M%S"),
                        help='directory where the model is stored')

    return parser.parse_args()


def main():
    """ Main function """
    # Get input
    args = parse_input()

    # Build a model
    model = build_model(args.log_file)

    # Save model
    save_model(args.out_dir, model)

    print('Finished!')

if __name__ == '__main__':
    main()
