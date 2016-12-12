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
    """ Defines the network architecture, following Nvidia's example on:
        http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf """

    # Parameters
    input_shape = preprocess_input.FINAL_IMG_SHAPE

    weight_init='normal'
    activation = 'relu'
    padding = 'valid'
    dropout_prob = 0.5

    # Define model
    model = Sequential()

    model.add(Convolution2D(24, 5, 5, input_shape = input_shape,
                            border_mode=padding, activation = activation,
                            init = weight_init, subsample = (2, 2)))
    model.add(Convolution2D(36, 5, 5,
                            border_mode=padding, activation = activation,
                            init = weight_init, subsample = (2, 2)))
    model.add(Convolution2D(48, 5, 5,
                            border_mode=padding, activation = activation,
                            init = weight_init, subsample = (2, 2)))
    model.add(Convolution2D(64, 3, 3,
                            border_mode=padding, activation = activation,
                            init = weight_init, subsample = (1, 1)))
    model.add(Convolution2D(64, 3, 3,
                            border_mode=padding, activation = activation,
                            init = weight_init, subsample = (1, 1)))
    model.add(Dropout(dropout_prob))

    model.add(Flatten())
    model.add(Dense(100, init = weight_init, activation = activation))
    model.add(Dropout(dropout_prob))
    model.add(Dense(50, init = weight_init, activation = activation))
    model.add(Dropout(dropout_prob))
    model.add(Dense(10, init = weight_init, activation = activation))
    model.add(Dropout(dropout_prob))
    model.add(Dense(1, init = weight_init, name = 'output'))

    model.summary()

    # Compile it
    model.compile(loss = 'mse', optimizer = Adam(lr = 0.0001))

    return model


def train_model(model, train_csv, val_csv):
    """ Trains model """
    print('Training model...')

    batch_size = 128
    n_epochs = 20

    n_train_samples = math.ceil(2 * len(train_csv)/batch_size) * batch_size
    n_val_samples = math.ceil(2 * len(val_csv)/batch_size) * batch_size

    gen_train = image_generator(train_csv, batch_size)
    gen_val = image_generator(val_csv, batch_size)

    model.fit_generator(generator = gen_train,
                        samples_per_epoch = n_train_samples,
                        validation_data = gen_val,
                        nb_val_samples = n_val_samples,
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


def evaluate_model(model, test_csv):
    """ Evaluates the model on test data, printing out the loss """
    print('Evaluating model on test set...')
    batch_size = 128
    n_test_samples = math.ceil(2 * len(test_csv)/batch_size) * batch_size

    gen_test = image_generator(test_csv, batch_size)
    score = model.evaluate_generator(gen_test, n_test_samples)

    print('Test loss: %.4f' % score[0])


def split_training_data(csv_data, train_ratio, val_ratio):
    """ Split input log file (CSV) into train, validation and test sets,
        according to train_ratio and val_ratio """
    assert train_ratio + val_ratio < 1.0

    n_total = len(csv_data)
    n_train = int(math.ceil(n_total * train_ratio))
    n_val = int(math.ceil(n_total * val_ratio))

    idx = np.arange(0, n_total)
    np.random.shuffle(idx)

    train_csv = csv_data.iloc[idx[0:n_train]]
    val_csv = csv_data.iloc[idx[n_train : n_train + n_val]]
    test_csv = csv_data.iloc[idx[n_train + n_val:]]

    return train_csv, val_csv, test_csv

def build_model(log_file_path):
    """ Builds and trains the network given the input data in train_dir """
    # Read CSV file with pandas
    data = pd.read_csv(log_file_path)

    # Split into train, validation and test sets
    train_csv, val_csv, test_csv = split_training_data(data, 0.8, 0.1)
    print ('Train set: %d, validation set: %d, test set: %d' %
            (len(train_csv), len(val_csv), len(test_csv)))

    # Build and train the network
    model = define_model()
    train_model(model, train_csv, val_csv)

    # Evaluate model on test data
    evaluate_model(model, test_csv)

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
