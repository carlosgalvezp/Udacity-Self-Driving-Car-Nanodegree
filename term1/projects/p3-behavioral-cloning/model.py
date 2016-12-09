import os
import cv2
import numpy as np
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

def load_dataset(log_files):
    """ Loads the raw training dataset """
    print('Loading training data...')

    # Declare outputs
    X_train = []
    y_train = []

    # Loop through log files
    for log_file in log_files:
        with open(log_file, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')

            # Read image from center camera and steering angle
            for row in csv_reader:
                X_train.append(cv2.imread(row[0]))
                y_train.append(float(row[3]))

    return np.array(X_train), np.array(y_train)

def flip_images(X_train, y_train):
    X_extra = []
    y_extra = []

    for i in range(X_train.shape[0]):
        X_extra.append(cv2.flip(X_train[i], 1))
        y_extra.append(-y_train[i])

    X_train = np.concatenate((X_train, np.array(X_extra)), axis = 0)
    y_train = np.concatenate((y_train, np.array(y_extra)), axis = 0)

    return X_train, y_train

def extend_training_data(X_train, y_train):
    # Horizontally flip images and negate steering angle
    X_train, y_train = flip_images(X_train, y_train)

    return X_train, y_train

def get_training_data(log_files):
    """ Loads and preprocesses the training data """
    # Load data data
    X_train, y_train = load_dataset(log_files)
    print('Input dataset: ', X_train.shape)

    # Extend dataset
    X_train, y_train = extend_training_data(X_train, y_train)
    print('Extended dataset: ', X_train.shape)

    # Preprocess input
    X_train = preprocess_input.main(X_train)

    # Pack and output
    out_data = {'X_train': X_train, 'y_train': y_train}
    return out_data

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

    # Define model
    model = Sequential()

    model.add(Convolution2D(24, conv1_filter_size, conv1_filter_size,
                            border_mode=padding, activation = 'relu',
                            init='normal', subsample=(2, 2),
                            input_shape=input_shape))
    model.add(Dropout(dropout_prob))

    model.add(Convolution2D(36, conv2_filter_size, conv2_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(2, 2), init='normal'))
    model.add(Dropout(dropout_prob))

    model.add(Convolution2D(48, conv3_filter_size, conv3_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(2, 2), init='normal'))
    model.add(Dropout(dropout_prob))

    model.add(Convolution2D(64, conv4_filter_size, conv4_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(1, 1), init='normal'))
    model.add(Dropout(dropout_prob))

    model.add(Convolution2D(64, conv5_filter_size, conv5_filter_size,
                            border_mode=padding, activation = 'relu',
                            subsample=(1, 1), init='normal'))
    model.add(Dropout(dropout_prob))

    model.add(Flatten())

    model.add(Dense(n_fc1, activation = 'relu', init='normal'))
    model.add(Dropout(dropout_prob))
    model.add(Dense(n_fc2, activation = 'relu', init='normal'))
    model.add(Dropout(dropout_prob))
    model.add(Dense(n_fc3, activation = 'relu', init='normal'))
    model.add(Dropout(dropout_prob))
    model.add(Dense(1))

    model.summary()

    # Compile it
    model.compile(loss='mean_squared_error', optimizer=Adam(lr=0.001),
                  metrics=['accuracy'])

    return model


def train_model(model, data):
    print('Training model...')

    batch_size = 128
    n_epochs = 5
    history = model.fit(data['X_train'], data['y_train'],
                        batch_size=batch_size, nb_epoch=n_epochs,
                        verbose=1, validation_split=0.2)

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

def build_model(log_files):
    """ Builds and trains the network given the input data in train_dir """
    # Get training data
    data = get_training_data(log_files)

    # Build and train the network
    model = define_model()
    train_model(model, data)
    return model


def parse_input():
    """ Sets up the required input arguments and parses them """
    parser = argparse.ArgumentParser()

    parser.add_argument('log_files', nargs='+',
                        help='list of CSV files of log data')
    parser.add_argument('-o, --out_dir', dest='out_dir', metavar='',
                        default=time.strftime("%Y%m%d_%H%M%S"),
                        help='directory where the model is stored')

    return parser.parse_args()


def main():
    """ Main function """
    # Get input
    args = parse_input()

    # Build a model
    model = build_model(args.log_files)

    # Save model
    save_model(args.out_dir, model)

    print('Finished!')

if __name__ == '__main__':
    main()
