import os
import cv2
import numpy as np
import csv
import time
import argparse
from keras.models import Sequential
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.core import Dense, Dropout, Flatten
from keras.optimizers import Adam

IMG_SHAPE_IN = (66, 200, 1)

def save_model(args, model):
    raise NotImplemented

def resize(x):
    height = x.shape[0]
    width = x.shape[1]

    factor = float(IMG_SHAPE_IN[1]) / float(width)

    resized_size = (int(width*factor), int(height*factor))
    x = cv2.resize(x, resized_size)
    crop_height = resized_size[1] - IMG_SHAPE_IN[0]

    return x[crop_height:, :, :]

def rgb_to_gray(x):
    return cv2.cvtColor(x, cv2.COLOR_RGB2GRAY)

def normalize(x):
    # Approximately zero-mean, unit variance
    return (np.array(x, dtype=np.float32) - 128.0) / 128.0

def preprocess_input(X):
    """ Preprocesses input data
        X is a tensor (n_img, height, width, depth) """
    print('Preprocessing training data...')

    X_out = []
    for i in range(X.shape[0]):
        img = X[i,:]
        img = resize(img)
        img = rgb_to_gray(img)
        img = normalize(img)

        X_out.append(np.reshape(img, IMG_SHAPE_IN))

    return np.array(X_out)


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


def get_training_data(log_files):
    """ Loads and preprocesses the training data """
    # Load data data
    X_train, y_train = load_dataset(log_files)

    # Preprocess input
    X_train = preprocess_input(X_train)

    # Pack and output
    out_data = {'X_train': X_train, 'y_train': y_train}
    return out_data

def train_model(model, data):
    raise NotImplemented

def define_model():
    # Parameters
    input_shape = IMG_SHAPE_IN
    conv1_filter_size = 5
    conv2_filter_size = 3
    conv3_filter_size = 3

    padding = 'valid'
    pool_size = (2,2)
    dropout_prob = 0.5

    n_fc1 = 128
    n_fc2 = 64

    # Define model
    model = Sequential()

    model.add(Convolution2D(32, conv1_filter_size, conv1_filter_size,
                            border_mode=padding, activation = 'relu',
                            input_shape=input_shape))
    model.add(MaxPooling2D(pool_size=pool_size))
    model.add(Dropout(dropout_prob))

    model.add(Convolution2D(64, conv2_filter_size, conv2_filter_size,
                            border_mode=padding, activation = 'relu'))
    model.add(MaxPooling2D(pool_size=pool_size))
    model.add(Dropout(dropout_prob))

    model.add(Convolution2D(128, conv3_filter_size, conv3_filter_size,
                            border_mode=padding, activation = 'relu'))
    model.add(MaxPooling2D(pool_size=pool_size))
    model.add(Dropout(dropout_prob))

    model.add(Flatten())

    model.add(Dense(n_fc1, activation = 'relu'))
    model.add(Dropout(dropout_prob))
    model.add(Dense(n_fc2, activation = 'relu'))
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
    n_epochs = 10
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
    with open(os.path.join(out_dir, 'model.json'), 'w+') as json_file:
        json_file.write(model_json)

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
