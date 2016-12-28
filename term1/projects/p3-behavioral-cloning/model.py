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
from keras.layers import Dense, Dropout, Flatten, Lambda, Activation, ELU
from keras.optimizers import Adam
from keras.callbacks import Callback
import matplotlib.image as mpimg
import preprocess_input

# Angle offset for the left and right cameras. It's and estimation of the
# additional steering angle (normalized to [-1,1]) that we would have to steer
# if the center camera was in the position of the left or right one
ANGLE_OFFSET = 0.25

# Angle offsets applied to center, left and right image
ANGLE_OFFSETS = [0.0, ANGLE_OFFSET, -ANGLE_OFFSET]

# Batch size
BATCH_SIZE = 64

def random_horizontal_flip(x, y):
    flip = np.random.randint(2)

    if flip:
        x = cv2.flip(x, 1)
        y = -y

    return x, y

def random_translation(img, steering):
    # Maximum shift of the image, in pixels
    trans_range = 50  # Pixels

    # Compute translation and corresponding steering angle
    tr_x = np.random.uniform(-trans_range, trans_range)
    steering = steering + (tr_x / trans_range) * ANGLE_OFFSET

    # Warp image using the computed translation
    rows = img.shape[0]
    cols = img.shape[1]

    M = np.float32([[1,0,tr_x],[0,1,0]])
    img = cv2.warpAffine(img,M,(cols,rows))

    return img, steering

def data_augmentation(x, y):
    # Random horizontal shift
    x, y = random_translation(x, y)

    # Random flip
    x, y = random_horizontal_flip(x, y)

    return x, y

def train_generator(X, y, batch_size):
    """ Provides a batch of images from a log file. The main advantage
        of using a generator is that we do not need to read the whole log file,
        only one batch at a time, so it will fit in RAM.
        This function also generates extended data on the fly. """
    # Supply training images indefinitely
    while 1:
        # Declare output data
        x_out = []
        y_out = []

        # Fill batch
        for i in range(0, batch_size):
            # Get random index to an element in the dataset.
            idx = np.random.randint(len(y))

            # Randomly select which of the 3 images (center, left, right) to use
            idx_img = np.random.randint(len(ANGLE_OFFSETS))

            # Read image and steering angle (with added offset)
            x_i = mpimg.imread(X[idx][idx_img].strip())
            y_i = y[idx] + ANGLE_OFFSETS[idx_img]

            # Preprocess image
            x_i = preprocess_input.main(x_i)

            # Augment data
            x_i, y_i = data_augmentation(x_i, y_i)

            # Add to batch
            x_out.append(x_i)
            y_out.append(y_i)

        yield (np.array(x_out), np.array(y_out))


def val_generator(X, y):
    """ Provides images for validation. This generator is different
        from the previous one in that it does **not** perform data augmentation:
        it just reads images from disk, preprocess them and yields them """
    # Validation generator
    while 1:
        for i in range(len(y)):
            # Read image and steering angle
            x_out = mpimg.imread(X[i][0].strip())
            y_out = np.array([[y[i]]])

            # Preprocess image
            x_out = preprocess_input.main(x_out)
            x_out = x_out[None, :, :, :]

            # Return the data
            yield x_out, y_out


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
    dropout_prob = 0.25

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
    model.compile(loss = 'mse', optimizer = Adam(lr = 0.001))

    return model


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


class EpochSaverCallback(Callback):
    def __init__(self, out_dir):
        self.out_dir = out_dir

    def on_train_begin(self, logs={}):
        self.losses = []

    def on_epoch_end(self, epoch, logs={}):
        current_loss = logs.get('val_loss')

        if not self.losses or current_loss < np.amin(self.losses):
            out_dir = os.path.join(self.out_dir, 'e' + str(epoch+1))
            save_model(out_dir, self.model)

        self.losses.append(current_loss)



def train_model(model, save_dir, n_epochs, X, y):
    """ Trains model """
    print('Training model...')

    batch_size = BATCH_SIZE

    n_train_samples = 5 * make_multiple(len(y), batch_size)
    n_val_samples = len(y)

    gen_train = train_generator(X, y, batch_size)
    gen_val = val_generator(X, y)

    checkpoint_callback = EpochSaverCallback(save_dir)

    model.fit_generator(generator = gen_train,
                        samples_per_epoch = n_train_samples,
                        validation_data = gen_val,
                        nb_val_samples = n_val_samples,
                        nb_epoch = n_epochs,
                        callbacks = [checkpoint_callback],
                        verbose = 1)


def get_training_data(log_file_path):
    """ Reads the CSV file and splits it into training and validation sets """
    # Read CSV file with pandas
    data = pd.read_csv(log_file_path)

    # Get image paths and steering angles
    X = np.column_stack((np.copy(data['center']), np.copy(data['left']), np.copy(data['right'])))
    y = np.copy(data['steering'])

    return X, y

def build_model(log_file_path, n_epochs, save_dir):
    """ Builds and trains the network given the input data in train_dir """

    # Get training and validation data
    X, y = get_training_data(log_file_path)

    # Build and train the network
    model = define_model()
    train_model(model, save_dir, n_epochs, X, y)

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
    model = build_model(args.log_file, args.n_epochs, args.out_dir)

    # Save model
    save_model(args.out_dir, model)

    print('Finished!')

if __name__ == '__main__':
    main()
