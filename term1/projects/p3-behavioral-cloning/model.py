import os
import cv2
import csv
import time
import argparse

def save_model(args, model):
    raise NotImplemented


def load_log_file(log_file):
    X_train = []
    y_train = []

    with open(log_file, 'rb') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')

        for row in csv_reader:
            X_train.append(cv2.imread(row[0]))
            y_train.append(float(row[3]))

    return X_train, y_train

def load_dataset(log_files):
    """ Loads the raw training dataset """
    # Declare outputs
    X_train = []
    y_train = []

    for log_file in log_files:
        X_train_i, y_train_i = load_log_file(log_file)
        X_train.append(X_train_i)
        y_train.append(y_train_i)

    return X_train, y_train


def get_training_data(log_files):
    """ Loads and preprocesses the training data """
    # Load data data
    X_train, y_train = load_dataset(log_files)

    # Preprocess input
    X_train = preprocess_input(X_train)

    # Pack and output
    out_data = {'X_train': X_train, 'y_train': y_train}
    return out_data


def build_model(log_files):
    """ Builds and trains the network given the input data in train_dir """
    # Get training data
    data = get_training_data(log_files)

    # Build and train the network
    model = define_model()
    model = train_model(model, data)

    return model


def parse_input():
    """ Sets up the required input arguments and parses them """
    parser = argparse.ArgumentParser()

    parser.add_argument('log_file(s)', nargs='+', dest='log_files',
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


if __name__ == '__main__':
    main()
