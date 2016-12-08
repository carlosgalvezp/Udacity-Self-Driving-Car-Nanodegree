import cv2
import numpy as np

FINAL_IMG_SHAPE = (66, 200, 1)

def resize(x):
    height = x.shape[0]
    width = x.shape[1]

    factor = float(FINAL_IMG_SHAPE[1]) / float(width)

    resized_size = (int(width*factor), int(height*factor))
    x = cv2.resize(x, resized_size)
    crop_height = resized_size[1] - FINAL_IMG_SHAPE[0]

    return x[crop_height:, :, :]

def rgb_to_gray(x):
    return cv2.cvtColor(x, cv2.COLOR_RGB2GRAY)

def normalize(x):
    # Approximately zero-mean, unit variance
    return (np.array(x, dtype=np.float32) - 128.0) / 128.0

def main(X):
    """ Preprocesses input data
        X is a tensor (n_img, height, width, depth) """
    X_out = []
    for i in range(X.shape[0]):
        img = X[i,:]
        img = resize(img)
        img = rgb_to_gray(img)
        img = normalize(img)

        X_out.append(np.reshape(img, FINAL_IMG_SHAPE))

    return np.array(X_out)
