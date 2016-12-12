import cv2
import numpy as np

FINAL_IMG_SHAPE = (66, 200, 3)

def resize(x):
    height = x.shape[0]
    width = x.shape[1]

    factor = float(FINAL_IMG_SHAPE[1]) / float(width)

    resized_size = (int(width*factor), int(height*factor))
    x = cv2.resize(x, resized_size)
    crop_height = resized_size[1] - FINAL_IMG_SHAPE[0]

    return x[crop_height:, :, :]

def rgb_to_gray(x):
    return cv2.cvtColor(x, cv2.COLOR_BGR2GRAY)

def rgb_to_yuv(x):
    return cv2.cvtColor(x, cv2.COLOR_BGR2YUV)


def normalize(x):
    # Normalize between -0.5 and 0.5
    return x / 255.0 - 0.5

def main(X):
    """ Preprocesses input data
        X is a tensor (n_img, height, width, depth) """
    X_out = []
    for i in range(X.shape[0]):
        img = X[i,:]
        img = resize(img)
        img = rgb_to_yuv(img)
        img = normalize(img)

        X_out.append(np.reshape(img, FINAL_IMG_SHAPE))

    return np.array(X_out)
