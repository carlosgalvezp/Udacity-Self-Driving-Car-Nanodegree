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

def rgb_to_yuv(x):
    return cv2.cvtColor(x, cv2.COLOR_BGR2YUV)

def main(img):
    """ Preprocesses input data
        img is an image of shape (height, width, depth) """
    img = resize(img)
    #img = rgb_to_yuv(img)

    return img
