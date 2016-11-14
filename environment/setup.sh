#!/bin/bash
set -e

echo 'Installing base environment...'
conda env create -f environment.yml
source activate carnd

echo 'Installing OpenCV...'
conda install -c https://conda.anaconda.org/menpo opencv3

echo 'Installing TensorFlow...'
export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-0.11.0-cp35-cp35m-linux_x86_64.whl
pip install --ignore-installed --upgrade $TF_BINARY_URL
source deactivate

echo 'Done!'
