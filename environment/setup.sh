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

echo 'Installing Keras...'
pip install keras
cat - > ~/.keras/keras.json <<END
{
    "image_dim_ordering": "tf",
    "epsilon": 1e-07,
    "backend": "tensorflow",
    "floatx": "float32"
}
END

source deactivate
echo 'Done!'
