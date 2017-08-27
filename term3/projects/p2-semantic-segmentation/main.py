import os.path
import tensorflow as tf
import numpy as np
from math import ceil
import helper
import warnings
import argparse
import shutil
from distutils.version import LooseVersion
import project_tests as tests

# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))

helper.maybe_download_pretrained_vgg('./data')

def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)

    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    return tf.get_default_graph().get_tensor_by_name(vgg_input_tensor_name),        \
           tf.get_default_graph().get_tensor_by_name(vgg_keep_prob_tensor_name),    \
           tf.get_default_graph().get_tensor_by_name(vgg_layer3_out_tensor_name),   \
           tf.get_default_graph().get_tensor_by_name(vgg_layer4_out_tensor_name),   \
           tf.get_default_graph().get_tensor_by_name(vgg_layer7_out_tensor_name)

#tests.test_load_vgg(load_vgg, tf)

# From
# https://github.com/MarvinTeichmann/tensorflow-fcn/blob/master/fcn8_vgg.py#L284
def get_bilinear_filter(f_shape):
    width = f_shape[0]
    heigh = f_shape[0]
    f = ceil(width/2.0)
    c = (2 * f - 1 - f % 2) / (2.0 * f)
    bilinear = np.zeros([f_shape[0], f_shape[1]])
    for x in range(width):
        for y in range(heigh):
            value = (1 - abs(x / f - c)) * (1 - abs(y / f - c))
            bilinear[x, y] = value
    weights = np.zeros(f_shape)
    for i in range(f_shape[2]):
        weights[:, :, i, i] = bilinear

    init = tf.constant_initializer(value=weights,
                                   dtype=tf.float32)

    var_name = 'up_filter_{}'.format(get_bilinear_filter.count)
    get_bilinear_filter.count += 1

    var = tf.get_variable(name=var_name, initializer=init,
                          shape=weights.shape)
    return var
get_bilinear_filter.count=0

#def upsample(input_tensor, num_classes, upsample_factor):
#    # Get input shape
#    input_shape = tf.shape(input_tensor)

#    input_batch_size = input_shape[0]
#    input_height = input_shape[1]
#    input_width = input_shape[2]
#    input_channels = input_tensor.get_shape()[3]

#    # Create initial kernel weights
#    k_size = upsample_factor * 2
#    k_shape = [k_size, k_size, num_classes, input_channels]
#    weights = get_bilinear_filter(k_shape)

#    # Define output shape
#    out_shape = [input_batch_size, input_height*upsample_factor, input_width*upsample_factor, num_classes]
#    out_shape_tensor = tf.stack(out_shape)

#    # Strides
#    strides = [1, upsample_factor, upsample_factor, 1]
#    return tf.nn.conv2d_transpose(input_tensor,
#                                  weights,
#                                  output_shape=out_shape_tensor,
#                                  strides=strides,
#                                  padding='SAME')

def upsample(input_tensor, num_classes, upsample_factor, name):
    return tf.layers.conv2d_transpose(input_tensor,
                                      filters=num_classes,
                                      kernel_size=2*upsample_factor,
                                      strides=upsample_factor,
                                      padding='same',
                                      name=name,
                                      kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))

def conv1x1(input_tensor, num_outputs, name):
    return tf.layers.conv2d(input_tensor,
                            filters=num_outputs,
                            kernel_size=1,
                            strides=1,
                            padding='same',
                            name=name,
                            kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))

def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # Upsample last layer 32 times to get output
    with tf.variable_scope('DecoderVars'):
        # Add 1x1 convolution on top of conv7 and upsample
        conv7_1x1 = conv1x1(vgg_layer7_out, num_classes, name='conv1x1_1')
        conv7_x2 = upsample(conv7_1x1, num_classes, 2, 'upsample_1')

        # Add 1x1 on top of pool4 to match num_classes
        pool4_1x1 = conv1x1(vgg_layer4_out, num_classes, 'conv1x1_2')

        # Create conv7_x2 + pool4 and upsample
        conv7x2_plus_pool4 = tf.add(conv7_x2, pool4_1x1)
        conv7_plus_pool4_x2 = upsample(conv7x2_plus_pool4, num_classes, 2, 'upsample_2')

        # Add 1x1 on top of pool3 to match num_classes
        conv3_1x1 = conv1x1(vgg_layer3_out, num_classes, 'conv1x1_3')

        # Add
        conv7x4_plus_pool4x2_plus_pool3 = tf.add(conv3_1x1, conv7_plus_pool4_x2)

        # Final upsampling to get FCN-8
        output = upsample(conv7x4_plus_pool4x2_plus_pool3, num_classes, 8, 'upsample_3')

    return output

#tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: value for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    logits = tf.reshape(nn_last_layer, (-1, num_classes))
    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=correct_label))
    optimizer = tf.train.AdamOptimizer(learning_rate)

    # Add loss to summary
    tf.summary.scalar('cross-entropy loss', cross_entropy_loss)

    # Optimize only the decode weights
    decoder_vars = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='DecoderVars')
    train_op = optimizer.minimize(cross_entropy_loss, var_list=decoder_vars)

    return logits, train_op, cross_entropy_loss
#tests.test_optimize(optimize)  # Interferes with run() function - "No variables to optimize" error


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    """
    # Create summary
    logdir = './logs'

    # Delete old logs
    if os.path.isdir(logdir):
        shutil.rmtree(logdir)

    # Create a fresh new log directory
    os.makedirs(logdir)

    # Create writer and summary merge operation
    summary_merged = tf.summary.merge_all()
    writer = tf.summary.FileWriter(logdir, tf.get_default_graph())

    # Initialize variables
    sess.run(tf.global_variables_initializer())
    sess.run(tf.local_variables_initializer())

    # Loop through epocs
    step = 0
    for i_epoch in range(epochs):
        batch_nr = 0
        for batch_x, batch_y in get_batches_fn(batch_size):
            batch_nr += 1
            # Create feed dictionary
            feed_data_train = {input_image: batch_x,
                               correct_label: batch_y,
                               keep_prob: 0.5}

            # Feed it to the network and update the weights
            out = sess.run([train_op, cross_entropy_loss, summary_merged],
                            feed_dict=feed_data_train)

            loss_value = out[1]
            summary_out = out[2]

            writer.add_summary(summary_out, step)
            step += 1

            # Compute current loss for display purposes
            print('[Epoch {}/{}][Batch {}] Loss: {}'
                  .format(i_epoch+1, epochs, batch_nr, loss_value))

    # Close summary writter
    writer.close()

#tests.test_train_nn(train_nn)

def parse_arguments():
    parser = argparse.ArgumentParser(description='Runs the Semantic Segmentation Project')

    parser.add_argument('-e', '--epochs',        dest='epochs',        type=int,   default=100)
    parser.add_argument('-b', '--batch',         dest='batch_size',    type=int,   default=8)
    parser.add_argument('-l', '--learning_rate', dest='learning_rate', type=float, default=0.0001)

    return parser.parse_args()

def run():
    args = parse_arguments()

    num_classes = 2
    epochs = args.epochs
    batch_size = args.batch_size
    learning_rate = args.learning_rate
    image_shape = (160, 576)

    data_dir = './data'

    runs_dir = os.path.join('./runs',
                            'e{}_b{}_l{}'.format(epochs, batch_size,
                            str(learning_rate).replace ('.', '_')))

    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    # Create placeholders
    correct_label = tf.placeholder(tf.float32, shape=[None, image_shape[0], image_shape[1], num_classes])

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')

        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # Build NN using load_vgg, layers, and optimize function
        input_image, keep_prob, vgg_layer3_out, vgg_layer4_out, vgg_layer7_out = load_vgg(sess, vgg_path)
        output = layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes)
        logits, train_op, cross_entropy_loss = optimize(output, correct_label, learning_rate, num_classes)

        # Train NN using the train_nn function
        train_nn(sess, epochs, batch_size, get_batches_fn, train_op,
                 cross_entropy_loss, input_image, correct_label, keep_prob)

        # Save inference data using helper.save_inference_samples
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)

        # OPTIONAL: Apply the trained model to a video

if __name__ == '__main__':
    run()
