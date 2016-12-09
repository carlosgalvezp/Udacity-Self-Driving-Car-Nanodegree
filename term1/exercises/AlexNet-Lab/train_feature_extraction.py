import pickle
import time
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from alexnet import AlexNet

# TODO: Load traffic signs data.
with open('train.p', mode='rb') as f:
    data = pickle.load(f)

X_train, y_train = data['features'], data['labels']

# TODO: Split data into training and validation sets.
X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, random_state=918273645)

# TODO: Define placeholders and resize operation.
x = tf.placeholder(tf.float32, (None, 32, 32, 3))
resized = tf.image.resize_images(x, [227, 227])
y_true_class = tf.placeholder(tf.int64, None)

# TODO: pass placeholder as first argument to `AlexNet`.
fc7 = AlexNet(resized, feature_extract=True)

# NOTE: `tf.stop_gradient` prevents the gradient from flowing backwards
# past this point, keeping the weights before and up to `fc7` frozen.
# This also makes training faster, less work to do!
fc7 = tf.stop_gradient(fc7)

# TODO: Add the final layer for traffic sign classification.
nb_classes = 43
shape = (fc7.get_shape().as_list()[-1], nb_classes)  # use this shape for the weight matrix

weights = tf.Variable(tf.truncated_normal(shape, stddev=0.05))
biases = tf.Variable(tf.zeros(nb_classes) + 0.05)

fc8 = tf.matmul(fc7, weights)
fc8 = tf.nn.bias_add(fc8, biases)

y_predicted = tf.nn.softmax(fc8)
y_predicted_class = tf.argmax(y_predicted, dimension=1)

# TODO: Define loss, training, accuracy operations.
# HINT: Look back at your traffic signs project solution, you may
# be able to reuse some the code.
cross_entropy = tf.nn.sparse_softmax_cross_entropy_with_logits(logits=fc8,
                                                        labels=y_true_class)
cost = tf.reduce_mean(cross_entropy)

learning_rate = 0.0001
optimizer = tf.train.AdamOptimizer(learning_rate).minimize(cost, var_list=[weights, biases])

### Performance evaluation
# Check if the predicted class is equal to the true class
is_correct_prediction = tf.equal(y_predicted_class, y_true_class)

# Compute the accuracy by averaging out the previous variable
accuracy = 100.0 * tf.reduce_mean(tf.cast(is_correct_prediction, tf.float32))

# TODO: Train and evaluate the feature extraction model
sess = tf.Session()
sess.run(tf.initialize_all_variables())

batch_size = 128
n_epochs = 10

def eval_on_data(X, y, sess):
    total_acc = 0
    total_loss = 0
    for offset in range(0, X.shape[0], batch_size):
        end = offset + batch_size
        X_batch = X[offset:end]
        y_batch = y[offset:end]

        loss, acc = sess.run([cost, accuracy], feed_dict={x: X_batch, y_true_class: y_batch})
        total_loss += (loss * X_batch.shape[0])
        total_acc += (acc * X_batch.shape[0])

    return total_loss/X.shape[0], total_acc/X.shape[0]

for i in range(n_epochs):
    # training
    X_train, y_train = shuffle(X_train, y_train)
    t0 = time.time()
    for offset in range(0, X_train.shape[0], batch_size):
        end = offset + batch_size
        sess.run(optimizer, feed_dict={x: X_train[offset:end], y_true_class: y_train[offset:end]})

    val_loss, val_acc = eval_on_data(X_val, y_val, sess)
    print("Epoch", i+1)
    print("Time: %.3f seconds" % (time.time() - t0))
    print("Validation Loss =", val_loss)
    print("Validation Accuracy =", val_acc)
    print("")

