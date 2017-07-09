from sklearn.naive_bayes import GaussianNB

class GNB(object):
    """ Implements a Gaussian Naive Bayes classifier """

    def __init__(self):
        self.possible_labels = ['left', 'keep', 'right']
        self.clf = GaussianNB()

    def train(self, data, labels):
        """
        Trains the classifier with N data points and labels.

        INPUTS
        data - array of N observations
          - Each observation is a tuple with 4 values: s, d,
            s_dot and d_dot.
          - Example : [
	          	[3.5, 0.1, 5.9, -0.02],
	          	[8.0, -0.3, 3.0, 2.2],
	          	...
          	]

        labels - array of N labels
          - Each label is one of "left", "keep", or "right".
        """
        # Get relevant features
        X_train = self._get_features(data)

        # Fit classifier
        self.clf.fit(X_train, labels)

    def predict(self, observation):
        """
        Once trained, this method is called and expected to return 
        a predicted behavior for the given observation.

        INPUTS

        observation - a 4 tuple with s, d, s_dot, d_dot.
          - Example: [3.5, 0.1, 8.5, -0.2]

        OUTPUT

        A label representing the best guess of the classifier. Can
        be one of "left", "keep" or "right".
        """
        # Get relevant features
        X_test = self._get_features(observation)

        # Predict class
        return self.clf.predict([X_test])

    def _get_features(self, data):
        return data
