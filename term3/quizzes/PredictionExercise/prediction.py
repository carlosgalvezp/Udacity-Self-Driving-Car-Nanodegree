#!/usr/bin/env python

from classifier import GNB
import json

def main():

    # Load training data
    with open('train.json', 'r') as f:
        j = json.load(f)

    X = j['states']
    Y = j['labels']

    # Train classifier
    gnb = GNB()
    gnb.train(X, Y)

    # Open test data
    with open('test.json', 'r') as f:
	    j = json.load(f)

    X = j['states']
    Y = j['labels']

    # Evaluate classifier
    score = 0
    for coords, label in zip(X,Y):
        predicted = gnb.predict(coords)
        if predicted == label:
	        score += 1
    fraction_correct = float(score) / len(X)
    print("You got {} percent correct".format(100 * fraction_correct))

if __name__ == "__main__":
	main()
