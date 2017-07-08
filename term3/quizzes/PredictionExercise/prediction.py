#!/usr/bin/env python

from classifier import GNB
import json

def main():
	gnb = GNB()
	with open('train.json', 'rb') as f:
		j = json.load(f)
	print j.keys()
	X = j['states']
	Y = j['labels']
	gnb.train(X, Y)

	with open('test.json', 'rb') as f:
		j = json.load(f)

	X = j['states']
	Y = j['labels']
	score = 0
	for coords, label in zip(X,Y):
		predicted = gnb.predict(coords)
		if predicted == label:
			score += 1
	fraction_correct = float(score) / len(X)
	print "You got {} percent correct".format(100 * fraction_correct)

if __name__ == "__main__":
	main()