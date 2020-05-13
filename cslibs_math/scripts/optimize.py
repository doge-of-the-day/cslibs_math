#!/usr/bin/python3

import numpy as np
from scipy.optimize import minimize

# value file : actual log2
# exponent file : exponent - 1
# fraction file : fraction - 0.5
# function to optimize = (exponent-1) + 2.3 * (fraction - 0.5);

exponent = []
fraction = []
log2 = []



with open('log2_values') as file:
	for line in file:
		log2.append(float(line))

with open('log2_exponent') as file:
	for line in file:
		exponent.append(float(line))

with open('log2_fraction') as file:
	for line in file:
		fraction.append(float(line))

np_exponent = np.array(exponent)
np_fraction = np.array(fraction)
np_log2 = np.array(log2)


def error(x):
	error = np.sum((log2 - (np_exponent + x * np_fraction))**2)
	return error


#print(log2)
#print(exponent)
#print(fraction)

res = minimize(error, 2.0)

for i in range(len(log2)):
	log2_approx=np_exponent[i] + res.x * np_fraction[i]
	diff = log2[i] - log2_approx
	line = f"{log2[i]} : {log2_approx} : {diff}"
	print(line)
	input()
