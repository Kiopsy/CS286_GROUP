'''
Harvard CS 286 Spring 2022
'''

from itertools import permutations
from scipy.optimize import linear_sum_assignment
import numpy as np

def solve(N, R, Q, C):
	'''
	Add your code here
	'''
	# initialize negative utility matrix
	Un = []

	for i in range(len(Q)):
		row = []
		for j in range(len(Q[0])):
			# subtract Qij from Cij for all values in the matrix
			row.append(C[i][j] - Q[i][j])
		Un.append(row)

	# initialize results
	result_choice = []

	# run linear_sum_assignment on the negative utility matrix and get the
	# row indexes and column indexes of the assignment for minimum cost
	cost = np.array(Un)
	row_ind, col_ind = linear_sum_assignment(cost)

	 
	for r, c in zip(row_ind, col_ind):
		# negate the negative minimum utility to get maximized utility
		util = -Un[r][c]

		# add an assignment to results if the utility of the assignment is greater than 0
		if util >= 0:
			result_choice.append((c, r, util))
	
	return result_choice

if __name__ == "__main__":
	print("Input N , R")
	N, R = map(int, input().split())

	print("Input Q")
	Q = []
	for i in range(N):
		Q.append(list(map(int, input().rstrip().split())))

	print("Input C")
	C = []
	for i in range(N):
		C.append(list(map(int, input().rstrip().split())))

	print(Q)
	print(C)

	print("Solving...")
	print(solve(N, R, Q, C))