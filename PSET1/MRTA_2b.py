'''
Harvard CS 286 Spring 2022
'''

used = set()

def solve(N, R, Q, C):
	tempmax = 0
	index = -1
	'''
	Your code here
	'''
	# ==== Implementation of Murdoch's Algorithm ====

	# calculate Utility list from Quality - Cost
	U = [Q[i] - C[i] for i in range(len(Q))]

	# for every task that we have already assigned, make the utiliy -inf
	for indx in used:
		U[indx] = -float("inf")
	
	# get the greatest possible utility
	tempmax = max(U)
	index = U.index(tempmax)


	# return the task index and the utility and add the task to seen
	used.add(index)
	return (index, len(used) - 1, tempmax)

if __name__ == "__main__":
	print("Input N , R")
	N, R = map(int, input().split())
	result = []

	for i in range(N):
		print("Input Q")
		Q = (list(map(int,input().rstrip().split())))
		
		print("Input C")
		C = (list(map(int,input().rstrip().split())))
		
		print("Solution for task ", str(i))
		cur_result = solve(1, R, Q, C)
		result.append(cur_result)
		# used.add(cur_result[0])
		print(result)
		print(used)

