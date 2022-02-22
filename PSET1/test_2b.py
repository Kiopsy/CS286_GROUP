# NOTE: MRTA_2a.py must be in the same folder as this file to run tests
from MRTA_2b import solve

test1 = {
    "N": 2,
    "R": 4,
    "Q": [[1, 3, 5, 4], [3, 2, 6, 4]],
    "C": [[0, 1, 0, 2], [1, 0, 1, 2]],
    "expected_ans": [(2, 0, 5), (0, 1, 2)]
}

test2 = {
    "N": 2,
    "R": 2,
    "Q": [[2, 10], [1, 11]],
    "C": [[0, 0], [0, 0]],

    # not the most optimal, but best by greedy choosing
    "expected_ans": [(1, 0, 10), (0, 1, 1)]
}


tests = [test1, test2]

for t in tests:
    result = []
    for i in range(t["N"]):
        print("Solution for task ", str(i))
        cur_result = solve(1, t["R"], t["Q"][i], t["C"][i])
        result.append(cur_result)
    assert result == t["expected_ans"]

print("All tests succeeded")
