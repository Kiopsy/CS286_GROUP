# NOTE: MRTA_2a.py must be in the same folder as this file to run tests
from MRTA_2a import solve

test1 = {
    "N": 4,

    "R": 4,

    "Q": [[5, 8, 9, 4],
        [13, 7, 28, 13], 
        [3, 14, 6, 41], 
        [14, 4, 5, 12]],

    "C":   [[2, 4, 5, 0],
            [12, 4, 24, 9], 
            [0, 12, 3, 37], 
            [10, 0, 1, 8]],
    "expected_ans": [(1, 0, 4), (2, 1, 4), (3, 2, 4), (0, 3, 4)]
}

test2 = {
    "N": 2,

    "R": 3,

    "Q": [[0, 0, 0], [0, 0, 0]],

    "C": [[1, 2, 3], [4, 5, 6]],

    # all utility is negative, so no actions are taken
    "expected_ans": []
}

test3 = {

    "N": 2,

    "R": 2,

    "Q": [[2, 10], [1, 11]],

    "C": [[0, 0], [0, 0]],

    # if it was greedy, it would chose 10 first, but it should not
    "expected_ans": [(0, 0, 2), (1, 1, 11)]

}


tests = [test1, test2, test3]

for t in tests:
    answer = solve(t["N"], t["R"], t["Q"], t["C"])
    assert answer == t["expected_ans"]

print("All tests succeeded")
