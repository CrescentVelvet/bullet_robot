import numpy as np
import scipy.sparse.linalg
A = np.array([[2, 1, 0, 5], [1, 2, 1, 2], [0, 1, 2, 4], [1, 3, 6, 4.5]])
B = np.array([9, 10, -2, 3])
coef_tmp1 = np.linalg.solve(A, B)
coef_tmp2 = scipy.sparse.linalg.spilu(A).solve(B) # LU三角分解
print(coef_tmp1)
print(coef_tmp2)
# [ 3.83333333,  4.77777778, -2.01111111, -0.68888889]
# [ 3.83333333,  4.77777778, -2.01111111, -0.68888889]

# class A:
#     def __init__(self):
#         self.c = 0
#         self.d = 0
#     def __init__(self, a, b):
#         self.c = a
#         self.d = b
#         self.ppp()
#     def ppp(self):
#         print(111)
# aaa = A(3,4)
# print(aaa.c, aaa.d)