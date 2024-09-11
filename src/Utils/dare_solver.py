import numpy as np
from scipy.linalg import solve_discrete_are

# A = np.array([[0, 1], [-2, -3]])
# B = np.array([[0], [1]])
# Q = np.array([[2, 0], [0, 1]])
# R = np.array([[1]])

# P = solve_discrete_are(A, B, Q, R)
# print(P)

dt = 0.008

A = np.eye(12)
B = np.zeros((12,6))
Q = np.zeros((12,12))
R = np.zeros((6,6))

A_block = np.array([[1, dt], [0, 1]])
B_block = np.array([[dt*dt / 2, dt]])
for i in range(6):
    A[i*2:i*2 + 2, i*2:i*2 + 2]= A_block
    B[i*2:i*2 + 2, i] = B_block
    Q[i*2, i*2] = 1
    R[i,i] = 1

print(A)
print(B)
print(Q)
print(R)

P = solve_discrete_are(A, B, Q, R)
K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
print(np.round(P, 4))
print()
print(np.round(K, 4))
print(K.shape)