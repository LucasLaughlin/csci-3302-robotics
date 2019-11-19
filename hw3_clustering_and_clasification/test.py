import numpy as np
x=[4, 3, 5, 6]
y=[0, 1, 2, 3,2, 5, 6, 7, 8, 9, 10]

z=[y[i] for i in x]
print(z)


X=[(9,2), (3, 2), (6,2)]

print(np.average(X, axis=0))

x=[(2,3), (4, 5)]
y=[(2,2), (3,4)]

print(x[0]-y[0])