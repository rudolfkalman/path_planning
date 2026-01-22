import numpy as np
a = np.array([1, 2])
b = np.array([1, 2, 0])
norm = np.linalg.norm(a, ord=0)
print(norm)
