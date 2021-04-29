import numpy as np

i_coords, j_coords = np.meshgrid(range(5), range(4), indexing='ij')

coordinate_grid = np.array([i_coords, j_coords])

for i in range(5):
    for j in range(4):
        print 4*coordinate_grid[:, i, j]