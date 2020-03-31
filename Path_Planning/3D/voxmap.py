import numpy as np


def create_voxmap(data, voxel_size=5):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    alt_min = 0.0
    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min))) // voxel_size
    east_size = int(np.ceil((east_max - east_min))) // voxel_size
    alt_size = int(alt_max) // voxel_size

    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        obstacle = [
            int(north - north_min - d_north) // voxel_size,
            int(north - north_min + d_north) // voxel_size,
            int(east - east_min - d_east) // voxel_size,
            int(east - east_min + d_east) // voxel_size,
            int(alt - alt_min - d_alt) // voxel_size,
            int(alt - alt_min + d_alt) // voxel_size,
        ]
        voxmap[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1, obstacle[4]:obstacle[5]+1] = 1
        
    return voxmap