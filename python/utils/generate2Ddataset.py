import numpy as np
from dotmap import DotMap


def generate2Ddataset(dataset_str):
    #GENERATE2DDATASET Generate 2D dataset evidence grid
    #
    #   Usage: dataset = GENERATE2DDATASET(dataset_str)
    #   @dataset_str       dataset string, existing datasets:
    #                      'OneObstacleDataset', 'TwoObstaclesDataset'
    #
    #   Output Format:
    #   dataset.map        ground truth evidence grid
    #   dataset.rows       number of rows (y)
    #   dataset.cols       number of cols (x)
    #   dataset.origin_x   origin of map x
    #   dataset.origin_y   origin of map y
    #   dataset.cell_size  cell size

    dataset = DotMap()


    # dataset 0: 0 obs dataset for 2D Arm obs avoid
    if dataset_str == 'ZeroObstacleDataset':
        # params
        dataset.cols = 300
        dataset.rows = 300
        dataset.origin_x = -1
        dataset.origin_y = -1
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))

    # dataset 5: 1 obs dataset for 2D Arm obs avoid
    elif dataset_str == 'OneObstacleDataset':
        # params
        dataset.cols = 300
        dataset.rows = 300
        dataset.origin_x = -1
        dataset.origin_y = -1
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle([190, 160], [60, 80], dataset.map)
        
    # dataset 6: obs dataset for 3D Arm obs avoid
    elif dataset_str == 'TwoObstaclesDataset':
        # params
        dataset.cols = 300
        dataset.rows = 300
        dataset.origin_x = -1
        dataset.origin_y = -1
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle([200, 200], [80, 100], dataset.map)
        dataset.map = add_obstacle([160, 80], [30, 80], dataset.map)
        
    # dataset 7: multiple obs dataset for 2D Arm obs avoid
    elif dataset_str == 'MultiObstacleDataset':
        # params
        dataset.cols = 400 #x
        dataset.rows = 300 #y
        dataset.origin_x = -20
        dataset.origin_y = -10
        dataset.cell_size = 0.1
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle(get_center(12,10,dataset), get_dim(5,7,dataset), dataset.map)
        dataset.map = add_obstacle(get_center(-7,10,dataset), get_dim(10,7,dataset), dataset.map)
        dataset.map = add_obstacle(get_center(0,-5,dataset), get_dim(10,5,dataset), dataset.map)

    # mobile 2d map
    elif dataset_str == 'MobileMap1':
        # params
        dataset.cols = 500 #x
        dataset.rows = 500 #y
        dataset.origin_x = -5
        dataset.origin_y = -5
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle(get_center(0,0,dataset), get_dim(1,5,dataset), dataset.map)
        #     dataset.map = add_obstacle(get_center(-2.5,-2,dataset), get_dim(5,1,dataset), dataset.map)
        # wall
        dataset.map = add_obstacle(get_center(0,4.5,dataset), get_dim(10,1,dataset), dataset.map)
        dataset.map = add_obstacle(get_center(0,-4.5,dataset), get_dim(10,1,dataset), dataset.map)
        dataset.map = add_obstacle(get_center(4.5,0,dataset), get_dim(1,10,dataset), dataset.map)
        dataset.map = add_obstacle(get_center(-4.5,0,dataset), get_dim(1,10,dataset), dataset.map)

        
    # no such dataset
    else:
        raise NotImplementedError('No such dataset exist')

    return dataset

def add_obstacle(position, size, map, landmarks=None, origin_x=None, origin_y=None, cell_size=None):

    half_size_row = np.floor((size[0]-1)/2)
    half_size_col = np.floor((size[1]-1)/2)

    idx_a = int(position[0]-half_size_row) 
    idx_b = int(position[0] +half_size_row)
    idy_a = int(position[1]-half_size_col)
    idy_b = int(position[1] +half_size_col)
    dx = int(2*half_size_row+1)
    dy = int(2*half_size_col+1)
    # occupency grid
    map[idx_a: idx_b + 1, idy_a: idy_b + 1] = np.ones((dx, dy)) 

    # landmarks
    if landmarks is not None:
        x1 = position[0]-half_size_row-1
        x2 = position[0]+half_size_row-1
        for x in range(x1, x2 + 1, 4):
            y = position[1]-half_size_col-1
            landmarks = np.stack((landmarks, [origin_y+y*cell_size, origin_x+x*cell_size]))
            y = position[1]+half_size_col-1
            landmarks = np.stack((landmarks, [origin_y+y*cell_size, origin_x+x*cell_size]))

        y1,= position[1]-half_size_col+3
        y2 = position[1]+half_size_col-5
        for y in range(y1, y2 + 1, 4):
            x = position[0]-half_size_row-1
            landmarks = np.stack((landmarks, [origin_y+y*cell_size, origin_x+x*cell_size]))
            x = position[0]+half_size_row-1
            landmarks = np.stack((landmarks, [origin_y+y*cell_size, origin_x+x*cell_size]))

        return map, landmarks
    else:
        return map


def get_center(x,y,dataset):
    center = np.array([y - dataset.origin_y, x - dataset.origin_x])/dataset.cell_size;
    return center

def get_dim(w,h,dataset):
    dim = np.array([h, w])/dataset.cell_size
    return dim