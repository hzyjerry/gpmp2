import numpy as np
from scipy.ndimage.morphology import distance_transform_edt

def signedDistanceField2D(ground_truth_map, cell_size):
	#SIGNEDDISTANCEFIELD2D 2D signed distance field
	#   Given a ground truth 2D map defined in Matrix in 0-1,
	#   calculate 2D signed distance field, which is defined as a matrix
	#   map matrix and signed distance field matrix have the same resolution.
	#
	#   Usage: field = SIGNEDDISTANCEFIELD2D(ground_truth_map, cell_siz)
	#   @map        evidence grid from dataset, map use 0 show open area, 1 show objects. 
	#   @cell_size  cell sizeto given metric information
	#
	#   Output: 
	#   @field      sdf, row is Y, col is X


	# regularize unknow area to open area
	map = (ground_truth_map > 0.75)
	# inverse map
	inv_map = 1 - map

	# get signed distance from map and inverse map
	map_dist = distance_transform_edt(np.logical_not(map))
	inv_map_dist = distance_transform_edt(np.logical_not(inv_map))

	field = map_dist - inv_map_dist

	# metric
	field = field * cell_size
	field = np.array(field, dtype=np.float64)

	# limit inf
	if np.any(np.isinf(field[0, 0])):
		print("Got inf sdf")
		field = np.ones_like(field) * 1000

	return field