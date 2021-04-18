# @brief    Point Robot 2D example, building factor graph in matlab
# @author   Mustafa Mukadam
# @date     July 20, 2016

## Load libraries
from gtsam import *
from gpmp2 import *
from utils import *
import numpy as np

## small dataset
# dataset = generate2Ddataset('MultiObstacleDataset')
dataset = generate2Ddataset('OneObstacleDataset')
rows = dataset.rows
cols = dataset.cols
cell_size = dataset.cell_size
origin_point2 = Point2(dataset.origin_x, dataset.origin_y)

# signed distance field
field = signedDistanceField2D(dataset.map, cell_size)
sdf = PlanarSDF(origin_point2, cell_size, field)

# # plot sdf
# figure(2)
# plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size)
# title('Signed Distance Field')


## settings
total_time_sec = 5.0
total_time_step = 10
total_check_step = 50
delta_t = total_time_sec / total_time_step
check_inter = total_check_step / total_time_step - 1

# use GP interpolation
use_GP_inter = True

# point robot model
pR = PointRobot(2,1)
spheres_data = np.array([0,  0.0,  0.0,  0.0,  1.5])
nr_body = spheres_data.shape[0]
sphere_vec = BodySphereVector()
sphere_vec.push_back(BodySphere(int(spheres_data[0]), spheres_data[4], Point3(spheres_data[1:4])))
pR_model = PointRobotModel(pR, sphere_vec)

# GP
Qc = np.eye(2)
Qc_model = noiseModel.Gaussian.Covariance(Qc) 

# obstacle cost settings
cost_sigma = 0.3
epsilon_dist = 2

# prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001)
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001)

# start and  conf
start_conf = np.array([-15, -8], dtype=np.float64)
start_vel = np.array([0, 0], dtype=np.float64)
_conf = np.array([17, 14], dtype=np.float64)
_vel = np.array([0, 0], dtype=np.float64)
avg_vel = (_conf / total_time_step) / delta_t

# plot param
pause_time = total_time_sec / total_time_step

# # plot start /  configuration
# figure(1), hold on
# plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size)
# plotPointRobot2D(pR_model, start_conf)
# plotPointRobot2D(pR_model, _conf)
# title('Layout')
# hold off


## init optimization
graph = NonlinearFactorGraph()
init_values = Values()

for i in range(0, total_time_step + 1):
    key_pos = symbol('x', i)
    key_vel = symbol('v', i)
    
    # initial values: straght line
    pose = start_conf * (total_time_step-i)/total_time_step + _conf * i/total_time_step
    vel = avg_vel
    init_values.insert(key_pos, pose)
    init_values.insert(key_vel, vel)
    
    # priors
    if i==0:
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix))
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix))
    elif i==total_time_step:
        graph.add(PriorFactorVector(key_pos, _conf, pose_fix))
        graph.add(PriorFactorVector(key_vel, _vel, vel_fix))
    
    
    # GP priors and cost factor
    if i > 0:
        key_pos1 = symbol('x', i-1)
        key_pos2 = symbol('x', i)
        key_vel1 = symbol('v', i-1)
        key_vel2 = symbol('v', i)
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model))
        
        # cost factor
        graph.add(ObstaclePlanarSDFFactorPointRobot(key_pos, pR_model, sdf, cost_sigma, epsilon_dist))
        
        # GP cost factor
        if use_GP_inter & int(check_inter) > 0:
            for j in range(1, int(check_inter) + 1):
                tau = j * (total_time_sec / total_check_step)
                graph.add(ObstaclePlanarSDFFactorGPPointRobot(key_pos1, key_vel1, key_pos2, key_vel2,pR_model, sdf, cost_sigma, epsilon_dist, Qc_model, delta_t, tau))
            
        
    

## optimize!
use_trustregion_opt = False

if use_trustregion_opt:
    parameters = DoglegParams()
    parameters.setVerbosity('ERROR')
    optimizer = DoglegOptimizer(graph, init_values, parameters)
else:
    parameters = GaussNewtonParams()
    parameters.setVerbosity('ERROR')
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

optimizer.optimize()
result = optimizer.values()
# result.print('Final results')


# ## plot final values
# for i=0:total_time_step
#     figure(4), hold on
#     title('Optimized Values')
#     # plot world
#     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size)
#     # plot arm
#     conf = result.atVector(symbol('x', i))
#     plotPointRobot2D(pR_model, conf)
#     pause(pause_time), hold off

