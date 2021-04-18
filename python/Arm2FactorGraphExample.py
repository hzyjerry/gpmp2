# planar arm obstacle avoidance example, build factor graph in matlab
# @author Jing Dong
# @date Nov 16, 2015

from gtsam import *
from gpmp2 import *
from utils import *
import numpy as np


## small dataset
# dataset = generate2Ddataset('OneObstacleDataset')
dataset = generate2Ddataset('ZeroObstacleDataset')
rows = dataset.rows
cols = dataset.cols
cell_size = np.float64(dataset.cell_size)
# origin_point2 = Point2(dataset.origin_x, dataset.origin_y)
origin_point2 = Point2(0, 0)

# signed distance field
field = signedDistanceField2D(dataset.map, cell_size)
# field.fill(1.)
sdf = PlanarSDF(origin_point2, cell_size, field)

# plot sdf
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

# arm model
arm = generateArm('SimpleTwoLinksArm')

# GP
Qc = np.eye(2, dtype=np.float64)
Qc_model = noiseModel.Gaussian.Covariance(Qc) 

# Obstacle avoid settings
cost_sigma = np.float64(0.1)
epsilon_dist = np.float64(0.1)

# prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001)
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001)

# start and end conf
start_conf = np.array([0, 0], dtype=np.float64)
start_vel = np.array([0, 0], dtype=np.float64)
end_conf = np.array([np.pi/2, 0], dtype=np.float64)
end_vel = np.array([0, 0], dtype=np.float64)
avg_vel = (end_conf / total_time_step) / delta_t

# plot param
pause_time = total_time_sec / total_time_step

# # plot start / end configuration
# figure(1), hold on
# plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size)
# title('Layout')
# plotPlanarArm(arm.fk_model(), start_conf, 'b', 2)
# plotPlanarArm(arm.fk_model(), end_conf, 'r', 2)
# hold off


## init optimization
graph = NonlinearFactorGraph()
init_values = Values()

for i in range(0, total_time_step+1):
    key_pos = symbol('x', i)
    key_vel = symbol('v', i)
    
    # initialize as straight line in conf space
    pose = start_conf * (total_time_step-i)/total_time_step + end_conf * i/total_time_step
    vel = avg_vel
    init_values.insert(key_pos, pose)
    init_values.insert(key_vel, vel)
    
    # start/end priors
    if i==0:
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix))
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix))
    elif i == total_time_step:
        graph.add(PriorFactorVector(key_pos, end_conf, pose_fix))
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix))
    
    # GP priors and cost factor
    if i > 0:
        key_pos1 = symbol('x', i-1)
        key_pos2 = symbol('x', i)
        key_vel1 = symbol('v', i-1)
        key_vel2 = symbol('v', i)
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model))
        
        # cost factor
        graph.add(ObstaclePlanarSDFFactorArm(key_pos, arm, sdf, cost_sigma, epsilon_dist))
        
        # # GP cost factor
        # if use_GP_inter and check_inter > 0:
        #     for j in range(0, int(check_inter)):
        #         tau = (j + 1) * (total_time_sec / total_check_step)
        #         graph.add(ObstaclePlanarSDFFactorGPArm(
        #             key_pos1, key_vel1, key_pos2, key_vel2, 
        #             arm, sdf, cost_sigma, epsilon_dist,
        #             Qc_model, delta_t, tau))

# import pdb; pdb.set_trace()

# ## plot initial values
# for i=0:total_time_step
#     figure(3), hold on
#     title('Initial Values')
#     # plot world
#     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size)
#     # plot arm
#     conf = init_values.atVector(symbol('x', i))
#     plotPlanarArm(arm, conf, 'b', 2)
#     pause(pause_time), hold off
# end


## optimize!
use_trustregion_opt = True

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
#     plotPlanarArm(arm.fk_model(), conf, 'b', 2)
#     pause(pause_time), hold off
# end

