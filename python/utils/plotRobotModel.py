import matplotlib.pyplot as plt
import numpy as np

def sphere(N):
	N = 20
	thetavec = np.linspace(0, np.pi, N)
	phivec = np.linspace(0, 2*np.pi, 2*N)
	th, ph = np.meshgrid(thetavec, phivec)
	R = np.ones_like(th)

	x = R * np.sin(th) * np.cos(ph)
	y = R * np.sin(th) * np.sin(ph)
	z = R * np.cos(th)
	return x, y, z


def plotRobotModel(robot, conf, color_rgb=[0.4 0.4 0.4]):
	"""plotRobotModel Plot RobotModel class in 3D, visualize the body spheres
	   also it can plot any child class of RobotModelm like ArmModel
	
	   	Usage: plotRobotModel(robot, conf, color_rgb)
	   	@robot      RobotModel(or child) object
	   	@conf       robot configuration vector
	   	@color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]
	"""

	# points
	body_points = robot.sphereCentersMat(conf)

	# show
	# colormap(color_rgb)

	X_ball, Y_ball, Z_ball = sphere(16)

	fig = plt.figure()
	ax = fig.gca(projection='3d')

	for i in range(1:robot.nr_body_spheres()):
		X = X_ball * robot.sphere_radius(i-1) + body_points(1, i)
		Y = Y_ball * robot.sphere_radius(i-1) + body_points(2, i)
		Z = Z_ball * robot.sphere_radius(i-1) + body_points(3, i)
		ax.plot_surface(X, Y, Z)

	#Show the plot
	plt.show()