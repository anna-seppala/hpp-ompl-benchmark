

#------------------------------------------------------------------
# Script to solve ompl problem in hpp:
# name = Abstract
# robot = Absrtact_robot
# world = Abstract_env
from math import pi
from hpp.corbaserver.hpp_ompl_benchmark.Abstract import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import Viewer

v = Viewer (ps)
v.loadObstacleModel("hpp-ompl-benchmark", "Abstract_env", "env")



q_init = [2.158, -4.9, -1.524, 1 , 0, 0, 0]
q_goal = [-3.07, -4, 0.305, 0.7071, 0.7071, 0, 0]
v(q_init)

robot.setJointBounds ('base_joint_xyz', [-5.92, 6.09, -11.9, 0.1, -5.64, 6.35])
# robot.setJointBounds ('base_joint_SO3', [-pi, pi, -pi, pi, -pi, pi, -pi, pi])
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.client.problem.selectPathValidation("Progressive",0.05)
ps.client.problem.clearPathOptimizers()


from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
ps.client.problem.setRandomSeed(1927402002)
benchmark.seedRange = range (50)
benchmark.iterPerCase = 1
results = benchmark.do()

benchmark.writeDatabase("dbHPPCont.db","abstract","abstractLogsCont.log",True)



