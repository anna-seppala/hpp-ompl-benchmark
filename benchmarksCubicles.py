

# name = cubicles
# robot = cubicles_robot
# world = cubicles_env
from math import pi
from hpp.corbaserver.hpp_ompl_benchmark.cubicles import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import Viewer

v = Viewer (ps)
v.loadObstacleModel("hpp-ompl-benchmark", "cubicles_env", "env")



q_init = [-0.126, -1.792, -1.0317, 1, 0, 0, 0]
q_goal = [5.08, -1.792, -1.0317, 1, 0, 0, 0]
#q_init = [-0.0496, -0.4062, 0.7057, 1 , 0, 0, 0]
#q_goal = [2.0, -0.4062, 0.7057, 1, 0, 0, 0]

robot.setJointBounds ('base_joint_xyz', [-12.92, 8.11, -2.565, 3.14, -5.845, 13.5])
#robot.setJointBounds ('base_joint_SO3', [-pi, pi, -pi, pi, -pi, pi, -pi, pi])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.client.problem.selectPathValidation("Dichotomy",0.05)
ps.client.problem.clearPathOptimizers()

from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
#ps.client.problem.setRandomSeed(3530408688)
benchmark.seedRange = range (50)
benchmark.iterPerCase = 1
results = benchmark.do()

benchmark.writeDatabase("dbHPPDicho.db","cubicles","cubiclesLogsDicho.log")


