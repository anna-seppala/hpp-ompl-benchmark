

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




q_init = [-4.96, -70.57, -40.62, 1, 0, 0, 0]
q_goal = [200, -70.57, -40.62,1, 0, 0, 0]
#q_init = [-0.0496, -0.4062, 0.7057, 1 , 0, 0, 0]
#q_goal = [2.0, -0.4062, 0.7057, 1, 0, 0, 0]

robot.setJointBounds ('base_joint_xyz', [-508.88, 319.62, -101, 123.75, -230.13, 1531.87])
#robot.setJointBounds ('base_joint_SO3', [-pi, pi, -pi, pi, -pi, pi, -pi, pi])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.client.problem.selectPathValidation("Progressive",2)
ps.client.problem.clearPathOptimizers()


ps.client.problem.setRandomSeed(2105460788)

from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
#ps.client.problem.setRandomSeed(353040868)
benchmark.seedRange = range (50)
benchmark.iterPerCase = 1
results = benchmark.do()

benchmark.writeDatabase("temp.db","cubicles","cubiclesLogsCENTERCont.log")


