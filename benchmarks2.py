

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

from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
#ps.client.problem.setRandomSeed(3530408688)
benchmark.seedRange = range (50)
benchmark.iterPerCase = 1
results = benchmark.do()

benchmark.writeDatabase("dbHPP.db","cubicles","cubiclesLogs.log")
#------------------------------------------------------------------
# name = pipedream
# robot = ring
# world = pipedream_env
from math import pi
from hpp.corbaserver.hpp_ompl_benchmark.pipedream import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import Viewer

v = Viewer (ps)
v.loadObstacleModel("hpp-ompl-benchmark", "pipedream_env", "env")

q_init = [2.02,-0.0912,0.786,0.707,0.707,0,0]
q_goal = [1.587,-0.345,0.634,1,0,0,0]

robot.setJointBounds ('base_joint_xyz', [1.08, 2.43, -0.824, 0.28, 0.3, 1.7])
# robot.setJointBounds ('base_joint_SO3', [-pi, pi, -pi, pi, -pi, pi, -pi, pi])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
v(q_init)
from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
#ps.client.problem.setRandomSeed(3530408688)
benchmark.seedRange = range (50)
benchmark.iterPerCase = 1
results = benchmark.do()

benchmark.writeDatabase("dbHPP.db","pipedream","pipedreamLogs.log",True)


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

from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
ps.client.problem.setRandomSeed(1927402002)
benchmark.seedRange = range (50)
benchmark.iterPerCase = 1
results = benchmark.do()

benchmark.writeDatabase("dbHPP.db","abstract","abstractLogs.log",True)


