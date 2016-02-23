# Script to solve ompl problem in hpp:
# name = Abstract
# robot = Absrtact_robot
# world = Abstract_env
from math import pi
from hpp.corbaserver.hpp_ompl_benchmark.Abstract import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import ViewerFactory

r = ViewerFactory (ps)
r.loadObstacleModel("hpp-ompl-benchmark", "Abstract_env", "env")
r (robot.getCurrentConfig ())
v = r.createRealClient ()

q_init = [2.1, -5.81216, -1, 1 , 0, 0, 0]
q_goal = [-3, -4.3, 1, 0.7071, 0.7071, 0, 0]

robot.setJointBounds ('base_joint_xyz', [-5.5, 6.5, -12, 0, -5, 6])
# robot.setJointBounds ('base_joint_SO3', [-pi, pi, -pi, pi, -pi, pi, -pi, pi])
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
benchmark.seedRange = range (3)
benchmark.iterPerCase = 3 
results = benchmark.do()

#------------------------------------------------------------------
# name = cubicles
# robot = cubicles_robot
# world = cubicles_env
from math import pi
from hpp.corbaserver.hpp_ompl_benchmark.cubicles import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import ViewerFactory

r = ViewerFactory (ps)
r.loadObstacleModel("hpp-ompl-benchmark", "cubicles_env", "env")
r (robot.getCurrentConfig ())
v = r.createRealClient ()

q_init = [0, 0, 0, 1, 0, 0, 0]
q_goal = [5.3, 0.5, 0, 1, 0, 0, 0]
#q_init = [-0.0496, -0.4062, 0.7057, 1 , 0, 0, 0]
#q_goal = [2.0, -0.4062, 0.7057, 1, 0, 0, 0]

robot.setJointBounds ('base_joint_xyz', [-12, 8, -0.5, 1.5, -5, 13])
#robot.setJointBounds ('base_joint_SO3', [-pi, pi, -pi, pi, -pi, pi, -pi, pi])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
benchmark.seedRange = range (10)
benchmark.iterPerCase = 10 
results = benchmark.do()

#------------------------------------------------------------------
# name = pipedream
# robot = ring
# world = pipedream_env
from math import pi
from hpp.corbaserver.hpp_ompl_benchmark.pipedream import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import ViewerFactory

r = ViewerFactory (ps)
r.loadObstacleModel("hpp-ompl-benchmark", "pipedream_env", "env")
r (robot.getCurrentConfig ())
v = r.createRealClient ()

q_init = [2,-0,0.75,0.707,0.707,0,0]
q_goal = [1.6,-0.4,0.7,0,1,0,0]

robot.setJointBounds ('base_joint_xyz', [1, 2.5, -1, 0.5, 0, 1.7])
# robot.setJointBounds ('base_joint_SO3', [-pi, pi, -pi, pi, -pi, pi, -pi, pi])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
