
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
ps.client.problem.selectPathValidation("Dichotomy",0.05)
ps.client.problem.clearPathOptimizers()

v(q_init)
from hpp.corbaserver import Benchmark
benchmark = Benchmark (robot.client, robot, ps)
#ps.client.problem.setRandomSeed(3530408688)
benchmark.seedRange = range (50)
benchmark.iterPerCase = 1
results = benchmark.do()

benchmark.writeDatabase("dbHPPDicho.db","pipedream","pipedreamLogsDicho.log",True)



