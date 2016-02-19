# Script to solve ompl problem in hpp:
# name = Abstract
# robot = Absrtact_robot
# world = Abstract_env

from hpp.corbaserver.hpp_ompl_benchmark.Abstract import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import ViewerFactory
r = ViewerFactory (ps)
r.loadObstacleModel("hpp-ompl-benchmark", "Abstract_env", "env")
r (robot.getCurrentConfig ())
v = r.createRealClient ()

q_init = robot.getCurrentConfig ()
q_init = [0.8498, -0.600, 1.8016, 1 , 0, 0, 0]
# q_init = [2.1, -5.81216, -1, 1 , 0, 0, 0]; v(q_init); robot.isConfigValid(q_init)
# robot.client.obstacle.moveObstacle('env', [0,0,0,1,0,0,0])
q_goal = [-1.2102, 0.120, 1.5316, 0.7071, 0.7071, 0, 0]

offset = [1.2502, -5.212160000000001, -2.8016]
q_goal = [-1.2102+offset[0], 0.120+offset[1], 1.5316+offset[2], 0.7071, 0.7071, 0, 0]

robot.setJointBounds ([-2.33119232178, 2.29919021606, -2.22197250366, 2.5073979187, -0.0394512939453, 4.68982696533])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# ps.selectPathValidation ("Dichotomy", 0.)
# ps.addPathOptimizer ("RandomShortcut")


from hpp.corbaserver.hpp_ompl_benchmark.cubicles import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import ViewerFactory
r = ViewerFactory (ps)
r.loadObstacleModel("hpp-ompl-benchmark", "cubicles_env", "env")
r (robot.getCurrentConfig ())
v = r.createRealClient ()

q_init = robot.getCurrentConfig ()
q_init = [-0.0496, -0.4062, 0.7057, 1 , 0, 0, 0]
# q_init = [2.1, -5.81216, -1, 1 , 0, 0, 0]; v(q_init); robot.isConfigValid(q_init)
# robot.client.obstacle.moveObstacle('env', [0,0,0,1,0,0,0])
q_goal = [2.0, -0.4062, 0.7057, 1, 0, 0, 0]

#offset = [1.2502, -5.212160000000001, -2.8016]
#q_goal = [-1.2102+offset[0], 0.120+offset[1], 1.5316+offset[2], 0.7071, 0.7071, 0, 0]

robot.setJointBounds ([-5.0888, 3.1962, -2.3013, 5.3187, -1.2375, 1.01])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

