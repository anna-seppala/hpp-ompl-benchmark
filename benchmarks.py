# Script to solve ompl problem in hpp:
# name = Abstract
# robot = Absrtact_robot
# world = Abstract_env

from hpp.corbaserver.hpp_ompl_benchmark import Robot
robot = Robot('ompl')
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import ViewerFactory
r = ViewerFactory (ps)
r.loadObstacleModel("hpp-ompl-benchmark", "Abstract_env", "env")
r (robot.getCurrentConfig ())
v = r.createRealClient ()

q_init = robot.getCurrentConfig ()
q_init = [84.98, -60.0, 180.16, 1 , 0, 0, 0]

q_goal = [-121.02, 12.0, 153.16, 0.7071, 0.7071, 0, 0]

robot.setJointBounds ([-233.119232178, 229.919021606, -222.197250366, 250.73979187, -3.94512939453, 468.982696533])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# ps.selectPathValidation ("Dichotomy", 0.)
# ps.addPathOptimizer ("RandomShortcut")
