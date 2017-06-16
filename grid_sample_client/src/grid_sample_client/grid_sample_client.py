import rospy
import actionlib
import copy 

from graspit_interface.msg import Grasp
from grid_sample_msgs.msg import GridSampleAction, GridSampleGoal


class GridSampleClient(object): 
    """
    Python interface for interacting with GraspIt
    """

    ROS_NODE_NAME = "GridSampleClient"
    GRASPIT_NODE_NAME = "/graspit/"

    @classmethod
    def computePreGrasps(cls, resolution=2, sampling_type=0):
                   
        try:
            rospy.init_node(cls.ROS_NODE_NAME, anonymous=True)
        except rospy.ROSException:
            pass

        client = actionlib.SimpleActionClient(GridSampleClient.GRASPIT_NODE_NAME + 'gridSample', GridSampleAction)
        
        client.wait_for_server(timeout=rospy.Duration(1.0))
        goal = GridSampleGoal(resolution, sampling_type)

        client.send_goal_and_wait(goal)

        return client.get_result()

    @classmethod
    def evaluatePreGrasps(cls, pre_grasps, pre_grasp_dofs=None):
        import graspit_commander
        gc = graspit_commander.GraspitCommander()

        grasps = []
        for i, pre_grasp in enumerate(pre_grasps):
            gc.toggleAllCollisions(False)
            if not pre_grasp_dofs:
                gc.forceRobotDof(pre_grasp.dofs)
            else:
                gc.forceRobotDof(pre_grasp_dofs)
            gc.setRobotPose(pre_grasp.pose)

            gc.toggleAllCollisions(True)
            gc.findInitialContact()
                                                                                                                                                                                 
            gc.autoGrasp()
            robot_state = gc.getRobot(0)
                                                                                                                                                                                 
            try:
                quality = gc.computeQuality()
                volume_quality = quality.volume
                epsilon_quality = quality.epsilon
            except:
                volume_quality = -1
                epsilon_quality = -1
        
            result = gc.getRobot(0)
            grasp = copy.deepcopy(pre_grasp)
            grasp.pose = robot_state.robot.pose
            grasp.volume_quality = volume_quality
            grasp.epsilon_quality = epsilon_quality
            grasp.dofs = robot_state.robot.dofs
            grasps.append(grasp)

        grasps = sorted(grasps, key=lambda x: x.volume_quality, reverse=True)
                
        return grasps

  
