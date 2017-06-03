#include "grid_sample_node.h"

#include "grid_sampler.h"

//GraspIt! includes
#include "graspit_source/include/EGPlanner/searchState.h"
#include "graspit_source/include/EGPlanner/simAnnPlanner.h"

//Message includes
#include <graspit_interface/Grasp.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include "graspit_source/include/graspitCore.h"
#include "graspit_source/include/robot.h"
#include "graspit_source/include/world.h"
#include "graspit_source/include/ivmgr.h"

#include "graspit_source/include/quality.h"
#include "graspit_source/include/grasp.h"
#include "graspit_source/include/EGPlanner/searchState.h"
#include "graspit_source/include/EGPlanner/egPlanner.h"
#include "graspit_source/include/EGPlanner/simAnnPlanner.h"
#include "graspit_source/include/EGPlanner/guidedPlanner.h"

#include "cmdline/cmdline.h"


namespace GridSamplerNode
{

int GridSamplerNode::init(int argc, char** argv)
{
    ros::init(argc, argv, "graspit_grid_sample_node");


    const std::string node_name_help =
            "print Ros Node name\n";

    cmdline::parser* parser = new cmdline::parser();

    parser->add<std::string>("node_name", 'n', node_name_help,  false);
    parser->parse(argc, argv);

    std::string node_name = "";

    if(parser->exist("node_name")){
        node_name = parser->get<std::string>("node_name");
    }

    nh = new ros::NodeHandle(node_name);

    gridSampleActionServer = new actionlib::SimpleActionServer<grid_sample_msgs::GridSampleAction>(*nh, "gridSample",
                                                                                            boost::bind(&GridSamplerNode::gridSampleCB, this, _1), false);
    gridSampleActionServer->start();

    ROS_INFO("GraspIt Grid Sampler Node initialized!");

    return 0;
}

int GridSamplerNode::mainLoop()
{
    ros::spinOnce();
    return 0;
} 


void GridSamplerNode::gridSampleCB(const grid_sample_msgs::GridSampleGoalConstPtr &goal)
{
      grid_sample_msgs::GridSampleResult result;

      Hand* mHand = graspitCore->getWorld()->getHand(0);
      GraspableBody *gb = graspitCore->getWorld()->getGB(0);

      // EllipseSampler * sampler = new EllipseSampler(mHand, gb, goal->resolution);
      AboveSampler * sampler = new AboveSampler(mHand, gb, goal->resolution);
      //BoxGridSampler * sampler = new BoxGridSampler(mHand, gb, goal->resolution);
      sampler->sample();

      std::vector<GraspPlanningState*> *samples = sampler->getResults();

      for(int i = 0; i < samples->size(); i++)
      {
          GraspPlanningState *gps = samples->at(i);
        gps->execute(mHand);

        geometry_msgs::Pose pose;
        transf t = mHand->getTran();

        std::cout << "t: " << t << std::endl;
        pose.position.x = t.translation().x() / 1000.0;
        pose.position.y = t.translation().y() / 1000.0;;
        pose.position.z = t.translation().z() / 1000.0;;
        pose.orientation.w = t.rotation().w();
        pose.orientation.x = t.rotation().x();
        pose.orientation.y = t.rotation().y();
        pose.orientation.z = t.rotation().z();

        graspit_interface::Grasp g;

        double dof[mHand->getNumDOF()];
        mHand->getDOFVals(dof);
        for(int i = 0; i <mHand->getNumDOF(); ++i)
        {
            g.dofs.push_back(dof[i]);
        }

        g.pose = pose;

        result.grasps.push_back(g);
    }

    if(sampler != NULL)
    {
        delete sampler;
        sampler = NULL;
    }

    gridSampleActionServer->setSucceeded(result);
}

}
