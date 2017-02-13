#ifndef _GRASPIT_INTERFACE_H_
#define _GRASPIT_INTERFACE_H_ 

#include <ros/ros.h>
#include <graspit_source/include/plugin.h>
#include <actionlib/server/simple_action_server.h>

// ActionServer includes
#include <grid_sample_plugin/GridSampleAction.h>

namespace GridSamplerNode
{

class GridSamplerNode : public Plugin
{

private:
  ros::NodeHandle *nh;

  // ActionServer declarations
  actionlib::SimpleActionServer<grid_sample_plugin::GridSampleAction> *gridSampleActionServer;

  //ActionServer callbacks
  void gridSampleCB(const grid_sample_plugin::GridSampleGoalConstPtr &goal);

public: 
  GridSamplerNode(){}
  ~GridSamplerNode(){}

  virtual int init(int argc, char **argv);

  virtual int mainLoop();

};

}


#endif
