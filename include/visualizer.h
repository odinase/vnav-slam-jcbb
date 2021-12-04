#ifndef VISUALIZER_H
#define VISUALIZER_H


#include <ros/ros.h>


class VisualizerNode {
    public:
    VisualizerNode(const ros::NodeHandle& nh);
    private:
    ros::NodeHandle nh_;

};


#endif // VISUALIZER_H