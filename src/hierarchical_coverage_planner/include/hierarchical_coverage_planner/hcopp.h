#ifndef _HCOPP_H_
#define _HCOPP_H_

#include <active_perception/perception_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hierarchical_coverage_planner/hcplanner.h>
#include <hierarchical_coverage_planner/hctraj.h>

using namespace std;
using std::unique_ptr;

namespace predrecon {

struct PointCluster {
    double x, y, z;
    int cluster;
};

class PerceptionUtils;

class HCOPP {
  public:
    HCOPP();
    ~HCOPP();
    /* Func */
    void init(ros::NodeHandle &nh);
    void ExecPlan();
    void triggerCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg);
    void flightCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    /* Utils */
    unique_ptr<hierarchical_coverage_planner> PathPlanner_;
    unique_ptr<TrajGenerator> TrajGen_;
    unique_ptr<PerceptionUtils> percep_utils_;
    unique_ptr<PlanningVisualization> VisUtils_;
    ros::Subscriber trigger_sub;
    ros::Subscriber flight_sub;
    /* Param */
    ros::NodeHandle HCOPPnh;
    double compT, freq;
};

} // namespace predrecon

#endif