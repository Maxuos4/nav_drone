#include <nav_drone_core/planner.hpp>
#include <nav_drone_util/node_utils.hpp>
#include "nav_drone_a_star_planner/astar_planner.hpp"
#include <cmath>

namespace nav_drone_theta_star_planner
{
  class ThetaStarPlanner : public nav_drone_core::Planner
  {
    public:
      void configure(const rclcpp::Node::SharedPtr parent, 
                     std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                     std::shared_ptr<octomap::OcTree> costmap ) override
      {
           AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty, double greedy_penalty,
               double min_altitude, double max_altitude, double timeout_threshold, double max_waypoint_distance, bool unknown_is_occupied);

      }
      
      void updateMap(std::shared_ptr<octomap::OcTree> costmap) override
      {
        
      }
      
      nav_msgs::msg::Path createPlan( const geometry_msgs::msg::PoseStamped & start,
                                      const geometry_msgs::msg::PoseStamped & goal) override
      {            
        RCLCPP_DEBUG(logger_, "Requested AStar to plan a path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
        start.pose.position.x,
        start.pose.position.y,
        start.pose.position.z,
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.position.z);
        
        nav_msgs::msg::Path global_path;
        
        std::pair<std::vector<octomap::point3d>, PlanningResult> findPath(
            const octomap::point3d &start_coord, const octomap::point3d &goal_coord, const octomap::point3d &pos_cmd, std::shared_ptr<octomap::OcTree> mapping_tree,
            double timeout, std::function<void(const octomap::OcTree &)> visualizeTree,
            std::function<void(const std::unordered_set<Node, HashFunction> &, const std::unordered_set<Node, HashFunction> &, const octomap::OcTree &)>
                visualizeExpansions);
        
        return global_path;
      }
      
    protected:
      rclcpp::Node::SharedPtr node_;
      std::string plugin_name_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<octomap::OcTree> costmap_;
      rclcpp::Logger logger_ {rclcpp::get_logger("AStar")};
      rclcpp::Clock::SharedPtr clock_;
      
      float weight_;
  };

}  // namespace nav_drone_theta_star_planner

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav_drone_a_star_planner::AStarPlanner, nav_drone_core::Planner)
