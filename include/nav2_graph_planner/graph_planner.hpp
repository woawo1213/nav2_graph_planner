// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_GRAPH_PLANNER__GRAPH_PLANNER_HPP_
#define NAV2_GRAPH_PLANNER__GRAPH_PLANNER_HPP_

#include <algorithm>
#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

#include "graph_map_msgs/msg/graph_edge.hpp"
#include "graph_map_msgs/msg/graph_map.hpp"
#include "graph_map_msgs/msg/graph_node.hpp"

namespace nav2_graph_planner
{

class GraphPlanner : public nav2_core::GlobalPlanner
{
public:
  GraphPlanner();

  ~GraphPlanner();

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

protected:
  // Planner based on GraphMap

  graph_map_msgs::msg::GraphMap map_msg_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("GraphPlanner")};

  // Global Costmap
  nav2_costmap_2d::Costmap2D *costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  // Whether or not the planner should be allowed to plan through unknown space
  bool allow_unknown_;

  // If the goal is obstructed, the tolerance specifies how many meters the
  // planner can relax the constraint in x and y before failing
  double tolerance_;

  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      dyn_params_handler_;

  // GraphMap subscriber
  rclcpp::Subscription<graph_map_msgs::msg::GraphMap>::SharedPtr graph_map_sub_;

  // create map table
  std::map<std::string, int>
  createGraphNodeTable(const graph_map_msgs::msg::GraphMap &msg);

  std::vector<std::pair<int,int>>
  createGraphEdgeTable(const graph_map_msgs::msg::GraphMap &msg);

  // GraphMap Callback
  void mapReceived(const graph_map_msgs::msg::GraphMap &msg);

  // Get Node Id
  int getNodeId(const geometry_msgs::msg::PoseStamped &msg);

  // Graphmap table
  std::map<std::string, int> graph_node_table_;
  std::vector<std::pair<int,int>> graph_edge_table_;

  // Callback Detect
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dijkstra
  // Infinite


  // Pair ==> double Pair
  using Pair_ = std::pair<int, double>;

  // To add an edge
  void addEdge(std::vector<Pair_> adj[], int u, int v,
               double dist);

  // Calculate shortest path from src to other vertices
  nav_msgs::msg::Path
  calcShortestPath(int V, int src);
};



}  // namespace nav2_graph_planner

#endif  // NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_
