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

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf


#include "nav2_graph_planner/graph_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"


using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_graph_planner
{


GraphPlanner::GraphPlanner()
: tf_(nullptr)
{
  // graph_map_table_;
}

GraphPlanner::~GraphPlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type GraphPlanner",
    name_.c_str());
}

void
GraphPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap(); //받지만 사용하진 않음.
  global_frame_ = costmap_ros->getGlobalFrameID(); // costmap frame id 받아야하나?

  node_ = parent;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type GraphPlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);


  // subscriber declare -> graph_map_server 가 publish 하는  GraphMap.msg 기반으로 map 제작<pose, node_id>
  graph_map_sub_ = node->create_subscription<graph_map_msgs::msg::GraphMap>(
    "graph_map",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&GraphPlanner::mapReceived, this, std::placeholders::_1)
  );
}

void
GraphPlanner::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type GraphPlanner",
    name_.c_str());
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&GraphPlanner::dynamicParametersCallback, this, _1));
}

void
GraphPlanner::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type GraphPlanner",
    name_.c_str());
  dyn_params_handler_.reset();
}

void
GraphPlanner::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type GraphPlanner",
    name_.c_str());
  graph_map_sub_.reset(); //새로운 맵 들어올경우
  // planner_.reset();
}

nav_msgs::msg::Path GraphPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;

  // auto start_time = std::chrono::steady_clock::now();
  path.header.stamp=clock_->now();
  path.header.frame_id=global_frame_;

  /*
    여기다가 path 생성하는 코드 작성
    지정된 action에 의해서 start goal pose는 정해져있는 값이 들어올꺼임
    pose 를 이용한 unordered_map 에서 path 만들기 <pose_string, node_id>
    1. start와 goal 의 string(pose) 주고
    2. 받은 string(pose) 바탕으로 node id 검색
    3. node id 바탕으로 dijkstra 돌려
  */
  int start_node_id = getNodeId(start);
  int goal_node_id = getNodeId(goal);
  

  path = calcShortestPath(start_node_id, goal_node_id);

  return path;
}


rcl_interfaces::msg::SetParametersResult
GraphPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".tolerance") {
        tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".allow_unknown") {
        allow_unknown_ = parameter.as_bool();
      } 
    }
  }
  result.successful = true;
  return result;
}

std::map<std::string, int>
GraphPlanner::createGraphNodeTable(const graph_map_msgs::msg::GraphMap &msg) {
  std::map<std::string, int> table_;

  for (auto &node : msg.nodes) {

    std::string pos_x = std::to_string(node.pose.position.x);
    std::string pos_y = std::to_string(node.pose.position.y);
    std::string pos_z = std::to_string(node.pose.position.z);
    std::string orientaion_x = std::to_string(node.pose.orientation.x);
    std::string orientaion_y = std::to_string(node.pose.orientation.y);
    std::string orientaion_z = std::to_string(node.pose.orientation.z);
    std::string orientaion_w = std::to_string(node.pose.orientation.w);

    std::string key = pos_x + pos_y + pos_z + orientaion_x + orientaion_y +
                      orientaion_z + orientaion_w;

    table_.insert(std::pair<std::string, int>(key, node.id));
  }
  return table_;
}

std::vector<std::pair<int,int>>
GraphPlanner::createGraphEdgeTable(const graph_map_msgs::msg::GraphMap &msg){
  std::vector<std::pair<int,int>> table_;

  for(auto &edge :msg.edges){
    table_.push_back({edge.id_0,edge.id_1});
  }
  return table_;
}




void GraphPlanner::mapReceived(const graph_map_msgs::msg::GraphMap &msg) {
  // 넘겨줘서 맵만들기
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "GraphPlannerNode: A new graph map was received.");
  graph_node_table_ = createGraphNodeTable(msg);
  graph_edge_table_ = createGraphEdgeTable(msg);
}

int GraphPlanner::getNodeId(const geometry_msgs::msg::PoseStamped &msg) {
  int value_;

  std::string spos_x = std::to_string(msg.pose.position.x);
  std::string spos_y = std::to_string(msg.pose.position.y);
  std::string spos_z = std::to_string(msg.pose.position.z);
  std::string sorientaion_x = std::to_string(msg.pose.orientation.x);
  std::string sorientaion_y = std::to_string(msg.pose.orientation.y);
  std::string sorientaion_z = std::to_string(msg.pose.orientation.z);
  std::string sorientaion_w = std::to_string(msg.pose.orientation.w);

  std::string key = spos_x + spos_y + spos_z + sorientaion_x + sorientaion_y + sorientaion_z + sorientaion_w;

  value_ = graph_node_table_[key];

  return value_;
}

nav_msgs::msg::Path GraphPlanner::calcShortestPath(int start_node_id, int goal_node_id){
  nav_msgs::msg::Path shortest_path_;
  std::string start_pose;
  std::string goal_pose;
  std::vector<std::pair<double,double>> node_pos;



  // make node positon vector
  // find key form map value
  // http://devlab.neonkid.xyz/2019/08/09/c++/2019-08-09-C-Map/
  for(auto it=graph_node_table_.begin();it!=graph_node_table_.end();++it){
    // it->second  (int)node_id
    // it->first   (string)pose
    start_node_id++;
    goal_node_id++;

    std::string pos_x = it->first.substr(0,6); // first node x position from pose string
    std::string pos_y = it->first.substr(6,12); // first node y positin from pose string

    node_pos.push_back({std::stof(pos_x),std::stof(pos_y)});
  }

  /*
  node_pos 에 각 노드의 position(x,y)의 vector가 있음
  --------------------------------------------------
  https://www.javatpoint.com/cpp-dijkstra-algorithm-using-priority-queue
  1. 연결된 노드 사이에거리
  dijstra돌릴때 
  */
  int node_count = graph_node_table_.size();
  std::vector<Pair_> node_adj[100];

  //addEdge
  for (auto it=graph_edge_table_.begin();it!=graph_edge_table_.end();it++){
    int s = it -> first;
    int e = it -> second;

    // get 2 pos dist
    // http://wiki.ros.org/tf2_eigen
    // geometry_msgs/PointStamped tf2::Stamped<Eigen::Vector3d>  Eigen::Vector3d v(x,y,z)
    Eigen::Vector3d sv(node_pos[s].first, node_pos[s].second, 0);
    Eigen::Vector3d ev(node_pos[e].first, node_pos[e].second, 0);

    double dist = (sv - ev).norm();

    node_adj[s].push_back({e,dist});
  }

  //Dijstra Alogorithm
  std::priority_queue<Pair_, std::vector<Pair_>, std::greater<Pair_>> pq;

  const double INF = std::numeric_limits<double>::infinity();

  std::vector<double> dist(node_count, INF);
  int arr[1001];
  pq.push({0, start_node_id});
  dist[start_node_id] = 0;

  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();

    for (auto x : node_adj[u]) {
      int v = x.first;
      int weight = x.second;

      if (dist[v] > dist[u] + weight) {
        arr[v]=u;
        dist[v] = dist[u] + weight;
        pq.push({dist[v], v});
      }
    }
  }


  //push node path
  std::vector<int> nodeSeq;
  int temp = goal_node_id;
  while(temp){
    nodeSeq.push_back(temp);
    temp=arr[temp];
  }

  //nodeSeq 가 src 에서 goal 까지의 node들의 순서를 가짐.

  geometry_msgs::msg::PoseStamped pose;
  //nodeSeq -> shortest_path_
  for (auto node : nodeSeq) {
    for (auto it = graph_node_table_.begin(); it != graph_node_table_.end(); ++it) {
      if (node == it->second) {
        std::string pos_x = it->first.substr(0, 6); // first node x position from pose string
        std::string pos_y = it->first.substr(6, 12); // first node y positin from pose string
        // position of x   std::stof(pos_x)
        // position of y   std::stof(pos_y)
        pose.header.stamp=clock_->now();
        pose.pose.position.x = std::stof(pos_x);
        pose.pose.position.y = std::stof(pos_y);
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
      }
    }
  }
  shortest_path_.poses.push_back(pose);

  return shortest_path_;
}

}  // namespace nav2_graph_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_graph_planner::GraphPlanner, nav2_core::GlobalPlanner)
