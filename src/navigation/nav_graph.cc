//
// Created by amanda on 11/12/20.
//

#include <navigation/nav_graph.h>
#include <navigation/simple_queue.h>
#include <navigation/navigation.cc>
#include <navigation/navigation.h>

namespace nav_graph {

    std::pair<std::pair<int32_t, int32_t>, int8_t> NavGraphNode::convertToKeyForm() const {
        int32_t key_form_x = round(node_pos_.x() / kGridResolution);
        int32_t key_form_y = round(node_pos_.y() / kGridResolution);
        int8_t key_form_angle = round(node_orientation_ / kAngularOptionsFromNavGraph);

        std::pair<int32_t, int32_t> pos_pair = std::make_pair(key_form_x, key_form_y);
        return std::make_pair(pos_pair, key_form_angle);
    }
//we are defining our heurestic to be the euclidean distance
double Heurestic(const Eigen::Vector2f& node_pos,const Eigen::Vector2f& nav_goal_loc){
      return (nav_goal_loc - node_pos).norm();   
}

double ComputeCost(const NavGraphNode& current, const NavGraphNode& next){
    Eigen::Vector2f current_pos = current.getNodePos();
    Eigen::Vector2f next_pos = next.getNodePos(); 
    double current_angle = current.getNodeOrientation();
    double next_angle = next.getNodeOrientation();
    if (current_angle == next_angle){
        return (current_pos - next_pos).norm(); 
    }
    else{
       return M_PI_4 * (current_pos - next_pos).norm() * M_SQRT1_2;
    }
}

//we also need to define the start and goal nodes (assuming both are global variables) ///Please create nav_goal_loc_ and nav_start_loc_ as a NavGraphNode
void GetPathtoGoal( const NavGraphNode& nav_goal_loc, const NavGraphNode& nav_start_loc, const NavGraphNode& nodes_){
    std::unordered_map<NavGraphNode, NavGraphNode> came_from;
    std::unordered_map<NavGraphNode, double> cost_so_far;
    SimpleQueue<NavGraphNode,double > frontier;
    frontier.Push(nav_start_loc , 0.0);
    
    came_from[nav_start_loc] = nav_start_loc;
    cost_so_far[nav_start_loc] = 0;
    
    while (!frontier.Empty()){
        NavGraphNode current = frontier.Pop();
    
        if (current == nav_goal_loc){
            break;
        }
        std::vector<NavGraphNode> neighbors = getNeighbors(current); 
        for (NavGraphNode next : neighbors) {                      
            double new_cost = cost_so_far[current] + ComputeCost(current , next);
        if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]){
	    cost_so_far[next] = new_cost;
            Eigen::Vector2f next_pos = next.getNodePos();
            Eigen::Vector2f goal_pos = nav_goal_loc.getNodePos(); 
            double priority = new_cost + Heurestic(next_pos, goal_pos);
            frontier.Push(next , priority);
            came_from[next] = current;
            }
            }
}    
}
} // end nav_graph
