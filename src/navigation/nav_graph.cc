//
// Created by amanda on 11/12/20.
//

#include <navigation/nav_graph.h>
#include <simple_queue.h>
//using namespace Navigation;

namespace nav_graph {

    std::pair<std::pair<int32_t, int32_t>, int8_t> NavGraphNode::convertToKeyForm() const {
        int32_t key_form_x = round(node_pos_.x() / kGridResolution);
        int32_t key_form_y = round(node_pos_.y() / kGridResolution);
        int8_t key_form_angle = round(node_orientation_ / kAngularOptionsFromNavGraph);

        std::pair<int32_t, int32_t> pos_pair = std::make_pair(key_form_x, key_form_y);
        return std::make_pair(pos_pair, key_form_angle);
    }
//we are defining our heurestic to be the euclidean distance
double Heurestic(const Eigen::Vector2f& node_pos_,const Eigen::Vector2f& nav_goal_loc_){
    double x_difference = std::abs(node_pos_.x() - nav_goal_loc_.x());
    double y_difference = std::abs(node_pos_.y() - nav_goal_loc_.y());
    double squared_euclidean = std::pow(x_difference,2) + std::pow(y_difference,2);
    return std::abs(std::sqrt(squared_euclidean)); 
}

double ComputeCost(const NavGraphNode& current, const NavGraphNode& next){
    Eigen::Vector2f current_pos = current.getNodePos();
    Eigen::Vector2f next_pos = next.getNodePos(); 
    double current_angle = current.getNodeOrientation();
    double next_angle = next.getNodeOrientation();
    double x_difference_1 = std::abs(current_pos.x() - next_pos.x());
    double y_difference_1 = std::abs(current_pos.y() - next_pos.y());
    double squared_euclidean_1 = std::pow(x_difference_1,2) + std::pow(y_difference_1,2);
    if (current_angle == next_angle){
        return std::abs(std::sqrt(squared_euclidean_1)); 
    }
    else{
       return 3.14/2 * std::abs(std::sqrt(squared_euclidean_1));
    }
}

//we also need to define the start and goal nodes (assuming both are global variables)
void GetPathtoGoal(const std::vector<NavGraphNode>& nodes_, const std::unordered_map<NavGraphNode, std::vector<uint32_t>>& neighbors_, const Eigen::Vector2f& nav_goal_loc_, 
     const Eigen::Vector2f& nav_start_loc_){
    std::unordered_map<NavGraphNode, NavGraphNode> came_from;
    std::unordered_map<NavGraphNode, double> cost_so_far;
    simple_queue<NavGraphNode, double> frontier;
/*    
    frontier.put(nav_start_loc_ , 0);
    
    came_from[nav_start_loc_] = nav_start_loc_;
    cost_so_far[nav_start_loc_] = 0;
    
    while (!frontier.empty()){
        NavGraphNode current = frontier.get();
        
        void Navigation::ReachedGoal();
    
        if (nav_complete_ == true){
            break;
        }
        
        for (NavGraphNode next :  ) {                       //enter the neighbors
            double new_cost = cost_so_far[current] + /
*/
}

  

} // end nav_graph
