//
// Created by amanda on 11/12/20.
//

#include <navigation/nav_graph.h>
#include <navigation/simple_queue.h>
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
       return M_PI_4 * (current_pos - next_pos).norm();
    }
}
/*
//we also need to define the start and goal nodes (assuming both are global variables)
void nav_graph::GetPathtoGoal( const Eigen::Vector2f& nav_goal_loc_, const Eigen::Vector2f& nav_start_loc_){
    std::unordered_map<NavGraphNode, NavGraphNode> came_from;
    std::unordered_map<NavGraphNode, double> cost_so_far;
    SimpleQueue<NavGraphNode,double > frontier;
    //start reviewing the code from this part 
    frontier.push(nav_start_loc_ , 0);
    
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

}
*/
  

} // end nav_graph
