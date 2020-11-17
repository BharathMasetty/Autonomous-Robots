// Created by amanda on 11/12/20.
//

#include <navigation/nav_graph.h>
#include <navigation/simple_queue.h>
#include "visualization/visualization.h"
#include <navigation/simple_queue.h>
#include "shared/ros/ros_helpers.h"
#include "ros/ros.h"

using amrl_msgs::VisualizationMsg;
using geometry::line2f;
using Eigen::Vector2f;
using std::string;
using std::vector;

namespace nav_graph {

    std::pair<std::pair<int32_t, int32_t>, int8_t> NavGraphNode::convertToKeyForm() const {
        int32_t key_form_x = round(node_pos_.x() / kGridResolution);
        int32_t key_form_y = round(node_pos_.y() / kGridResolution);
        int8_t key_form_angle = round(node_orientation_ / kAngularOptionsFromNavGraph);

        std::pair<int32_t, int32_t> pos_pair = std::make_pair(key_form_x, key_form_y);
        return std::make_pair(pos_pair, key_form_angle);
    }

    //we are defining our heuristic to be the euclidean distance
    double computeHeuristic(const Eigen::Vector2f& node_pos,const Eigen::Vector2f& nav_goal_loc) {
        return (nav_goal_loc - node_pos).norm();
    }

    double ComputeCost(const NavGraphNode& current, const NavGraphNode& next){
        Eigen::Vector2f current_pos = current.getNodePos();
        Eigen::Vector2f next_pos = next.getNodePos();
        double current_angle = current.getNodeOrientation();
        double next_angle = next.getNodeOrientation();
        if (current_angle == next_angle) {
            return (current_pos - next_pos).norm();
        } else {
            // This should be the arc length instead of diagonal. Multiplying by 1/sqrt(2) gets us the grid size and
            // multiplying by PI/4 gets the arc length for a 90 degree arc.
            return M_PI_4 * (current_pos - next_pos).norm() * M_SQRT1_2;
        }
    }

    //we also need to define the start and goal nodes (assuming both are global variables) ///Please create nav_goal_loc_ and nav_start_loc_ as a NavGraphNode
    std::vector<NavGraphNode> GetPathToGoal(const NavGraphNode& nav_goal_loc, const NavGraphNode& nav_start_loc,
                                            const NavGraph &nav_graph) {
        std::unordered_map<NavGraphNode, NavGraphNode> came_from;
        std::unordered_map<NavGraphNode, double> cost_so_far;
        SimpleQueue<NavGraphNode, double> frontier;

        frontier.Push(nav_start_loc, computeHeuristic(nav_start_loc.getNodePos(), nav_goal_loc.getNodePos()));

        cost_so_far[nav_start_loc] = 0;

        while (!frontier.Empty()) {
            NavGraphNode current = frontier.Pop();

            if (current == nav_goal_loc) {
                std::vector<NavGraphNode> path;
                NavGraphNode node_to_find_parent_for = current;
                path.emplace_back(node_to_find_parent_for);
                while(came_from.find(node_to_find_parent_for) != came_from.end()) {
                    node_to_find_parent_for = came_from.at(node_to_find_parent_for);
                    path.emplace_back(node_to_find_parent_for);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            std::vector<NavGraphNode> neighbors = nav_graph.getNeighbors(current);
            for (NavGraphNode next : neighbors) {
                double new_cost = cost_so_far[current] + ComputeCost(current, next);
                if ((cost_so_far.find(next) == cost_so_far.end()) || (new_cost < cost_so_far[next])) {
                    cost_so_far[next] = new_cost;
                    Eigen::Vector2f next_pos = next.getNodePos();
                    Eigen::Vector2f goal_pos = nav_goal_loc.getNodePos();
                    double priority = new_cost + computeHeuristic(next_pos, goal_pos);
                    frontier.Push(next, priority);
                    came_from[next] = current;
                }
            }
        }

        return {};
    }

void NavGraph::visualizeNavigationGraphPoints(const uint32_t &node_color, amrl_msgs::VisualizationMsg &viz_msg) {
    // Visualize Nodes
    for (const NavGraphNode& node : nodes_){
        visualization::DrawPoint(node.getNodePos(), node_color, viz_msg);
    }
}

void NavGraph::visualizeNavigationGraphEdges(const uint32_t &node_color, amrl_msgs::VisualizationMsg &viz_msg){

    // Visualize Edges
    for (const auto& lattice: neighbors_) {
        NavGraphNode node = lattice.first;
        Vector2f nodeLoc = node.getNodePos();
        double nodeAngle = node.getNodeOrientation();
        std::vector<uint32_t> neighbors = lattice.second;
        for (const uint32_t i : neighbors){
            NavGraphNode otherNode = nodes_[i];
            Vector2f otherLoc = otherNode.getNodePos();
            double otherAngle = otherNode.getNodeOrientation();
            visualization::DrawLine(nodeLoc, otherLoc, node_color, viz_msg);
            double startAngle = 0.0;
            double endAngle = 0.0;
            if (nodeAngle == otherAngle) {
                visualization::DrawLine(nodeLoc, otherLoc, node_color, viz_msg);
            } else {
                if (nodeAngle == 0.0 && otherAngle == kAngularOptionsFromNavGraph) {
                    startAngle = -kAngularOptionsFromNavGraph;
                    endAngle = 0.0;
                } else if ((nodeAngle == kAngularOptionsFromNavGraph) && (otherAngle == 2*kAngularOptionsFromNavGraph)) {
                    startAngle = 0.0;
                    endAngle = kAngularOptionsFromNavGraph;
                } else if ((nodeAngle == 2*kAngularOptionsFromNavGraph) && (otherAngle == 3*kAngularOptionsFromNavGraph)) {
                    startAngle =  kAngularOptionsFromNavGraph;
                    endAngle = 2*kAngularOptionsFromNavGraph;
                } else if ((nodeAngle == 3*kAngularOptionsFromNavGraph) && (otherAngle == 0.0)) {
                    startAngle =  2*kAngularOptionsFromNavGraph;
                    endAngle = 3*kAngularOptionsFromNavGraph;
                }

                double theta1 = nodeAngle;
                double theta2 = otherAngle;
                if (otherAngle > M_PI) {
                    theta2 = otherAngle -  M_PI;
                }
                if (nodeAngle > M_PI) {
                    theta1 = nodeAngle - M_PI;
                }
                float centerX = otherLoc.x()*cos(theta2) + nodeLoc.x()*cos(theta1);
                float centerY = otherLoc.y()*sin(theta2) + nodeLoc.y()*sin(theta1);
                float radius = kGridResolution;
        		Vector2f center(centerX, centerY);
        		visualization::DrawArc(center, radius, startAngle, endAngle, node_color, viz_msg);
            }
		}
   	}
}

void NavGraph::createNavigationGraph(const vector_map::VectorMap& map_){

   std::cout << "Nav Graph compute Started" << std::endl;
   //Get extreme x,y's of the map
   float x_min = std::numeric_limits<float>::infinity();
   float y_min = std::numeric_limits<float>::infinity();
   float x_max = -std::numeric_limits<float>::infinity();
   float y_max = -std::numeric_limits<float>::infinity();

   for(const line2f& l : map_.lines){
        std::vector<float> xs{l.p0.x(), l.p1.x()};
        std::vector<float> ys{l.p0.y(), l.p1.y()};
        for(const float& x : xs){
                if (x < x_min) x_min = x;
                if (x > x_max) x_max = x;
        }
        for(const float& y : ys){
                if (y < y_min) y_min = y;
                if (y > y_max) y_max = y;
        }
   }
   std::cout << "Map extremens obtained" << std::endl;
   std::cout << x_min << "  " << x_max << "  " << y_min << " " << y_max << std::endl;

   // To create an initial grid centered around initial pos in map range
   std::vector<Vector2f> initial2DGrid;
   float x_frontier = x_min;
   float y_frontier = y_min;

   while (x_frontier <= x_max) {
       float x_new_node = x_frontier;
       y_frontier =  y_min;
       while (y_frontier <= y_max) {
           float y_new_node = y_frontier;
           initial2DGrid.emplace_back(x_new_node, y_new_node);
           Vector2f node(x_new_node, y_new_node);
           y_frontier += kGridResolution;
       }
       x_frontier += kGridResolution;
   }
   std::cout << "Initial Grid defined" << std::endl;
   std::cout << initial2DGrid.size() << std::endl;

   /* Removing the intersection points with map lines considering car dimensions
    * For every line segment in the amp, we identify the points that are between the ends of lines
    * and delete it from the initial2DGrid if it is too close to the line segment or intersects it.
    */
   for (const line2f& l :map_.lines) {
       Vector2f A(l.p0.x(), l.p0.y());
       Vector2f B(l.p1.x(), l.p1.y());
       bool isHorizontal = false;
       bool isVertical = false;
       if (A.y() == B.y()) {
           isHorizontal = true;
       } else if (A.x() == B.x()) {
           isVertical = true;
       }
       for (uint32_t i=0; i< initial2DGrid.size(); i++) {
           bool deleteNode = false;
           Vector2f P = initial2DGrid[i];
           Vector2f AP = P - A;
           Vector2f BP = P - B;
           if (isHorizontal && AP.x()*BP.x()<=0) {
               if (std::abs(AP.y()) <= kCarSafetyDistance) {
                   deleteNode = true;
               }
           }
           if (isVertical && AP.y()*BP.y()<=0) {
               if (std::abs(AP.x()) <= kCarSafetyDistance) {
                   deleteNode = true;
               }
           }

           if (deleteNode) {
               initial2DGrid.erase(std::remove(initial2DGrid.begin(), initial2DGrid.end(), P), initial2DGrid.end());
           }
       }
   }
   std::cout << initial2DGrid.size() << std::endl;
   std::cout << "Intersections deleted" << std::endl;

   // Filling up Nodes vector
   for (const Vector2f& point : initial2DGrid) {
       float possible_node_angle = math_util::DegToRad(0);
       while (possible_node_angle < math_util::DegToRad(360)) {
           nodes_.emplace_back(point, possible_node_angle, true, 0);
           possible_node_angle += kAngularOptionsFromNavGraph;
       }
   }

   std::cout << nodes_.size() << std::endl;

   // Create the neighbours by considering intersection with the map lines
   for (uint32_t i=0; i<nodes_.size(); i++) {
       std::vector<uint32_t> neighbors;
       NavGraphNode node = nodes_[i];
       Vector2f nodeLoc = node.getNodePos();
       float nodeX = nodeLoc.x();
       float nodeY = nodeLoc.y();
       double nodeAngle = node.getNodeOrientation();
       float cosOffset = kGridResolution * cos(nodeAngle);
       float sinOffset = kGridResolution * sin(nodeAngle);
       //std::cout << "Node " << nodeX << " " << nodeY << " " << nodeAngle << std::endl;
       double tempNodeOrientation1 = nodeAngle + kAngularOptionsFromNavGraph;
       if (tempNodeOrientation1 >= 2 * M_PI) {
           tempNodeOrientation1 -= 2 * M_PI;
       }
       double tempNodeOrientation2 = nodeAngle + 3 * kAngularOptionsFromNavGraph;
       if (tempNodeOrientation2 >= 2 * M_PI) {
           tempNodeOrientation2 -= 2 * M_PI;
       }

       NavGraphNode tempNode1(Vector2f(nodeX + cosOffset, nodeY + sinOffset), nodeAngle, true, 0);
       NavGraphNode tempNode2(Vector2f(nodeX + cosOffset - sinOffset, nodeY + sinOffset + cosOffset),
                              tempNodeOrientation1, true, 0);
       NavGraphNode tempNode3(Vector2f(nodeX + cosOffset + sinOffset, nodeY + sinOffset - cosOffset),
                              tempNodeOrientation2, true, 0);

       for (uint32_t j = 0; j < nodes_.size(); j++) {
           NavGraphNode otherNode = nodes_[j];
           //Vector2f otherNodeLoc = otherNode.getNodePos();
           //float otherX = otherNodeLoc.x();
           //float otherY = otherNodeLoc.y();
           //double otherNodeAngle = nodes_[j].getNodeOrientation();
           bool intersection = true;
           if (otherNode == tempNode1 || otherNode == tempNode2 || otherNode == tempNode3) {
               //std::cout << "Other Node " << otherX << " " << otherY << " " << otherNodeAngle << std::endl;
               intersection = checkIntersectionWithMap(node, otherNode, map_);
           }
           if (!intersection) {
               neighbors.push_back(j);
           }
       }
       if (!neighbors.empty()) {
           neighbors_[nodes_[i]] = neighbors;
       }
   }
   std::cout << "final purning done!" << std::endl;
   std::cout << kAngularOptionsFromNavGraph << std::endl;
}

void NavGraph::createUnalignedNode(const Eigen::Vector2f& loc, const float& angle, const uint32_t& ID) {
  nodes_.emplace_back(loc, angle, false, ID);
}

bool NavGraph::checkIntersectionWithMap(const NavGraphNode& node1, const NavGraphNode& node2,
                                        const vector_map::VectorMap& map_) {
    bool intersects = false;
    Vector2f nodeLoc = node1.getNodePos();
    double nodeAngle = node1.getNodeOrientation();
    float nodeX = nodeLoc.x();
    float nodeY = nodeLoc.y();
    Vector2f otherNodeLoc = node2.getNodePos();
    double otherNodeAngle = node2.getNodeOrientation();
    float otherNodeX = otherNodeLoc.x();
    float otherNodeY = otherNodeLoc.y();
    // No change in incoming and outgoing angle
    if (nodeAngle == otherNodeAngle){
  	    line2f straightEdge(nodeX, nodeY, otherNodeX, otherNodeY);
  	    intersects = checkLineIntersectionWithMap(straightEdge, map_);
    } else {
 	    intersects = checkCurveIntersectionWithMap(nodeX, nodeY, nodeAngle, otherNodeX, otherNodeY, otherNodeAngle, map_);
    }

    return intersects;
}

bool NavGraph::checkLineIntersectionWithMap(const line2f& line, const vector_map::VectorMap& map_){
   bool intersection = false;
   for (const line2f& mapline : map_.lines) {
       intersection = mapline.Intersects(line);
       if (intersection) {
           break;
       }
   }
   return intersection;
}

bool NavGraph::checkCurveIntersectionWithMap(const float& x1,
					     const float& y1,
					     double& theta1,
					     const float& x2,
					     const float& y2,
					     double& theta2,
					     const vector_map::VectorMap& map_){
    double subAngle1 = math_util::DegToRad(30.0);
    double subAngle2 = math_util::DegToRad(60.0);
    double angleDiff = theta2-theta1;
    float dir = cos(theta1) + sin(theta1);
    if (theta1 >= M_PI) {
        theta1 -= M_PI;
    }
    if (theta2 >= M_PI) {
        theta2 -= M_PI;
    }
    float centerX = x2*cos(theta2)+x1*cos(theta1);
    float centerY = y2*sin(theta2)+y1*sin(theta1);
    float change = kGridResolution*sin(angleDiff);

    Vector2f MidPoint1(centerX+change*dir*sin(subAngle1), centerY-change*dir*cos(subAngle1));
    Vector2f MidPoint2(centerX+change*dir*sin(subAngle2), centerY-change*dir*cos(subAngle2));

    //std::cout << "M1 " << MidPoint1.x() << " " << MidPoint1.y() << std::endl;
    //std::cout << "M2 " << MidPoint2.x() << " " << MidPoint2.y() << std::endl;

    line2f Edge1(x1, y1, MidPoint1.x(), MidPoint1.y());
    line2f Edge2(MidPoint1.x(), MidPoint1.y(), MidPoint2.x(), MidPoint2.y());
    line2f Edge3(x2, y2, MidPoint2.x(), MidPoint2.y());

    bool intersection1 = checkLineIntersectionWithMap(Edge1, map_);
    bool intersection2 = checkLineIntersectionWithMap(Edge2, map_);
    bool intersection3 = checkLineIntersectionWithMap(Edge3, map_);

    bool intersection  = (intersection1 || intersection2 || intersection3);

    return intersection;
}
} // end nav_graph
