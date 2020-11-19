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
        int32_t key_form_x = round(node_pos_.x() / kStaggeredGridResolution);
        int32_t key_form_y = round(node_pos_.y() / kStaggeredGridResolution);
        int8_t key_form_angle = round(node_orientation_ / kAngularOptionsFromNavGraph);

        std::pair<int32_t, int32_t> pos_pair = std::make_pair(key_form_x, key_form_y);
        return std::make_pair(pos_pair, key_form_angle);
    }

    std::pair<NavGraphNode, NavGraphNode> NavGraph::addStartAndGoal(
            const std::pair<Eigen::Vector2f, float> &start, Eigen::Vector2f &goal) {

        // Orientation doesn't matter for the goal
        NavGraphNode goal_node = createUnalignedNode(goal, 0);
        std::vector<NavGraphNode> near_goal_nodes = getNearbyNodes(goal);
        size_t goal_index = nodes_.size() - 1;
        neighbors_[goal_node] = {};
        for (const NavGraphNode &nearby_node : near_goal_nodes) {

            // If the curvature is too high, then we won't connect the nodes because they're not kinematically feasible.
            double optimal_curvature = getCurvatureFromPoseToPosition(
                    std::make_pair(nearby_node.getNodePos(), nearby_node.getNodeOrientation()), goal);

            if (abs(optimal_curvature) <= kMaxCurvature) {

                // Make sure the goal is in front of the other node
                Vector2f goal_rel_to_node = inverseTransformPoint(goal, 0, nearby_node.getNodePos(),
                                                                  nearby_node.getNodeOrientation()).first;
                if (goal_rel_to_node.x() < 0) {
                    continue;
                }

                // Approximate the connection as a line instead of an arc and only include if the line doesn't intersect
                // the map
                if (!checkLineIntersectionWithMap(line2f(nearby_node.getNodePos(), goal), map_)) {
                    std::vector<uint32_t> neighbors_of_nearby = neighbors_[nearby_node];
                    neighbors_of_nearby.emplace_back(goal_index);
                    neighbors_[nearby_node] = neighbors_of_nearby;
                    goal_neighbors_.emplace_back(nearby_node);
                }
            }
        }
        return std::make_pair(updateStart(start), goal_node);
    }

    float NavGraph::getAngleChangeAfterCurvatureExecution(const double &curvature, const std::pair<Vector2f, float> &start,
                                                          const Vector2f &end) {
        // If the curvature is 0, the angle will not change (will be same as start angle)
        if (curvature == 0) {
            return start.second;
        }

        // Get the angle of the arc traversed when using the given curvature to get from the start to the end.
        double rad_of_turn = 1.0 / curvature;
        Vector2f translation = inverseTransformPoint(end, 0, start.first, start.second).first;
        double angle_change = acos(((2 * pow(rad_of_turn, 2)) - translation.squaredNorm()) / (2 * pow(rad_of_turn, 2)));
        if (translation.x() < 0) {
            angle_change = M_2PI - angle_change;
        }

        // Apply the change in angle to the initial angle
        if (curvature > 0) {
            return start.second + angle_change;
        } else {
            return start.second - angle_change;
        }
    }

    NavGraphNode NavGraph::updateStart(const std::pair<Eigen::Vector2f, float> &start) {

        // Get the nodes near the start
        std::vector<NavGraphNode> near_start_nodes = getNearbyNodes(start.first);

        // Create the start node
        NavGraphNode start_node = createUnalignedNode(start.first, start.second);
        std::vector<uint32_t> start_neighbors;
        for (const NavGraphNode &near_start_node : near_start_nodes) {

            // Get the curvature from the start node to the neighbor node
            double optimal_curvature = getCurvatureFromPoseToPosition(start, near_start_node.getNodePos());

            // Only if the curvature is feasible will we consider adding the node
            if (abs(optimal_curvature) <= kMaxCurvature) {

                // Only keep nodes in front of start
                Vector2f node_rel_to_start = inverseTransformPoint(near_start_node.getNodePos(), 0, start.first, start.second).first;
                if (node_rel_to_start.x() < 0) {
                    continue;
                }

                // Approximate the connection as a line instead of an arc and only include if the line doesn't intersect
                // the map
                bool intersects_with_map = checkLineIntersectionWithMap(
                        line2f(start.first, near_start_node.getNodePos()), map_);

                // Check if the angle when the car gets to the neighbor node is close to the angle that the car should
                // be at the node
                bool angle_close_enough = true;
                if (near_start_node.isGridAligned()) {
                    float end_angle = getAngleChangeAfterCurvatureExecution(
                            optimal_curvature, start, near_start_node.getNodePos());
                    angle_close_enough =
                            math_util::AngleDist(end_angle, near_start_node.getNodeOrientation()) <= kCloseAngleMargin;
                }
                if (angle_close_enough && (!intersects_with_map)) {
                    ROS_INFO_STREAM("Adding edge from start to " << near_start_node.getNodePos().x() << ", " << near_start_node.getNodePos().y() << ", " << near_start_node.getNodeOrientation());
                    uint32_t neighbor_index = node_index_map_[near_start_node];
                    start_neighbors.emplace_back(neighbor_index);
                }
            }
        }
        neighbors_[nodes_.back()] = start_neighbors;
        return start_node;
    }

    std::vector<NavGraphNode> NavGraph::getNearbyNodes(const Eigen::Vector2f &search_pos) {
        box_type query_box(point_type(search_pos.x() - kSearchRangeOneDirection,
                                      search_pos.y() - kSearchRangeOneDirection),
                           point_type(search_pos.x() + kSearchRangeOneDirection,
                                      search_pos.y() + kSearchRangeOneDirection));

        std::vector<value> results;
        rtree_.query(bg::index::intersects(query_box), std::back_inserter(results));
        std::vector<NavGraphNode> result_nodes;
        for (const value &result : results) {
            result_nodes.emplace_back(nodes_[result.second]);
        }
        return result_nodes;
    }

    double getCurvatureFromPoseToPosition(const std::pair<Vector2f, float> &coming_from, const Vector2f &going_to) {
        Vector2f going_rel_to_coming_from = inverseTransformPoint(going_to, 0,
                                                                  coming_from.first, coming_from.second).first;
        return (2 * going_rel_to_coming_from.y()) / going_rel_to_coming_from.squaredNorm();
    }

    std::pair<Vector2f, float> transformPoint(const Vector2f &src_frame_point, const float &src_frame_angle,
                                              const Vector2f &src_frame_pos_rel_target_frame,
                                              const float &src_frame_angle_rel_target_frame) {

        // Rotate the point first
        Eigen::Rotation2Df rotation_mat(src_frame_angle_rel_target_frame);
        Vector2f rotated_still_src_transl = rotation_mat * src_frame_point;

        // Then translate
        Vector2f rotated_and_translated = src_frame_pos_rel_target_frame + rotated_still_src_transl;
        float target_angle = math_util::AngleMod(src_frame_angle_rel_target_frame + src_frame_angle);

        return std::make_pair(rotated_and_translated, target_angle);
    }

    std::pair<Vector2f, float> inverseTransformPoint(const Vector2f &src_frame_point,
                                                     const float &src_frame_angle,
                                                     const Vector2f &target_frame_pos_rel_src_frame,
                                                     const float &target_frame_angle_rel_src_frame) {

        // Translate the point
        Vector2f translated = src_frame_point - target_frame_pos_rel_src_frame;

        // Then rotate
        Eigen::Rotation2Df rotation_mat(-target_frame_angle_rel_src_frame);
        Vector2f rotated_and_translated = rotation_mat * translated;

        float target_angle = math_util::AngleMod(src_frame_angle - target_frame_angle_rel_src_frame);
        return std::make_pair(rotated_and_translated, target_angle);
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
            if (current.isGridAligned() && next.isGridAligned()) {
                // This should be the arc length instead of diagonal. Multiplying by 1/sqrt(2) gets us the grid size and
                // multiplying by PI/4 gets the arc length for a 90 degree arc.
                return M_PI_4 * (current_pos - next_pos).norm() * M_SQRT1_2;
            } else {
                if (current.isGridAligned()) {
                    // This is going to the goal
                    double curvature = getCurvatureFromPoseToPosition(std::make_pair(current_pos, current_angle),
                                                                      next.getNodePos());
                    if (curvature == 0) {
                        return (current_pos - next_pos).norm();
                    }

                    double rad_of_turn = 1.0 / curvature;
                    Vector2f translation = inverseTransformPoint(next_pos, 0, current_pos, current_angle).first;
                    double angle_change = acos(((2 * pow(rad_of_turn, 2)) - translation.squaredNorm()) / (2 * pow(rad_of_turn, 2)));
                    if (translation.x() < 0) {
                        angle_change = M_2PI - angle_change;
                    }
                    return abs(angle_change * rad_of_turn);
                } else {

                    // Just approximate with straight line distance
                    return (current_pos - next_pos).norm();
                }
            }
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
                    frontier.Push(next, -1 * priority);
                    came_from[next] = current;
                }
            }
        }
        ROS_INFO_STREAM("Could not find path to goal");
        return {};
    }

void NavGraph::visualizeConnectionsFromNode(const NavGraphNode &node, amrl_msgs::VisualizationMsg &viz_msg) {
    Vector2f nodeLoc = node.getNodePos();
    double nodeAngle = node.getNodeOrientation();
    std::vector<uint32_t> neighbors = neighbors_[node];
    for (const uint32_t i : neighbors){
        NavGraphNode otherNode = nodes_[i];
        Vector2f otherLoc = otherNode.getNodePos();
        double otherAngle = otherNode.getNodeOrientation();
        visualization::DrawLine(nodeLoc, otherLoc, kGraphColor, viz_msg);
        if (nodeAngle == otherAngle) {
            visualization::DrawLine(nodeLoc, otherLoc, kGraphColor, viz_msg);
        } else {
            double theta1 = nodeAngle;
            double theta2 = otherAngle;
            if (theta1 >= M_PI) theta1 -= M_PI;
            if (theta2 >= M_PI) theta2 -= M_PI;

            float centerX = otherLoc.x()*cos(theta2)+nodeLoc.x()*cos(theta1);
            float centerY = otherLoc.y()*sin(theta2)+nodeLoc.y()*sin(theta1);
            Vector2f center(centerX, centerY);

            double CenterAngle1 = atan2(nodeLoc.y()-centerY, nodeLoc.x()-centerX);
            double CenterAngle2 = atan2(otherLoc.y()-centerY, otherLoc.x()-centerX);
            double angleChange = CenterAngle2-CenterAngle1;

            if (angleChange < -M_PI) angleChange = M_PI/2;
            if (angleChange > M_PI) angleChange = -M_PI/2;

            double startAngle = std::min(CenterAngle1, CenterAngle1+angleChange);
            double endAngle = std::max(CenterAngle1, CenterAngle1+angleChange);
            visualization::DrawArc(center, kGridResolution, startAngle, endAngle, 0xeb34d2, viz_msg);
            double subAngle1 = CenterAngle1 + angleChange*0.3;
            double subAngle2 = CenterAngle1 + angleChange*0.7;
            Vector2f MidPoint1(centerX+  kGridResolution*cos(subAngle1), centerY+kGridResolution*sin(subAngle1));
            Vector2f MidPoint2(centerX+  kGridResolution*cos(subAngle2), centerY+ kGridResolution*sin(subAngle2));
            visualization::DrawPoint(MidPoint1, kGraphColor, viz_msg);
            visualization::DrawPoint(MidPoint2, kGraphColor, viz_msg);


        }
    }
}

void NavGraph::visualizeGoalEdges(const Vector2f &goal_pos, amrl_msgs::VisualizationMsg &viz_msg) {
    for (const NavGraphNode &goal_node_neighbor : goal_neighbors_) {
        visualization::DrawLine(goal_pos, goal_node_neighbor.getNodePos(), kGraphColor, viz_msg);
    }
}

void NavGraph::visualizeNavigationGraphPoints(amrl_msgs::VisualizationMsg &viz_msg) {
    // Visualize Nodes
    for (const NavGraphNode& node : nodes_){
        visualization::DrawPoint(node.getNodePos(), kGraphColor, viz_msg);
    }
}

void NavGraph::visualizeNavigationGraphEdges(amrl_msgs::VisualizationMsg &viz_msg){

    // Visualize Edges
    for (const NavGraphNode &node_to_display_connections: nodes_) {
        visualizeConnectionsFromNode(node_to_display_connections, viz_msg);
   	}
}

std::pair<Vector2f, Vector2f> NavGraph::getMapCorners(const vector_map::VectorMap &map) {
    float x_min = std::numeric_limits<float>::infinity();
    float y_min = std::numeric_limits<float>::infinity();
    float x_max = -std::numeric_limits<float>::infinity();
    float y_max = -std::numeric_limits<float>::infinity();

    for (const line2f &l : map.lines) {
        std::vector<float> xs{l.p0.x(), l.p1.x()};
        std::vector<float> ys{l.p0.y(), l.p1.y()};
        for (const float &x : xs) {
            if (x < x_min) x_min = x;
            if (x > x_max) x_max = x;
        }
        for (const float &y : ys) {
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }
    }

    std::cout << "Map extremens obtained" << std::endl;
    std::cout << x_min << "  " << x_max << "  " << y_min << " " << y_max << std::endl;
    return std::make_pair(Vector2f(x_min, y_min), Vector2f(x_max, y_max));
}

std::vector<Vector2f> NavGraph::createInitialNodePositions(const std::pair<Vector2f, Vector2f> &map_corners) {
    float x_min = map_corners.first.x();
    float y_min = map_corners.first.y();
    float x_max = map_corners.second.x();
    float y_max = map_corners.second.y();

    std::vector<Vector2f> initial2DGrid;

    for (int x_stagger_index = 0; x_stagger_index < kStaggerCount; x_stagger_index++) {
        float x_stagger = kGridResolution * ((float) x_stagger_index) / kStaggerCount;
        ROS_INFO_STREAM("X stagger " << x_stagger);
        for (int y_stagger_index = 0; y_stagger_index < kStaggerCount; y_stagger_index++) {
            float x_frontier = x_min + x_stagger;
            float y_stagger = kGridResolution * ((float) y_stagger_index) / kStaggerCount;
            ROS_INFO_STREAM("Y stagger " << y_stagger);
            while (x_frontier <= x_max) {
                float x_new_node = x_frontier;
                float y_frontier = y_min + y_stagger;
                while (y_frontier <= y_max) {
                    float y_new_node = y_frontier;
                    initial2DGrid.emplace_back(x_new_node, y_new_node);
                    Vector2f node(x_new_node, y_new_node);
                    y_frontier += kGridResolution;
                }
                x_frontier += kGridResolution;
            }
        }
    }
    return initial2DGrid;
}

std::vector<Vector2f> NavGraph::pruneNodesNearObstacles(const std::vector<Vector2f> &unpruned_nodes,
                                              const vector_map::VectorMap &map) {

    /* Removing the intersection points with map lines considering car dimensions
     * For every line segment in the amp, we identify the points that are between the ends of lines
     * and delete it from the initial2DGrid if it is too close to the line segment or intersects it.
     */

    std::vector<Vector2f> nodes_after_pruning = {};
    for (const Vector2f &node : unpruned_nodes) {
        bool deleteNode = false;
        Vector2f P = node;
        for (const line2f& l :map.lines) {
            Vector2f A(l.p0.x(), l.p0.y());
            Vector2f B(l.p1.x(), l.p1.y());
            bool isHorizontal = false;
            bool isVertical = false;
            if (A.y() == B.y()) {
                isHorizontal = true;
            } else if (A.x() == B.x()) {
                isVertical = true;
            }
            Vector2f AP = P - A;
            Vector2f BP = P - B;
            if (isHorizontal && AP.x() * BP.x() <= 0) {
                if (std::abs(AP.y()) <= kCarSafetyDistance) {
                    deleteNode = true;
                }
            }
	    else if (isVertical && AP.y() * BP.y() <= 0) {
                if (std::abs(AP.x()) <= kCarSafetyDistance) {
                    deleteNode = true;
                }
            }

        }

        if (!deleteNode) {
            nodes_after_pruning.emplace_back(node);
        }
    }
    return nodes_after_pruning;
}


void NavGraph::createNavigationGraph(const vector_map::VectorMap& map){
    map_ = map;

   std::cout << "Nav Graph compute Started" << std::endl;

   std::vector<Vector2f> initial2DGrid = createInitialNodePositions(getMapCorners(map));
   std::cout << "Initial Grid defined" << std::endl;
   std::cout << initial2DGrid.size() << std::endl;

   std::vector<Vector2f> nodes_after_pruning = pruneNodesNearObstacles(initial2DGrid, map);

   std::cout << nodes_after_pruning.size() << std::endl;
   std::cout << "Intersections deleted" << std::endl;

   node_index_map_.reserve(nodes_after_pruning.size() * kNumAngularOptions);
   nodes_.reserve(nodes_after_pruning.size() * kNumAngularOptions);

   // Filling up Nodes vector
   size_t node_index = 0;
   for (const Vector2f& node_pos : nodes_after_pruning) {
       for (int rot_num = 0; rot_num < kNumAngularOptions; rot_num++) {
           NavGraphNode new_node(node_pos, rot_num * kAngularOptionsFromNavGraph, true, 0);
           node_index_map_[new_node] = node_index;
           rtree_.insert(std::make_pair(box_type(point_type(node_pos.x(), node_pos.y()), point_type(node_pos.x() + kBoxAroundNodeDim, node_pos.y() + kBoxAroundNodeDim)), node_index));
           node_index++;
           nodes_.emplace_back(new_node);
       }
   }

   std::cout << nodes_.size() << std::endl;

   // Create the neighbours by considering intersection with the map lines
   size_t edges_count = 0;
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
       if (tempNodeOrientation1 >= 2*M_PI){
		tempNodeOrientation1 -= 2*M_PI;
	}
       double tempNodeOrientation2 = nodeAngle + 3 * kAngularOptionsFromNavGraph;
	if (tempNodeOrientation2 >= 2*M_PI){
                tempNodeOrientation2 -= 2*M_PI;
        }
       NavGraphNode tempNode1(Vector2f(nodeX + (cosOffset / ((float) kStaggerCount)), nodeY + (sinOffset / ((float) kStaggerCount))), nodeAngle, true, 0);
       NavGraphNode tempNode2(Vector2f(nodeX + cosOffset - sinOffset, nodeY + sinOffset + cosOffset),
                              tempNodeOrientation1, true, 0);
       NavGraphNode tempNode3(Vector2f(nodeX + cosOffset + sinOffset, nodeY + sinOffset - cosOffset),
                              tempNodeOrientation2, true, 0);

       std::vector<NavGraphNode> possible_neighbors = {tempNode1, tempNode2, tempNode3};

       for (const NavGraphNode &possible_neighbor : possible_neighbors) {
           if (node_index_map_.find(possible_neighbor) != node_index_map_.end()) {
               size_t neighbor_num = node_index_map_.at(possible_neighbor);
               if (!checkIntersectionWithMap(node, possible_neighbor, map)) {
                   neighbors.push_back(neighbor_num);
                   edges_count++;
               }
           }
       }
       neighbors_[nodes_[i]] = neighbors;
   }
   ROS_INFO_STREAM("Num edges " << edges_count);
   std::cout << "final purning done!" << std::endl;
   std::cout << kAngularOptionsFromNavGraph << std::endl;
}

NavGraphNode NavGraph::createUnalignedNode(const Eigen::Vector2f& loc, const float& angle) {
    uint32_t node_index = nodes_.size();
    NavGraphNode new_node(loc, angle, false, next_non_aligned_id_);
    nodes_.emplace_back(new_node);
    node_index_map_[new_node] = node_index;
    // TODO figure out why storing points in the rtree wasn't working. It would be nice not to approximate points with tiny boxes.
    rtree_.insert(std::make_pair(box_type(point_type(loc.x(), loc.y()), point_type(loc.x() + kBoxAroundNodeDim, loc.y() + kBoxAroundNodeDim)), node_index));
    next_non_aligned_id_++;
    return new_node;
}

bool NavGraph::checkIntersectionWithMap(const NavGraphNode& node1, const NavGraphNode& node2,
                                        const vector_map::VectorMap& map) {
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
  	    intersects = checkLineIntersectionWithMap(straightEdge, map);
    } else {
 	    intersects = checkCurveIntersectionWithMap(nodeX, nodeY, nodeAngle, otherNodeX, otherNodeY, otherNodeAngle, map);
    }

    return intersects;
}

bool NavGraph::checkLineIntersectionWithMap(const line2f& line, const vector_map::VectorMap& map){
   bool intersection = false;
   for (const line2f& mapline : map.lines) {
       intersection = mapline.Intersects(line);
       if (intersection) {
           break;
       }
   }
   return intersection;
}

bool NavGraph::checkCurveIntersectionWithMap(const float& x1,
					     const float& y1,
					     const double& theta1,
					     const float& x2,
					     const float& y2,
					     const double& theta2,
					     const vector_map::VectorMap& map){
   
    double temptheta1 = theta1;
    double temptheta2 = theta2;
    
    if (temptheta1 >= M_PI) temptheta1 -= M_PI;
    if (temptheta2 >= M_PI) temptheta2 -= M_PI;
	
    float centerX = x2*cos(temptheta2)+x1*cos(temptheta1);
    float centerY = y2*sin(temptheta2)+y1*sin(temptheta1);

    double CenterAngle1 = atan2(y1-centerY, x1-centerX);
    double CenterAngle2 = atan2(y2-centerY, x2-centerX);    
    double angleChange = CenterAngle2-CenterAngle1;

    if (angleChange < -M_PI) angleChange = M_PI/2;
    if (angleChange > M_PI) angleChange = -M_PI/2;

    double subAngle1 = CenterAngle1 + angleChange*0.3;
    double subAngle2 = CenterAngle1 + angleChange*0.7;
    Vector2f MidPoint1(centerX+  kGridResolution*cos(subAngle1), centerY+  kGridResolution*sin(subAngle1));
    Vector2f MidPoint2(centerX+  kGridResolution*cos(subAngle2), centerY+  kGridResolution*sin(subAngle2));

    line2f Edge1(x1, y1, MidPoint1.x(), MidPoint1.y());
    line2f Edge2(MidPoint1.x(), MidPoint1.y(), MidPoint2.x(), MidPoint2.y());
    line2f Edge3(x2, y2, MidPoint2.x(), MidPoint2.y());

    bool intersection1 = checkLineIntersectionWithMap(Edge1, map);
    bool intersection2 = checkLineIntersectionWithMap(Edge2, map);
    bool intersection3 = checkLineIntersectionWithMap(Edge3, map);

    bool intersection  = (intersection1 || intersection2 || intersection3);

    return intersection;
}
} // end nav_graph
