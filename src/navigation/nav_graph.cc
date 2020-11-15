//
// Created by amanda on 11/12/20.
//

#include <navigation/nav_graph.h>
#include <navigation/simple_queue.h>
#include "visualization/visualization.h"
#include <navigation/simple_queue.h>

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
    std::vector<NavGraphNode> GetPathToGoal( const NavGraphNode& nav_goal_loc, const NavGraphNode& nav_start_loc,
                                             const NavGraph &nav_graph) {
//        std::unordered_map<NavGraphNode, NavGraphNode> came_from;
//        std::unordered_map<NavGraphNode, double> cost_so_far;
//        SimpleQueue<NavGraphNode, double> frontier;
//
//        //start reviewing the code from this part
//        frontier.Push(nav_start_loc, computeHeuristic(nav_start_loc.getNodePos(), nav_goal_loc.getNodePos()));
//
//        cost_so_far[nav_start_loc] = 0;
//
//        while (!frontier.Empty()){
//            NavGraphNode current = frontier.Pop();
//            // TODO
//        }

        return {}; // TODO
    }

void NavGraph::visualizeNavigationGraph(const uint32_t &node_color, amrl_msgs::VisualizationMsg &viz_msg){

   // Visualize Nodes
   for (const NavGraphNode& node : nodes_){
   	visualization::DrawPoint(node.getNodePos(), node_color, viz_msg); 
	/*
	for (const auto& lattice: neighbors_){
		Vector2f node = lattice.first.getNodePos();
		std::vector<uint32_t> neighbors = lattice.second;
		for(const uint32_t i : neighbors){
			visualization::DrawLine(node, nodes_[i].getNodePos(), node_color, viz_msg);
		}
	}*/	
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

   while (x_frontier <= x_max){
   	float x_new_node = x_frontier;
	y_frontier =  y_min;
	while (y_frontier <= y_max){
		float y_new_node = y_frontier;
		initial2DGrid.emplace_back(x_new_node, y_new_node);
		Vector2f node(x_new_node, y_new_node);
		y_frontier += kGridResolution;
	}
	x_frontier += kGridResolution;
   }
   std::cout << "Initial Grid defined" << std::endl;
   std::cout << initial2DGrid.size() << std::endl;

   // Removing the intersection points with map lines considering car dimensions
   for (const line2f& l :map_.lines){
        Vector2f A(l.p0.x(), l.p0.y());
        Vector2f B(l.p1.x(), l.p1.y());
	bool isHorizontal = false;
	bool isVertical = false;
	if (A.y() == B.y()) isHorizontal = true;
	else if (A.x() == B.x()) isVertical = true;
	//std::cout << "New Line" << isHorizontal << " " <<  isVertical << std::endl;
	//std::cout << A.x() << " " << B.x() << " " << A.y() << " " << B.y() << " " << isHorizontal << " " << isVertical << " " << std::endl;
	for (uint i=0; i< initial2DGrid.size(); i++){
		bool deleteNode = false;
		Vector2f P = initial2DGrid[i];
        	Vector2f AP = P - A;
		Vector2f BP = P - B;

		//std::cout << P.x() << " " << P.y() << " " << AP.x()*BP.x() <<std::endl;
		if (isHorizontal && AP.x()*BP.x()<0){
			//std::cout << "Here" << " " << AP.y() << std::endl;
			if (std::abs(AP.y()) <= kCarSafetyDistance) {
				deleteNode = true;
				//std::cout << "DeleteFlag"<< std::endl;
			}
		}
		if (isVertical && AP.y()*BP.y()<0){
			if (std::abs(AP.x()) <= kCarSafetyDistance) deleteNode = true;
		}

		if (deleteNode) {
			initial2DGrid.erase(initial2DGrid.begin() + i);
			//std::cout<<"Node Deleted"<< std::endl;
		}
	}

   }
   std::cout << initial2DGrid.size() << std::endl;
   std::cout << "Intersections deleted" << std::endl;
   
   // Filling up Nodes vector
   for (const Vector2f& point : initial2DGrid){
  
	float possible_node_angle = math_util::DegToRad(-180);
        while (possible_node_angle < math_util::DegToRad(90)){
        	nodes_.emplace_back(point, possible_node_angle, true, 0);
                possible_node_angle += kAngularOptionsFromNavGraph;
        }
   } 
	
  std::cout << nodes_.size() << std::endl;

  // Create the neighbours by considering intersection with the map lines
  for (uint32_t i=0; i<nodes_.size(); i++){
 	std::vector<uint32_t> neighbors;
	Vector2f nodeLoc = nodes_[i].getNodePos();
	float nodeX = nodeLoc.x();
	float nodeY = nodeLoc.y();
	float nodeAngle = nodes_[i].getNodeOrientation();
	float xOffset = kGridResolution*cos(M_PI_2*nodeAngle);
	float yOffset = kGridResolution*sin(M_PI_2*nodeAngle);
	float angOffsetx = kGridResolution*cos(M_PI_2*(nodeAngle+kAngularOptionsFromNavGraph));
	float angOffsety = kGridResolution*sin(M_PI_2*(nodeAngle+kAngularOptionsFromNavGraph));
	for (uint32_t j=0; j<nodes_.size(); j++){
		Vector2f otherNodeLoc = nodes_[j].getNodePos();
		float otherX = otherNodeLoc.x();
		float otherY = otherNodeLoc.y();
		float otherNodeAngle = nodes_[j].getNodeOrientation();
		bool intersection = false;
		//float distance = (nodeLoc - otherNodeLoc).norm();
		//float angleDiff = std::abs(nodeAngle - otherNodeAngle);
		
		if ((otherX == nodeX+xOffset && otherY == nodeY+yOffset && nodeAngle == otherNodeAngle) || 
	            (otherX == nodeX+xOffset+angOffsetx && otherY == nodeY+yOffset+angOffsety && otherNodeAngle == nodeAngle+kAngularOptionsFromNavGraph) || 
		    (otherX == nodeX+xOffset-angOffsetx && otherY == nodeY+yOffset-angOffsety && otherNodeAngle == nodeAngle-kAngularOptionsFromNavGraph)) {
			
		    // check for intersection with map line
			if (otherNodeAngle == nodeAngle){
				line2f straightEdge(nodeX, nodeY, otherX, otherY);
				for (const line2f& mapline :map_.lines){
					intersection = mapline.Intersects(straightEdge);
					if (intersection) break;
				}
			}
			else if (nodeAngle >= otherNodeAngle){
				line2f Edge1(nodeX, nodeY, nodeX+kGridResolution*cos(math_util::DegToRad(30)), nodeY-kGridResolution*sin(math_util::DegToRad(30)));
				line2f Edge2(nodeX+kGridResolution*cos(math_util::DegToRad(30)), nodeY-kGridResolution*sin(math_util::DegToRad(30)),
						nodeX+kGridResolution*cos(math_util::DegToRad(60)), nodeY-kGridResolution*sin(math_util::DegToRad(60)));
				line2f Edge3(nodeX+kGridResolution*cos(math_util::DegToRad(60)), nodeY-kGridResolution*sin(math_util::DegToRad(60)), otherX, otherY);
		 		std::vector<line2f> splineEdges {Edge1, Edge2, Edge3};	
				for(const line2f& edge : splineEdges){
					for (const line2f& mapline :map_.lines){
                                        	intersection = mapline.Intersects(edge);
                                		if (intersection) break;
                                	}
                                	if (intersection) break;
				}
			}
			else {
				line2f Edge1(nodeX, nodeY, nodeX+kGridResolution*cos(math_util::DegToRad(30)), nodeY+kGridResolution*sin(math_util::DegToRad(30)));
                                line2f Edge2(nodeX+kGridResolution*cos(math_util::DegToRad(30)), nodeY+kGridResolution*sin(math_util::DegToRad(30)),
                                                nodeX+kGridResolution*cos(math_util::DegToRad(60)), nodeY+kGridResolution*sin(math_util::DegToRad(60)));
                                line2f Edge3(nodeX+kGridResolution*cos(math_util::DegToRad(60)), nodeY+kGridResolution*sin(math_util::DegToRad(60)), otherX, otherY);
		 		std::vector<line2f> splineEdges {Edge1, Edge2, Edge3};	
				for(const line2f& edge : splineEdges){
                                        for (const line2f& mapline :map_.lines){
                                        	intersection = mapline.Intersects(edge);
                                        	if (intersection) break;
                                        }
                                        if (intersection) break;
                                }	
			}
		//std::cout<< "Intersection " << intersection << std::endl;
	  	if (!intersection) neighbors.push_back(j);
		}
	}
	if (!neighbors.empty()){
		std::pair<NavGraphNode, std::vector<uint32_t>> newMapping (nodes_[i], neighbors);
		neighbors_.insert(newMapping);
		//std::cout<< neighbors.size() << std::endl;
		}
	}
   std::cout << "final purning done!" << std::endl;
}

void NavGraph::createUnalignedNode(const Eigen::Vector2f& loc, 
				      const float& angle,
				      const uint32_t& ID){
  nodes_.emplace_back(loc, angle, false, ID);
}

} // end nav_graph
