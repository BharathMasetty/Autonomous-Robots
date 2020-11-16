//
// Created by amanda on 11/12/20.
//


#ifndef CS393R_STARTER_NAV_GRAPH_H
#define CS393R_STARTER_NAV_GRAPH_H

#include "eigen3/Eigen/Dense"
#include <shared/math/math_util.h>
#include <boost/functional/hash.hpp>
#include <ros/ros.h>
#include <unordered_map>
#include "vector_map/vector_map.h"
#include <amrl_msgs/VisualizationMsg.h>

//using namespace Navigation;

namespace nav_graph {

/**
 * Resolution of the navigation graph (provides how far apart the nearest two nodes are).
 */
static const double kGridResolution = 0.5; // TODO tune

/**
 * Angular resolution of the navigation graph. With 90 degrees, there are 4 possible angles that a node could need
 * to be in. (If we allow backward motion, we could probably consider 0 and 180 to be the same and 90/-90 to be the
 * same, since the car can reach the node still, but we'll handle this in edge connections instead.
 */
static const double kAngularOptionsFromNavGraph = math_util::DegToRad(90.0);

class NavGraphNode {
public:

    /**
     * Create the nav graph node.
     *
     * @param node_pos                  Position of the node. Each component should be a multiple of kGridResolution
     *                                  unless this is a start/goal node.
     * @param node_orientation          Orientation of the node (orientation that car will be at when entering/exiting
     *                                  the node. Should be a multiple of kAngularOptionsFromNavGraph unless
     *                                  grid_aligned is false.
     * @param grid_aligned              True if the object aligns with the grid, false if it doesn't. Should only be
     *                                  false for start/goal nodes that are dynamically added to the graph.
     * @param id_for_not_grid_aligned   Identifier to distinguish between non-grid-aligned (start/goal) nodes. Must be
     *                                  unique for nodes where grid_aligned is false. Value doesn't matter for other
     *                                  nodes.
     */
    NavGraphNode(const Eigen::Vector2f &node_pos, const float &node_orientation, const bool &grid_aligned,
                 const uint32_t &id_for_not_grid_aligned) : node_pos_(node_pos),
                 node_orientation_(node_orientation),
                 grid_aligned_(grid_aligned),
                 id_for_not_grid_aligned_(id_for_not_grid_aligned) {
        if (grid_aligned) {
            id_for_not_grid_aligned_ = kGridAlignedIdentifierValue;
        }
    }

    /**
     * Default constructor. Should never be used explicitly (only by other data structures that require a default
     * constructor).
     */
    NavGraphNode() = default;

    bool operator==(const NavGraphNode &other) const {
        if (grid_aligned_) {
            if (other.grid_aligned_) {
                return convertToKeyForm() == other.convertToKeyForm();
            } else {
                return false;
            }
        }
        if (other.grid_aligned_) {
            return false;
        }
        return id_for_not_grid_aligned_ == other.id_for_not_grid_aligned_;
    }

    /**
     * Convert the nav graph node to ints based on the resolution for translation and angles.
     * @return
     */
    std::pair<std::pair<int32_t, int32_t>, int8_t> convertToKeyForm() const;

    Eigen::Vector2f getNodePos() const {
        return node_pos_;
    }

    float getNodeOrientation() const {
        return node_orientation_;
    }

    /**
     * Get if the node is grid aligned and if not, populate the non_aligned_identifier field with the identifier for
     * the node (used to distinguish start and end).
     *
     * @param non_aligned_identifier[out] Identifier for non-grid aligned nodes. Will not be modified if this function
     * returns true.
     *
     * @return True if the node is grid aligned, false if not.
     */
    bool isGridAligned(uint32_t &non_aligned_identifier) const {
        if (!grid_aligned_) {
            non_aligned_identifier = id_for_not_grid_aligned_;
            return false;
        }
        return true;
    }

private:

    /**
     * Value that the grid aligned nodes will have for the non-grid-aligned identifier. Will overwrite what is passed
     * in constructor to ensure this value doesn't matter for grid aligned nodes.
     */
    static const uint32_t kGridAlignedIdentifierValue = 6482;

    /**
     * Location of the node. X and y should both be multiples of kGridResolution (unless grid_aligned_ is false).
     */
    Eigen::Vector2f node_pos_;

    /**
     * Orientation for the node. Should be a multiple of kAngularOptonsFromNavGraph (unless grid_aligned_ is false).
     */
    float node_orientation_;

    /**
     * True if the node is aligned with the grid, false if it is not (start or goal).
     */
    bool grid_aligned_;

    /**
     * Identifier to distinguish between non-grid aligned nodes (should just be start/end).
     */
    uint32_t id_for_not_grid_aligned_;
};
}

// Adding hash function for Node so we can use it as key in unordered map.
namespace std {
    template<>
    class hash<nav_graph::NavGraphNode> {
    public:
        size_t operator()(const nav_graph::NavGraphNode &node) const {
            std::pair<std::pair<int32_t, int32_t>, int8_t> key_form = node.convertToKeyForm();
            std::size_t seed = 0;

            boost::hash_combine(seed, key_form.first.first);
            boost::hash_combine(seed, key_form.first.second);
            boost::hash_combine(seed, key_form.second);

            uint32_t non_grid_id;
            bool grid_aligned = node.isGridAligned(non_grid_id);
            boost::hash_combine(seed, grid_aligned);
            boost::hash_combine(seed, non_grid_id);
            return seed;
        }
    };
}

namespace nav_graph {

/**
 * Contains nodes and edges for navigation graph.
 */
class NavGraph {
public:
    
    /*
     * Creates the navigation graph without initial and final node
     */
    void createNavigationGraph(const vector_map::VectorMap& map_);
    
    /*
     * To visualize the nodes
     */
    void visualizeNavigationGraphPoints(const uint32_t &node_color, amrl_msgs::VisualizationMsg &viz_msg);

    /*
     * To visualize the nodes
     */
    void visualizeNavigationGraphEdges(const uint32_t &node_color, amrl_msgs::VisualizationMsg &viz_msg);

    /*
     * To add initial and final node. 
     */
    void createUnalignedNode(const Eigen::Vector2f& loc, const float& angle, const uint32_t& ID);
    
    /**
     * Get the neighbors of a given node.
     *
     * @param node Node to get neighbors for.
     *
     * @return Vector of neighbor nodes.
     */
    std::vector<NavGraphNode> getNeighbors(const NavGraphNode &node) const {
        if (neighbors_.find(node) == neighbors_.end()) {
            ROS_INFO("Unknown query node");
            return {};
        }
        std::vector<uint32_t> neighbor_nums = neighbors_.at(node);
        size_t nodes_num = nodes_.size();
        std::vector<NavGraphNode> neighbor_nodes;
        for (const auto neighbor_num : neighbor_nums) {
            if (neighbor_num < nodes_num) {
                neighbor_nodes.emplace_back(nodes_.at(neighbor_num));
            } else {
                ROS_ERROR("Neighbor did not exist in nodes list");
            }
        }
        return neighbor_nodes;
    }

private:

    /**
     * Nodes in the graph.
     */
    std::vector<NavGraphNode> nodes_;

    /**
     * Defines the neighbors of each graph node. Entries in the vector will give the index of the neighboring node in
     * the nodes_ list.
     */
    std::unordered_map<NavGraphNode, std::vector<uint32_t>> neighbors_;
 	
     /*
     * safety window for nodes, equal to max dimension of the car
     */
    const float kCarSafetyDistance = 0.535;  //TODO: Verify this

    /*
     * Given 2 potential neighebouring nodes, check the intersection with the map
     */
    bool checkIntersectionWithMap(const NavGraphNode& node1, const NavGraphNode& node2, const vector_map::VectorMap& map_);

    /*
     * Line intersection with map
     */
    bool checkLineIntersectionWithMap(const geometry::line2f& line, const vector_map::VectorMap& map_);

    /*
     * Check curved line intersection with map
     */
    bool checkCurveIntersectionWithMap(const float& startX, 
                                             const float& startY, 
                                             double& startAngle,
                                             const float& endX, 
                                             const float& endY, 
                                             double& endAngle, 
					     const vector_map::VectorMap& map_);
};

/**
 * Compute the heuristic to use in A*.
 *
 * @param node_pos      Current node position.
 * @param nav_goal_loc  Goal position.
 *
 * @return Heuristic estimating cost to goal.
 */
double computeHeuristic(const Eigen::Vector2f& node_pos,const Eigen::Vector2f& nav_goal_loc);

/**
 * Compute the cost between two nodes.
 *
 * @param current   Current node.
 * @param next      Next node.
 *
 * @return Cost to go from current node to next node.
 */
double ComputeCost(const NavGraphNode& current, const NavGraphNode& next);

/**
 * Get a path from the start location to the goal location using the fixed nav graph (will need to add temporary nodes).
 *
 * @param nav_goal_loc      Goal location.
 * @param nav_start_loc     Start location.
 * @param nav_graph         Navigation graph with fixed resolution to plan across.
 */
std::vector<NavGraphNode> GetPathToGoal(const NavGraphNode& nav_goal_loc, const NavGraphNode& nav_start_loc,
                                        const NavGraph &nav_graph);
} // end nav_graph




#endif //CS393R_STARTER_NAV_GRAPH_H
