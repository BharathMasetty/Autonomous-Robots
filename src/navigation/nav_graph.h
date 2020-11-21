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
#include <boost/geometry/index/rtree.hpp>

//using namespace Navigation;
namespace bg = boost::geometry;
typedef bg::model::point<float, 2, bg::cs::cartesian> point_type;
typedef bg::model::box<point_type> box_type;
typedef std::pair<box_type, uint32_t> value;

namespace nav_graph {


/**
 * Maximum curvature (assumed to be the same on both sides of the car, so c_min = -1 *c_max).
 *
 * TODO Investigate this value.
 */
const double kMaxCurvature = 1.0;

/**
 * Resolution of the navigation graph (provides how far apart the nearest two nodes are).
 */
static const double kGridResolution = 1.0; // TODO tune

/**
 * Number of increments in the grid resolution.
 */
static const int kStaggerCount = 4;

/**
 * Staggered (finer) resolution.
 */
static const double kStaggeredGridResolution = kGridResolution / kStaggerCount;

/**
 * Angular resolution of the navigation graph. With 90 degrees, there are 4 possible angles that a node could need
 * to be in. (If we allow backward motion, we could probably consider 0 and 180 to be the same and 90/-90 to be the
 * same, since the car can reach the node still, but we'll handle this in edge connections instead.
 */
static const double kAngularOptionsFromNavGraph = M_PI_2;

/**
 * Number of angular options.
 */
static const int kNumAngularOptions = M_2PI / kAngularOptionsFromNavGraph;

/**
 * Convert the given angle to be properly in the nav graph node range. Assumes that this should be a multiple of
 * kAngularOptionsFromNavGraph.
 *
 * @return Angle that is one of the options for the discretized node angles in the proper NavGraphNode range.
 */
static double convertAngleToNavGraphNodeRange(const double &angle) {
    int orientation_index = (((int) round(angle / kAngularOptionsFromNavGraph)) % kNumAngularOptions);
    if (orientation_index < 0) {
        orientation_index += kNumAngularOptions;
    }
    return math_util::AngleMod(orientation_index * kAngularOptionsFromNavGraph);
}

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
            node_orientation_ = convertAngleToNavGraphNodeRange(node_orientation_);
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

    /**
     * Get if the node is grid aligned.
     *
     * @return True if the node is grid aligned, false if not.
     */
    bool isGridAligned() const {
        return grid_aligned_;
    }

private:

    /**
     * Value that the grid aligned nodes will have for the non-grid-aligned identifier. Will overwrite what is passed
     * in constructor to ensure this value doesn't matter for grid aligned nodes.
     */
    static const uint32_t kGridAlignedIdentifierValue = 0;


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

            uint32_t non_grid_id = 0;
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

    /**
     * Default constructor.
     *
     * Setting next_non_aligned_index to 1 since 0 is taken for aligned nodes
     */
    NavGraph() : next_non_aligned_id_(1) {}
    
    /*
     * Creates the navigation graph without initial and final node
     */
    void createNavigationGraph(const vector_map::VectorMap& map, const vector_map::VectorMap &inflated_map);
    
    /*
     * To visualize the nodes
     */
    void visualizeNavigationGraphPoints(amrl_msgs::VisualizationMsg &viz_msg);

    /*
     * To visualize the nodes
     */
    void visualizeNavigationGraphEdges(amrl_msgs::VisualizationMsg &viz_msg);

    /**
     * Visualize the edges from other nodes to the goal. Has a separate message because other display methods are from
     * edges not to edges.
     *
     * @param goal_pos[in]  Goal position (end edges here).
     * @param viz_msg[out]  Vis msg to add the edges to the goal to.
     */
    void visualizeGoalEdges(const Eigen::Vector2f &goal_pos, amrl_msgs::VisualizationMsg &viz_msg);

    /**
     * Visualize connections from the given node.
     *
     * @param node[in]      Node to visualize connections from.
     * @param viz_msg[out]  Visualization message to add the edges to.
     */
    void visualizeConnectionsFromNode(const NavGraphNode &node, amrl_msgs::VisualizationMsg &viz_msg);
    
    /**
     * Get the neighbors of a given node.
     *
     * @param node Node to get neighbors for.
     *
     * @return Vector of neighbor nodes.
     */
    std::vector<NavGraphNode> getNeighbors(const NavGraphNode &node) const {
        if (neighbors_.find(node) == neighbors_.end()) {
            ROS_INFO_STREAM("Unknown query node " << node.getNodePos().x() << ", " << node.getNodePos().y() << ", " << node.getNodeOrientation());
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

    /**
     * Create a copy of the nav graph. This should be done before adding the start/goal nodes to the graph so we don't
     * have to deal with removing those later.
     *
     * @return Copy of the nav graph.
     */
    NavGraph createCopy() {
        return NavGraph(neighbors_, nodes_, rtree_, map_, node_index_map_, next_non_aligned_id_);
    }

    /**
     * Add the start and goal to the graph. This should only be done on copies so that there's only one goal at a time.
     *
     * @param start Start position and angle.
     * @param goal  Goal position.
     *
     * @return Pair of nodes for the start and goal, respectively.
     */
    std::pair<NavGraphNode, NavGraphNode> addStartAndGoal(const std::pair<Eigen::Vector2f, float> &start, Eigen::Vector2f &goal);

    /**
     * Update the start node. This can be done to replan when the goal is the same, but if there is a new goal, you
     * should make a copy of the graph without the start and goal nodes and then call addStartAndGoal.
     *
     * @param start Start position and angle.
     *
     * @return Node for the start.
     */
    NavGraphNode updateStart(const std::pair<Eigen::Vector2f, float> &start);

private:

    /**
     * Constructor used to copy nav graphs.
     *
     * @param neighbors             Map of nodes to the list of indices for its neighbors.
     * @param nodes                 List of nodes.
     * @param rtree                 Data structure that stores node indices by their positions.
     * @param map                   Map to use to check for collisions.
     * @param node_index_map        Map of a node to its index in the nodes list
     * @param next_non_aligned_id   Index for the next non-aligned node (this is the key for the node, not the
     *                              index in the nodes list)
     */
    NavGraph(const std::unordered_map<NavGraphNode, std::vector<uint32_t>> neighbors,
             const std::vector<NavGraphNode> nodes,
             const bg::index::rtree<value, bg::index::quadratic<16>> &rtree,
             const vector_map::VectorMap &map,
             const std::unordered_map<NavGraphNode, uint32_t> node_index_map,
             const uint32_t &next_non_aligned_id) {
        neighbors_ = neighbors;
        nodes_ = nodes;
        rtree_ = rtree;
        map_ = map;
        node_index_map_ = node_index_map;
        next_non_aligned_id_ = next_non_aligned_id;
    }

    /*
    * safety window for nodes, equal to max dimension of the car
    */
    static constexpr float kCarSafetyDistance = 0.3;  //TODO: Verify this

    /**
     * Maximum difference for an angle to be considered close.
     */
    static constexpr double kCloseAngleMargin = M_PI / 6;

    /**
     * Area to search within to get connections for start/goal nodes.
     */
    static constexpr double kSearchRangeOneDirection = 1.5;

    /**
     * For the rtree, point lookups weren't working, so this is the dimension of the box that represents the point for
     * each node.
     */
    static constexpr double kBoxAroundNodeDim = 0.000001;

    /**
     * Color to use when displaying the graph nodes and edges.
     */
    static const uint32_t kGraphColor = 0x34b4eb;

    /*
     * Constant used t0 magnifiy the wall dimensions to avoid node at gaps between walls.
     */
    static constexpr float kWallMultiplier = 0.9;

    /**
     * Data structure that stores indices of nodes based on their location (not angle).
     */
    bg::index::rtree<value, bg::index::quadratic<16>> rtree_;

    /**
     * Next id to use for non-aligned nodes.
     */
    uint32_t next_non_aligned_id_;

    /**
     * Map of the environment to check collisions against.
     */
    vector_map::VectorMap map_;

    /**
     * Inflated map of the environment.
     */
    vector_map::VectorMap inflated_map_;

    /**
     * Map of node to its index in the nodes list.
     */
    std::unordered_map<NavGraphNode, uint32_t> node_index_map_;

    /**
     * Nodes that have connections to the goal.
     */
    std::vector<NavGraphNode> goal_neighbors_;

    /**
     * Nodes in the graph.
     */
    std::vector<NavGraphNode> nodes_;

    /**
     * Defines the neighbors of each graph node. Entries in the vector will give the index of the neighboring node in
     * the nodes_ list.
     */
    std::unordered_map<NavGraphNode, std::vector<uint32_t>> neighbors_;

    /**
     * Get the angle after the curvature is executed from the start position.
     *
     * @param curvature Curvature that is executed.
     * @param start     Start position and angle.
     * @param end       Position that is reached with the curvature
     *
     * @return Angle that the car is at when it reaches the end position if it executes the given curvature and starts
     * at the start position.
     */
    float getAngleChangeAfterCurvatureExecution(const double &curvature, const std::pair<Eigen::Vector2f, float> &start,
                                                const Eigen::Vector2f &end);

    /*
     * To add initial and final node.
     */
    NavGraphNode createUnalignedNode(const Eigen::Vector2f& loc, const float& angle);

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
                                       const double& startAngle,
                                       const float& endX,
                                       const float& endY,
                                       const double& endAngle,
                                       const vector_map::VectorMap& map);

    /**
     * Get the minimum and maximum corners from the map.
     *
     * @param map Map defining the walls, etc of the world.
     *
     * @return Corners (min and max) that provide an axis aligned bounding box of the map.
     */
    std::pair<Eigen::Vector2f, Eigen::Vector2f> getMapCorners(const vector_map::VectorMap &map);

    /**
     * Create the initial set of node positions.
     *
     * @param map_corners Corners providing bounding box for the map
     *
     * @return Set of positions where we should consider putting nodes in the map.
     */
    std::vector<Eigen::Vector2f> createInitialNodePositions(
            const std::pair<Eigen::Vector2f, Eigen::Vector2f> &map_corners);

    /**
     * Prune positions for nodes near obstacles in the map from the nodes list and return the remaining positions.
     *
     * @param unpruned_nodes    Positions to consider putting nodes.
     * @param map               Map containing static obstacles.
     *
     * @return Node positions that aren't too close to an obstacle.
     */
    std::vector<Eigen::Vector2f> pruneNodesNearObstacles(const std::vector<Eigen::Vector2f> &unpruned_nodes,
                                                         const vector_map::VectorMap &map);

    /**
     * Get nodes near the given position.
     *
     * @param search_pos Position to get nodes near.
     *
     * @return Nodes near the given position.
     */
    std::vector<NavGraphNode> getNearbyNodes(const Eigen::Vector2f &search_pos);
};

/**
 * Get the pose of the src frame object in the target frame given the pose of the source frame in the target frame.
 *
 * Ex. get the pose of an object in the map frame given the pose in the base link frame and the pose of the base
 * link in the robot's frame.
 *
 * @param src_frame_point                     Location of the point in the source frame (base link frame in the example).
 * @param src_frame_angle                     Angle of the point in the source frame (base link frame in the example).
 * @param src_frame_pos_rel_target_frame      Position of the origin of the source frame in the target frame
 *                                            (position of the base link frame in the map frame in the example).
 * @param src_frame_angle_rel_target_frame    Angle of the source frame relative to the target frame (angle from map
 *                                            x axis to base link x axis in the example).
 *
 * @return Pose of point in the target frame (map frame in this example).
 */
std::pair<Eigen::Vector2f, float> transformPoint(const Eigen::Vector2f &src_frame_point, const float &src_frame_angle,
                                                 const Eigen::Vector2f &src_frame_pos_rel_target_frame,
                                                 const float &src_frame_angle_rel_target_frame);

/**
 * Get the pose of the src frame object in the target frame given the pose of the target frame in the source frame.
 *
 * Ex. get the pose of an object relative to base link given the pose of the object in the map frame and the pose of
 * the robot in the map frame
 *
 * @param src_frame_point                     Location of the point in the source frame (map frame in the example).
 * @param src_frame_angle                     Angle of the point in the source frame (map frame in the example).
 * @param target_frame_pos_rel_src_frame      Position of the origin of the target frame in the src frame
 *                                            (position of the base link frame in the map frame in the example).
 * @param target_frame_angle_rel_src_frame    Angle of the target frame relative to the src frame (angle from map
 *                                            x axis to base link x axis in the example).
 * @return Pose of point in the target frame (base link frame in this example).
 */
std::pair<Eigen::Vector2f, float> inverseTransformPoint(const Eigen::Vector2f &src_frame_point,
                                                        const float &src_frame_angle,
                                                        const Eigen::Vector2f &target_frame_pos_rel_src_frame,
                                                        const float &target_frame_angle_rel_src_frame);

/**
 * Get the curvature from the given pose to the given position.
 *
 * @param coming_from   Position and angle that car is starting at.
 * @param going_to      Location that car should be at after executing the returned curvature for some amount of time.
 *
 * @return Curvature that will take the car from the coming_from position to the going_to position.
 */
double getCurvatureFromPoseToPosition(const std::pair<Eigen::Vector2f, float> &coming_from, const Eigen::Vector2f &going_to);

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
