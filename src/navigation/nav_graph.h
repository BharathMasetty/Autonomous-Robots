//
// Created by amanda on 11/12/20.
//


#ifndef CS393R_STARTER_NAV_GRAPH_H
#define CS393R_STARTER_NAV_GRAPH_H

#include "eigen3/Eigen/Dense"
#include <shared/math/math_util.h>
#include <boost/functional/hash.hpp>
#include <unordered_map>

namespace nav_graph {

/**
 * Resolution of the navigation graph (provides how far apart the nearest two nodes are).
 */
static const double kGridResolution = 0.25; // TODO tune

/**
 * Angular resolution of the navigation graph. With 90 degrees, there are 4 possible angles that a node could need
 * to be in. (If we allow backward motion, we could probably consider 0 and 180 to be the same and 90/-90 to be the
 * same, since the car can reach the node still, but we'll handle this in edge connections instead.
 */
static const double kAngularOptionsFromNavGraph = math_util::DegToRad(90);

class NavGraphNode {
public:

    NavGraphNode(const Eigen::Vector2f &node_pos, const float &node_orientation) : node_pos_(node_pos),
    node_orientation_(node_orientation) {}

    bool operator==(const NavGraphNode &other) {
        return convertToKeyForm() == other.convertToKeyForm();
    }

    /**
     * Convert the nav graph node to ints based on the resolution for translation and angles.
     * @return
     */
    std::pair<std::pair<int32_t, int32_t>, int8_t> convertToKeyForm() const;

    Eigen::Vector2f getNodePos() {
        return node_pos_;
    }

    float getNodeOrientation() {
        return node_orientation_;
    }

private:

    /**
     * Location of the node. X and y should both be multiples of kGridResolution.
     */
    Eigen::Vector2f node_pos_;

    /**
     * Orientation for the node. Should be a multiple of kAngularOptonsFromNavGraph
     */
    float node_orientation_;
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
    // TODO fill in

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
};
} // end nav_graph




#endif //CS393R_STARTER_NAV_GRAPH_H
