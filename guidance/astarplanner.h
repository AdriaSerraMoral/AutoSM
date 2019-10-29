////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       AstarPlanner.h
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-27-2019
///
///     @brief      This file includes an A* solver for planning given a grid
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "common/types.h"
#include "common/constants.h"
#include "searching/tree_search.h"

namespace AutoSM {
namespace guidance {

// TODO:
// To make it general, I could have a virtual Node Class
// that contains cost f(x), g(x), h(x) as doubles,
// contains the bool < operator for comparison,
// and a pointer to a parent node. Then, make the specific
// Node (e.g., MapPosNode) inherit from base Node class such that
// I can manipulate derived node (e.g., MapPosNode) using pointers
// to base node (Node). The virtual Astar function class would then
// Include a map of nodes, a virtual function to get a set or vector of
// valid nodes given my node (nodes connected to my current node),
// and a virtual function to get the Heuristic and cost to travel
// from current Node to potential Node, and last, a virtual function
// to check if node is goal.
// Once path is found, travel from child to parent to get
// list of Nodes that is the path, and have a function that translates
// this into whatever output (e.g., XYZ path in motion planning, strings, ...)

// For now, to do things faster, just create a 2D/3D planning Astar
// algorithm using squares/cubes in map.

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///                         NODES
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class AstarPlannerNode final : public searching::SearchNode {

public:

    AstarPlannerNode(const int N, const double x = 0, const double y = 0, const double z = 0);
    ~AstarPlannerNode() = default;

    // ---------------------- Node Data -----------------------------------
    double      x_          {0.0};          ///< X-position [m]
    double      y_          {0.0};          ///< Y-position [m]
    double      z_          {0.0};          ///< Z-position [m]

private:
    // ---------------------- Planner Dimension ----------------------------
    const int   Ndim        {1};            ///< World Dimension (1D, 2D, 3D)

};

using AstarPlannerNodeUniquePtr = std::unique_ptr<AstarPlannerNode>;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///                         SOLVERS
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




} //namespace guidance
} // namespace AutoSM

