////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       tree_search.h
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-27-2019
///
///     @brief      This file includes an A* solver for planning given a grid
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "common/types.h"

namespace AutoSM {
namespace searching {

// Forward-Declaration
class SearchNode;
struct CompareMinTotalCostFirst;
struct CompareMinTravelCostFirst;
struct CompareMaxLevelFirst;
struct CompareMinLevelFirst;

// Useful Declarations
using vpSearchNode = std::vector<SearchNode*>;
using SearchNodeUniquePtr = std::unique_ptr<SearchNode>;
using AstarPriorityQueue = std::priority_queue< SearchNode*, vpSearchNode, CompareMinTotalCostFirst>;
using DijkstraPriorityQueue = std::priority_queue< SearchNode*, vpSearchNode, CompareMinTravelCostFirst>;
using DFSPriorityQueue = std::priority_queue< SearchNode*, vpSearchNode, CompareMaxLevelFirst>;
using BFSPriorityQueue = std::priority_queue< SearchNode*, vpSearchNode, CompareMinLevelFirst>;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///                         NODES
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *  @brief  SearchNode class
 *
 *          This class is inherited by all Custom Nodes that we want
 *          To use within our Astar solver.
 *
 */
class SearchNode {
protected:

    double          f_cost       {0.0};         ///< Total cost of Node
    double          g_cost       {0.0};         ///< Cost of traveling to Node
    double          h_cost       {0.0};         ///< Heuristic Cost

    bool            visited      {false};       ///< Flag to know if Node has been visited

    SearchNode*     parent       {nullptr};     ///< Pointer to Parent Node

    vpSearchNode    children;

    int             level       {0};            ///< Level of this Node in the Tree

public:

    // -------------- Getters ---------------------------------------------------------------

    const bool& isVisited() const {return visited;}

    const double& getTotalCost() const { return f_cost; }

    const double& getTravelCost() const { return g_cost; }

    const double& getHeuristicCost() const { return h_cost; }

    SearchNode* getParent() const { return parent; }

    const int& getLevel() const { return level; }

    const vpSearchNode& getChildren() const { return children; }

    // ---------- Setters --------------------------------------------------------------------

    void setVisisted() {visited = true;}

    void setTotalCost(const double& f_cost ) { this->f_cost = f_cost; }

    void setTravelCost(const double& g_cost ) {
        // set new travel cost
        this->g_cost = g_cost;

        // update total cost
        this->f_cost = this->g_cost + this->h_cost;

    }

    void setHeuristicCost(const double& h_cost ) {
        // set new heuristic cost
        this->h_cost = h_cost;

        // update total cost
        this->f_cost = this->g_cost + this->h_cost;
    }

    void setParent(SearchNode* parent) {
        this->parent = parent;
        this->level = parent->level + 1;
    }

    void setParent( SearchNode& parent ) {
        this->parent = &parent;
        this->level = parent.level + 1;
    }

    void setChildren( vpSearchNode& children ) {
        this->children = children;
    }

    void addChild( SearchNode* child ) {
        this->children.push_back(child);
    }

    void addChild( SearchNode& child ) {
        this->children.push_back(&child);
    }

    void setLevel( const int level ) {
        this->level = level;
    }

};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///                      Search Comparator
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CompareMaxLevelFirst {
    bool operator()( const SearchNode& lhs, const SearchNode& rhs ) {
        return (lhs.getLevel() < rhs.getLevel());
    }
};

struct CompareMinLevelFirst {
    bool operator()( const SearchNode& lhs, const SearchNode& rhs ) {
        return (lhs.getLevel() > rhs.getLevel());
    }
};

struct CompareMaxTotalCostFirst {
    bool operator()( const SearchNode& lhs, const SearchNode& rhs ) {
        return (lhs.getTotalCost() < rhs.getTotalCost());
    }
};

struct CompareMinTotalCostFirst {
    bool operator()( const SearchNode& lhs, const SearchNode& rhs ) {
        return (lhs.getTotalCost() > rhs.getTotalCost());
    }
};

struct CompareMaxTravelCostFirst {
    bool operator()( const SearchNode& lhs, const SearchNode& rhs ) {
        return (lhs.getTravelCost() < rhs.getTravelCost());
    }
};

struct CompareMinTravelCostFirst {
    bool operator()( const SearchNode& lhs, const SearchNode& rhs ) {
        return (lhs.getTravelCost() > rhs.getTravelCost());
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///                      Searchers
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< typename T>
class TreeSearch
{
public:

//    std::set<SearchNode> solveTreeSearch(const SearchNode& origin, const SearchNode& goal);

    static double getTrajectoryCost( std::set<SearchNode>& trajectory );

protected:

    std::set<SearchNode> makePath();

    virtual void addValidNodesToOpenList(SearchNode* Node);

    virtual bool isGoalNode(SearchNode*  Node) = 0;

    virtual void setNodeHeuristic(SearchNode* origin, SearchNode* destination);

    virtual void setNodeTravelCost(SearchNode* origin, SearchNode* destination);

    virtual void handleGoalNode(SearchNode* goal) = 0;

    virtual bool shouldAddChildNode(SearchNode* parent, SearchNode* child);

    virtual bool stopLoopCriteria() = 0;

    SearchNode                       origin_;
    SearchNode                       goal_;

    double                          costTrajectory          {0.0};

    bool                            run_loop                {true};

    T                               openNodes;

};

}
}
