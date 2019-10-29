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

#include "tree_search.h"

namespace AutoSM {
namespace searching {

template< typename T>
double TreeSearch<T>::getTrajectoryCost(std::set<SearchNode>& trajectory) {
    double total_cost {0.0};
    for(const auto& node : trajectory) {
        total_cost += node.getTravelCost();
    }

    return total_cost;
}

template< typename T>
void TreeSearch<T>::addValidNodesToOpenList(SearchNode* Node) {

    // get node children
    const vpSearchNode& children = Node->getChildren();

    // Add children to priority list
    for ( SearchNode* n_idx : children ) {
        // First, check if node is goal
        if( isGoalNode(n_idx) ) {
            // it is goal, handle goal
            handleGoalNode(n_idx);
        }

        // check if we have to add node
        if( shouldAddChildNode(Node, n_idx) ) {
            // Add child to openList
            openNodes.push(n_idx);
        }
    }

}

template< typename T>
void TreeSearch<T>::setNodeHeuristic(SearchNode *origin, SearchNode *destination) {
    // Empty by default, no heuristic
    destination->setHeuristicCost(0.0);
}

template< typename T>
void TreeSearch<T>::setNodeTravelCost(SearchNode* origin, SearchNode* destination) {
    // By default, unit cost to travel all nodes
    destination->setTravelCost(1.0);
}

template< typename T>
bool TreeSearch<T>::shouldAddChildNode(SearchNode* parent, SearchNode* child) {

    // By default, skip visited nodes except if cost is lower
    // if for some reason we made it here with goal node, do not add
    if( isGoalNode(child) ){
        return false;
    }

    // check if visited
    if( child->isVisited() ) {
        // it is visited, check if cost is lower

        // get current cost
        double cost_prev = child->getTotalCost();

        // update travel and heuristic cost
        setNodeHeuristic(parent, child);
        setNodeTravelCost(parent, child);

        // check cost
        if( child->getTotalCost() < cost_prev ){

            // cost is lower, change parent and add
            child->setParent(parent);
            return true;

        } else {
            // cost is n ot lower, do not add
            return false;
        }

    } else {
        // Node is not visited update cost and add
        setNodeHeuristic(parent, child);
        setNodeTravelCost(parent, child);
        return true;
    }

}

//std::set<SearchNode> TreeSearch::solveTreeSearch(const SearchNode& origin, const SearchNode& goal) {
//    // store inputs to member variables
//    origin_ = origin;
//    goal_ = goal;

//    // Add origin to open set
//    // Origin is assumed to be feasible, do check in Derived class that
//    // will call this function.
//    if(origin_.getTotalCost() > 0.0)
//        origin_.setTotalCost(0.0);

//    // Origin has obviously been visited
//    origin_.setVisisted();

//    // Add origin to openNodes list
//    openNodes.push(&origin_);

//    // Create flag to know when we found the goal
//    bool found_goal {false};

//    // MAIN LOOP, While true
//    while(true) {

//        if ( stopLoopCriteria() );
//            return makePath();

//        // get the best node in the openNodes set
//        AstarNode*  node_idx = openNodes.top();
//        openNodes.pop();

//        // Check if node is the goal
//        if ( isGoalNode(node_idx) ) {
//            found_goal = true;
//            goal_ = *node_idx;
//            return makePath();
//        }

//        // Get New Children
//        // This virtual function adds the pointers to new
//        // valid nodes, which have updated costs to the
//        // openNodes priority queue
//        addValidNodesToOpenList(node_idx);

//    }

//    return makePath();

//}

template< typename T>
std::set<SearchNode> TreeSearch<T>::makePath() {
    std::set<SearchNode> trajectory;

    if( !goal_.getParent() ){
        return trajectory;
    }

    trajectory.insert(goal_);

    SearchNode* node_idx = &goal_;

    while( node_idx->getParent() ) {

        trajectory.insert( *(node_idx->getParent()) );

        node_idx = node_idx->getParent();

    }

    return trajectory;
}


}   // namespace searching
}   // namespace AutoSM


