/**
 * We will model the robots environment as a grid where each cell/state
 * represents 0.17 m^2 of space because footbots are 17 cm in diameter.
 *
 * To be able to keep track of which paths have been attempted and compare
 * between paths, each node will store its parent (the previous state) and its 
 * cost. The cost is the distance travelled from the start state to the 
 * current state, so it's a global cost, not a local cost (distance/difficulty
 * to arrive from parent state). If we decide to introduce inclined planes
 * or rough terrain, we can add a local cost field in the future.
 *
 * Since the robots will more than likely only visit a small subset of the 
 * states in the grid, it'll probably be smarter to use another, more memory 
 * efficient, data structure. This one is quick and easy to implement though,
 * and it's easier to print the grid to debug what's going on if we have the 
 * whole state space available.
 *
 * In order to abstract away the details of indexing, the index(CVector2) 
 * method returns the index into the 1D array storing all the states, and the 
 * subscript operator just calls index. Basically, we will map Position 
 * vectors (2 continuous coordinates in meters) to a unique integer index. We 
 * will need to keep in mind that the axes are flipped in our model as well. 
 * I decided to do it this way since it may need to be resized in order 
 * to accomodate more states, and increasing the grid width makes the 
 * computation more complex. So my assumptions are:
 *     - the x will probably remain within [0, GRID_START_WIDTH]
 *     - the target is in the +y direction
 *     - the states in the -y direction won't need 
 *       to be explored (before the start state)
 *
 * Bearing in mind that the grid model is a 1D array, and the indeces are 
 * computed like this: i <- gridY * gridWidth + gridX, it will look like the following:
 * 
 *    Grid model:                 Simulation:
 * --------------- -> +x       --------------- -> -y
 * |o     S      |             |      T      |
 * |             |             |             |
 * |             |             |      o      |
 * |             |             |             |
 * |      T      |             |      S      |
 * ---------------             ---------------
 *               |                           |
 *               V                           V
 *              +y                          -x
 * 
 * where Simulation is the top-down view with camera 0, and
 *   - S is the start state,
 *   - T is the target state,
 *   - o is the origin (0, 0)
 */
#ifndef GRID_MODEL_H
#define GRID_MODEL_H

#include <iostream>
#include <vector>
//#include <unordered_map>
#include <argos3/core/utility/math/vector2.h>

#include "PQueue.h"

using namespace argos;

struct Node {
	Node * parent;
	bool   explored;
	double cost;
};

class Grid {
public:
	Grid();
	
	/**
	 * Insert the current position and its parent to remember path. Return the 
	 * item with the lowest priority from the priority queue.
	 *
	 * The robot will need to keep the previous time step's positioning sensor 
	 * reading. This will be the parent position when transitioning states.
	 */
	CVector2 insert (CVector2 pos, CVector2 parentPos, double cost);
	
	
	bool contains (CVector2 pos);
	
	/**
	 * We have to set the target location here (called in Init) since we don't 
	 * have access to the target in the robot constructor
	 */
	void setTarget (CVector2 pos);
	
	/**
	 * Return the next item from the priority queue.
	 *
	 * This will be called when there's no path to the state popped previously,
	 * like if there's a barrier.
	 */
	CVector2 getNextState();
	
	/**
	 * Overloaded getStateMidPoint since Node shouldn't be accessed outside of
	 * object definition
	 */
	CVector2 getStateMidPoint (CVector2 pos);
	
	/**
	 * Get neighbor states
	 */
	std::vector<CVector2> getNeighborMidPoints (CVector2 pos);
	
private:
	int index (CVector2 pos);
	Node & operator[](CVector2 pos);
	
	/**
	 * This is the inverse of index(vector), except that we can only return an 
	 * approximation: the middle point of the state
	 */
	CVector2 getStateMidPoint (Node * pNode);
	
	
	
	
	std::vector        <Node>     stateSpace;
	PQueue             <CVector2> frontierStates;
	//std::unordered_map <CVector2> closedSet;
	
	CVector2 startPos, targetPos;
	int nodesExplored;
};

#endif