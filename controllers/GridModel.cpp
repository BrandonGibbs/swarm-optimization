#include "GridModel.h"

#include <cmath>  // div, fabs, sqrt, INFINITY
#include <string> // used when overloading the '<<' operator for Node
#include <array>  // use when printing the grid: the left-most, right-most, and lowest values are stored here

#define GRID_START_WIDTH 64
#define GRID_START_AREA (GRID_START_WIDTH * GRID_START_WIDTH)
#define STATE_SIZE 0.17
#define IND_FACTOR 5.88235294117647 // 1/0.17 for mapping meters to grid units

/**
 * The heuristics functions determine the priority with which we will explore
 * states in the frontier (states not currently in the grid we may need to 
 * explore and are neighbors of the outermost states we've explored).
 *
 * The lower the priority, the sooner the value will get popped from the queue.
 */
double manhattanHeuristic(CVector2 pos, CVector2 targetPos){
	double cX = pos.GetX(),
	       cY = pos.GetY(),
		   tX = targetPos.GetX(),
		   tY = targetPos.GetY();
		   
	return fabs(cX - tX) + fabs(cY - tY);
}

double euclideanHeuristic(CVector2 pos, CVector2 targetPos){
	double cX = pos.GetX(),
	       cY = pos.GetY(),
		   tX = targetPos.GetX(),
		   tY = targetPos.GetY();
		   
	double diffX = cX - tX,
	       diffY = cY - tY;
		   
	return sqrt(diffX * diffX  +  diffY * diffY);
}



std::ostream & operator<<(std::ostream &os, Node n){
	std::string parent   = (n.parent == NULL)? "NULL":"a node",
	            explored = (n.explored)? "true":"false";
	os << "Node: explored: " << explored << ", cost: " << n.cost
	   << ", parent -> " << parent;
	return os;
}



Grid::Grid():
	stateSpace(GRID_START_AREA, {NULL, false, INFINITY}),
	nodesExplored(0){}
	
int Grid::index(CVector2 pos){
	// first, notice that we're swapping x and y axes
	/**
     * shift the simulation x axis (grid y axis) so that it's positive,
	 * then convert from meters to indeces by scaling and truncating
	 *
	 * add 1 so that all of the root's neighbors are in range
	 */
	int y = (pos.GetX() - startPos.GetX()) * IND_FACTOR + 1;
	
	/**
	 * shift the simulation y axis (grid x axis) to ensure it ends up 
	 * around the middle of the grid by first converting from meters to indeces,
	 * then shifting, and finally truncating
	 *
	 * we want the grid x in the middle so it can have some space to move
	 * along the grid x axis (simulation y axis) without having to resize the 
	 * array
	 */
	int x = pos.GetY() * IND_FACTOR + (GRID_START_WIDTH/2);
	
	int index = y * GRID_START_WIDTH + x;
	
	//std::cout << "index: " << pos << " -> " 
	//          << x << ", " << y << " -> " 
	//		  << index << std::endl;
	
	return index;
}

Node & Grid::operator[](CVector2 pos){
	return stateSpace[index(pos)];
}

/**
 * The robot will need to keep the previous time step's positioning sensor 
 * reading. This will be the parent position when transitioning states.
 *
 * Return the next target state.
 */
CVector2 Grid::insert(CVector2 pos, CVector2 parentPos, double cost){
	Node * parentNode;
	CVector2 nextState; // the value from the queue we will return
	
	if (nodesExplored == 0){
		// then this is the root node
		startPos = pos;
		parentNode = NULL;
		cost = 0;
	} else {
		parentNode = &(*this)[parentPos];
	}
	
	/**
	 * This has to be declared here since 
	 *   - the startPos needs to be set beforehand so that indexing will be 
	 *     correct
	 *   - we want a reference to the value in stateSpace, not a copy
	 */
	Node & currentNode = (*this)[pos];
	
	if (!currentNode.explored){
		currentNode = {
			parentNode,
			true,
			((parentNode != NULL)? parentNode->cost : 0) + cost
		};
		std::cout << "insert: " << currentNode << std::endl;
		nodesExplored += 1;
		
		std::vector <CVector2> neighbors = getNeighborMidPoints(pos);

		for (std::vector <CVector2>::iterator iter = neighbors.begin();
		     iter != neighbors.end();
			 ++iter){
			float priority = cost + euclideanHeuristic(*iter, targetPos);
			frontierStates.push(*iter, priority);
		}
		//frontierStates.printHeap();
		do {
			nextState = frontierStates.pop();
			std::cout << "popped from PQ: " << nextState << std::endl;
		} while (!frontierStates.empty() && contains(nextState));
		
		return nextState;
	}
	//std::cout << "The thing that wasn\'t supposed to happen happened. ;-;" << std::endl;
	return CVector2(0, 0); // this shouldn't ever happen
}

bool Grid::contains(CVector2 pos){
	return (*this)[pos].explored;
}

void Grid::setTarget(CVector2 pos){
	targetPos = pos;
}

CVector2 Grid::getNextState(){
	CVector2 nextState;
	do {
		nextState = frontierStates.pop();
		std::cout << "popped from PQ: " << nextState << std::endl;
	} while (!frontierStates.empty() && contains(nextState));
	
	return nextState;
}

/**
 * This is the inverse of index(vector), except that we can only return an 
 * approximation: the middle point of the state
 */
CVector2 Grid::getStateMidPoint(Node * pNode){
	int index = pNode - &stateSpace[0];
	
	div_t d = std::div(index, GRID_START_WIDTH);
	int x = d.rem,
	    y = d.quot;
		
	CVector2 corner (
		y / IND_FACTOR + startPos.GetX(), 
		(x - GRID_START_WIDTH / 2) / IND_FACTOR
	),
	center;
	
	int xSign = corner.GetX() >= 0? 1:-1,
	    ySign = corner.GetY() >= 0? 1:-1;
		
	center = corner + CVector2(xSign * STATE_SIZE / 2, ySign * STATE_SIZE / 2);
	//std::cout << "getStateMidPoint: " << index << " -> " << center << std::endl;
	return center;
}

CVector2 Grid::getStateMidPoint(CVector2 pos){
	return getStateMidPoint (&(*this)[pos]);
}

/**
 * Return an array of at most 8 neighbor states which aren't in the grid.
 *
 *    -------------------
 *    |  n  |  n  |  n  |
 *    -------------------
 *    |  n  |  C  |  n  |
 *    -------------------
 *    |  n  |  n  |  n  |
 *    -------------------
 *
 */
 std::vector<CVector2> Grid::getNeighborMidPoints (CVector2 pos){
	std::vector <CVector2> vecs;
	CVector2 topLeftMP = getStateMidPoint(pos) - CVector2(STATE_SIZE, STATE_SIZE);
	
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			CVector2 neighbor = topLeftMP + CVector2(i * STATE_SIZE, j * STATE_SIZE);
			if (!contains(neighbor)){
				vecs.push_back(neighbor);
			}
		}
	}
	
	 return vecs;
 }
 
 /*
 void Grid::printGrid (){
	 
	 
 }
 */