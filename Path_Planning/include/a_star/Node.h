#ifndef _Node_H_
#define _Node_H_


// creat a node
struct Node{

	Node* parent;

	// obstacle 
	bool is_walkable = false;

	// initial position
	int pose_X = 0, pose_Y = 0;

	// initial G cost and H cost (Heuristics)
	double g_cost = 1000.0, h_cost = 1000.0;
};

#endif

