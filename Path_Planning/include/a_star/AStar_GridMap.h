#ifndef AStar_GridMap_H_
#define AStar_GridMap_H_

#include "Node.h"
#include <iostream>
#include <vector> 
#include <SFML/Graphics.hpp>
#include <sstream>
#include <list>
#include <set>
#include <chrono>
#include <thread>

// declare chrono_literals namesoace
using namespace std::chrono_literals;


class AStar_GridMap{
	int grid_size_X_ = 0, grid_size_Y_ = 0;
    
	double node_radius = 0.0;
	
// initiate the start and goal position    
    std::vector<std::vector<Node> > grid_;
	int start_pose_X_ = 0, start_pose_Y_ = 0;
	int goal_pose_X_ = 0, goal_pose_Y_ = 0;

    public:
        AStar_GridMap(int grid_X, int grid_Y, double radius) 
                    :   grid_size_X_(grid_X), grid_size_Y_(grid_Y),
                        node_radius(radius){
              // number of nodes
              int x_nodes = grid_size_X_/(2*node_radius);
              int y_nodes = grid_size_Y_/(2*node_radius);
		
		// Grid matrix
              grid_ = std::vector<std::vector<Node> >(x_nodes, std::vector<Node>(y_nodes));
        }

        void create_grid();
        void start(int,int);
        void goal(int,int);
        void calc_node_weights(Node&, Node&);
        std::vector<Node> retrace_path(std::vector<Node>&);
        bool push_neighbours_to_list(std::set<std::pair<int, int>>&,
                                     std::vector<Node>& ,
                                     int, int);
        void generate_shortest_path();
        void draw_grid(std::vector<Node>&);
};

#endif

