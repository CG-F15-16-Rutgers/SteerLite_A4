//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );

			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		maze.resize(gSpatialDatabase->getNumCellsX());
		for (int i = 0; i < maze.size(); ++i) {
			maze[i].resize(gSpatialDatabase->getNumCellsZ());
			for ( int j = 0; j < maze[i].size(); ++j) {
				maze[i][j].cell.x = i;
				maze[i][j].cell.z = j;
				maze[i][j].cell.y = 0;
				maze[i][j].index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				gSpatialDatabase->getLocationFromIndex(maze[i][j].index, maze[i][j].point);
			}
		}
		
		unsigned int start_index = gSpatialDatabase->getCellIndexFromLocation(start);
		unsigned int goal_index = gSpatialDatabase->getCellIndexFromLocation(goal);
		unsigned int start_x, start_z;
	       unsigned int goal_x, goal_z;
       		gSpatialDatabase->getGridCoordinatesFromIndex(start_index, start_x, start_z); 
       		gSpatialDatabase->getGridCoordinatesFromIndex(goal_index, goal_x, goal_z); 
		AStarPlannerNode* startNode = &maze[start_x][start_z];//new AStarPlannerNode(start, 0, heuristicManhattan(start_id, goal_id), NULL);               //use euclidean as alternative 
		AStarPlannerNode* goalNode = &maze[goal_x][goal_z];//new AStarPlannerNode(goal, 0, 0, NULL);

		openSet.insert(startNode);

		while(!openSet.empty()){
			AStarPlannerNode* currentNode = *(openSet.begin());
			openSet.erase(openSet.begin());
			closeSet.insert(currentNode);

			if(*currentNode == *goalNode){
				std::cout << "We found a path!!!!" << std::endl;
				return constructPath(agent_path, currentNode);
			}

			std::vector<AStarPlannerNode*> neighbors = getNeighborNodes(currentNode);//gSpatialDatabase->getCellIndexFromLocation(currentNode->point.x, currentNode->point.z));

			for (std::vector<AStarPlannerNode*>::iterator it = neighbors.begin(); 
					it != neighbors.end(); ++it){
				if(closeSet.find(*it) != closeSet.end()){
					continue;
				}

				// calculate distance from current node to this neighbor
				double new_g = currentNode->g + 1;

				if(new_g < (*it)->g){
					(*it)->parent = currentNode;
					(*it)->g = new_g;
					(*it)->f = new_g +  heuristicManhattan((*it)->index, goalNode->index);

					if(openSet.find(*it) == openSet.end()) {
						openSet.insert(*it);
					} else {
						// need to modify to update openSet
						openSet.erase(openSet.find(*it));
						openSet.insert(*it);
					}
				}
			}
		}

		std::cout << "No path!!!!" << std::endl;
		return false;
	}

	std::vector<AStarPlannerNode*> AStarPlanner::getNeighborNodes(AStarPlannerNode* currentNode) { //unsigned int current_id){
		unsigned int x = currentNode->cell.x;
		unsigned int z = currentNode->cell.z;

		std::vector<AStarPlannerNode*> neighbors;

		for (int i = x - 1; i < x + 1; ++i) {
			for (int j = z - 1; j < z + 1; ++j) {
				if (i == x && j == z) continue;
				if (i < 0 || i >= maze.size()) continue;
				if (j < 0 || j >= maze[i].size()) continue;

				if(!canBeTraversed(maze[i][j].index)){
					continue;
				}
				neighbors.push_back(&maze[i][j]);
			}
		}

		return neighbors;
	}

	bool AStarPlanner::constructPath(std::vector<Util::Point>& agent_path, AStarPlannerNode* endNode)
	{
		if(!endNode){
			return false;
		}
		while(endNode->parent){
			agent_path.insert(agent_path.begin(), endNode->point);
			endNode = endNode->parent;
		}    
		agent_path.insert(agent_path.begin(), endNode->point);
		return true;
	}

	int AStarPlanner::heuristicManhattan(int start_id, int goal_id){
		unsigned int start_x, start_z, goal_x, goal_z;
		gSpatialDatabase->getGridCoordinatesFromIndex(start_id, start_x, start_z);
		gSpatialDatabase->getGridCoordinatesFromIndex(goal_id, goal_x, goal_z);

		return double(abs(goal_x - start_x) + abs(goal_z - start_z));
	}

	int AStarPlanner::heuristicEuclidean(int start_id, int goal_id){
		unsigned int start_x, start_z, goal_x, goal_z;
		gSpatialDatabase->getGridCoordinatesFromIndex(start_id, start_x, start_z);
		gSpatialDatabase->getGridCoordinatesFromIndex(goal_id, goal_x, goal_z);

		return sqrt(pow(double((goal_x - start_x)), 2.0) + pow(double(goal_z - start_z), 2.0));   
	}

	}
