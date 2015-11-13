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

        int start_id = gSpatialDatabase->getCellIndexFromLocation(start.x, start.z);
        int goal_id = gSpatialDatabase->getCellIndexFromLocation(goal.x, goal.z);

        AStarPlannerNode startNode = new AStarPlannerNode(start, 0, heuristicManhattan(start_id, goal_id), null);               //use euclidean as alternative 
        AStarPlannerNode goalNode = new AStarPlannerNode(goal, 0, 0, null);

        openSet.push(&startNode);

        while(!openSet.empty()){
            const AStarPlannerNode* currentNode = openSet.top();
            openSet.pop();
            closeSet.push(currentNode);

            if(*currentNode == goalNode){
                return constructPath(agent_path, currentNode);
            }

            std::vector<AStarPlannerNode> neighbors = getNeighborNodes(gSpatialDatabase->getCellIndexFromLocation(currentNode->poit.x, currentNode->point.z));

            for (std::vector<AStarPlannerNode>::iterator it = neighbors.begin(); 
                    it != neighbors.end(); ++it){
                if(it->point.x< 0 || it->point.x > gSpatialDatabase->getNumCellsX - 1
                        || it->point.z < 0 || it->point.z > gSpatialDatabase->getNumCellsZ - 1)
                {
                    continue;
                }

                int neighbor_id = gSpatialDatabase->getCellIndexFromLocation(it->point); 

                if(!canBeTraversed(neighbor_id)){
                    continue;
                }

                if(closeSet.find(it) != closeSet.end()){
                    continue;
                }
                double new_g = currentNode.g + 1;

                if(new_g < it->g){
                    it->parent = currentNode;
                    it->g = new_g;
                    it->f = new_g +  heuristicManhattan(neighbor_id, goal_id);
                }
                
                if(openSet.find(it) == openSet.end()){
                    openSet.push(it);
                }
            }

        }

		return false;
	}

    std::vector<AStarPlannerNode> getNeighborNodes(unsigned int current_id){
        unsigned int x, z;
        gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
        std::vector<AStarPlannerNode> neighbors;
        Util::Point result;
        std::vector<std::pair<unsigned int, unsigned int>> neiCells(
                std::pair<x-1, z-1>, std::pair<x-1, z>, std::pair<x-1, z+1>,
                std::pair<x+1, z-1>, std::pair<x+1, z>, std::pair<x+1, z+1>,
                std::pair<x,z-1>, std::pair<x,z+1>);
        //very stupid implementation here...........
        
        for(std::vector<std::pair<unsigned int, unsigned int>>::iterator it = neiCells.begin(); it != neiCells.end(); ++it){

            gSpatialDatabase->getLocationFromIndex(gSpatialDatabase->getCellIndexFromGridCoords(it->first, it->second), result);
            neighbors.push_back(AStarPlannerNode(result, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), null));

        }
        
        return neighbors;
    }

    bool constructPath(std::vector<Util::Point>& agent_path, AStarPlannerNode&* endNode)
    {
        if(!endNode){
            return false;
        }
        while(endNode->parent){
            std::vector<Utill::Point>::iterator it = agent_path.begin();
            agent_path.insert(it, endNode->point);
            endNode = endNode->parent;
        }    
        return true;
    }

    int heuristicManhattan(int start_id, int goal_id){
        unsigned int start_x, start_z, goal_x, goal_z;
        gSpatialDatabase->getGridCoordinatesFromIndex(start_id, start_x, start_z);
        gSpatialDatabase->getGridCoordinatesFromIndex(goal_id, goal_x, goal_z);

        return double(abs(goal_x - start_x) + abs(goal_z - start_z));
    }

    int heuristicEuclidean(int start_id, int goal_id){
        unsigned int start_x, start_z, goal_x, goal_z;
        gSpatialDatabase->getGridCoordinatesFromIndex(start_id, start_x, start_z);
        gSpatialDatabase->getGridCoordinatesFromIndex(goal_id, goal_x, goal_z);

        return sqrt(pow(double((goal_x - start_x)), 2.0) + pow(double(goal_z - start_z), 2.0));   
    }

}

