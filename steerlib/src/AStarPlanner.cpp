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

// Set to false for Euclidean distance
#define USE_MANHATTAN_DISTANCE false

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

		std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator> NodeMap;
		agent_path.clear();
		std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator> CameFromNodeMap;

		//TODO
		//std::cout<<"\nIn A*";

		std::vector<Util::Point> ClosedSet;
		std::vector<Util::Point> OpenSet;
		std::vector<Util::Point> CameFrom;
		std::vector<Util::Point> NeighborVector;

		int lowestFScore = 0;
		int lowestFIndex = 0;
		double CostSoFar;

		SteerLib::AStarPlannerNode StartNode(start, 0, Heuristic(start,goal), nullptr);
		NodeMap.emplace(start,StartNode);
		OpenSet.push_back(StartNode.point);

		while(!OpenSet.empty())
		{
			//Get Node with the lowest Fscore

			lowestFScore = NodeMap.at(OpenSet[0]).f;
			lowestFIndex = 0;

			for( int i = 0; i< OpenSet.size(); i++)
			{
				if(NodeMap.at(OpenSet[i]).f < lowestFScore)
				{
					lowestFScore = NodeMap.at(OpenSet[i]).f;
					lowestFIndex = i;
				}
			}

			SteerLib::AStarPlannerNode CurrentNode = NodeMap.at(OpenSet[lowestFIndex]);

			if(CurrentNode.point == goal)
			{
				agent_path.push_back(CurrentNode.point);

			
				while(CurrentNode.point != start)
				{
					CurrentNode = CameFromNodeMap.at(CurrentNode);
					agent_path.push_back(CurrentNode.point);
				
				}

			
				agent_path.push_back(start);

				std::reverse(agent_path.begin(), agent_path.end());
				
				return true;
			}

			ClosedSet.push_back(OpenSet[lowestFIndex]);
			OpenSet.erase(OpenSet.begin() + lowestFIndex);

			NeighborNodes(CurrentNode.point, goal,NodeMap,ClosedSet,OpenSet,CameFromNodeMap);
		}
		return false;
	}


	void AStarPlanner::NeighborNodes(Util::Point OriginPoint, Util::Point goal,	std::map<Util::Point,SteerLib::AStarPlannerNode,epsilonComparator>& NodeMap,std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& OpenSet, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap)
	{
		int x;
		int y;
		int z;
		double NonDiagonalCost;
		double DiagonalCost;
		
		SteerLib::AStarPlannerNode OriginNode = NodeMap.at(OriginPoint);

		DiagonalCost =std::numeric_limits<double>::infinity();
		NonDiagonalCost =std::numeric_limits<double>::infinity();

		x = OriginNode.point.x;
		y = OriginNode.point.y;
		z = OriginNode.point.z;

		Util::Point North = Util::Point(x,y,z+1);
		Util::Point South = Util::Point(x,y,z-1);
		Util::Point East =  Util::Point(x+1,y,z);
		Util::Point West =  Util::Point(x-1,y,z);

		//std::cout << "Start " << OriginNode.point << '\n';


		AddNode(North, NonDiagonalCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet,CameFromNodeMap);
		AddNode(South, NonDiagonalCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet,CameFromNodeMap);
		AddNode(East, NonDiagonalCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, CameFromNodeMap);
		AddNode(West, NonDiagonalCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, CameFromNodeMap);
		
		if(!USE_MANHATTAN_DISTANCE)
		{
			Util::Point NorthEast = Util::Point(x+1,y,z+1);
			Util::Point SouthEast = Util::Point(x+1,y,z-1);
			Util::Point NorthWest = Util::Point(x-1,y,z+1);
			Util::Point SouthWest = Util::Point(x-1,y,z-1);

			AddNode(NorthEast, DiagonalCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, CameFromNodeMap);
			AddNode(SouthEast, DiagonalCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, CameFromNodeMap);
			AddNode(NorthWest, DiagonalCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, CameFromNodeMap);
			AddNode(SouthEast, DiagonalCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, CameFromNodeMap);
		}
	}


	void AStarPlanner::AddNode(Util::Point CurrentPoint, double cost, SteerLib::AStarPlannerNode FromNode, Util::Point goal, std::map<Util::Point,SteerLib::AStarPlannerNode,epsilonComparator>& NodeMap, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& OpenSet, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap )
	{
		int NodeIndex;
		float DistanceInBetween;
		double TentativeScore;
		NodeIndex = gSpatialDatabase->getCellIndexFromLocation(CurrentPoint);
		std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>::iterator CameFromMapIt; 

		if(!canBeTraversed (NodeIndex))
		{
			return;
		}

		if(NodeMap.count(CurrentPoint) == 0)
		{
			SteerLib::AStarPlannerNode InsertNode(CurrentPoint, cost, cost, &FromNode);
			NodeMap.emplace(CurrentPoint,InsertNode);
		
		}

		if(std::find(ClosedSet.begin(), ClosedSet.end(), CurrentPoint) != ClosedSet.end())
		{
			return;
		}
		
		TentativeScore = FromNode.g + distanceBetween(FromNode.point,CurrentPoint);

		if(std::find(OpenSet.begin(), OpenSet.end(), CurrentPoint) == OpenSet.end())
		{
			OpenSet.push_back(CurrentPoint);
			
		}
		else if(TentativeScore >=NodeMap.at(CurrentPoint).g)
		{
			return;
		}

		SteerLib::AStarPlannerNode InsertNode(CurrentPoint, TentativeScore, TentativeScore + Heuristic(CurrentPoint,goal), &FromNode);
		NodeMap.erase(CurrentPoint);
		NodeMap.emplace(CurrentPoint,InsertNode);
	
		if(CameFromNodeMap.count(InsertNode) != 0)
		{
					CameFromNodeMap.erase(InsertNode);
		
		}

		CameFromNodeMap.emplace(InsertNode,FromNode);
		//std::cout << InsertNode.point <<  "From" << FromNode.point <<'\n';
		//for(auto elem : CameFromNodeMap)
		//		{
   					
		//			std::cout << elem.first.point << " " << elem.second.point << "\n";
		//		}
		//while (std::cin.get() != '\n')
		//	    {
		//			 std::cout << '\n' <<'Press the Enter key to continue.';
	//			} 


		
	}

	double AStarPlanner::Manhattan(Util::Point FirstPoint, Util::Point SecondPoint)
	{
		Util::Vector diff = FirstPoint - SecondPoint;
		return abs(diff.x) + abs(diff.y) + abs(diff.z);
	}

	double AStarPlanner::Heuristic(Util::Point a, Util::Point b)
	{
		if(USE_MANHATTAN_DISTANCE) {
			return Manhattan(a,b);
		} else {
			return (double)distanceBetween(a,b);
		}
	}
}
