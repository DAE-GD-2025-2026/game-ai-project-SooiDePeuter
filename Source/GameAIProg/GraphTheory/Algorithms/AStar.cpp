#include "AStar.h"
#include <queue>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	struct WeightedNode
	{
		Node* pNode{};
		int GCost{-1};
		Node* pPreviousNode = nullptr;
	};
	
	std::vector<Node*> path{};
	std::vector<WeightedNode> closedList{};
	WeightedNode pCurrentNode{};
	
	pCurrentNode = {pStartNode, 0};
	while (pCurrentNode.pNode != pGoalNode)
	{
		//get node with lowest f-score
		int LowestFCost{INT_MAX};
		int LowestFcostId{Graphs::InvalidNodeId};
		for (Connection* Connection : pGraph->FindConnectionsFrom(pCurrentNode.pNode->GetId()))
		{
			int GCost = pCurrentNode.GCost + Connection->GetWeight();
			
			//discard if in closed list
			bool BreakForLoop = false;
			for (int index{}; index < closedList.size(); index++)
			{
				if (closedList[index].pNode == pGraph->GetActiveNodes()[Connection->GetToId()])
				{
					if (GCost >= closedList[index].GCost)
					{
						BreakForLoop = true;
						break;
					}
					else
					{
						closedList.erase(closedList.begin() + index);
					}
				}
			}
			if (BreakForLoop)
			{
				continue;
			}
			
			
			int HCost = GetHeuristicCost(pGraph->GetActiveNodes()[Connection->GetToId()], pGoalNode);
			int FCost = HCost + GCost;
			
			if (FCost < LowestFCost)
			{
				LowestFCost = FCost;
				LowestFcostId = Connection->GetToId();
			}
		}
		
		//copy to closedList
		closedList.push_back(pCurrentNode);
		
		//set new node: 
		//new pNode
		//Gcost increases with 1
		//pNode now becomes the previous Node
		pCurrentNode = 
		{
			pGraph->GetActiveNodes()[LowestFcostId],
			pCurrentNode.GCost + 1,
			pCurrentNode.pNode,
		};
	}
	
	//backtrack
	closedList.push_back(pCurrentNode);
	Node* BackTrackNode{pCurrentNode.pNode};
	while (BackTrackNode != pStartNode)
	{
		for (const WeightedNode& ClosedNode : closedList)
		{
			if (ClosedNode.pNode == BackTrackNode)
			{
				path.push_back(BackTrackNode);
				BackTrackNode = ClosedNode.pPreviousNode;
			}
		}
	}
	
	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}