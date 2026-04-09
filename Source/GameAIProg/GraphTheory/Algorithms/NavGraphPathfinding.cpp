#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> FinalPath{};

	//Get the start and endTriangle
	const TriPolygon::Triangle* StartTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(startPos, true);
	const TriPolygon::Triangle* EndTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(endPos, true);
	
	if (StartTriangle == nullptr || EndTriangle == nullptr)
	{
		return FinalPath;
	}
	if (StartTriangle == EndTriangle)
	{
		FinalPath.push_back(startPos);
		FinalPath.push_back(endPos);
		return FinalPath;
	}
	
	//=>Start looking for a path
	//Copy the graph
	NavGraph GraphCopy = *pNavGraph;
	
	//Create Extra node for the Start Node
	const int StartNodeId{GraphCopy.AddNode(std::make_unique<NavGraphNode>(startPos, -1))};
	for (TriPolygon::Edge Edge: StartTriangle->GetEdges())
	{
		auto edgeIndex = pNavGraph->GetNavPolygon()->FindEdgeIndex(Edge);
		if (!edgeIndex.has_value())
		{
			continue;
		}
		
		int NodeId = GraphCopy.GetNodeIdFromEdgeIndex(edgeIndex.value());
		
		if (NodeId != Graphs::InvalidNodeId)
		{
			FVector2D nodePos = GraphCopy.GetNode(NodeId)->GetPosition();

			GraphCopy.AddConnection(StartNodeId, NodeId);
		}
	}
	
	//Create extra node for the endNode
	const int EndNodeId{GraphCopy.AddNode(std::make_unique<NavGraphNode>(endPos, -1))};
	for (TriPolygon::Edge Edge: EndTriangle->GetEdges())
	{
		auto edgeIndex = pNavGraph->GetNavPolygon()->FindEdgeIndex(Edge);
		if (!edgeIndex.has_value())
		{
			continue;
		}
		
		int NodeId = GraphCopy.GetNodeIdFromEdgeIndex(edgeIndex.value());
		
		if (NodeId != Graphs::InvalidNodeId)
		{
			FVector2D nodePos = GraphCopy.GetNode(NodeId)->GetPosition();

			GraphCopy.AddConnection(EndNodeId, NodeId);
		}
	}
	
	//set weights
	GraphCopy.SetConnectionCostsToDistances();
	
	//Run A star on new graph
	AStar pathfinder(&GraphCopy, HeuristicFunctions::Euclidean);
	std::vector<Node*> NodePath = pathfinder.FindPath(
		GraphCopy.GetNode(StartNodeId).get(),
		GraphCopy.GetNode(EndNodeId).get() );
	
	//convert to positions
	for (Node* Node : NodePath)
	{
		FinalPath.push_back(Node->GetPosition());
	}
	
	//Debug Visualisation

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	// debugPortals = SSFA::FindPortals(nodes, *pNavGraph->GetNavPolygon());
	// finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return FinalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}