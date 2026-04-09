#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	// Go over all the edges of the navigation mesh and create nodes
	for (int index{}; index <  pNavPoly->GetEdges().size(); index++)
	{
		const TriPolygon::Edge* Line = &pNavPoly->GetEdges()[index];
		
		//count touching triangles
		int triangleCount{};
		for (auto const & triangle : pNavPoly->GetTriangles())
		{
			if (triangle.HasEdge(*Line))
			{
				triangleCount++;
			}
		}
		
		//if in between two triangles
		if (triangleCount >= 2)
		{
			const FVector2D Position{Line->GetP1(*pNavPoly) * 0.5 + Line->GetP2(*pNavPoly) * 0.5};
			this->AddNode(std::make_unique<NavGraphNode>(Position, index));
		}
	}

	// Create connections now that every node is created
	for (const TriPolygon::Triangle& Triangle : pNavPoly->GetTriangles())
	{
		std::array<TriPolygon::Edge, 3> Edges = Triangle.GetEdges();
		
		//store node indexes
		std::vector<int> NodeIndexes{};
		NodeIndexes.reserve(Edges.size());
		for (TriPolygon::Edge Edge : Edges)
		{
			auto EdgeIndex = pNavPoly->FindEdgeIndex(Edge);
			if (!EdgeIndex.has_value())
			{
				continue;
			}
			
			int nodeId = GetNodeIdFromEdgeIndex(EdgeIndex.value());
			if (nodeId != Graphs::InvalidNodeId)
			{
				NodeIndexes.emplace_back(nodeId);
			}
		}
		
		//make connections and set their weight
		if (NodeIndexes.size() == 2)
		{
			this->AddConnection(std::make_unique<Connection>(NodeIndexes[0], NodeIndexes[1]));
		}
		else if (NodeIndexes.size() == 3)
		{
			this->AddConnection(std::make_unique<Connection>(NodeIndexes[0], NodeIndexes[1]));
			this->AddConnection(std::make_unique<Connection>(NodeIndexes[1], NodeIndexes[2]));
			this->AddConnection(std::make_unique<Connection>(NodeIndexes[2], NodeIndexes[0]));
		}
	}
	
	//set all weights
	SetConnectionCostsToDistances();
}
