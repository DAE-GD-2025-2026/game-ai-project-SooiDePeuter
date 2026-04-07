#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		
		// If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
		{
			return Eulerianity::notEulerian;
		}
		
		// Count nodes with odd degree 
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		int TotalOddNodes{};
		for (int index{}; index < Nodes.size(); ++index)
		{
			std::vector<Connection*> Connections = m_pGraph->FindConnectionsFrom(index);
			if (Connections.size() % 2 == 1)
			{
				TotalOddNodes++;
			}
		}
		// determine euleranity
		if (TotalOddNodes == 1 || TotalOddNodes >= 3)
		{
			return Eulerianity::notEulerian;
		}
		else if (TotalOddNodes == 0)
		{
			return Eulerianity::eulerian;
		}
		else if (TotalOddNodes == 2)
		{
			return Eulerianity::semiEulerian;
		}
		
		return Eulerianity::notEulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		// If this graph is not eulerian, return the empty path
		if (eulerianity == Eulerianity::notEulerian)
		{
			return Path;
		}
		
		if (eulerianity == Eulerianity::eulerian)
		{
			currentNodeId = 0;
		}
		else if (eulerianity == Eulerianity::semiEulerian)
		{
			//find index with odd number of connections
			for (int index{}; index < Nodes.size(); ++index)
			{
				if (graphCopy.FindConnectionsFrom(index).size() % 2 == 1)
				{
					currentNodeId = index;
					break;
				}
			}
		}
		
		// Start algorithm loop
		std::stack<int> nodeStack;
		
		while (graphCopy.FindConnectionsFrom(currentNodeId).size() != 0)
		{
			//add node to the path
			Path.push_back(m_pGraph->GetActiveNodes()[currentNodeId]);
			//set id to next node
			currentNodeId = graphCopy.FindConnectionsFrom(currentNodeId)[0]->GetToId();
			//remove connection between stored and next node
			graphCopy.RemoveConnection(Path[Path.size() - 1]->GetId(), currentNodeId);
		}
		//add the last new node to the path
		Path.push_back(m_pGraph->GetActiveNodes()[currentNodeId]);

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// Mark the visited node
		visited[startIndex] = true;

		// Ask the graph for the connections from that node
		std::vector<Connection*> Connections = m_pGraph->FindConnectionsFrom(startIndex);
		
		// recursively visit any valid connected nodes that were not visited before
		for (Connection* connection : Connections)
		{
			if (visited[connection->GetToId()] == false)
			{
				VisitAllNodesDFS(Nodes, visited, connection->GetToId());
			}
		}
		
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;
		
		std::vector<bool> VisitedNodes;
		VisitedNodes.reserve(Nodes.size());
		for (int index{}; index < Nodes.size(); ++index)
		{
			VisitedNodes.emplace_back(false);
		}
		
		// start a depth-first-search traversal from the node that has at least one connection
		VisitAllNodesDFS(Nodes, VisitedNodes, 0);
		
		// if a node was never visited, this graph is not connected
		for (bool Node : VisitedNodes)
		{
			if (Node == false)
			{
				return false;
			}
		}
		return true;
	}
}