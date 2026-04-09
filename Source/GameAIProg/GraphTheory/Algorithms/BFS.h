#pragma once
#include <vector>

namespace GameAI
{
	class Graph;
	class Node;

	class BFS
	{
	public:
		BFS(Graph* const pGraph);

		std::vector<Node*> FindPath(Node* const pStartNode, Node* const pDestinationNode) const;
		std::vector<Node*> FindPathSooi(Node* const pStartNode, Node* const pDestinationNode) const;
		std::vector<Node*> FindPathDamian(Node* const pStartNode, Node* const pDestinationNode) const;

	private:
		Graph* pGraph;
	};
}
