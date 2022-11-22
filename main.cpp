/*
@Author Xiao Luo
@Date Nov. 11th 2022
*/

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
using namespace std;

const int NODE_NUMBER = 7;
const int INF = static_cast<int>(2e9+7);

class NodeInfo;

using GraphType = array<array<int, NODE_NUMBER + 1>, NODE_NUMBER + 1>;
using ShortestPathType = array<int, NODE_NUMBER + 1>;
using NodeInfoListType = array<NodeInfo, NODE_NUMBER + 1>;

// mainly use in DV
class NodeInfo
{
	// node i send a Vector to node j when shortest path changed
	struct DVPacket
	{
		int sourceNode;
		int destinationNode;
		ShortestPathType shortestPath;
	};

	// the received packets of this node
	vector<DVPacket> receivedPackets;
	// the packets need to be sent
	vector<DVPacket> sendingPackets;
	// forwardingTable[i] = to which neighbor if a packet want to goto destination i
	array<int, NODE_NUMBER + 1> forwardingTable;

	// if shortest path to all other nodes not change, this node converge
	bool convergeFlag = true;

	// id of this node
	int node;

	// when link cost change or receive a distance vector, update shortest path and propogate 
	// new shortest path to its neighbor
	void propogateChange();

public:

	NodeInfo(int node): node(node){ }
	NodeInfo() = default;

	struct 
	{
		unordered_map<int, ShortestPathType> neighborShortestPaths;
		ShortestPathType shortestPath;
	}DVAttribute;

	void receiveNewVector(const DVPacket& newPacket);

	void processReceivedDVPackets();

	void costToNeighborChange(const int neighborNode);

	void deliverPackets();

	bool isConverge()const;

	void outputForwardingTable();
};

// graph[i][j] = cost from i to j if they are neighbors. or INF
GraphType graph;
NodeInfoListType nodeInfos;

GraphType initGraph();
NodeInfoListType initNodeInfoList();

bool isNeighbor(const int x, const int y);
ShortestPathType dijsktra(const int sourceNode);
void initDV();

int doDVUntilConverge();
int linkCostChange(const int x, const int y, const int newCost);


int main()
{

	graph = initGraph();
	nodeInfos = initNodeInfoList();
	initDV();
	doDVUntilConverge();

	auto gap = []() {printf("-------------------------------------------------------\n"); };


	{// Q1
		printf("For question1 \n");
		printf("Shortest Path from v1 to v7 by Dijkstra is %d\n", dijsktra(1).at(7));
		printf("Shortest Path from v1 to v7 by DV is %d\n", nodeInfos[1].DVAttribute.shortestPath[7]);
	}
	gap();
	{// Q2
		printf("For question2 \n");
		for (int i = 0; i <= NODE_NUMBER; ++i)
		{
			if (i == NODE_NUMBER / 2)puts("To");
			printf("\t", i);
		}
		putchar('\n');
		for (int i = 0; i <= NODE_NUMBER; ++i)
		{
			printf("%d\t", i);
		}
		putchar('\n');
		for (int i = 1; i <= NODE_NUMBER; ++i)
		{
			printf("%d\t", i);
			nodeInfos[i].outputForwardingTable();
			if (i == NODE_NUMBER / 2)printf("From\t");
			putchar('\n');
		}
	}
	gap();
	{// Q3 不会中毒。因为v3-v4-v6 cost也是3，和v3-v6变化前一样。不像课件例子从5迭代到50
		printf("For Question3, iteration time is %d\n", linkCostChange(3, 6, 50));
		linkCostChange(3, 6, 3);//change back
	}
	gap();
	{// Q4
		printf("For Question4, iteration time is %d\n", linkCostChange(3, 6, 1));
		linkCostChange(3, 6, 3);//change back
	}
	gap();
	{// Q5 v3-v5=-10, v3-v5-v3=-20, v3-v5-v3-v5=-30. 越来越小
		printf("For Question5, iteration time is %d\n", linkCostChange(3, 5, -10));
	}

	getchar();
	return 0;
}



GraphType initGraph()
{
	GraphType graph;
	for (auto&subArray : graph)
	{
		for (int&ele : subArray)
		{
			ele = INF;
		}
	}

	auto setEdge = [&graph](const int x, const int y, const int cost)
	{
		graph[x][y] = graph[y][x] = cost;
	};

	setEdge(1, 4, 4);
	setEdge(1, 3, 2);
	setEdge(1, 2, 5);
	setEdge(2, 3, 8);
	setEdge(2, 5, 1);
	setEdge(3, 4, 1);
	setEdge(3, 5, 4);
	setEdge(3, 6, 3);
	setEdge(3, 7, 2);
	setEdge(4, 6, 2);
	setEdge(5, 7, 6);
	setEdge(6, 7, 10);

	for (int node = 1; node <= NODE_NUMBER; ++node)
	{
		setEdge(node, node, 0);
	}

	return graph;
}

NodeInfoListType initNodeInfoList()
{
	NodeInfoListType nodeInfoList;
	for (int node = 1; node <= NODE_NUMBER; ++node)
	{
		nodeInfoList[node] = NodeInfo(node);
	}

	return nodeInfoList;
}

ShortestPathType dijsktra(const int sourceNode)
{
	// N~
	ShortestPathType shortestPath;
	ShortestPathType isInN_;
	fill(isInN_.begin(), isInN_.end(), false);
	
	int N_Size = 1;
	isInN_[sourceNode] = true;
	for (int node = 1; node <= NODE_NUMBER; ++node)
	{
		shortestPath[node] = graph[node][sourceNode];
	}

	do
	{
		// find w not in N~ 
		int w = 1;

		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			if (isInN_[node] == false)
			{
				w = node;
				break;
			}
		}

		// such that D(w) is a minimum
		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			if (isInN_[node] == false && shortestPath[node] <= shortestPath[w])
			{
				w = node;
			}
		}

		// add w to N`
		isInN_[w] = true;
		N_Size++;

		//update
		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			if (isNeighbor(node, w))
			{
				shortestPath[node] = min(shortestPath[node], shortestPath[w] + graph[w][node]);
			}
		}
	} while (N_Size < NODE_NUMBER);

	return shortestPath;
}

bool isNeighbor(const int x, const int y)
{
	return graph[x][y] != INF && x != y;
}

void initDV()
{
	for (int sourceNode = 1; sourceNode <= NODE_NUMBER; ++sourceNode)
	{
		auto& nodeAttribute = nodeInfos[sourceNode].DVAttribute;

		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			nodeAttribute.shortestPath[node] = graph[node][sourceNode];
		}

		// for each neighbor w, set to inf initially
		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			if (isNeighbor(node, sourceNode))
			{
				ShortestPathType neighborShortestPath;
				fill(neighborShortestPath.begin(), neighborShortestPath.end(), INF);
				nodeAttribute.neighborShortestPaths[node] = neighborShortestPath;
			}
		}

		// for each neighbor w, send distance vector D_x
		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			if (isNeighbor(node, sourceNode))
			{
				nodeInfos[node].receiveNewVector({ sourceNode, node, nodeAttribute.shortestPath });
			}
		}
	}

}

int doDVUntilConverge()
{
	int iteration = 0;
	while (true)
	{
		++iteration;
		//for all nodes, proess receiving packets
		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			nodeInfos[node].processReceivedDVPackets();
		}

		// send their new shortest path to their neighbors
		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			nodeInfos[node].deliverPackets();
		}

		// if all nodes converge at the same time, break
		int convergeFlag = true;
		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			convergeFlag &= nodeInfos[node].isConverge();
		}
		if (convergeFlag)
		{
			break;
		}
		if (iteration % 100 == 0)
		{
			printf("iteration %d now! Maybe deadloop!\n", iteration);
		}
		if (iteration >= 500)
		{
			printf("Too much iteration! Abort!\n");
			exit(0);
		}
	}
	return iteration;
}

int linkCostChange(const int x, const int y, const int newCost)
{
	graph[x][y] = graph[y][x] = newCost;
	nodeInfos[x].costToNeighborChange(y);
	nodeInfos[y].costToNeighborChange(x);

	if (nodeInfos[x].isConverge() && nodeInfos[y].isConverge())
	{
		return 1;
	}

	return doDVUntilConverge() + 1;
}



inline void NodeInfo::propogateChange()
{
	// x = this->node
	// for each y, D_x(y) = min_v{c_(x,v)+D_v(y)}
	convergeFlag = true;
	
	for (int y = 1; y <= NODE_NUMBER; ++y)
	{
		if (this->node == y)continue;
		const int historyShortestCost = DVAttribute.shortestPath[y];
		DVAttribute.shortestPath[y] = INF;
		for (int v = 1; v <= NODE_NUMBER; ++v)
		{
			if (isNeighbor(this->node, v))
			{
				if (DVAttribute.shortestPath[y] >= graph[this->node][v] + DVAttribute.neighborShortestPaths[v][y])
				{
					DVAttribute.shortestPath[y] = graph[this->node][v] + DVAttribute.neighborShortestPaths[v][y];
					forwardingTable[y] = v;
				}
			}
		}

		// if D_x(y) changed for any desination y, send D_x to all its neighbor
		if (historyShortestCost != DVAttribute.shortestPath[y])
		{
			convergeFlag = false;
		}
	}

	if (convergeFlag == false)
	{
		for (int node = 1; node <= NODE_NUMBER; ++node)
		{
			if (isNeighbor(node, this->node))
			{
				sendingPackets.push_back({ this->node, node, DVAttribute.shortestPath });
			}
		}
	}
}

inline void NodeInfo::receiveNewVector(const DVPacket & newPacket)
{
	receivedPackets.push_back(newPacket);
}

inline void NodeInfo::processReceivedDVPackets()
{
	for (const DVPacket&packet : receivedPackets)
	{
		// update your knowledge of neighbor
		DVAttribute.neighborShortestPaths[packet.sourceNode] = packet.shortestPath;
	}
	propogateChange();
	receivedPackets.clear();
}

inline void NodeInfo::costToNeighborChange(const int neighborNode)
{
	//this->DVAttribute.shortestPath[neighborNode] = graph[this->node][neighborNode];
	this->propogateChange();
	this->deliverPackets();
}

inline void NodeInfo::deliverPackets()
{
	for (const DVPacket&packet : sendingPackets)
	{
		nodeInfos.at(packet.destinationNode).receiveNewVector(packet);
	}
	sendingPackets.clear();
}

inline bool NodeInfo::isConverge() const { return convergeFlag; }

void NodeInfo::outputForwardingTable()
{
	for (int node = 1; node <= NODE_NUMBER; ++node)
	{
		if (node == this->node)
		{
			printf("NA.\t");
		}
		else 
		{
			printf("%d\t", forwardingTable[node]);
		}
	}
}
