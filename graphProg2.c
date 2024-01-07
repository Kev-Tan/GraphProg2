#include "basicDS.h"
//Check if you can include #include <algorithm>
#include <algorithm>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

/* You can add more functions or variables in each class.
   But you "Shall Not" delete any functions variables that TAs defined. */

std::vector<std::pair<Tree, int>> mstContainer;

class Problem1 {
public:

	Problem1(Graph G);  //constructor
	~Problem1();        //destructor
	void insert(int id, int s, Set D, int t, Graph &G, Tree &MTid);
	void stop(int id, Graph &G, Forest &MTidForest);
	void rearrange(Graph &G, Forest &MTidForest);
};

Problem1::Problem1(Graph G) {
	/* Write your code here. */
		cout << "P1 constructor Triggered" << endl;
}

Problem1::~Problem1() {
	/* Write your code here. */

}

//Priority queue comparator
struct CompareCost {
    bool operator()(const graphEdge& e1, const graphEdge& e2) {
        return e1.ce > e2.ce; // Min-heap based on bandwidth cost
    }
};

//Create a temporary graph
Graph tempGraphCreator(Graph G, int t){
    Graph copyGraph = G;
    for (auto it = copyGraph.E.begin(); it != copyGraph.E.end();) {
        if (t > it->b) {
            it = copyGraph.E.erase(it); // Remove the edge that meets the condition
        } else {
            ++it; // Move to the next edge
        }
    }
    return copyGraph;
}

//Find suitable graphEdge source for prim
vector<graphEdge> primEdgeFinder(const Graph& g, int startVertex){
    vector<graphEdge> pathsConnected;
    for (const auto& edge : g.E) {
        //cout << "Edge: " << edge.vertex[0] << " - " << edge.vertex[1] << ", Weight: " << edge.ce << endl;
        if(edge.vertex[0]==startVertex || edge.vertex[1]==startVertex) pathsConnected.push_back(edge);
    }
    return pathsConnected;
}

void prim(const Graph& G, int startVertex, Tree &MTid) {

    //Set total for path to be added
    int total = 0;

    //Heap based Prim algorithm
    priority_queue<graphEdge, vector<graphEdge>, CompareCost> minHeap;
    vector<graphEdge> pathsConnected = primEdgeFinder(G, startVertex);

    //Declaration
    int V = G.V.size();
    vector<int> key(V, INT_MAX);
    vector<bool> inMST(V, false);

    // Mark the starting vertex as part of the MST
    inMST[startVertex] = true;
    MTid.V.push_back(startVertex);
    key[startVertex] = 0;

    // Add edges connected to the starting vertex to the minHeap
    for (const auto& edge : pathsConnected) {
        minHeap.push(edge);
    }

    while (!minHeap.empty()) {
        //Start from top heap
        graphEdge currentEdge = minHeap.top();
        minHeap.pop();

        //Find out which one is the node that hasn't been visited
        int u = currentEdge.vertex[0];
        int v = currentEdge.vertex[1];

        if (inMST[u] && inMST[v]) continue; // Both vertices are in MST, skip

        //Push directly to MTid
        treeEdge edgeToBePushed;
        edgeToBePushed.vertex[0] = currentEdge.vertex[0];
        edgeToBePushed.vertex[1] = currentEdge.vertex[1];
        MTid.E.push_back(edgeToBePushed);
        total += currentEdge.ce;

        int newVertex = inMST[u] ? v : u;
        inMST[newVertex] = true;
        MTid.V.push_back(newVertex);

        // Find new connections for the newVertex
        vector<graphEdge> newPathsConnected = primEdgeFinder(G, newVertex);
        for (const auto& edge : newPathsConnected) {
            int nextVertex = (edge.vertex[0] == newVertex) ? edge.vertex[1] : edge.vertex[0];
            if (!inMST[nextVertex]) {
                minHeap.push(edge);
            }
        }
    }
        MTid.ct = total;
}

void printGraphEdges(Graph G){
    for (auto& graphEdge : G.E) {
        cout << graphEdge.vertex[0] << "->" << graphEdge.vertex[1] << " B: "<< graphEdge.b << " Be " << graphEdge.be << " Ce: " << graphEdge.ce <<endl;
    }
}

void printVertices(Tree MTid){
    for (auto& vertice : MTid.V) {
        cout << vertice << " ";
    }
    cout << endl;
}

void printStoredMST(vector<std::pair<Tree, int>> mstContainer){

    for(auto& pair: mstContainer){
        const Tree& treeChosen = pair.first;

        cout << "ID of " << treeChosen.id << endl;
        for (auto& edge: treeChosen.E){
            cout << edge.vertex[0] << " -> " << edge.vertex[1] << endl;
        }
        cout << "Cost is " << treeChosen.ct << endl;
    }

}


void Problem1::insert(int id, int s, Set D, int t, Graph &G, Tree &MTid) {
	/* Store your output graph and multicast tree into G and MTid */
	//Clear the MTid's contents
	MTid.E.clear();
	MTid.V.clear();
	Graph tempGraph = tempGraphCreator(G, t);
	prim(tempGraph,s, MTid);
	MTid.ct *= t;
	MTid.id = id;
	MTid.s = s;

	//Push MSTs into a container
	mstContainer.push_back(make_pair(MTid, t));

	cout << MTid.ct << endl;
    for (const auto& mtEdge  : MTid.E) {
            cout << "Transmitting through " << mtEdge.vertex[0] << " -> " << mtEdge.vertex[1] << endl;
            for (auto& graphEdge : G.E) {
                    if((mtEdge.vertex[0]==graphEdge.vertex[0] || mtEdge.vertex[0]==graphEdge.vertex[1]) && (mtEdge.vertex[1]==graphEdge.vertex[0] || mtEdge.vertex[1]==graphEdge.vertex[1])){
                        graphEdge.b -= t;
                        break;
                    }
            }
    }

    //printGraphEdges(G);
    //printVertices(MTid);
    //printStoredMST(mstContainer);


    printGraphEdges(G);
	//cout << "Finish" <<endl;
}

bool compareByID(const std::pair<Tree, int>& a, const std::pair<Tree, int>& b) {
    return a.first.id < b.first.id;
}

void Problem1::stop(int id, Graph &G, Forest &MTidForest) {
    /* Store your output graph and multicast tree forest into G and MTidForest
       Note: Please "only" include mutlicast trees that you added nodes in MTidForest. */

    cout << "GRAPH EDGES BEFORE STOP" << endl;
    printGraphEdges(G);

    // Sort MST in order by ID
    std::sort(mstContainer.begin(), mstContainer.end(), compareByID);

    // Remove appropriate MST
    for (auto it = mstContainer.begin(); it != mstContainer.end(); /* no increment here */) {
        const Tree& treeChosen = it->first;
        if (treeChosen.id == id) {
            cout << "Stopping tree of id " << id << endl;
            cout << endl;

            // Revise edge values
            for (auto& originalEdge : treeChosen.E) {
                for (auto& adjustedEdge : G.E) {
                    if ((originalEdge.vertex[0] == adjustedEdge.vertex[0] || originalEdge.vertex[0] == adjustedEdge.vertex[1]) &&
                        (originalEdge.vertex[1] == adjustedEdge.vertex[0] || originalEdge.vertex[1] == adjustedEdge.vertex[1])) {
                        adjustedEdge.b += it->second;
                    }
                }
            }
            it = mstContainer.erase(it); // Remove the element and update iterator
        } else {
            ++it; // Move to the next element
        }
    }


    int counter = 0;
    for (auto& pair : mstContainer) {
        counter += 1;
        cout << "counter val is " << counter << endl;
        Tree& tree = pair.first;
        int traffic = pair.second;
        int originalVertexNumber;


        //Make priority heap
        priority_queue<graphEdge, vector<graphEdge>, CompareCost> minHeap;
        //Find paths connected to source
        vector<graphEdge> pathsConnected = primEdgeFinder(G, tree.s);

    for (auto& verticeSource : tree.V) {
        int V = G.V.size();
        vector<int> key(V, INT_MAX);
        vector<bool> inMST(V, false);
        inMST[verticeSource] = true;

        for(auto& vertice: tree.V){
            inMST[vertice] = true;
        }


        for (auto& edge : pathsConnected) {
            minHeap.push(edge);
        }

        while (!minHeap.empty()) {
            graphEdge currentEdge = minHeap.top();
            minHeap.pop();

            //Find out which one is the node that hasn't been visited
            int u = currentEdge.vertex[0];
            int v = currentEdge.vertex[1];

            if (inMST[u] && inMST[v]) continue; // Both vertices are in MST, skip
            treeEdge edgeToBePushed;
            edgeToBePushed.vertex[0] = currentEdge.vertex[0];
            edgeToBePushed.vertex[1] = currentEdge.vertex[1];

            //Push the edge
            tree.E.push_back(edgeToBePushed);
            //Adjust tree cost edge
            tree.ct += currentEdge.ce*traffic;
            //Make new node visited to be true
            int newVertex = inMST[u] ? v : u;
            inMST[newVertex] = true;
            tree.V.push_back(newVertex);
            //Adjust edge value
            currentEdge.b -= traffic;

            vector<graphEdge> newPathsConnected = primEdgeFinder(G, newVertex);
            for (const auto& edge : newPathsConnected) {
                int nextVertex = (edge.vertex[0] == newVertex) ? edge.vertex[1] : edge.vertex[0];
                if (!inMST[nextVertex]) {
                    minHeap.push(edge);
                }
            }

        }
    }



    }

    printStoredMST(mstContainer);

    printStoredMST(mstContainer);
    cout << "GRAPH EDGES AFTER RECONNECTION" << endl;
    printGraphEdges(G);

    for (const auto& pair : mstContainer) {
        const Tree& tree = pair.first;
        int value = pair.second;

        MTidForest.trees.push_back(tree);
    }

    cout << endl;
    for (const auto& tree : MTidForest.trees) {
        for (const auto& edge : tree.E) {
            cout << edge.vertex[0] << " -> " << edge.vertex[1] << endl;
        }
        cout << "Final Tree cost is: " << tree.ct << endl;
    }

    return;
}


void Problem1::rearrange(Graph &G, Forest &MTidForest) {
	/* Store your output graph and multicast tree forest into G and MTidForest
	   Note: Please include "all" active mutlicast trees in MTidForest. */

        MTidForest.trees.clear();

        //1. Reset all the bandwidth on the graph
        for (auto& edge : G.E) {
            edge.b = edge.be;
        }

        //2. Sort the available MST based on the ID
        std::sort(mstContainer.begin(), mstContainer.end(), compareByID);

        //3. Reset all graph and vertices on the MST container
         for (auto& pair : mstContainer) {
            pair.first.V.clear();  // Clear the vector inside Tree
            pair.first.E.clear();  // Clear another vector inside Tree
            pair.first.ct = 0;
        }

        printStoredMST(mstContainer);

        //4. Insert Again

         for (auto& pair : mstContainer) {
            Tree newTree;
            newTree.V.clear();
            newTree.E.clear();
            auto tree = pair.first;
            newTree.id = tree.id;
            int traffic = pair.second;
            Graph tempGraph = tempGraphCreator(G, traffic);
            prim(tempGraph,tree.s, newTree);
            newTree.ct *= traffic;

            MTidForest.trees.push_back(newTree);
            pair.first = newTree;



            for (const auto& mtEdge  : newTree.E) {
                    for (auto& graphEdge : G.E) {
                            if((mtEdge.vertex[0]==graphEdge.vertex[0] || mtEdge.vertex[0]==graphEdge.vertex[1]) && (mtEdge.vertex[1]==graphEdge.vertex[0] || mtEdge.vertex[1]==graphEdge.vertex[1])){
                                graphEdge.b -= traffic;
                                break;
                            }
                    }
            }


        }


        for (const auto& tree : MTidForest.trees) {
        for (const auto& edge : tree.E) {
            cout << edge.vertex[0] << " -> " << edge.vertex[1] << endl;
        }
        cout << "Final Tree cost is: " << tree.ct << endl;
    }

//        printStoredMST(mstContainer);

    printGraphEdges(G);
	/* Write your code here. */

	return;
}
