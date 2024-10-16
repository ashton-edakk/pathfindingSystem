//NAME:Ashton Edakkunnathu
//CLASS: CS251 - Data Structures
//PROJECT: 5 - Open Street Map
//IDE: Visual Studio Code
//DATE: 4/25/24
//
//SUMMARY:  This project is is a path finder on Open Street Map. Rather than shortest paths from A to B,
//          our application will take two starting points, and have them “meet in the middle”. Assume one
//          person starts at building A, and another person starts at building B. We use a slightly altered
//          version of Dijkstra’s Alogrithm: the paths do not include buildings, except for the buildings
//          at the start and end of the path.More generally, we want to be able to specify a set of nodes
//          that the shortest paths should not go through.

#include "application.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

double INF = numeric_limits<double>::max();

class prioritize {
public:
    bool operator()(const pair<long long, double>& p1,
        const pair<long long, double>& p2) const {
        return p1.second > p2.second;
    }
};


graph<long long, double> buildGraph(
    const map<long long, Coordinates>& Nodes,
    const vector<FootwayInfo>& Footways,
    const vector<BuildingInfo>& Buildings) {
    graph<long long, double> G;

    for (auto& node : Nodes) { //add the nodes
        G.addVertex(node.first);
    }

    for (auto& ftway : Footways) { //adds edegs
        for (size_t i = 0; i + 1 < ftway.Nodes.size(); i++) {
            double wgt = distBetween2Points(Nodes.at(ftway.Nodes[i]).Lat,
                Nodes.at(ftway.Nodes[i]).Lon,
                Nodes.at(ftway.Nodes[i + 1]).Lat,
                Nodes.at(ftway.Nodes[i + 1]).Lon);
            G.addEdge(ftway.Nodes[i], ftway.Nodes[i + 1], wgt);
            G.addEdge(ftway.Nodes[i + 1], ftway.Nodes[i], wgt);
        }
    }

    for (const auto& b : Buildings) { //loops will connect buildings to all node nearby
        if (!G.addVertex(b.Coords.ID)) {
            cerr << "Failed to add building vertex: " << b.Coords.ID << endl;
        }
        for (const auto& p : Nodes) {
            if (p.second.OnFootway) {
                double wgt = distBetween2Points(b.Coords.Lat, b.Coords.Lon, p.second.Lat, p.second.Lon);
                
                if (wgt <= 0.041) {
                    G.addEdge(b.Coords.ID, p.first, wgt);
                    G.addEdge(p.first, b.Coords.ID, wgt);
                }
            }
        }
    }
    return G;
}

vector<long long> dijkstra(
    const graph<long long, double>& G,
    long long start,
    long long target,
    const set<long long>& ignoreNodes) {
    vector<long long> path;

    if (start == target) {
        path.push_back(start);
        return path;
    }

    unordered_map<long long, double> distances; 
    unordered_map<long long, long long> previous;

    //pq
    priority_queue<pair<long long, double>,vector<pair<long long, double>>,prioritize> worklist;

    for (const auto& n : G.getVertices()) {//sets all distances to infinty and previsou to -1, adding them to queue
        previous.emplace(n, -1);
        distances.emplace(n,INF);

        worklist.push({ n, distances.at(n) });
    }

    //this creates the starting point for the alogrutim
    distances.at(start) = 0.0;
    worklist.push({ start, distances.at(start) });
    

    while (!worklist.empty()) {
        long long currVal = worklist.top().first;
        double currDist = worklist.top().second;
        worklist.pop();
        
        //checks value is in the ignoredNodes, and that it is not the start or target
        if (ignoreNodes.find(currVal) != ignoreNodes.end() && currVal != start && currVal != target) {
            continue;
        }
        
        for (const auto& neighbor : G.neighbors(currVal)) {//checks through the neighbors

            //does the same thing as the last if statment but for the neighbors of the value
            if (ignoreNodes.find(neighbor) != ignoreNodes.end() && neighbor != start && neighbor != target) {
                continue;
            }

            //gets the distance
            double edgeWgt = 0.0;
            G.getWeight(currVal, neighbor, edgeWgt);
            double dist = currDist + edgeWgt;

            //changes the maps if the distance calculated is shorter/more effecient
            if (dist < distances.at(neighbor)) {
                previous.at(neighbor) = currVal;
                distances.at(neighbor) = dist;
                worklist.push({ neighbor, dist });
            }
        }
    }

    if (distances.at(target) == INF) {
        return path;
    }
    

    //adds values to the path vector, in reverse.
    long long temp = target;
    while (temp != -1) {
        path.push_back(temp);
        temp = previous.at(temp);
    }

    // Reverse the path
    reverse(path.begin(), path.end());

    return path;

}

//gets the length of the path with weight
double pathLength(const graph<long long, double>& G, const vector<long long>& path) {
    double length = 0.0;
    double weight;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
        assert(res);
        length += weight;
    }
    return length;
}

//creates a visual repersentation of the path via '->' as the traversal from one node to the other
void outputPath(const vector<long long>& path) {
    for (size_t i = 0; i < path.size(); i++) {
        cout << path.at(i);
        if (i != path.size() - 1) {
            cout << "->";
        }
    }
    cout << endl;
}

void application(
    const vector<BuildingInfo>& Buildings,
    const graph<long long, double>& G) {
    string person1Building, person2Building;

    set<long long> buildingNodes;
    for (const auto& building : Buildings) {
        buildingNodes.insert(building.Coords.ID);
    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);

        //
        // find the building coordinates
        //
        bool foundP1 = false;
        bool foundP2 = false;
        Coordinates P1Coords, P2Coords;
        string P1Name, P2Name;

        for (const BuildingInfo& building : Buildings) {
            if (building.Abbrev == person1Building) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (building.Abbrev == person2Building) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        for (const BuildingInfo& building : Buildings) {
            if (!foundP1 &&
                building.Fullname.find(person1Building) != string::npos) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (!foundP2 && building.Fullname.find(person2Building) != string::npos) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        if (!foundP1) {
            cout << "Person 1's building not found" << endl;
        } else if (!foundP2) {
            cout << "Person 2's building not found" << endl;
        } else {
            cout << endl;
            cout << "Person 1's point:" << endl;
            cout << " " << P1Name << endl;
            cout << " (" << P1Coords.Lat << ", " << P1Coords.Lon << ")" << endl;
            cout << "Person 2's point:" << endl;
            cout << " " << P2Name << endl;
            cout << " (" << P2Coords.Lat << ", " << P2Coords.Lon << ")" << endl;

            string destName;
            Coordinates destCoords;

            Coordinates centerCoords = centerBetween2Points(
                P1Coords.Lat, P1Coords.Lon, P2Coords.Lat, P2Coords.Lon);

            double minDestDist = numeric_limits<double>::max();

            for (const BuildingInfo& building : Buildings) {
                double dist = distBetween2Points(
                    centerCoords.Lat, centerCoords.Lon,
                    building.Coords.Lat, building.Coords.Lon);
                if (dist < minDestDist) {
                    minDestDist = dist;
                    destCoords = building.Coords;
                    destName = building.Fullname;
                }
            }

            cout << "Destination Building:" << endl;
            cout << " " << destName << endl;
            cout << " (" << destCoords.Lat << ", " << destCoords.Lon << ")" << endl;

            vector<long long> P1Path = dijkstra(G, P1Coords.ID, destCoords.ID, buildingNodes);
            vector<long long> P2Path = dijkstra(G, P2Coords.ID, destCoords.ID, buildingNodes);

            // This should NEVER happen with how the graph is built
            if (P1Path.empty() || P2Path.empty()) {
                cout << endl;
                cout << "At least one person was unable to reach the destination building. Is an edge missing?" << endl;
                cout << endl;
            } else {
                cout << endl;
                cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
                cout << " miles" << endl;
                cout << "Path: ";
                outputPath(P1Path);
                cout << endl;
                cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
                cout << " miles" << endl;
                cout << "Path: ";
                outputPath(P2Path);
            }
        }

        //
        // another navigation?
        //
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
    }
}
