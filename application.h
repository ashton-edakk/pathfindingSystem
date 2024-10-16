#pragma once

#include <map>
#include <vector>

#include "graph.h"
#include "osm.h"

graph<long long, double> buildGraph(
    const map<long long, Coordinates>& Nodes,
    const vector<FootwayInfo>& Footways,
    const vector<BuildingInfo>& Buildings);

// Run Dijkstra's algorithm to find the path from `start` to `target`.
// Path may not go through any node in `skipNodes`, but the path may
// start or end in such a node.
vector<long long> dijkstra(
    const graph<long long, double>& G,
    long long start,
    long long target,
    const set<long long>& ignoreNodes);

void application(
    const vector<BuildingInfo>& Buildings,
    const graph<long long, double>& G);
