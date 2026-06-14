#pragma once
#include <iostream>
#include <list>
#include <vector>
#include <unordered_map>

struct Edge {
    int to;
    float length;
    float speed;
    float capacity;
    float vehicles;
    float waiting;
    float time;
    float queue = 0;    // Q_ij(t)
    bool green = false; // g_ij(t)

    Edge(int t, float l, float s, float c, float v, float w, float ti) {
        to = t;
        length = l;
        speed = s;
        capacity = c;
        vehicles = v;
        waiting = w;
        time = ti;
    }
};

struct rnode {
    int vert;
    std::list<Edge> l;

    rnode(int v) {
        vert = v;
    }
};

class Road {

    std::unordered_map<int, rnode*> node;

public:
    bool isEmpty() {
        return node.empty();
    }

    int totalVertices() {
        return (int)node.size();
    }

    int totalEdges() {
        int count = 0;
        for (auto& pair : node)
            count += (int)pair.second->l.size();
        return count;
    }

    bool searchVertex(int v) {
        return node.count(v) > 0; 
    }

    void addVertex(int v) {
        if (searchVertex(v)) return;
        node[v] = new rnode(v); 
    }

   
    rnode* getNode(int v) {
        auto it = node.find(v);
        return (it != node.end()) ? it->second : nullptr;
    }

 
    std::vector<rnode*> getNodes() {
        std::vector<rnode*> result;
        result.reserve(node.size());
        for (auto& pair : node) result.push_back(pair.second);
        return result;
    }

    void addedge(int v1, int v2, float l = 0, float s = 0, float c = 0,
        float v = 0, float w = 0, float t = 0) {

        rnode* src = getNode(v1);
        rnode* dst = getNode(v2);
        if (!src || !dst) return;
        src->l.push_back(Edge(v2, l, s, c, v, w, t));
    }

    ~Road() {
        for (auto& pair : node)
            delete pair.second;
        node.clear();
    }
};