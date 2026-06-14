#include<iostream>
#include<list>
#include<vector>
#include "Vehicle.h"
#include "Road.h"
#include<map>
#include<set>
#include<unordered_map>
#include<cmath>
#include<algorithm>
#include <string>

using namespace std;

class TrafficSimulator {
    Road* road;
    vector<Vehicle> vehicles;
    float totalSystemCost = 0;
    int stepCount = 0;

    float getTrafficMin(float queue, float dischargeLimit, float nextCapacity) {
        float smallest = queue;
        if (dischargeLimit < smallest) smallest = dischargeLimit;
        if (nextCapacity < smallest) smallest = nextCapacity;
        return (smallest < 0) ? 0 : smallest;
    }


public:
    // helper
    float calculatePathIdealTime(const vector<int>& path) {
        float ideal = 0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            rnode* n = road->getNode(path[i]);
            if (!n) continue;
            for (auto& e : n->l) {
                if (e.to == path[i + 1]) {
                    ideal += (e.speed > 0) ? (e.length / e.speed) : 0;
                    break;
                }
            }
        }
        return ideal;
    }
    TrafficSimulator(Road* r) { road = r; }

    void addVehicle(int id, int s, int d, int st) {
        Vehicle v(id, s, d, st);

        v.path = dijkstra(s, d);

        if (v.path.size() < 2) {
            v.status = "arrived";
            vehicles.push_back(v);
            return;
        }

        v.pathIndex = 0;
        v.currentFrom = s;

        v.idealTravelTime = calculatePathIdealTime(v.path);

        v.edgeTotalTime = getTravelTime(s, v.path[1], 0.15f, 4.0f);
        v.remainingTime = v.edgeTotalTime;

        // increment edge load
        rnode* n = road->getNode(s);
        for (auto& e : n->l) {
            if (e.to == v.path[1]) {
                e.vehicles += 1;
                break;
            }
        }

        vehicles.push_back(v);
    }

    float computeCongestion(int from, int to) {
        rnode* n = road->getNode(from);
        if (!n) return -1;
        for (auto& e : n->l) {
            if (e.to == to) {
                if (e.capacity <= 0) return 0;
                return e.vehicles / e.capacity;
            }
        }
        return -1;
    }

    float getTravelTime(int from, int to, float alpha, float beta) {
        rnode* n = road->getNode(from);
        if (!n) return 1e9;
        for (auto& e : n->l) {
            if (e.to == to) {
                if (e.speed <= 0 || e.capacity <= 0) return 1e9;
                float w_free = e.length / e.speed;
                float ratio = e.vehicles / e.capacity;
                return w_free * (1 + alpha * pow(ratio, beta)); // BPR formula
            }
        }
        return 1e9;
    }

    void updateEdgeTimes(float alpha, float beta) {
        for (auto nodePtr : road->getNodes()) {
            for (auto& e : nodePtr->l) {
                e.time = getTravelTime(nodePtr->vert, e.to, alpha, beta);
            }
        }
    }

    vector<int> dijkstra(int src, int dest) {
        unordered_map<int, float> dist;
        unordered_map<int, int> parent;
        set<pair<float, int>> pq;

        for (auto nodePtr : road->getNodes()) {
            dist[nodePtr->vert] = 1e9;
            parent[nodePtr->vert] = -1;
        }

        dist[src] = 0;
        pq.insert({ 0, src });

        while (!pq.empty()) {
            auto top = *pq.begin();
            pq.erase(pq.begin());

            int u = top.second;
            if (u == dest) break;

            rnode* un = road->getNode(u);
            if (!un) continue;
            for (auto& e : un->l) {
                int v = e.to;
                float cost = e.time;
                if (dist[u] + cost < dist[v]) {
                    pq.insert({ dist[v], v });
                    dist[v] = dist[u] + cost;
                    parent[v] = u;
                }
            }
        }

        vector<int> path;
        if (dist[dest] == 1e9) return path;

        for (int cur = dest; cur != -1; cur = parent[cur]) {
            path.push_back(cur);
        }

        reverse(path.begin(), path.end());
        return path;
    }

    void updateSignals() {
        for (auto nodePtr : road->getNodes()) {
            int intersectionId = nodePtr->vert;

            Edge* bestEdge = nullptr;
            float maxQueue = -1;

            for (auto sourceNode : road->getNodes()) {
                for (auto& e : sourceNode->l) {
                    if (e.to == intersectionId) {
                        e.green = false;

                        if (e.queue > maxQueue) {
                            maxQueue = e.queue;
                            bestEdge = &e;
                        }
                    }
                }
            }

            if (bestEdge != nullptr && maxQueue > 0) {
                bestEdge->green = true;
            }
        }
    }

    float calculateCurrentStepCost(float alpha, float beta) {
        float stepCost = 0;

        for (auto nodePtr : road->getNodes()) {
            for (auto& e : nodePtr->l) {
                float queueTerm = alpha * e.queue;
                float ratio = (e.capacity > 0) ? (e.vehicles / e.capacity) : 0;
                float congestionTerm = beta * pow(ratio, 2);
                stepCost += (queueTerm + congestionTerm);
            }
        }
        return stepCost;
    }

    void calculateTravelMetrics() {
        int arrivedCount = 0;
        float sumTravelTime = 0;
        float sumDelay = 0;
        int N = 0;

        for (auto& v : vehicles) {
            if (v.status == "arrived") {
                float T_sd = v.endTime - v.startTime;
                float T_free = v.idealTravelTime;

                sumTravelTime += T_sd;
                sumDelay += (T_sd - T_free);
                N++;
            }
        }
        for (auto& v : vehicles) {
            if (v.status == "arrived") arrivedCount++;
        }

        float throughput = (stepCount > 0) ? (float)arrivedCount / stepCount : 0;
        cout << "Throughput: " << throughput << " vehicles/step" << endl;

        cout << "Average Travel Time: " << (N ? sumTravelTime / N : 0) << endl;
        cout << "Total Delay: " << sumDelay << endl;

    }
    float getThroughput() {
        int arrived = 0;
        for (auto& v : vehicles)
            if (v.status == "arrived") arrived++;

        return (stepCount > 0) ? (float)arrived / stepCount : 0;
    }
    float getAverageTravelTime() {
        float sum = 0;
        int count = 0;

        for (auto& v : vehicles) {
            if (v.status == "arrived") {
                sum += (v.endTime - v.startTime);
                count++;
            }
        }

        return (count ? sum / count : 0);
    }
    float getTotalCost() {
        return totalSystemCost;
    }
    float getTotalDelay() {
        float delay = 0;

        for (auto& v : vehicles) {
            if (v.status == "arrived") {
                float actual = v.endTime - v.startTime;
                delay += (actual - v.idealTravelTime);
            }
        }

        return delay;
    }

    float getAverageCongestion() {
        float totalRatio = 0;
        int edgeCount = 0;

        for (rnode* nodePtr : road->getNodes()) {
            for (auto& e : nodePtr->l) {

                if (e.capacity > 0) {
                    totalRatio += (e.vehicles / e.capacity);
                }
                edgeCount++;
            }
        }
        return (edgeCount > 0) ? (totalRatio / edgeCount) : 0;
    }

    vector<Vehicle>& getVehicles() { return vehicles; }

    void simulateStep(float dt, float alpha, float beta, float dischargeRate = 2.0f) {

        updateSignals();

        // =====================================================
        // 1. MOVE VEHICLES ALONG CURRENT EDGE (FLOW UPDATE)
        // =====================================================
        for (auto& v : vehicles) {

            if (v.status != "moving") continue;
            if (v.path.empty() || v.pathIndex + 1 >= (int)v.path.size()) {
                v.status = "arrived";
                v.endTime = stepCount;
                continue;
            }

            v.remainingTime -= dt;
            v.totalTravelTime += dt;

            if (v.remainingTime <= 0.0f) {

                // Vehicle reaches end of edge → enters queue
                v.status = "waiting";

                rnode* n = road->getNode(v.currentFrom);
                if (n) {
                    for (auto& e : n->l) {
                        if (e.to == v.path[v.pathIndex + 1]) {

                            e.queue += 1.0f;

                            // remove from flow on edge
                            e.vehicles = std::max(0.0f, e.vehicles - 1.0f);

                            break;
                        }
                    }
                }
            }
        }

        // =====================================================
        // 2. DISCHARGE QUEUES (d_ij MODEL)
        // =====================================================
        for (auto nodePtr : road->getNodes()) {

            for (auto& e : nodePtr->l) {

                if (!e.green || e.queue <= 0)
                    continue;

                float availableCapacity = 999.0f;

                int nextNode = -1;

                // find next edge in vehicle path
                for (auto& v : vehicles) {
                    if (v.status == "waiting" &&
                        v.currentFrom == nodePtr->vert &&
                        v.pathIndex + 1 < (int)v.path.size() &&
                        v.path[v.pathIndex + 1] == e.to) {

                        if (v.pathIndex + 2 < (int)v.path.size())
                            nextNode = v.path[v.pathIndex + 2];

                        break;
                    }
                }

                // downstream capacity check
                if (nextNode != -1) {
                    rnode* next = road->getNode(e.to);

                    if (next) {
                        for (auto& ne : next->l) {
                            if (ne.to == nextNode) {
                                availableCapacity = ne.capacity - ne.vehicles;
                                break;
                            }
                        }
                    }
                }

                // d_ij = min(Q, mu, capacity)
                int discharge = (int)getTrafficMin(e.queue, dischargeRate, availableCapacity);

                for (int i = 0; i < discharge; i++) {

                    for (auto& v : vehicles) {

                        if (v.status != "waiting") continue;

                        if (v.currentFrom == nodePtr->vert &&
                            v.pathIndex + 1 < (int)v.path.size() &&
                            v.path[v.pathIndex + 1] == e.to) {

                            // reduce queue
                            e.queue = std::max(0.0f, e.queue - 1.0f);

                            // move vehicle forward
                            v.pathIndex++;
                            v.currentFrom = v.path[v.pathIndex];

                            // ARRIVAL CHECK
                            if (v.currentFrom == v.destination) {
                                v.status = "arrived";
                                v.endTime = stepCount;
                            }
                            else {
                                v.status = "moving";

                                // update edge time (BPR model)
                                v.edgeTotalTime = getTravelTime(
                                    v.currentFrom,
                                    v.path[v.pathIndex + 1],
                                    alpha,
                                    beta
                                );

                                v.remainingTime = v.edgeTotalTime;

                                // increase flow on new edge
                                rnode* n2 = road->getNode(v.currentFrom);

                                if (n2) {
                                    for (auto& e2 : n2->l) {
                                        if (e2.to == v.path[v.pathIndex + 1]) {
                                            e2.vehicles += 1.0f;
                                            break;
                                        }
                                    }
                                }
                            }

                            break;
                        }
                    }
                }
            }
        }

     
        updateEdgeTimes(alpha, beta);

     
        totalSystemCost += calculateCurrentStepCost(alpha, beta);

        stepCount++;
    }
};