#pragma once
#include <iostream>
#include <vector>
#include <string>
#pragma once
#include <vector>
#include <string>

class Vehicle {
public:
    int id, source, destination;

    int currentFrom;
    float remainingTime;
    float edgeTotalTime;

    std::string status;

    std::vector<int> path;
    int pathIndex;

    int startTime;
    int endTime;

    float idealTravelTime;
    float totalTravelTime;

    Vehicle(int i, int s, int d, int st) {
        id = i;
        source = s;
        destination = d;

        currentFrom = s;
        remainingTime = 0;
        edgeTotalTime = 0;

        status = "moving";

        pathIndex = 0;

        startTime = st;
        endTime = -1;

        idealTravelTime = 0;
        totalTravelTime = 0;
    }
};