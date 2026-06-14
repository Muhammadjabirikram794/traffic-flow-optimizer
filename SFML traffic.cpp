#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <cmath>
#include <algorithm>
#include <unordered_map>

#include "Road.h"
#include "Vehicle.h"
#include "road_network.cpp"

const int WIDTH = 1200;
const int HEIGHT = 900;
const float NODE_RADIUS = 10.0f;

// -------------------------
// TIMERS
// -------------------------
sf::Clock simClock;
sf::Clock spawnClock;

// -------------------------
// TRAFFIC STATE
// -------------------------
std::unordered_map<int, bool> nodeGreen;

const float STOP_DISTANCE = 25.0f;

int main() {

    sf::RenderWindow window(sf::VideoMode({ WIDTH, HEIGHT }), "Traffic Flow Simulator (FIXED)");
    window.setFramerateLimit(60);

    // -------------------------
    // ROAD NETWORK
    // -------------------------
    Road road;
    std::map<int, sf::Vector2f> pos;

    for (int i = 0; i < 12; i++) {
        road.addVertex(i);
        pos[i] = {
            200.f + (i % 4) * 250.f,
            150.f + (i / 4) * 250.f
        };
    }

    for (int i = 0; i < 12; i++) {
        if ((i + 1) % 4 != 0 && i + 1 < 12)
            road.addedge(i, i + 1, 100, 50, 10);

        if (i + 4 < 12)
            road.addedge(i, i + 4, 100, 50, 10);
    }

    // -------------------------
    // SIMULATOR
    // -------------------------
    TrafficSimulator sim(&road);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, 11);

    int carId = 0;

    // -------------------------
    // FONT
    // -------------------------
    sf::Font font;
    if (!font.openFromFile("C:/Windows/Fonts/arial.ttf")) {
        std::cout << "Font load failed\n";
    }

    // =========================
    // MAIN LOOP
    // =========================
    while (window.isOpen()) {

        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // ======================================================
        // 🚗 CONTINUOUS SPAWNING
        // ======================================================
        if (spawnClock.getElapsedTime().asSeconds() > 0.15f) {

            int s = dist(gen);
            int d = dist(gen);
            if (s == d) d = (d + 1) % 12;

            if (sim.getVehicles().size() < 500) {
                sim.addVehicle(carId++, s, d, 0);
            }

            spawnClock.restart();
        }

        // ======================================================
        // 🚦 SIMULATION STEP
        // ======================================================
        sim.simulateStep(0.05f, 4.0f, 2.5f);

        float time = simClock.getElapsedTime().asSeconds();

        window.clear(sf::Color(25, 25, 25));

        // =========================
        // EDGES
        // =========================
        for (auto node : road.getNodes()) {
            for (auto& e : node->l) {

                sf::Vector2f a = pos[node->vert];
                sf::Vector2f b = pos[e.to];

                float ratio = (e.capacity > 0) ? (e.vehicles / e.capacity) : 0;

                sf::Color color;
                if (e.green)
                    color = sf::Color::Green;
                else if (ratio > 0.8)
                    color = sf::Color::Red;
                else if (ratio > 0.4)
                    color = sf::Color::Yellow;
                else
                    color = sf::Color(120, 120, 120);

                sf::Vector2f diff = b - a;
                float length = std::sqrt(diff.x * diff.x + diff.y * diff.y);

                sf::RectangleShape roadShape({ length, 6 });
                roadShape.setPosition(a);

                float angle = std::atan2(diff.y, diff.x) * 180.0f / 3.14159f;
                roadShape.setRotation(sf::degrees(angle));

                roadShape.setFillColor(color);

                window.draw(roadShape);
            }
        }

        // =========================
        // NODES + TRAFFIC LIGHTS
        // =========================
        for (auto& [id, p] : pos) {

            float localTime = time + id * 0.7f;
            bool green = ((int)localTime % 6 < 3);

            nodeGreen[id] = green;

            sf::CircleShape node(NODE_RADIUS);
            node.setOrigin({ NODE_RADIUS, NODE_RADIUS });
            node.setPosition(p);
            node.setFillColor(sf::Color::Blue);
            window.draw(node);

            sf::CircleShape light(6);
            light.setOrigin({ 6.f, 6.f });
            light.setPosition({ p.x, p.y - 20 });
            light.setFillColor(green ? sf::Color::Green : sf::Color::Red);
            window.draw(light);
        }

        // =========================
        // VEHICLES (REAL STOP-LINE LOGIC)
        // =========================
        for (auto& v : sim.getVehicles()) {

            if (v.status == "arrived") continue;
            if (v.path.size() < 2) continue;

            sf::CircleShape car(7);
            car.setOrigin({ 7.f, 7.f });

            bool isWaiting = (v.status == "waiting");

            car.setFillColor(isWaiting ? sf::Color::Red : sf::Color::Cyan);
            car.setOutlineThickness(2.f);
            car.setOutlineColor(sf::Color::White);

            sf::Vector2f position = pos[v.currentFrom];

            if (v.pathIndex + 1 < (int)v.path.size()) {

                int nextNode = v.path[v.pathIndex + 1];

                sf::Vector2f a = pos[v.currentFrom];
                sf::Vector2f b = pos[nextNode];

                float dx = b.x - a.x;
                float dy = b.y - a.y;
                float distToNode = std::sqrt(dx * dx + dy * dy);

                float speedFactor = 0.6f;

                float t = 1.0f - (v.remainingTime / (v.edgeTotalTime / speedFactor + 0.0001f));
                t = std::clamp(t, 0.0f, 1.0f);

                float maxAllowed = (distToNode - STOP_DISTANCE) / distToNode;
                maxAllowed = std::clamp(maxAllowed, 0.0f, 1.0f);

                bool green = nodeGreen[nextNode];

                // =========================
                // 🚦 HARD STOP LOGIC
                // =========================
                if (!green && t >= maxAllowed) {

                    v.status = "waiting";
                    position = a + (b - a) * maxAllowed;

                }
                else {

                    v.status = "moving";
                    position = a + (b - a) * t;
                }
            }

            car.setPosition(position);
            window.draw(car);
        }

        // =========================
        // STATS
        // =========================
        int moving = 0, waiting = 0, arrived = 0;

        for (auto& v : sim.getVehicles()) {
            if (v.status == "moving") moving++;
            else if (v.status == "waiting") waiting++;
            else if (v.status == "arrived") arrived++;
        }

        sf::Text stats(font);
        stats.setCharacterSize(16);
        stats.setFillColor(sf::Color::White);
        stats.setPosition({ 20, 20 });

        stats.setString(
            "TRAFFIC SYSTEM (FIXED)\n"
            "-----------------\n" +
            std::string("Moving: ") + std::to_string(moving) + "\n" +
            "Waiting: " + std::to_string(waiting) + "\n" +
            "Arrived: " + std::to_string(arrived) + "\n\n" +

            "Avg Travel Time: " + std::to_string(sim.getAverageTravelTime()) + "\n" +
            "Throughput: " + std::to_string(sim.getThroughput()) + "\n" +
            "Total Delay: " + std::to_string(sim.getTotalDelay()) + "\n" +
            "Avg Congestion: " + std::to_string(sim.getAverageCongestion()) + "\n" +
            "System Cost: " + std::to_string(sim.getTotalCost())
        );

        window.draw(stats);

        window.display();
    }

    return 0;
}