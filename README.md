# traffic-flow-optimizer
# Traffic Flow Optimization System Using Graph Theory

A high-performance, simulation-based system designed to model urban road networks and dynamically optimize vehicle movement in real-time. Built from scratch in C++ using a modular architecture, this project utilizes graph theory and dynamic shortest-path routing paired with an SFML-based graphical user interface to visualize and minimize urban traffic congestion.

## 🚀 Core Features
* **Graph-Based Road Network:** Models an urban road network as a directed graph $G=(V,E)$, mapping intersections as nodes and road segments as edges.
* **Adaptive Traffic Signal Control:** Dynamically adjusts traffic signal timings at intersections, prioritizing green signals for roads with the longest vehicle queues.
* **Congestion-Aware Routing:** Uses a modified, real-time implementation of Dijkstra's Algorithm to update vehicle paths dynamically based on changing road capacities and delays.
* **Macro-Simulation Framework:** Simulates queue formation, vehicle discharge rates, and multi-source to multi-destination routing across discrete simulation cycles.

---

## 📐 Mathematical & System Design

The system maps physical traffic conditions to mathematical models updated at each discrete simulation step:

### 1. Traffic Flow & Queue Model
Traffic density on any given road segment evolves over time based on vehicle entry and departure rates:
$$f_{ij}(t+1) = f_{ij}(t) + a_{ij}(t) - x_{ij}(t)$$

When vehicles reach an intersection during a red signal or heavy downstream congestion, they enter a downstream queue ($Q_{ij}$), which updates as:
$$Q_{ij}(t+1) = Q_{ij}(t) + x_{ij}(t) - d_{ij}(t)$$

The vehicle discharge rate ($d_{ij}(t)$) past a green light is restricted by physical constraints:
$$d_{ij}(t) = g_{ij}(t) \cdot \min(Q_{ij}(t), \mu_{ij}, c_{jk} - f_{jk}(t))$$
Where $g_{ij}(t)$ is the binary signal state, $\mu_{ij}$ is the maximum intersection discharge rate, and $c_{jk} - f_{jk}(t)$ represents available capacity on the next road segment.

### 2. Travel Time & Congestion Cost
To prevent vehicles from routing into heavily congested areas, the cost of an edge is dynamically scaled using a non-linear travel time function:
$$w_{ij}(t) = w_{ij}^{\text{free}} \left(1 + \alpha \left(\frac{f_{ij}(t)}{c_{ij}}\right)^{\beta}\right)$$
Where $w_{ij}^{\text{free}}$ is the ideal travel time ($\frac{\text{length}}{\text{max speed}}$), $\alpha$ is congestion sensitivity, and $\beta$ is the non-linearity factor.

---

## 🛠️ Project Structure

```text
├── Road.h                # Defines Road edge attributes (length, capacity, velocity, flow)
├── Vehicle.h             # Tracks independent vehicle agents (source, destination, remaining time)
├── road_network.cpp      # Directed graph infrastructure using custom adjacency structures
├── SFML traffic.cpp      # GUI initialization, visualization loop, and SFML rendering 
├── Source.cpp            # Main entry point managing simulation cycle steps
├── SFML traffic.sln      # Visual Studio Solution configurations
└── SFML traffic.vcxproj  # Visual Studio Project build configurations

