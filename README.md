# A* Path Planner for Bumperbot
### A ROS 2 Global Motion Planning Implementation
**A high-performance C++ implementation of the A* search algorithm for autonomous grid-based navigation.**

---

## 📖 Table of Contents
* [Overview](#overview)
* [Algorithm Implementation](#algorithm-implementation)
* [System Architecture](#system-architecture)
* [Coordinate Transformations](#coordinate-transformations)
* [Technical Specifications](#technical-specifications)
* [Installation & Usage](#installation--usage)

---

## 🔍 Overview
This package provides a global path planner for the Bumperbot mobile robot. It utilizes the **A* (A-Star) algorithm** to find the shortest path between the robot's current pose and a user-defined goal on a 2D occupancy grid. 

The planner integrates seamlessly with the ROS 2 navigation stack, utilizing TF2 for localization and publishing standard `nav_msgs/Path` messages for local controllers to follow.



---

## 🚀 Algorithm Implementation

The core logic resides in the `plan()` function, which improves upon standard Dijkstra by incorporating a **Manhattan Distance Heuristic**. This heuristic guides the search toward the goal, drastically reducing the state-space exploration.

### Cost Function
The node evaluates each grid cell using the standard A* formula:
$$f(n) = g(n) + h(n)$$

Where:
* $g(n)$: The actual cost (distance) from the start node to the current node.
* $h(n)$: The estimated cost to the goal (Manhattan Distance).

### Dead-Zone & Safety
* **Collision Avoidance:** The planner strictly checks the `OccupancyGrid` data. Any cell with a value other than `0` (free space) is treated as an obstacle.
* **Visited Map:** A secondary map is published to `/a_star/visited_map` using a custom intensity value (`-106`) to allow real-time visualization of the search frontier in RViz.

---

## 🏗 System Architecture

The node follows a standard ROS 2 asynchronous pattern, subscribing to map and goal updates while providing path data to the navigation system.

```text
┌──────────────────┐      nav_msgs       ┌──────────────────┐      nav_msgs      ┌────────────────┐
│      /map        │ ──────────────────> │                  │ ─────────────────> │  /a_star/path  │
│ (OccupancyGrid)  │                     │  AStarPlanner    │                    │     (Path)     │
└──────────────────┘                     │      Node        │                    └────────────────┘
          ^                              │                  │                             ^
          │           geometry_msgs      │                  │      nav_msgs               │
          └───────────────────────────── │                  │ ─────────────────>  /a_star/visited_map
                    /goal_pose           └──────────────────┘                    (OccupancyGrid)


```
---

## 📍 Coordinate Transformations
The planner operates across two distinct coordinate systems. Robust conversion logic ensures the mathematical search matches the physical world.

* **World Frame (Meters):** The continuous space where the robot resides (retrieved via `tf_buffer_` from `map` to `base_footprint`).
* **Grid Frame (Pixels):** The discrete space where the A* algorithm explores.



### Conversion Logic:
* **World to Grid:** $$Grid_{x} = \frac{World_{x} - Origin_{x}}{Resolution}$$
* **Grid to World:** $$World_{x} = (Grid_{x} \times Resolution) + Origin_{x}$$

---

## 📊 Technical Specifications

| Feature | Specification |
| :--- | :--- |
| **Language** | C++ 17 / ROS 2 Humble |
| **Heuristic** | Manhattan Distance |
| **Search Connectivity** | 4-Connected Neighbors (Up, Down, Left, Right) |
| **Data Structure** | `std::priority_queue` (Min-Heap) |
| **Transform Library** | `tf2_ros` |
| **QoS Policy** | Transient Local (for Map Durability) |



---

## 🛠 Installation & Usage

### Building the Package
Navigate to your Colcon workspace and build:

```bash
colcon build --packages-select bumperbot_planning
source install/setup.bash
ros2 run bumperbot_planning a_star_node
```
### Visualizing in RViz
To see the planner in action, configure your RViz2 workspace with the following settings:

1. **Fixed Frame**: Set this to `map`.
2. **Path Display**: Add a `Path` display and subscribe to the topic `/a_star/path`.
3. **Map Display**: Add a `Map` display and subscribe to `/a_star/visited_map`.
    * **Pro Tip**: Set the **Color Scheme** to `Costmap`. This will render the visited nodes in a distinct orange hue, allowing you to see the A* exploration frontier in real-time.
