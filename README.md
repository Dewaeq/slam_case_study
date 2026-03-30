# UGent Racing SLAM Case Study

Welcome to the SLAM Case Study! We are looking for a sharp, driven engineer to help build the Simultaneous Localization and Mapping (SLAM) pipeline for our autonomous racecar. 

This task is designed to test your C++ programming skills, your understanding of autonomous systems, and your ability to think critically about noisy sensor data in a dynamic environment.

**Important Note:** We are evaluating your *thought process, code architecture, and problem-solving skills*. SLAM is notoriously difficult. **Do not stress if you cannot get the map to close perfectly.** A partial solution with a brilliant `ANALYSIS.md` explaining *why* it failed is worth far more to us than a hacked-together solution that works by accident. Show us how you think!

**Expected Time Commitment:** 3 to 10 hours.

## The Scenario

Inside this repository, you will find a 2D SLAM simulator written in C++. 
* **The Track:** An elliptical circuit lined with 30 cones.
* **The Car:** Drives 2 full laps.
* **The Sensors:** * A simulated LiDAR that perfectly detects the local position of cones within a 6-meter radius.
  * A simulated Odometry system (wheel speeds + IMU) that accumulates severe angular and lateral **drift** over time.

**The Problem:** The current `slam_algorithm.cpp` uses the drifted odometry to place LiDAR observations onto a global map. Because it relies on a naive Nearest-Neighbor search, the system works fine for the first half of a lap. However, as the odometry drift increases, the system fails to recognize previously mapped cones. Instead of "closing the loop," it creates duplicate cones, resulting in a severely "ghosted" and inaccurate map.

## Your Mission

Your evaluation is split into three parts:

### 1. Build & Visualize (The Setup)

We highly recommend using GitHub Codespaces for this task. It provides a pre-configured Linux environment in your browser with zero setup required.
1. Click the green **"<> Code"** button at the top of this repository, select the **"Codespaces"** tab, and create a new codespace.
2. Once the editor loads, run the default build task `(Menu -> Terminal -> Run Build Task)`. Alternatively, open the terminal at the bottom and run our automated script:

```bash
./run.sh
```


**Task**: Look closely at the output: notice what happens during the second lap when the car returns to the start line. (Ground truth is shown as black circles; your algorithm's output is shown as blue triangles).

### 2. Algorithm Improvement

Open `src/slam_algorithm.cpp` and look at the `process_observations()` method. Currently, observations are matched to map landmarks one by one based on a strict 1.0m threshold. This is brittle. If two observations are close to the same map cone, or if the car's drift exceeds 1 meter, the logic fails.

**Task**: Implement a more robust data association method. You could explore assignment algorithms (like the Hungarian algorithm), incorporate landmark constellations, or implement better validation gating. Write clean, efficient, and well-structured C++.
**You may only use the C++ Standard Library and Eigen3.** 

*(Remember: It is perfectly fine if your solution doesn't completely fix the map! Just try your best to improve the association logic and comment your code so we can follow your logic).*

### 3. Architecture Review
Even with perfect data association, your map will still not be entirely correct because of a fundamental flaw in how this specific algorithm is designed to handle state.

**Task**: In a file named `ANALYSIS.md`, write a concise technical explanation (max 500 words) answering these questions:
- Explain the approach you took to improve the data association. Why did you choose it?
- Even if your data association perfectly matches lap 2 observations to lap 1 cones, why does the map still distort?
- What fundamental SLAM concept is missing from this codebase?

**Good luck, have fun, and we look forward to reading your analysis!**
