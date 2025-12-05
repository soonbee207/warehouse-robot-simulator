# 🚚 Warehouse Robot Simulator
### CS 5100 Final Project – Multi-Robot Navigation & Path Planning  
**Author:** Soonbee Hwang, Siting Wen
**Repository:** https://github.com/soonbee207/warehouse-robot-simulator

---

## 1. Overview

This project implements a **multi-robot warehouse simulation** where autonomous robots navigate a 10×10 grid, avoid collisions, compute shortest paths using **A\***, and complete delivery tasks.  
The system includes:

- A\* Path Planning  
- Dynamic Velocity & Acceleration  
- Priority-Based Collision Avoidance  
- Task Pickup & Dropoff  
- Animated Visualization (Matplotlib)  
- Streamlit Web Interface  

Four robots move simultaneously, plan paths around shelves, avoid collisions, and deliver packages to dropoff zones.

### Guideline Compliance
- Robots modulate speed continuously to smooth motion and penalize abrupt acceleration, mirroring the proposal’s safety-focused cost term.
- Pairwise conflict detection enforces separation by pausing lower-priority robots whenever potential collisions or swaps are predicted.
- Each robot completes a pickup-and-drop-off cycle, supporting higher throughput and task completion tracking across four concurrent agents.

---

## 2. System Architecture
```
┌─────────────────┐
│ Streamlit UI │
└────────┬────────┘
│
┌────────▼──────────┐
│ Simulation Core │
│ (Robots & Tasks) │
└────────┬──────────┘
│
┌──────▼───────┐
│ Path Planner │
│ (A*) │
└──────────────┘
│
┌────────▼────────┐
│ Collision Logic │
└─────────────────┘
```

### Local Setup

1. **Clone the repository:**
```bash
git clone https://github.com/soonbee207/warehouse-robot-simulator.git
cd warehouse-robot-simulator
```

2. **Install dependencies:**
```bash
pip install -r requirements.txt
```

3. **Run the Streamlit app:**
```bash
streamlit run warehouse_sim.py
```
