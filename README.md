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

Three robots move simultaneously, plan paths around shelves, avoid collisions, and deliver packages to dropoff zones.

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
