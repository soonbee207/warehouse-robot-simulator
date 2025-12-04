# warehouse-robot-simulator
Warehouse robot simulation for CS 5100 Final Project

A dynamic warehouse robot simulation built with Streamlit that demonstrates multi-robot path planning, collision avoidance, and task management in a grid-based environment.


Warehouse Robot Simulation
A dynamic warehouse robot simulation built with Streamlit that demonstrates multi-robot path planning, collision avoidance, and task management in a grid-based 

Features
A Path Planning Algorithm*: Efficient pathfinding for each robot
Real-time Collision Avoidance: Priority-based conflict resolution system
Dynamic Velocity Control: Robots adjust speed based on proximity to others
Visual Path Display: Dotted lines showing planned routes for each robot
Task Management: Automated pickup and delivery system
Interactive Animation: Smooth visualization of robot movements
Start/End Markers: Clear visualization of robot starting and destination points

Key Components:
S markers: Starting positions (light colored circles)
E markers: Ending/drop-off positions (light colored circles)
Yellow squares: Task pickup locations (T1-T4)
Brown squares: Static obstacles
Dotted lines: Planned paths for each robot
Green boxes: Carried tasks

Installation
Prerequisites

Python 3.8 or higher
pip package manager

Step 1: Clone or Download
bash# If using git
git clone <repository-url>
cd warehouse-robot-simulation

# Or simply download the files to a folder
Step 2: Create Virtual Environment (Recommended)
bash# Windows
python -m venv venv
venv\Scripts\activate

# macOS/Linux
python3 -m venv venv
source venv/bin/activate
Step 3: Install Dependencies
bashpip install -r requirements.txt
If you don't have a requirements.txt file, create one with:
txtstreamlit>=1.28.0
numpy>=1.24.0
matplotlib>=3.7.0
Then run:
bashpip install -r requirements.txt
OR install packages directly:
bashpip install streamlit numpy matplotlib

Usage
Running the Simulation

Navigate to the project directory:

bashcd path/to/warehouse-robot-simulation

Run the Streamlit app:

bashstreamlit run warehouse_sim.py

Your default browser will automatically open to http://localhost:8501
Click the "Generate Animation" button to start the simulation
