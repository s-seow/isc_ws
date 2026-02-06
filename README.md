# `isc_ws` ROS 2 Workspace

`isc_ws` is a ROS 2 (Humble) workspace containing two Python (`ament_python`) packages:

- **`isc_task_optimizer`**: Runs the iShopChangi optimization pipeline (MySQL → packing → routing → robot assignment) and publishes summaries to a ROS topic.
- **`isc_subscriber`**: Subscribes to the optimizer summary topic.

The current workflow uses a **local MySQL 8.0** database connection configured via a `.env` file.

## Setup

### System Requirements

- OS: Ubuntu 22.04
- ROS 2 Distribution: ROS 2 Humble Hawksbill
- Python Version: Python 3.10.12
- MySQL Server Version: MySQL 8.0.44

### Dependencies 

- ROS 2 is required to build and run this program.
  - ament_python, rclpy, std_msgs, ament_index_python
- Python is required.
    - PyMySQL==1.1.2, python-dotenv==1.2.1

### Workspace Repo Structure (High-Level)
```
isc_ws/
├─ README.md                                  # Workspace overview (you are here)
├─ .env                                       # DB credentials (gitignored)
├─ src/                                       # ROS 2 workspace source space
│  ├─ isc_task_optimizer/                     # ROS 2 package: optimizer node + optimization logic
│  │  ├─ package.xml
│  │  ├─ setup.py
│  │  ├─ resource/
│  │  │  └─ isc_task_optimizer
│  │  ├─ README.md                            # Package-level docs (optimizer)
│  │  └─ src/                                 
│  │     └─ isc_task_optimizer/               # Python import package
│  │        ├─ optimizer_node.py              # ROS 2 Node: polling, timers, triggers, publish summary
│  │        ├─ optimizer_node_settings.py     # Node settings 
│  │        └─ optimizer_logic/               # Non-ROS logic (called by optimizer_node)
│  │           ├─ main_logic.py               # run_logic(): DB → packing → routing → assignment
│  │           ├─ main_logic_settings.py      # Logic settings 
│  │           └─ helpers/
│  │              ├─ logic/                   # packing + assignment helpers
│  │              ├─ routing/                 # routing helpers + maps loader
│  │              │  └─ maps/                 # JSON routing maps (installed via setup.py data_files)
│  │              └─ utils.py
│  └─ isc_subscriber/                         # ROS 2 package: subscribes to optimizer summary topic
│     ├─ package.xml
│     ├─ setup.py
│     ├─ resource/
│     │  └─ isc_subscriber
│     ├─ README.md                            # Package-level docs (subscriber)
│     └─ src/
│        └─ isc_subscriber/
│           └─ summary_subscriber.py          # ROS 2 node: caches payload + interactive merchant query
├─ db/
│  └─ schema.sql                              # Database schema (optional)
└─ tools/
   └─ mysql_loader/                           # CSV → MySQL loading scripts (optional)
```

## Steps

### Build
```bash
cd ~/isc_ws
colcon build
source install/setup.bash
```

### Run

Terminal 1: Optimizer node
```bash
cd ~/isc_ws
source install/setup.bash
ros2 run isc_task_optimizer optimizer_node
```

Terminal 2: Subscriber node
```bash
cd ~/isc_ws
source install/setup.bash
ros2 run isc_subscriber summary_subscriber
```