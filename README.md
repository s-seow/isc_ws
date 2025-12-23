# Optimizer Program for Delivery Robots

This repository contains an optimization program for delivery robots across 3 terminals.

## Dependencies / Prerequisites

- ROS2 is required to build and run this program.
- A MySQL Database Connection is also required, with a database containing 3 tables labelled iSCOrderData, ItemDictionary and MerchantData.

## How to Run

Build after any changes made
``colcon build``     

Source
``source install/setup.bash``

Export /src/ path
``export PYTHONPATH="$PWD/src:$PYTHONPATH"``

Run the optimizer node
``ros2 run isc_task_optimizer optimizer_node``
