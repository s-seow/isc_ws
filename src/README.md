# `isc_ws` Workspace

This is a ROS2/Python workspace, containing the `isc_task_optimizer` node and the `isc_main_logic` package that is called by the former to perform iShopChangi optimization tasks. 

The current workflow makes use of a local MySQL Database connection.

## Set-up

### Dependencies 

- ROS2 is required to build and run this program.
- A MySQL Database Connection is also required, with a database containing 3 tables labelled iSCOrderData, ItemDictionary and MerchantData.

### Steps

1. Build the workspace
    ```bash
    colcon build
    ```

2. Source and export path
    ```bash
    source install/setup.bash
    export PYTHONPATH="$PWD/src:$PYTHONPATH"
    ```

3. Launch the node
    ```bash
    ros2 run isc_task_optimizer optimizer_node
    ```

### Structure 
```
├─ src/
│  ├─ isc_main_logic/ 
│  │  └─ main_logic.py                  #
│  └─ isc_task_optimizer/src/optimizer  
│     └─ optimizer_node.py              #
├─ db/
│  └─ schema.sql                        #
└─ tools/mysql_loader
   ├─ itemDict_to_mysql.py              #
   └─ orderData_to_mysql.py             #
```

### Parameters

1. **`isc_main_logic` package**

    The configurable parameters are found in `isc_main_logic/settings.py`.

    - `LOW_TIME_WINDOW`: Denotes the time window given to low volume merchants.
    - `HIGH_TIME_WINDOW`: Denotes the time window given to high volume merchants.
    - `HIGH_WINDOW_MERCHANTS`: Refers to merchants that are given a longer time window, e.g. Lotte Duty Free.
    - `robot_dimensions`: The dimensions of the delivery robot in millimeters.

2. **`isc_task_optimizer` package** 

    The configurable parameters are handled by `isc_task_optimizer/settings.py`.
    - `TARGET_DATE_DEFAULT`: Denotes the order date that is ran by the optimizer.
    - `POLL_PERIOD_SEC_DEFAULT`: Denotes how often the node polls MySQL for new orders for date D.
    - `DEBOUNCE_SEC_DEFAULT`: Denotes how long to wait to batch new orders after first detecting a new order.

3. **MySQL Environment Variables**

    For the MySQL database, the connection is handled in the root `.env` file. `.env.example` provides an example of how it should be configured.

    For the MySQL schema, it can be found in the root /db/ folder.