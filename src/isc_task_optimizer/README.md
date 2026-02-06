# `isc_task_optimizer` (ROS 2 package)

`isc_task_optimizer` provides the optimizer ROS 2 node and the underlying (non-ROS) optimization logic.

## Configuration

### 1) Optimizer logic settings (packing / time windows / robot dimensions)

Configurable parameters for the optimization pipeline are defined in:

- `src/isc_task_optimizer/optimizer_logic/main_logic_settings.py`

Key parameters:
- `LOW_TIME_WINDOW`: Time window (minutes) for low-volume merchants.
- `HIGH_TIME_WINDOW`: Time window (minutes) for high-volume merchants.
- `HIGH_WINDOW_MERCHANTS`: Merchants eligible for the high time window (e.g., Lotte Duty Free).
- `ROBOT_DIMENSIONS`: Delivery robot dimensions in millimeters.

### 2) Optimizer node settings (polling / debounce / scheduling / default date)

Configurable parameters for the ROS 2 node are defined in:

- `src/isc_task_optimizer/optimizer_node_settings.py`

Key parameters:
- `TARGET_DATE_DEFAULT`: Default order date used by the optimizer (DD/MM/YYYY).
- `POLL_PERIOD_SEC_DEFAULT`: How often the node polls MySQL for new orders for date D (seconds).
- `DEBOUNCE_SEC_DEFAULT`: Debounce duration to batch triggers after detecting new orders (seconds).
- `SCHEDULED_RUN_PERIOD_SEC_DEFAULT`: How often the node triggers a scheduled optimization run (seconds).

> Note: These defaults are currently defined as Python constants. If ROS parameters are added later, they can override these defaults at runtime.

### 3) MySQL environment variables (`.env`)

MySQL connection credentials are loaded from the workspace root `.env` file:

- `~/isc_ws/.env`

An example template is provided in `.env.example`.

The MySQL schema (optional reference) is located at:

- `~/isc_ws/db/schema.sql`
