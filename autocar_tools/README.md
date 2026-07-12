# autocar_tools

Provides tools for data collection, performance analysis, decision-making playback, and runtime telemetry visualization. Nothing inside this package is critical for robot movement, making it safe to isolate for development and debugging.

## 📂 Structure
- **`src/`**
  - `mission_performance_logger.cpp`: Captures high-level mission details (duration, distance, collisions) and writes reports to the fleet SQLite database.
  - `decision_trace_logger.cpp`: Records step-by-step state representations, actions, and arbitrator confidence levels.
  - `experience_logger_node.cpp`: Records transition tuples (state, action, reward, next_state) for offline reinforcement learning.
- **`scripts/`**
  - `benchmark_suite.py`: Automatically runs the vehicle through pre-defined scenarios under different planner configurations to benchmark execution.
  - `navigation_analytics.py`: Queries SQLite databases and plots performance graphs (CTE, rewards, success rates).
  - `decision_replay.py`: Command-line tool to step through and inspect recorded decision traces.
  - `visualizer_node.py`: Publishes custom visualization markers (arrows, text) to display robot plans and goals in RViz.
- **`analytics/`**
  - Destination directory for exported CSV sheets and Matplotlib evaluation plots.
