#!/usr/bin/env python3
"""
"""

import json
import time

import paho.mqtt.client as mqtt

MAZE_ROWS = 13
MAZE_COLS = 13
MQTT_BROKER = "localhost" 
MQTT_PORT = 1883
POSE_TOPIC = "robot/pose"

# wall bit per side, OR'd together
WALL_N, WALL_E, WALL_S, WALL_W = 0x1, 0x2, 0x4, 0x8

WALLS = [
    [12, 6, 12, 6, 13, 4, 0, 4, 5, 6, 12, 5, 6],
    [10, 11, 10, 10, 12, 3, 8, 2, 12, 1, 1, 6, 10],
    [8, 5, 3, 8, 1, 6, 9, 2, 9, 6, 13, 2, 10],
    [10, 12, 4, 3, 12, 1, 4, 0, 6, 9, 4, 2, 10],
    [10, 10, 8, 5, 3, 13, 2, 10, 9, 6, 10, 11, 10],
    [8, 3, 9, 4, 5, 6, 8, 1, 6, 10, 9, 5, 2],
    [10, 12, 4, 1, 6, 10, 9, 6, 10, 8, 5, 5, 2],
    [8, 1, 2, 12, 3, 10, 12, 3, 9, 2, 12, 6, 10],
    [9, 6, 10, 10, 12, 1, 2, 12, 5, 1, 0, 1, 3],
    [14, 8, 1, 1, 3, 12, 1, 3, 12, 4, 2, 12, 6],
    [8, 0, 4, 7, 12, 3, 12, 6, 10, 9, 1, 2, 10],
    [10, 10, 9, 6, 10, 12, 2, 8, 1, 7, 12, 0, 2],
    [9, 1, 5, 1, 1, 3, 8, 1, 5, 5, 3, 9, 3],
]
# Maze map -- the same data as WALLS above, drawn. row 0 = SOUTH (bottom),
# col 0 = WEST (left). The gaps in the top and bottom edges are EXIT_CELLS.
#
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#   row 12 |                 |              |     |
#          +  +  +--+  +  +  +  +  +--+--+  +  +  +
#   row 11 |  |  |     |  |     |        |        |
#          +  +  +  +--+  +--+  +  +  +--+--+  +  +
#   row 10 |           |     |     |  |        |  |
#          +  +  +--+--+--+  +--+--+  +  +  +  +  +
#   row  9 |  |           |        |        |     |
#          +--+  +  +  +  +--+  +  +--+--+  +--+--+
#   row  8 |     |  |  |        |                 |
#          +  +--+  +  +--+  +  +--+--+  +  +  +  +
#   row  7 |        |     |  |     |     |     |  |
#          +  +  +  +--+  +  +--+  +  +  +--+--+  +
#   row  6 |  |           |  |     |  |           |
#          +  +--+--+  +--+  +  +--+  +  +--+--+  +
#   row  5 |     |           |        |  |        |
#          +  +  +  +--+--+--+  +  +--+  +  +--+  +
#   row  4 |  |  |        |     |  |     |  |  |  |
#          +  +  +  +--+  +--+  +  +  +--+  +  +  +
#   row  3 |  |        |              |        |  |
#          +  +--+--+  +--+  +--+  +--+  +--+  +  +
#   row  2 |        |        |     |     |     |  |
#          +  +--+  +  +  +--+  +  +  +--+--+  +  +
#   row  1 |  |  |  |  |     |     |           |  |
#          +  +  +  +  +--+  +  +  +--+  +  +--+  +
#   row  0 |     |     |                 |        |
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col

# the 2 known exits: (row, col, facing)
EXIT_CELLS = [
    (0, 6, 'south'),
    (MAZE_ROWS - 1, 6, 'north'),
]

BOT_CMD_TOPIC = "bot/cmd"         
PELLETS_TOPIC = "pellets/pose"    
CMD_VEL_TOPIC = "robot/cmd_vel"  

# yaw -> dr, dc, wall bit. 0=EAST, 90=NORTH, 180=WEST, 270=SOUTH
HEADING_DELTA = {
    0.0:   (0, 1, WALL_E),
    90.0:  (1, 0, WALL_N),
    180.0: (0, -1, WALL_W),
    270.0: (-1, 0, WALL_S),
}


# ============================================================================
# YOUR ALGORITHM GOES HERE. Everything above and below is plumbing.
# ============================================================================
def choose_command(pacbot_cell, pacbot_yaw, pellets_remaining):
    from collections import deque

    start = pacbot_cell

    directions = {
        0.0:   (0, 1, WALL_E),
        90.0:  (1, 0, WALL_N),
        180.0: (0, -1, WALL_W),
        270.0: (-1, 0, WALL_S),
    }

    def get_neighbors(cell):
        row, col = cell
        neighbours = []

        for yaw, (dr, dc, wall) in directions.items():
            new_row = row + dr
            new_col = col + dc

            if not (0 <= new_row < MAZE_ROWS and
                    0 <= new_col < MAZE_COLS):
                continue

            if WALLS[row][col] & wall:
                continue

            neighbours.append(((new_row, new_col), yaw))

        return neighbours

    def bfs(source, target):
        queue = deque([source])
        parent = {source: None}
        direction_taken = {}

        while queue:
            current = queue.popleft()

            if current == target:
                break

            for neighbour, direction in get_neighbors(current):
                if neighbour not in parent:
                    parent[neighbour] = current
                    direction_taken[neighbour] = direction
                    queue.append(neighbour)

        if target not in parent:
            return None

        path = []
        current = target

        while current != source:
            direction = direction_taken[current]
            path.append((current, direction))
            current = parent[current]

        path.reverse()
        return path

    # Find nearest pellet
    if pellets_remaining:
        best_path = None

        for pellet in pellets_remaining:
            path = bfs(start, tuple(pellet))

            if path is not None:
                if best_path is None or len(path) < len(best_path):
                    best_path = path

        if best_path is None:
            return None

        # Already standing on pellet
        if len(best_path) == 0:
            return "FRONT"

        next_cell, required_yaw = best_path[0]

    # No pellets → go to nearest exit
    else:
        best_path = None
        best_exit_direction = None
        best_exit_cell = None

        for exit_row, exit_col, exit_direction in EXIT_CELLS:
            exit_cell = (exit_row, exit_col)
            path = bfs(start, exit_cell)

            if path is not None:
                if best_path is None or len(path) < len(best_path):
                    best_path = path
                    best_exit_direction = exit_direction
                    best_exit_cell = exit_cell

        if best_path is None:
            return None

        if start == best_exit_cell:
            exit_yaw = {
                'east': 0.0,
                'north': 90.0,
                'west': 180.0,
                'south': 270.0
            }[best_exit_direction]

            required_yaw = exit_yaw
        else:
            next_cell, required_yaw = best_path[0]

    # Convert direction into command
    current_yaw = pacbot_yaw % 360
    required_yaw = required_yaw % 360

    diff = (required_yaw - current_yaw) % 360

    if diff == 0:
        return "FRONT"
    elif diff == 90:
        return "LEFT"
    elif diff == 180:
        return "BACK"
    elif diff == 270:
        return "RIGHT"

    return None
# ============================================================================


def parse_pellets(payload):
    return {tuple(cell) for cell in json.loads(payload)}


def main():
    state = {
        "running": False,
        "pellets": set(),
        "cell": (MAZE_ROWS // 2, MAZE_COLS // 2),
        "yaw": 0.0,
        "in_flight": False,
        "got_pose": False,
        "got_pellets": False,
    }

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Controller")

    def decide_and_send():
        if (not state["running"] or state["in_flight"]
                or not state["got_pose"] or not state["got_pellets"]):
            return
        print(f"[debug] pose={state['cell']} yaw={state['yaw']} pellets={state['pellets']}")
        cmd = choose_command(state["cell"], state["yaw"], set(state["pellets"]))
        if cmd is not None:
            state["in_flight"] = True
            client.publish(CMD_VEL_TOPIC, cmd)
            print(f"[controller] {state['cell']} yaw={state['yaw']} -> {cmd}, "
                  f"pellets_left={len(state['pellets'])}")

    def on_message(client, userdata, msg):
        try:
            if msg.topic == BOT_CMD_TOPIC:
                running = msg.payload.decode().startswith("1")
                was_running = state["running"]
                state["running"] = running
                if running and not was_running:
                    decide_and_send()   # kick off the reactive loop on Start
            elif msg.topic == PELLETS_TOPIC:
                state["pellets"] = parse_pellets(msg.payload.decode())
                state["got_pellets"] = True
                decide_and_send()
            elif msg.topic == POSE_TOPIC:
                data = json.loads(msg.payload.decode())
                state["cell"] = (int(data["col"]), int(data["row"]))   # wire is swapped
                state["got_pose"] = True
                state["yaw"] = float(data.get("yaw", 0.0))
                state["in_flight"] = False   # this pose is the ack for our last command
                decide_and_send()   # every pose/command-ack triggers the next step
        except Exception as e:
            print("[controller] mqtt parse error:", e)

    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe([(BOT_CMD_TOPIC, 0), (PELLETS_TOPIC, 0), (POSE_TOPIC, 0)])
    client.loop_start()

    print(f"[controller] ready; sending one '{CMD_VEL_TOPIC}' command at a time, "
          f"reacting to '{POSE_TOPIC}'/'{PELLETS_TOPIC}' feedback")

    try:
        while True:
            time.sleep(0.2)  
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()
    state["cell"]= (6, 0)
    state["yaw"] = 0
    state["pellets"] = {(12, 0), (10, 1)}
    cmd = choose_command(
        state["cell"],
        state["yaw"],
        set(state["pellets"])
    )

    print("CHOOSE_COMMAND RETURNED:", cmd)

    if cmd is not None:
        state["in_flight"] = True
        client.publish(CMD_VEL_TOPIC, cmd)
        print(f"[controller] {state['cell']} yaw={state['yaw']} -> {cmd}, "
            f"pellets_left={len(state['pellets'])}")

if __name__ == "__main__":
    main()
