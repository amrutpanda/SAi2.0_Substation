import keyboard
import redis

# keys
ROBOT_STATE_REQUESTED_KEY = "sai2::HapticApplications::01::robot_state_requested"
ROBOT_STATE_TRANSITION_KEY = "sai2::HapticApplications::01::robot_state_transition"
ROBOT_CLEANING_DIRECTION_KEY = "sai2::HapticApplications::01::cleaning_direction"
r = redis.Redis(host='localhost', port=6379, decode_responses=True)

# Define a dictionary to map keys to actions
key_actions = {
    "1": lambda: (
        print("State 1 requested"),
        r.set(ROBOT_STATE_REQUESTED_KEY, "1")
    ),
    "2": lambda: (
        print("State 2 requested"),
        r.set(ROBOT_STATE_REQUESTED_KEY, "2")
    ),
    "3": lambda: (
        print("State 3 requested"),
        r.set(ROBOT_STATE_REQUESTED_KEY, "3")
    ),
    "4": lambda: (
        print("State 4 requested"),
        r.set(ROBOT_STATE_REQUESTED_KEY, "4")
    ),
    "5": lambda: (
        print("State 5 requested"),
        r.set(ROBOT_STATE_REQUESTED_KEY, "5")
    ),
    "6": lambda: (
        print("State 6 requested"),
        r.set(ROBOT_STATE_REQUESTED_KEY, "6")
    ),
    "7": lambda: (
        print("State 7 requested"),
        r.set(ROBOT_STATE_REQUESTED_KEY, "7")
    ),
    "8": lambda: (
        print("State 8 requested"),
        r.set(ROBOT_STATE_REQUESTED_KEY, "8")
    ),
    "p": lambda: (
        print("State change accepted"),
        r.set(ROBOT_STATE_TRANSITION_KEY, "1")
    ),
    "up": lambda: (
        print("Up arrow"),
        r.set(ROBOT_CLEANING_DIRECTION_KEY, "1")
    ),
    "down": lambda: (
        print("Down arrow"),
        r.set(ROBOT_CLEANING_DIRECTION_KEY, "-1")
    )
}

# Register key event handlers for each target key
for key, action in key_actions.items():
    keyboard.on_press_key(key, lambda e, action=action: action())

try:
    # Keep the script running
    keyboard.wait()
except KeyboardInterrupt:
    pass
finally:
    # Unregister all event handlers when done
    keyboard.unhook_all()
