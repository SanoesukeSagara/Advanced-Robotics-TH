"""A robot drives towards a wall and stops at a safe distance.

Three "nodes" talk to each other only through topics:

    sensor node  --/scan-->  controller node  --/cmd_vel-->  robot (motor driver)

Extra subscribers ("ros2 topic echo") print the messages into the log window.
Use the checkboxes to subscribe/unsubscribe at runtime and watch what happens.
"""

import tkinter as tk

from topic import Topic

CANVAS_W, CANVAS_H = 640, 160
ROBOT_R = 20          # robot radius [px]
SAFE_DISTANCE = 60    # desired gap to the wall [px]
WARNING_DISTANCE = 30  # distance to wall to trigger warning [px]
MAX_SPEED = 150       # [px/s]
GAIN = 2.0            # P-controller gain [1/s]
DT = 0.05             # simulation step [s] -> 20 Hz

# --- Topics (the subjects) -------------------------------------------------
scan_topic = Topic("/scan")         # msg: distance to wall [px]
cmd_vel_topic = Topic("/cmd_vel")   # msg: velocity command [px/s]
battery_topic = Topic("/battery")     # msg: battery level [%]

robot = {"x": 60.0, "v": 0.0, "crashed": False, "battery": 100.0}  # robot state


# --- Subscriber callbacks (the observers) ---------------------------------

# ---- Robot node: subscribes to /cmd_vel and sets its wheel speed ---------
def motor_callback(v):
    """Robot node: subscribes to /cmd_vel and sets its wheel speed."""
    if robot["battery"] <= 0.0:
        robot["v"] = 0.0
    else:
        robot["v"] = v

# --- Controller node: subscribes to /scan and publishes to /cmd_vel ---------    
def controller_callback(distance):
    """Controller node: subscribes to /scan, publishes to /cmd_vel."""
    v = GAIN * (distance - SAFE_DISTANCE)
    v = max(-MAX_SPEED, min(MAX_SPEED, v))
    cmd_vel_topic.publish(v)


def main():
    root = tk.Tk()
    root.title("Observer pattern / ROS2 pub-sub demo")

    canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H, bg="white")
    canvas.pack(padx=10, pady=10)

    wall_x = tk.IntVar(value=500)
    tk.Scale(root, from_=150, to=CANVAS_W - 20, orient="horizontal", length=CANVAS_W,
             label="Wall position (drag me)", variable=wall_x).pack(padx=10)

    panel = tk.Frame(root)
    panel.pack(fill="x", padx=10)
    info = tk.Label(root, font=("Consolas", 10), justify="left")
    info.pack(anchor="w", padx=10)
    warning_label = tk.Label(panel, font=("Consolas", 10), justify="right", fg="red")
    warning_label.pack(anchor="w", padx=10)

    battery_label = tk.Label(panel, font=("Consolas", 10), justify="left", fg="green")
    battery_label.pack(anchor="e", padx=10)
    #log = tk.Text(root, height=10, width=80, font=("Consolas", 9))
    #log.pack(padx=10, pady=10)
    #create a separate log for each topic, so that the user can see the messages of each topic separately
    scan_log = tk.Text(root, height=10, width=30, font=("Consolas", 9)) 
    scan_log.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    cmd_vel_log = tk.Text(root, height=10, width=30, font=("Consolas", 9))
    cmd_vel_log.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    battery_log = tk.Text(root, height=10, width=30, font=("Consolas", 9))
    battery_log.pack(side="right", fill="both", expand=True, padx=10, pady=10)

    # --- Callback for an observer that prints messages of a topic (like `ros2 topic echo`) ---
    def echo(topic, log_widget):
        def callback(msg):
            log_widget.insert("end", "[{}] {:7.1f}\n".format(topic.name, msg))
            if int(log_widget.index("end-1c").split(".")[0]) > 200:
                log_widget.delete("1.0", "2.0")
            log_widget.see("end")
        return callback

    def update_info():
        info.config(text="subscribers: {} -> {}   /   {} -> {}   /   {} -> {}".format(
            scan_topic.name, scan_topic.subscriber_count(),
            cmd_vel_topic.name, cmd_vel_topic.subscriber_count(),
            battery_topic.name, battery_topic.subscriber_count()))
        
    def warning_callback(distance):
        """Warning node: subscribes to /scan and prints a warning if too close."""
        if distance < WARNING_DISTANCE:
            warning_label.config(text="WARNING: Too close to the wall!") 
        else:
            warning_label.config(text="") 

    def battery_callback(level):
        """Battery node: subscribes to /battery and prints the battery level."""
        battery_label.config(text="Battery level: {:.2f}%".format(level))
        
    def add_subscription_checkbox(text, topic, callback, enabled=True):
        var = tk.BooleanVar(value=enabled)

        def toggle():
            if var.get():
                topic.subscribe(callback)
            else:
                topic.unsubscribe(callback)
            update_info()

        tk.Checkbutton(panel, text=text, variable=var, command=toggle).pack(side="left")
        toggle()

    add_subscription_checkbox("controller ← /scan", scan_topic, controller_callback)
    add_subscription_checkbox("motor ← /cmd_vel", cmd_vel_topic, motor_callback)
    add_subscription_checkbox("warning ← /scan", scan_topic, warning_callback)
    add_subscription_checkbox("echo /scan", scan_topic, echo(scan_topic, scan_log), enabled=False)
    add_subscription_checkbox("echo /cmd_vel", cmd_vel_topic, echo(cmd_vel_topic, cmd_vel_log), enabled=False)
    add_subscription_checkbox("battery display ← /battery", battery_topic, battery_callback)
    add_subscription_checkbox("echo /battery", battery_topic, echo(battery_topic, battery_log), enabled=False)

    def reset():
        robot.update(x=60.0, v=0.0, crashed=False, battery=100.0)
        for log_widget in (scan_log, cmd_vel_log, battery_log):
            log_widget.delete("1.0", "end")

    tk.Button(panel, text="Reset robot", command=reset).pack(side="right")

    def draw():
        canvas.delete("all")
        y = CANVAS_H / 2
        x, w = robot["x"], wall_x.get()
        canvas.create_rectangle(w, 10, w + 15, CANVAS_H - 10, fill="gray30")
        canvas.create_line(w - SAFE_DISTANCE, 10, w - SAFE_DISTANCE, CANVAS_H - 10, dash=(4, 4), fill="green")
        canvas.create_line(x + ROBOT_R, y, w, y, fill="red", dash=(2, 2))  # sensor ray
        color = "red" if robot["crashed"] else "steelblue"
        canvas.create_oval(x - ROBOT_R, y - ROBOT_R, x + ROBOT_R, y + ROBOT_R, fill=color)
        canvas.create_text(x, y - ROBOT_R - 12, text="v = {:.0f} px/s".format(robot["v"]))
        if robot["crashed"]:
            canvas.create_text(CANVAS_W / 2, 20, text="CRASH !!!", fill="orange", font=("Arial", 16, "bold"))
        elif robot["battery"] <= 0.0:
            canvas.create_text(CANVAS_W / 2, 20, text="Battery empty!", fill="orange", font=("Arial", 16, "bold"))
    def tick():
        """Timer callback: sensor node publishes, then the physics moves the robot."""
        w = wall_x.get()

        #Battery Drain Simulation
        if robot["battery"] > 0.0:
            idle_drain = 0.01  # battery drain per tick when idle
            sensor_drain = 0.02  # battery drain per tick when moving

            # Drain calculation based on robot's velocity
            movement_drain = 0.0010 * abs(robot["v"])  # battery drain proportional to speed
            robot["battery"] -= (idle_drain + sensor_drain + movement_drain)
            robot["battery"] = max(0.0, robot["battery"])  # Ensure battery doesn't go below 0

        if robot["battery"] <= 0.0:
            robot["v"] = 0.0  # Stop the robot if battery is empty

        scan_topic.publish(w - robot["x"] - ROBOT_R)   # sensor node (publisher)
        battery_topic.publish(robot["battery"])

        robot["x"] += robot["v"] * DT
        robot["x"] = max(ROBOT_R, robot["x"])
        robot["crashed"] = robot["x"] + ROBOT_R >= w
        if robot["crashed"]:
            robot["x"] = w - ROBOT_R

        draw()
        root.after(int(DT * 1000), tick)

    tick()
    root.mainloop()


if __name__ == "__main__":
    main()

#Task 3 Why does `publish` iterate over `list(self._callbacks)` instead of `self._callbacks`? 
# The `publish` method iterates over `list(self._callbacks)` instead of iterating over 
# `self._callbacks` to avoid problems  that can occur if the list of `self._callbacks` is modified while `publish` is iterating over it meaning 
# that some callbacks might be skipped or called multiple times.