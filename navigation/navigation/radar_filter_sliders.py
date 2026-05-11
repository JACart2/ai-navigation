#!/usr/bin/env python3
"""Tk slider window that drives the radar_xyz_filter node's bounds live.

Six sliders (x_min, x_max, y_min, y_max, z_min, z_max) call SetParameters on
the filter node every time you move them. On startup we read the current
values back from the filter node so the sliders reflect reality.

ROS parameters on this node:
    target_node (string, default "/radar_xyz_filter"):
        Fully-qualified name of the filter node whose params we drive.
"""
from __future__ import annotations

import threading
from typing import Dict

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node


PARAM_NAMES = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
SLIDER_LIMITS = {
    "x_min": (-10.0, 10.0),
    "x_max": (-10.0, 10.0),
    "y_min": (-10.0, 10.0),
    "y_max": (-10.0, 10.0),
    "z_min": (-3.0, 3.0),
    "z_max": (-3.0, 3.0),
}
DEFAULTS = {
    "x_min": 0.25, "x_max": 3.58,
    "y_min": -2.10, "y_max": 5.00,
    "z_min": -0.65, "z_max": 1.11,
}


class FilterClientNode(Node):
    def __init__(self):
        super().__init__("radar_filter_sliders")
        self.declare_parameter("target_node", "/radar_xyz_filter")
        target = self.get_parameter("target_node").value.rstrip("/")
        self.set_cli = self.create_client(SetParameters, f"{target}/set_parameters")
        self.get_cli = self.create_client(GetParameters, f"{target}/get_parameters")
        self.target = target

    def set_value(self, name: str, value: float):
        if not self.set_cli.service_is_ready():
            return None
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(value),
                ),
            )
        ]
        return self.set_cli.call_async(req)

    def fetch_values(self, on_done):
        """Async fetch of all six bound params; on_done(dict) called when ready."""
        if not self.get_cli.service_is_ready():
            on_done(None)
            return
        req = GetParameters.Request()
        req.names = list(PARAM_NAMES)
        future = self.get_cli.call_async(req)

        def _cb(_fut):
            try:
                resp = future.result()
                out: Dict[str, float] = {}
                for name, val in zip(PARAM_NAMES, resp.values):
                    out[name] = float(val.double_value)
                on_done(out)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"GetParameters failed: {exc}")
                on_done(None)

        future.add_done_callback(_cb)


def _run_gui(ros_node: FilterClientNode):
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title(f"Radar XYZ Filter — {ros_node.target}")
    root.geometry("520x420")

    main = ttk.Frame(root, padding=10)
    main.pack(fill="both", expand=True)

    ttk.Label(
        main,
        text=(
            "Sliders drive radar_xyz_filter live. Units = meters, radar frame.\n"
            "RViz shows: green = kept, red = rejected, green box = filter region."
        ),
        wraplength=480,
    ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

    vars_: Dict[str, tk.DoubleVar] = {}
    value_labels: Dict[str, ttk.Label] = {}
    scales: Dict[str, ttk.Scale] = {}
    suppress = {"flag": False}

    def on_slider_change(name: str, _value: str):
        if suppress["flag"]:
            return
        v = vars_[name].get()
        value_labels[name].config(text=f"{v:+.2f}")
        ros_node.set_value(name, v)

    for i, name in enumerate(PARAM_NAMES):
        lo, hi = SLIDER_LIMITS[name]
        var = tk.DoubleVar(value=DEFAULTS[name])
        vars_[name] = var

        ttk.Label(main, text=name, width=8).grid(
            row=i + 1, column=0, sticky="w", padx=(0, 6), pady=2
        )
        scale = ttk.Scale(
            main, from_=lo, to=hi, orient="horizontal", variable=var,
            command=lambda v, n=name: on_slider_change(n, v),
        )
        scale.grid(row=i + 1, column=1, sticky="ew", pady=2)
        scales[name] = scale

        label = ttk.Label(main, text=f"{DEFAULTS[name]:+.2f}", width=8, anchor="e")
        label.grid(row=i + 1, column=2, sticky="e", padx=(6, 0), pady=2)
        value_labels[name] = label

    main.columnconfigure(1, weight=1)

    btns = ttk.Frame(main)
    btns.grid(row=len(PARAM_NAMES) + 1, column=0, columnspan=3, sticky="ew", pady=(12, 0))

    def apply_values(values: Dict[str, float] | None):
        if values is None:
            status.config(text="Filter node not reachable — sliders inactive.")
            return
        suppress["flag"] = True
        for name, v in values.items():
            vars_[name].set(v)
            value_labels[name].config(text=f"{v:+.2f}")
        suppress["flag"] = False
        status.config(text=f"Synced from {ros_node.target}.")

    def schedule_apply(values):
        root.after(0, lambda: apply_values(values))

    def on_refresh():
        status.config(text="Reading current values…")
        ros_node.fetch_values(schedule_apply)

    def on_reset():
        suppress["flag"] = True
        for name, v in DEFAULTS.items():
            vars_[name].set(v)
            value_labels[name].config(text=f"{v:+.2f}")
            ros_node.set_value(name, v)
        suppress["flag"] = False
        status.config(text="Reset to defaults.")

    ttk.Button(btns, text="Refresh from node", command=on_refresh).pack(side="left")
    ttk.Button(btns, text="Reset to defaults", command=on_reset).pack(side="left", padx=8)

    status = ttk.Label(main, text="Connecting…", foreground="#444")
    status.grid(row=len(PARAM_NAMES) + 2, column=0, columnspan=3, sticky="w", pady=(8, 0))

    def initial_sync():
        if ros_node.set_cli.wait_for_service(timeout_sec=0.1):
            on_refresh()
        else:
            status.config(text=f"Waiting for {ros_node.target}/set_parameters…")
            root.after(500, initial_sync)

    root.after(200, initial_sync)
    root.mainloop()


def main():
    rclpy.init()
    node = FilterClientNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        _run_gui(node)
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
