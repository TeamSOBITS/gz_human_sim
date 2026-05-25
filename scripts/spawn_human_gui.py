#!/usr/bin/env python3
import argparse
import math
import os
import threading
import tkinter as tk
from tkinter import ttk
import logging

import rclpy
from rclpy.executors import MultiThreadedExecutor
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity


def _yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SpawnHumanGui:
    def __init__(self, args):
        self._logger = logging.getLogger("spawn_human_gui")
        self._models_root = args.models_root
        self._world_name = args.world_name
        self._default_name = args.model_name
        self._pose = (args.x, args.y, args.z, args.yaw)
        self._refresh_ms = int(args.refresh_ms)
        self._status_queue = []
        self._queue_lock = threading.Lock()
        self._model_names = []

        rclpy.init()
        self._node = rclpy.create_node("spawn_human_gui")
        self._spawn_client = self._node.create_client(
            SpawnEntity,
            f"/world/{self._world_name}/create",
        )
        self._delete_client = self._node.create_client(
            DeleteEntity,
            f"/world/{self._world_name}/remove",
        )
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._root = tk.Tk()
        self._root.title("Spawn Human GUI")
        self._build_ui()
        self._refresh_models()
        self._root.after(200, self._drain_status_queue)

    def _build_ui(self):
        frame = ttk.Frame(self._root, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        label = ttk.Label(frame, text="Models")
        label.pack(anchor=tk.W)

        list_frame = ttk.Frame(frame)
        list_frame.pack(fill=tk.BOTH, expand=True)

        self._listbox = tk.Listbox(list_frame, height=12)
        self._listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self._listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self._listbox.configure(yscrollcommand=scrollbar.set)

        button_frame = ttk.Frame(frame)
        button_frame.pack(fill=tk.X, pady=(8, 0))

        spawn_button = ttk.Button(button_frame, text="Spawn", command=self._on_spawn)
        spawn_button.pack(side=tk.LEFT, padx=(0, 8))

        remove_button = ttk.Button(button_frame, text="Remove", command=self._on_remove)
        remove_button.pack(side=tk.LEFT)

        self._status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(frame, textvariable=self._status_var)
        status_label.pack(anchor=tk.W, pady=(8, 0))

    def _set_status(self, message):
        self._status_var.set(message)

    def _enqueue_status(self, message):
        with self._queue_lock:
            self._status_queue.append(message)

    def _drain_status_queue(self):
        with self._queue_lock:
            queued = list(self._status_queue)
            self._status_queue.clear()
        for message in queued:
            self._set_status(message)
        self._root.after(200, self._drain_status_queue)

    def _list_model_dirs(self):
        if not os.path.isdir(self._models_root):
            return []
        names = []
        for entry in sorted(os.listdir(self._models_root)):
            path = os.path.join(self._models_root, entry)
            if not os.path.isdir(path):
                continue
            if os.path.isfile(os.path.join(path, "model.sdf")):
                names.append(entry)
        return names

    def _refresh_models(self):
        names = self._list_model_dirs()
        if names != self._model_names:
            selection = self._get_selected()
            self._listbox.delete(0, tk.END)
            for name in names:
                self._listbox.insert(tk.END, name)
            self._model_names = names
            if selection in names:
                index = names.index(selection)
                self._listbox.selection_set(index)
        self._root.after(self._refresh_ms, self._refresh_models)

    def _get_selected(self):
        selection = self._listbox.curselection()
        if not selection:
            return None
        return self._listbox.get(selection[0])

    def _wait_for_service(self, client, name):
        if client.service_is_ready():
            return True
        if client.wait_for_service(timeout_sec=1.0):
            return True
        self._logger.warning("Service not available: %s", name)
        self._enqueue_status(f"Service not available: {name}")
        return False

    def _on_spawn(self):
        model_name = self._get_selected()
        if not model_name:
            self._enqueue_status("Select a model to spawn")
            return
        if not self._wait_for_service(self._spawn_client, "create"):
            return
        sdf_path = os.path.join(self._models_root, model_name, "model.sdf")
        if not os.path.isfile(sdf_path):
            self._enqueue_status(f"Missing model.sdf for {model_name}")
            return

        req = SpawnEntity.Request()
        req.entity_factory.name = model_name or self._default_name
        req.entity_factory.sdf_filename = sdf_path
        x, y, z, yaw = self._pose
        req.entity_factory.pose.position.x = float(x)
        req.entity_factory.pose.position.y = float(y)
        req.entity_factory.pose.position.z = float(z)
        qx, qy, qz, qw = _yaw_to_quaternion(float(yaw))
        req.entity_factory.pose.orientation.x = qx
        req.entity_factory.pose.orientation.y = qy
        req.entity_factory.pose.orientation.z = qz
        req.entity_factory.pose.orientation.w = qw

        future = self._spawn_client.call_async(req)
        future.add_done_callback(lambda fut: self._handle_spawn_result(model_name, fut))
        self._enqueue_status(f"Spawn requested: {model_name}")

    def _handle_spawn_result(self, model_name, future):
        try:
            response = future.result()
        except Exception as exc:
            self._logger.exception("Spawn failed: %s", exc)
            self._enqueue_status(f"Spawn failed: {model_name}")
            return
        if response.success:
            self._logger.info("Spawned: %s", model_name)
            self._enqueue_status(f"Spawned: {model_name}")
        else:
            self._logger.warning("Spawn failed for %s", model_name)
            self._enqueue_status(f"Spawn failed: {model_name}")

    def _on_remove(self):
        model_name = self._get_selected()
        if not model_name:
            self._enqueue_status("Select a model to remove")
            return
        if not self._wait_for_service(self._delete_client, "remove"):
            return

        req = DeleteEntity.Request()
        req.entity.name = model_name
        req.entity.type = Entity.MODEL
        future = self._delete_client.call_async(req)
        future.add_done_callback(lambda fut: self._handle_remove_result(model_name, fut))
        self._enqueue_status(f"Remove requested: {model_name}")

    def _handle_remove_result(self, model_name, future):
        try:
            response = future.result()
        except Exception as exc:
            self._logger.exception("Remove failed: %s", exc)
            self._enqueue_status(f"Remove failed: {model_name}")
            return
        if response.success:
            self._logger.info("Removed: %s", model_name)
            self._enqueue_status(f"Removed: {model_name}")
        else:
            self._logger.warning("Remove failed for %s", model_name)
            self._enqueue_status(f"Remove failed: {model_name}")

    def run(self):
        try:
            self._root.mainloop()
        finally:
            self._executor.shutdown()
            self._node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Spawn human models via GUI.")
    parser.add_argument("--models-root", required=True, help="Path to models directory")
    parser.add_argument("--world-name", default="rcjo2026_arena", help="Gazebo world name")
    parser.add_argument("--model-name", default="gz_human", help="Default model name")
    parser.add_argument("--x", type=float, default=-2.0, help="Spawn x position")
    parser.add_argument("--y", type=float, default=1.5, help="Spawn y position")
    parser.add_argument("--z", type=float, default=0.0, help="Spawn z position")
    parser.add_argument("--yaw", type=float, default=0.0, help="Spawn yaw angle")
    parser.add_argument("--refresh-ms", type=int, default=1000, help="List refresh interval (ms)")

    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    app = SpawnHumanGui(args)
    app.run()


if __name__ == "__main__":
    main()
