#!/usr/bin/env python

import functools
import os
import threading
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class StaticFileHandler(SimpleHTTPRequestHandler):
    """HTTP handler with quiet logging and no-cache headers for fast iteration."""

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, format, *args):
        return


class CartVizServer(Node):
    def __init__(self):
        super().__init__("cart_viz_server")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("auto_open_browser", False)
        self.declare_parameter("rosbridge_host", "127.0.0.1")
        self.declare_parameter("rosbridge_port", 9090)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.auto_open_browser = self.parse_bool(
            self.get_parameter("auto_open_browser").value
        )
        self.rosbridge_host = str(self.get_parameter("rosbridge_host").value)
        self.rosbridge_port = int(self.get_parameter("rosbridge_port").value)

        self.server = None
        self.server_thread = None

        share_dir = get_package_share_directory("navigation")
        web_root = os.path.join(share_dir, "web")
        handler = functools.partial(StaticFileHandler, directory=web_root)
        self.server = ThreadingHTTPServer((self.host, self.port), handler)
        self.server_thread = threading.Thread(
            target=self.server.serve_forever, name="cart-viz-http", daemon=True
        )
        self.server_thread.start()

        self.browser_url = (
            f"http://{self.host}:{self.port}/"
            f"?rosbridgeHost={self.rosbridge_host}&rosbridgePort={self.rosbridge_port}"
        )
        self.get_logger().info(f"Cart browser visualization available at {self.browser_url}")

        if self.auto_open_browser:
            self.open_timer = self.create_timer(1.0, self.open_browser_once)
        else:
            self.open_timer = None

    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {"1", "true", "yes", "on"}

    def open_browser_once(self):
        if self.open_timer is not None:
            self.destroy_timer(self.open_timer)
            self.open_timer = None

        self.get_logger().info("Opening cart browser visualization")
        webbrowser.open_new_tab(self.browser_url)

    def destroy_node(self):
        if self.open_timer is not None:
            self.destroy_timer(self.open_timer)
            self.open_timer = None

        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None

        if self.server_thread is not None:
            self.server_thread.join(timeout=2.0)
            self.server_thread = None

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CartVizServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
