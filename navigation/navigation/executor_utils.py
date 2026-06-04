import os

from rclpy.executors import MultiThreadedExecutor


def cpu_thread_count():
    raw_count = os.environ.get("RCLPY_EXECUTOR_THREADS")
    try:
        count = int(raw_count) if raw_count else os.cpu_count()
    except ValueError:
        count = os.cpu_count()
    return max(1, count or 1)


def spin_with_cpu_pool(node):
    executor = MultiThreadedExecutor(num_threads=cpu_thread_count())
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
