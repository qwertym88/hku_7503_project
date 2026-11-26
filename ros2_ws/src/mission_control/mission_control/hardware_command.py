"""Wait for the hardware mission service and report the result of preparation."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


def run(node, operation):
    status = {}

    def receive(msg):
        status.update(json.loads(msg.data))

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/mission/status', receive, qos)
    client = node.create_client(Trigger, f'/mission/{operation}')
    ready_by = time.monotonic() + 45
    while not (status and client.service_is_ready()):
        if time.monotonic() >= ready_by:
            raise RuntimeError('等待真机任务节点超时，请查看任务日志')
        rclpy.spin_once(node, timeout_sec=0.2)
    if status.get('simulation_only') is not False:
        raise RuntimeError('当前 ROS 域未连接真机任务节点')
    if operation != 'cancel' and 'preparation_id' not in status:
        raise RuntimeError('任务节点仍在运行旧版本，请先用同一脚本执行 shutdown')

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    if not future.done():
        raise RuntimeError('任务服务未返回，请查看任务日志确认状态')
    response = future.result()
    if not response.success:
        raise RuntimeError(response.message)
    if operation == 'cancel':
        print(response.message)
        return

    operation_id = int(response.message)
    deadline = time.monotonic() + 35
    last_detail = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if status['preparation_id'] < operation_id:
            continue
        if status['preparation_id'] > operation_id:
            raise RuntimeError('该操作已被后续请求替换，请查看任务日志')
        detail = status['preparation_detail']
        if not status['preparation_pending']:
            if not status['preparation_result']:
                raise RuntimeError(detail)
            print(detail, flush=True)
            return
        if detail != last_detail:
            print(detail, flush=True)
            last_detail = detail
    raise RuntimeError('等待准备结果超时，请查看任务日志')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('start', 'reset', 'cancel'))
    operation = parser.parse_args().operation
    rclpy.init(args=[])
    node = Node('fixed_route_hardware_command')
    try:
        run(node, operation)
    except RuntimeError as exc:
        parser.exit(1, f'{exc}\n')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
