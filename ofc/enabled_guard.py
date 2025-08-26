#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent, Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters

class EnabledGuard(Node):
    """
    targets 중 한 노드의 'enabled'가 True로 바뀌면, 나머지 노드의 enabled를 자동으로 False로 내린다.
    시작 시 둘 다 True면 prefer만 남기고 모두 False로 맞춘다.
    """
    def __init__(self):
        super().__init__('enabled_guard')

        # 제어 대상 노드와 선호 노드(동시에 True일 때 유지할 노드)
        self.targets = list(self.declare_parameter(
            'targets', ['tln_inference', 'ftg_controller']
        ).get_parameter_value().string_array_value)
        self.prefer = self.declare_parameter(
            'prefer', 'tln_inference'
        ).get_parameter_value().string_value

        self.create_subscription(ParameterEvent, '/parameter_events', self._on_event, 10)
        self.get_logger().info(f'[enabled_guard] targets={self.targets}, prefer={self.prefer}')

        # 초기 상태 정합(둘 이상 True면 prefer만 True로 유지)
        self._did_check = False
        self.create_timer(1.0, self._startup_check_once)

    def _startup_check_once(self):
        if self._did_check:
            return
        self._did_check = True
        enabled = [n for n in self.targets if self._get_enabled(n)]
        if len(enabled) <= 1:
            return
        keep = self.prefer if self.prefer in enabled else enabled[0]
        for n in enabled:
            if n != keep:
                self._set_enabled(n, False)
        self.get_logger().info(f'[enabled_guard] multiple True at startup: {enabled} -> keep {keep}')

    def _on_event(self, evt: ParameterEvent):
        node_base = evt.node.split('/')[-1] if '/' in evt.node else evt.node
        if node_base not in self.targets:
            return
        for p in list(evt.new_parameters) + list(evt.changed_parameters):
            if p.name != 'enabled':
                continue
            if p.value.type == ParameterType.PARAMETER_BOOL and p.value.bool_value:
                # node_base가 True로 올라갔으니 나머지는 False로 내림
                for other in self.targets:
                    if other != node_base:
                        self._set_enabled(other, False)
                self.get_logger().info(f'[enabled_guard] {node_base}=True -> others OFF')
                break

    def _set_enabled(self, node_name: str, val: bool):
        cli = self.create_client(SetParameters, f'/{node_name}/set_parameters')
        if not cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'set_parameters not available: {node_name}')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name='enabled',
                                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=val))]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    def _get_enabled(self, node_name: str):
        cli = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        if not cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'get_parameters not available: {node_name}')
            return False
        req = GetParameters.Request(); req.names = ['enabled']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        res = fut.result()
        if not res or not res.values: return False
        v = res.values[0]
        return v.type == ParameterType.PARAMETER_BOOL and bool(v.bool_value)

def main(args=None):
    rclpy.init(args=args)
    node = EnabledGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
