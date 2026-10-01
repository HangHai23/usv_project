"""Bounded nonblocking UDP reception; only valid packets are published."""
import socket
import time
import json
import math
import ipaddress

import rclpy
from rclpy.node import Node
from usv_interfaces.msg import GlobalPosition
from std_msgs.msg import Bool, String
from .protocol import ReceiverState


class UdpPositionNode(Node):
    def __init__(self):
        super().__init__('udp_position_receiver')
        for key, value in [('bind_address', '0.0.0.0'), ('port', 5700),
                           ('allowed_sender_ip', ''), ('frame_id', 'map'),
                           ('output_topic', '/usv/global_position'), ('boat_id', 0),
                           ('publish_position', False), ('clock_synchronized', False),
                           ('processed_to_measurement_ms', 0.0), ('maximum_network_delay_ms', 100.0),
                           ('tag_to_body_heading_deg', 0.0), ('tag_to_body_xy_m', [0.0,0.0])]:
            self.declare_parameter(key, value)
        port = self.get_parameter('port').value
        if not 1 <= port <= 65535:
            raise ValueError('port must be in 1..65535')
        self._allowed = str(self.get_parameter('allowed_sender_ip').value)
        self._frame = str(self.get_parameter('frame_id').value)
        if self._allowed:
            ipaddress.IPv4Address(self._allowed)
        self._boat_id = self.get_parameter('boat_id').value
        self._publish_position = self.get_parameter('publish_position').value
        self._synced = self.get_parameter('clock_synchronized').value
        self._delay = self.get_parameter('processed_to_measurement_ms').value
        self._network = self.get_parameter('maximum_network_delay_ms').value
        self._heading = self.get_parameter('tag_to_body_heading_deg').value
        self._offset = self.get_parameter('tag_to_body_xy_m').value
        if self._publish_position and not self._synced:
            raise ValueError('Position fusion requires verified clock_synchronized=true')
        if len(self._offset)!=2 or not all(math.isfinite(v) for v in [*self._offset,self._delay,self._network,self._heading]) or self._delay<0 or self._network<=0 or self._boat_id<0:
            raise ValueError('Invalid calibration/timing parameters')
        self._state = ReceiverState()
        self._last_frame = None
        self._received = None
        self._sender = ''
        self._valid_pub = self.create_publisher(Bool, '/usv/global_position_valid', 10)
        self._snapshot_pub = self.create_publisher(String, '/vision/global_observation', 10)
        self._diagnostic_pub = self.create_publisher(String, '/vision/udp_diagnostics', 200)
        self._received_monotonic = None
        self._publisher = self.create_publisher(
            GlobalPosition, str(self.get_parameter('output_topic').value), 10)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind((str(self.get_parameter('bind_address').value), port))
            self._socket.setblocking(False)
        except Exception:
            self._socket.close()
            raise
        self._last_warning = -float('inf')
        self.create_timer(0.005, self._poll)
        self.get_logger().info(f'UDP position receiver listening on port {port}')
        if not self._allowed:
            self.get_logger().warning('Set allowed_sender_ip to the Mac IPv4; empty rejects all packets')

    def _poll(self):
        # Limit work each tick so bursts cannot monopolize the ROS executor.
        for _ in range(100):
            try:
                data, sender = self._socket.recvfrom(65535)
            except BlockingIOError:
                break
            if sender[0] != self._allowed:
                continue
            arrival = time.monotonic()
            self._diagnostic_pub.publish(String(data=json.dumps(dict(
                received_monotonic=arrival, received_unix_ns=time.time_ns(),
                sender=list(sender), size_bytes=len(data), stage='socket_receive',
                payload_hex=data.hex(),
                calibration=dict(boat_id=self._boat_id, tag_to_body_heading_deg=self._heading,
                                 tag_to_body_xy_m=list(self._offset), clock_synchronized=self._synced)
            ))))
            if len(data)>1400:
                continue
            received = self.get_clock().now().to_msg()
            try:
                if self._synced:
                    packet = json.loads(data)
                    delay = time.time()*1000-packet['sent_unix_ms']
                    if not math.isfinite(delay) or delay < -20 or delay > self._network:
                        self._state.latest = None
                        continue
                if not self._state.accept(data,time.monotonic()):
                    continue
            except (ValueError, KeyError, TypeError, UnicodeError, OverflowError, RecursionError) as error:
                now = time.monotonic()
                if now - self._last_warning > 5.0:
                    self.get_logger().warning(f'Rejected UDP packet from {sender}: {error}')
                    self._last_warning = now
                continue
            self._received = received
            self._received_monotonic = arrival
            self._sender = f'{sender[0]}:{sender[1]}'
        observation = self._state.current(time.monotonic())
        boat = next((b for b in observation['boats'] if b['id']==self._boat_id), None) if observation else None
        valid = boat is not None and boat.get('xy_m') is not None and boat.get('tag_heading_deg') is not None
        self._valid_pub.publish(Bool(data=valid))
        report = dict(valid=valid, reset_required=self._state.reset_required,
                      session=self._state.session, coord=self._state.coord,
                      seq=self._state.seq, observation=observation,
                      received_monotonic=self._received_monotonic)
        if valid:
            heading = math.radians(boat['tag_heading_deg']+self._heading)
            dx,dy = self._offset
            x = boat['xy_m'][0]+math.cos(heading)*dx-math.sin(heading)*dy
            y = boat['xy_m'][1]+math.sin(heading)*dx+math.cos(heading)*dy
            report['own_boat'] = dict(id=self._boat_id, xy_m=[x,y],
                heading_rad=math.atan2(math.sin(heading),math.cos(heading)), possession=boat['possession'])
            key = (self._state.session,self._state.coord,observation['frame'])
            if self._publish_position and key != self._last_frame:
                sec,nanosec = divmod(round((observation['processed_unix_ms']-self._delay)*1e6),1000000000)
                if 0<=sec<=2147483647:
                    message = GlobalPosition()
                    message.header.stamp.sec = sec
                    message.header.stamp.nanosec = nanosec
                    message.header.frame_id = self._frame
                    message.received_stamp = self._received
                    message.x,message.y,message.z = x,y,0.0
                    message.sender = self._sender
                    self._publisher.publish(message)
                    self._last_frame = key
        self._snapshot_pub.publish(String(data=json.dumps(report,allow_nan=False)))

    def destroy_node(self):
        self._socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UdpPositionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
