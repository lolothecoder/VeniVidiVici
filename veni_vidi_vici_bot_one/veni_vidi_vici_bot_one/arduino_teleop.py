#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import signal

import serial
import sys
import termios
import tty
import select
import time


class ArduinoTeleop(Node):
    def __init__(self):
        super().__init__('arduino_teleop')
        # PARAMETERS
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('forward_speed', 120)
        self.declare_parameter('backward_speed', -120)
        self.declare_parameter('turn_speed', 80)
        self.declare_parameter('stop_speed', 0)
        self.declare_parameter('boot_delay', 2.0)
        self.declare_parameter('idle_timeout', 0.1)
        self.declare_parameter('door_keepalive', 1.0)
        self.declare_parameter('grabber_keepalive', 1.0)
        self.declare_parameter('ramp_keepalive', 1.0)
        # load values
        port       = self.get_parameter('port').value
        baud       = self.get_parameter('baud').value
        self.FW    = self.get_parameter('forward_speed').value
        self.BW    = self.get_parameter('backward_speed').value
        self.TS    = self.get_parameter('turn_speed').value
        self.ST    = self.get_parameter('stop_speed').value
        self.boot  = self.get_parameter('boot_delay').value
        self.idle_timeout = self.get_parameter('idle_timeout').value
        self.door_int     = self.get_parameter('door_keepalive').value
        self.grab_int     = self.get_parameter('grabber_keepalive').value
        self.ramp_int     = self.get_parameter('ramp_keepalive').value
        # SERIAL
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"❌ Could not open {port} @ {baud}: {e}")
            rclpy.shutdown()
            return
        self.get_logger().info(f"✅ Opened {port} @ {baud}. Waiting {self.boot}s for Arduino…")
        time.sleep(self.boot)
        while self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)
        # state
        self.last_drive = (None, None)
        self.door_open  = False; self.last_door = time.time()
        self.grab_on    = False; self.last_grab = time.time()
        self.ramp_ext   = False; self.last_ramp = time.time()
        # instructions
        self.get_logger().info(
            "Teleop started. Use keys (no Enter required):\n"
            "  W=forward S=backward A=left D=right SPACE=stop\n"
            "  O=open P=close door K/;=grabber on L=grabber off M=extend ,=retract ramp Q=quit"
        )
        # timer
        self.create_timer(self.idle_timeout, self.timer_cb)

    def _get_key(self, timeout):
        fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            r, _, _ = select.select([fd], [], [], timeout)
            if r: return sys.stdin.read(1)
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def send(self, cmd: str):
        self.ser.write(cmd.encode('ascii'))
        # defer flush until queue is low, or every N commands:
        self.cmd_count = getattr(self, 'cmd_count', 0) + 1
        if self.cmd_count % 5 == 0:            
            self.ser.flush()
    def send_drive(self, l, r):
        self.send(f"o {l*1.038} {r}\r")

    def timer_cb(self):
        now = time.time(); ch = self._get_key(self.idle_timeout).lower()
        # commands
        if ch == 'w': s=(self.FW,self.FW); self.send_drive(*s); self.last_drive=s
        elif ch == 's': s=(self.BW,self.BW); self.send_drive(*s); self.last_drive=s
        elif ch == 'a': s=(-self.TS,self.TS); self.send_drive(*s); self.last_drive=s
        elif ch == 'd': s=(self.TS,-self.TS); self.send_drive(*s); self.last_drive=s
        elif ch == ' ': s=(self.ST,self.ST); self.send_drive(*s); self.last_drive=s
        elif ch == 'o': self.send("h 0 1\r"); self.door_open=True; self.last_door=now
        elif ch == 'p': self.send("h 0 0\r"); self.door_open=False
        elif ch in ('k',';'): self.send("g 100\r"); self.grab_on=True; self.last_grab=now
        elif ch == 'l': self.send("g 0\r"); self.grab_on=False
        elif ch == 'm': self.send("h 1 1\r"); self.ramp_ext=True; self.last_ramp=now
        elif ch == ',': self.send("h 1 0\r"); self.ramp_ext=False
        elif ch == 'q': rclpy.shutdown(); return
        # idle
        elif ch == '':
            if self.last_drive != (self.ST,self.ST): self.send_drive(self.ST,self.ST); self.last_drive=(self.ST,self.ST)
        # keepalive
        if self.door_open  and now - self.last_door >= self.door_int:  self.send("h 0 1\r"); self.last_door=now
        if self.grab_on    and now - self.last_grab >= self.grab_int:  self.send("g 100\r"); self.last_grab=now
        if self.ramp_ext   and now - self.last_ramp >= self.ramp_int:  self.send("h 1 1\r"); self.last_ramp=now

    def destroy_node(self):
        # cleanup
        try:
            self.send_drive(self.ST,self.ST)
            self.send("h 0 0\r"); self.send("g 0\r"); self.send("h 1 0\r")
        except: pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoTeleop()
    # handle SIGINT
    signal.signal(signal.SIGINT, lambda s,f: rclpy.shutdown())
    rclpy.spin(node)
    node.destroy_node()
    # no second shutdown

if __name__ == '__main__': main()
