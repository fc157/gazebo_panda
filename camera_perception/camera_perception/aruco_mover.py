#!/usr/bin/env python3
"""
Aruco Marker Mover GUI

Provides a tkinter GUI to dynamically reposition aruco_marker_1 in Gazebo.
Supports smooth interpolation from current pose to target pose with
configurable speed. Continuously sends pose commands during movement.

Usage:
    ros2 run camera_perception aruco_mover.py
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState


class ArucoMoverNode(Node):
    """ROS2 node that provides set_entity_state service client."""

    def __init__(self):
        super().__init__('aruco_mover_node')
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        self.get_logger().info('/gazebo/set_entity_state service is ready.')

        # Track last sent pose (x, y, z, roll, pitch, yaw in radians)
        self.last_pose = (0.8, 0.0, 0.5, 0.0, 0.0, 0.0)

    def move_marker_async(self, x, y, z, roll, pitch, yaw):
        """Send set_entity_state asynchronously. Non-blocking."""
        request = SetEntityState.Request()
        request.state.name = 'aruco_marker_1'
        request.state.reference_frame = 'world'

        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z

        qw, qx, qy, qz = self.euler_to_quaternion(roll, pitch, yaw)
        request.state.pose.orientation.w = qw
        request.state.pose.orientation.x = qx
        request.state.pose.orientation.y = qy
        request.state.pose.orientation.z = qz

        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0

        future = self.cli.call_async(request)
        return future

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles (radians) to quaternion (w, x, y, z)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qw, qx, qy, qz


class ArucoMoverGUI:
    """Tkinter GUI for controlling aruco marker pose with smooth interpolation."""

    SEND_RATE = 60.0  # Hz

    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.running = False
        self._stop_event = threading.Event()
        self._move_thread = None

        self.root.title('Aruco Marker Mover')
        self.root.geometry('480x600')
        self.root.resizable(False, False)

        # ===================== Target Pose Controls =====================
        target_frame = ttk.LabelFrame(root, text='Target Pose', padding=10)
        target_frame.pack(fill='x', padx=10, pady=5)

        self.target_x = tk.DoubleVar(value=self.node.last_pose[0])
        self.target_y = tk.DoubleVar(value=self.node.last_pose[1])
        self.target_z = tk.DoubleVar(value=self.node.last_pose[2])

        self.create_slider(target_frame, 'X', self.target_x, -2.0, 2.0, 0)
        self.create_slider(target_frame, 'Y', self.target_y, -2.0, 2.0, 1)
        self.create_slider(target_frame, 'Z', self.target_z, -2.0, 2.0, 2)

        ttk.Separator(target_frame, orient='horizontal').grid(row=3, column=0, columnspan=3, sticky='ew', pady=8)

        self.target_roll = tk.DoubleVar(value=0.0)
        self.target_pitch = tk.DoubleVar(value=0.0)
        self.target_yaw = tk.DoubleVar(value=0.0)

        self.create_angle_slider(target_frame, 'Roll', self.target_roll, -180, 180, 4)
        self.create_angle_slider(target_frame, 'Pitch', self.target_pitch, -180, 180, 5)
        self.create_angle_slider(target_frame, 'Yaw', self.target_yaw, -180, 180, 6)

        # ===================== Motion Settings =====================
        motion_frame = ttk.LabelFrame(root, text='Motion Settings', padding=10)
        motion_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(motion_frame, text='Duration:', width=8, anchor='e').grid(row=0, column=0, sticky='e', padx=(0, 5), pady=2)
        self.duration_var = tk.DoubleVar(value=2.0)
        duration_scale = ttk.Scale(
            motion_frame, from_=0.1, to=10.0, orient='horizontal',
            variable=self.duration_var,
        )
        duration_scale.grid(row=0, column=1, sticky='ew', padx=5, pady=2)
        self.duration_label = ttk.Label(motion_frame, text='2.0 s', width=7, anchor='w')
        self.duration_label.grid(row=0, column=2, sticky='w', pady=2)
        self.duration_var.trace_add('write', lambda *_: self.duration_label.config(
            text=f'{self.duration_var.get():.1f} s'))
        motion_frame.columnconfigure(1, weight=1)

        ttk.Label(motion_frame, text='Rate:', width=8, anchor='e').grid(row=1, column=0, sticky='e', padx=(0, 5), pady=2)
        ttk.Label(motion_frame, text=f'{self.SEND_RATE:.0f} Hz', width=7, anchor='w').grid(row=1, column=2, sticky='w', pady=2)

        ttk.Label(motion_frame, text='Mode:', width=8, anchor='e').grid(row=2, column=0, sticky='e', padx=(0, 5), pady=2)
        self.move_mode = tk.StringVar(value='once')
        mode_combo = ttk.Combobox(
            motion_frame, textvariable=self.move_mode,
            values=['once', 'loop'], state='readonly', width=10,
        )
        mode_combo.grid(row=2, column=1, sticky='w', padx=5, pady=2)
        ttk.Label(motion_frame, text='once=单次  loop=循环').grid(row=2, column=2, sticky='w', pady=2)

        # ===================== Current Pose Display =====================
        current_frame = ttk.LabelFrame(root, text='Last Sent Pose', padding=10)
        current_frame.pack(fill='x', padx=10, pady=5)

        self.current_x = tk.StringVar(value=f'{self.node.last_pose[0]:.3f}')
        self.current_y = tk.StringVar(value=f'{self.node.last_pose[1]:.3f}')
        self.current_z = tk.StringVar(value=f'{self.node.last_pose[2]:.3f}')
        self.current_r = tk.StringVar(value='0.0°')
        self.current_p = tk.StringVar(value='0.0°')
        self.current_yaw = tk.StringVar(value='0.0°')

        labels = [
            ('X:', self.current_x), ('Y:', self.current_y), ('Z:', self.current_z),
            ('Roll:', self.current_r), ('Pitch:', self.current_p), ('Yaw:', self.current_yaw),
        ]
        for i, (text, var) in enumerate(labels):
            row, col = divmod(i, 3)
            ttk.Label(current_frame, text=text, width=5, anchor='e').grid(
                row=row, column=col * 3, sticky='e', padx=(0, 2), pady=1)
            ttk.Label(current_frame, textvariable=var, width=8, anchor='w').grid(
                row=row, column=col * 3 + 1, sticky='w', pady=1)

        # ===================== Control Buttons =====================
        btn_frame = ttk.Frame(root)
        btn_frame.pack(fill='x', padx=10, pady=10)

        self.start_btn = ttk.Button(
            btn_frame, text='▶  Start', command=self.on_start,
        )
        self.start_btn.pack(side='left', expand=True, fill='x', padx=2)

        self.stop_btn = ttk.Button(
            btn_frame, text='■  Stop', command=self.on_stop, state='disabled',
        )
        self.stop_btn.pack(side='left', expand=True, fill='x', padx=2)

        self.reset_btn = ttk.Button(
            btn_frame, text='Reset Target', command=self.on_reset,
        )
        self.reset_btn.pack(side='right', expand=True, fill='x', padx=2)

        # ===================== Status & Progress =====================
        self.status_var = tk.StringVar(value='Ready')
        self.status_bar = ttk.Label(root, textvariable=self.status_var, relief='sunken', anchor='w')
        self.status_bar.pack(fill='x', padx=10, pady=(0, 10))

        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_bar = ttk.Progressbar(
            root, orient='horizontal', mode='determinate',
            variable=self.progress_var,
        )
        self.progress_bar.pack(fill='x', padx=10, pady=(0, 5))

    # ---------- Slider Helpers ----------

    def create_slider(self, parent, label, variable, from_, to, row, resolution=0.01):
        lbl = ttk.Label(parent, text=f'{label}:', width=6, anchor='e')
        lbl.grid(row=row, column=0, sticky='e', padx=(0, 5), pady=2)
        scale = ttk.Scale(parent, from_=from_, to=to, orient='horizontal', variable=variable)
        scale.grid(row=row, column=1, sticky='ew', padx=5, pady=2)
        val_lbl = ttk.Label(parent, text=f'{variable.get():.2f}', width=6, anchor='w')
        val_lbl.grid(row=row, column=2, sticky='w', pady=2)
        variable.trace_add('write', lambda *_, v=variable, vl=val_lbl: vl.config(text=f'{v.get():.2f}'))
        parent.columnconfigure(1, weight=1)

    def create_angle_slider(self, parent, label, variable, from_, to, row):
        lbl = ttk.Label(parent, text=f'{label}:', width=6, anchor='e')
        lbl.grid(row=row, column=0, sticky='e', padx=(0, 5), pady=2)
        scale = ttk.Scale(parent, from_=from_, to=to, orient='horizontal', variable=variable)
        scale.grid(row=row, column=1, sticky='ew', padx=5, pady=2)
        val_lbl = ttk.Label(parent, text=f'{variable.get():.1f}°', width=7, anchor='w')
        val_lbl.grid(row=row, column=2, sticky='w', pady=2)
        variable.trace_add('write', lambda *_, v=variable, vl=val_lbl: vl.config(text=f'{v.get():.1f}°'))
        parent.columnconfigure(1, weight=1)

    # ---------- Actions ----------

    def _read_target(self):
        """Read target values from sliders. Angles returned in radians."""
        return (
            self.target_x.get(),
            self.target_y.get(),
            self.target_z.get(),
            math.radians(self.target_roll.get()),
            math.radians(self.target_pitch.get()),
            math.radians(self.target_yaw.get()),
        )

    def _update_last_pose_display(self, x, y, z, roll, pitch, yaw):
        """Update the current pose display labels."""
        self.current_x.set(f'{x:.3f}')
        self.current_y.set(f'{y:.3f}')
        self.current_z.set(f'{z:.3f}')
        self.current_r.set(f'{math.degrees(roll):.1f}°')
        self.current_p.set(f'{math.degrees(pitch):.1f}°')
        self.current_yaw.set(f'{math.degrees(yaw):.1f}°')

    def on_start(self):
        """Start smooth interpolation from last pose to target."""
        if self.running:
            return

        target = self._read_target()

        # Use last sent pose as the starting point
        start = self.node.last_pose
        sx, sy, sz, sr, sp, syaw = start

        # If start and target are the same, just send once
        if (abs(sx - target[0]) < 0.001 and abs(sy - target[1]) < 0.001 and
            abs(sz - target[2]) < 0.001 and abs(sr - target[3]) < 0.001 and
            abs(sp - target[4]) < 0.001 and abs(syaw - target[5]) < 0.001):
            self.status_var.set('Target same as current, nothing to move.')
            return

        self.status_var.set(
            f'Start: x={sx:.2f} y={sy:.2f} z={sz:.2f} → '
            f'Target: x={target[0]:.2f} y={target[1]:.2f} z={target[2]:.2f}'
        )

        self.running = True
        self._stop_event.clear()
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')

        duration = self.duration_var.get()
        mode = self.move_mode.get()
        self._move_thread = threading.Thread(
            target=self._smooth_move_loop,
            args=(start, target, duration, mode),
            daemon=True,
        )
        self._move_thread.start()

    def on_stop(self):
        """Stop the current movement."""
        if self.running:
            self._stop_event.set()
            self.running = False
            self.start_btn.config(state='normal')
            self.stop_btn.config(state='disabled')
            self.progress_var.set(0.0)
            self.status_var.set('Movement stopped.')

    def _smooth_move_loop(self, start, target, duration, mode):
        """Interpolate from start to target, sending commands at SEND_RATE."""
        sx, sy, sz, sr, sp, syaw = start
        tx, ty, tz, tr, tp, tyaw = target

        dt = 1.0 / self.SEND_RATE
        total_steps = max(1, int(duration * self.SEND_RATE))

        try:
            while self.running and not self._stop_event.is_set():
                self._interpolate_move(sx, sy, sz, sr, sp, syaw,
                                       tx, ty, tz, tr, tp, tyaw,
                                       total_steps, dt)
                if self._stop_event.is_set():
                    break

                # Update last_pose to final target position
                self.node.last_pose = (tx, ty, tz, tr, tp, tyaw)

                if mode != 'loop':
                    break

                # Swap start ↔ target for return trip
                sx, tx = tx, sx
                sy, ty = ty, sy
                sz, tz = tz, sz
                sr, tr = tr, sr
                sp, tp = tp, sp
                syaw, tyaw = tyaw, syaw
        except Exception as e:
            self.node.get_logger().error(f'Move error: {e}')
        finally:
            self.running = False
            self.root.after(0, self._move_finished)

    def _interpolate_move(self, sx, sy, sz, sr, sp, syaw,
                          tx, ty, tz, tr, tp, tyaw,
                          total_steps, dt):
        """Linearly interpolate from start to target, sending commands."""
        for step in range(total_steps):
            if self._stop_event.is_set():
                return

            t = (step + 1) / total_steps
            t_smooth = t * t * (3.0 - 2.0 * t)  # SmoothStep easing

            x = sx + (tx - sx) * t_smooth
            y = sy + (ty - sy) * t_smooth
            z = sz + (tz - sz) * t_smooth
            roll = sr + (tr - sr) * t_smooth
            pitch = sp + (tp - sp) * t_smooth
            yaw = syaw + (tyaw - syaw) * t_smooth

            # Send command asynchronously (ROS2 spin runs in bg thread)
            self.node.move_marker_async(x, y, z, roll, pitch, yaw)

            # Update the last pose after final step
            if step == total_steps - 1:
                self.node.last_pose = (x, y, z, roll, pitch, yaw)

            # Update GUI periodically (every ~5 steps)
            progress = (step + 1) / total_steps * 100.0
            if step % 5 == 0 or step == total_steps - 1:
                self.root.after(0, lambda p=progress, px=x, py=y, pz=z, pr=roll, pp=pitch, pyaw=yaw: (
                    self.progress_var.set(p),
                    self.status_var.set(
                        f'Moving... {p:.0f}%  |  '
                        f'x={px:.3f} y={py:.3f} z={pz:.3f} '
                        f'r={math.degrees(pr):.1f} p={math.degrees(pp):.1f} y={math.degrees(pyaw):.1f}'
                    ),
                    self._update_last_pose_display(px, py, pz, pr, pp, pyaw),
                ))

            time.sleep(dt)

    def _move_finished(self):
        """Called when movement completes (from main thread via after)."""
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        self.progress_var.set(0.0)
        if not self._stop_event.is_set():
            self.status_var.set('Movement complete.')

    def on_reset(self):
        """Reset target to initial values."""
        self.target_x.set(0.8)
        self.target_y.set(0.0)
        self.target_z.set(0.5)
        self.target_roll.set(0.0)
        self.target_pitch.set(0.0)
        self.target_yaw.set(0.0)
        self.status_var.set('Target reset to defaults.')

    def on_close(self):
        """Clean up on window close."""
        self._stop_event.set()
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMoverNode()

    # Spin ROS2 in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Start tkinter GUI (main thread)
    root = tk.Tk()
    gui = ArucoMoverGUI(root, node)
    root.protocol('WM_DELETE_WINDOW', lambda: (gui.on_close(), root.destroy()))
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()