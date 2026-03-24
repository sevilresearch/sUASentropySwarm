#!/usr/bin/env python3
# entropy_swarm/Strategies/Crazyswarm2Strategy.py

import math
import threading
from typing import Optional, Dict, Tuple, List

from .. import Strategy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration as DurationMsg
from crazyflie_interfaces.srv import Takeoff, Land, GoTo


def _duration_msg(seconds: float) -> DurationMsg:
    sec = int(max(0.0, seconds))
    nanosec = int((max(0.0, seconds) - sec) * 1e9)
    return DurationMsg(sec=sec, nanosec=nanosec)


def _norm_frame(s: str) -> str:
    return (s or "").lstrip("/").strip()


class _TFStore:
    """child -> (parent, (tx,ty,tz), (qx,qy,qz,qw))"""
    def __init__(self):
        self._lock = threading.Lock()
        self._latest: Dict[str, Tuple[str, Tuple[float, float, float], Tuple[float, float, float, float]]] = {}

    def cb(self, msg: TFMessage):
        with self._lock:
            for t in msg.transforms:
                parent = _norm_frame(t.header.frame_id)
                child = _norm_frame(t.child_frame_id)
                tr = t.transform.translation
                rot = t.transform.rotation
                self._latest[child.lower()] = (
                    parent,
                    (float(tr.x), float(tr.y), float(tr.z)),
                    (float(rot.x), float(rot.y), float(rot.z), float(rot.w)),
                )

    def get_for(self, cf_name: str):
        key = (cf_name or "").strip().lower()
        if not key:
            return None
        with self._lock:
            if key in self._latest:
                return self._latest[key]
            candidates = [(child, data) for child, data in self._latest.items() if key in child]
            if not candidates:
                return None
            candidates.sort(key=lambda x: len(x[0]))
            return candidates[0][1]


class Crazyswarm2Strategy(Strategy.Strategy):
    # shared
    _node = None
    _tf_store = None
    _tf_ready = False

    # ---- motion tuning (matches your stable square script style) ----
    STEP_MAX = 0.30       # meters per command (cap)
    STEP_MIN = 0.05
    CRUISE_V = 0.30       # m/s used to compute duration
    DUR_MIN  = 0.30       # seconds
    DUR_MAX  = 2.00

    TAKEOFF_T = 3.0
    MAX_CLIMB_MPS = 1.0   # gentler vertical corrections

    # yaw mode: keep stable (like your working script)
    USE_YAW = False       # if True, we can set yaw to face motion (but stable default is False)

    @classmethod
    def configure_ros(cls, node):
        cls._node = node
        if cls._tf_store is None:
            cls._tf_store = _TFStore()
            node.create_subscription(TFMessage, "/tf", cls._tf_store.cb, 50)
            node.create_subscription(TFMessage, "/tf_static", cls._tf_store.cb, 10)
        cls._tf_ready = True

    def __init__(self, parent=None):
        self.parent = parent
        self.master = None
        self.ros_name: Optional[str] = None

        self._cli_takeoff = None
        self._cli_land = None
        self._cli_goto = None

        self._airborne = False
        self._pending: List = []

        # cached pose estimate (fallback)
        self._est_xw = 0.0
        self._est_yw = 0.0
        self._est_zw = 0.0
        self._est_yaw = 0.0  # radians

    def set_parent(self, parent):
        self.parent = parent
        self.ros_name = self._map_drone_to_cf(parent.ID)

    @staticmethod
    def _map_drone_to_cf(drone_id: str) -> str:
        s = (drone_id or "").strip()
        if s.lower().startswith("drone"):
            num = "".join([c for c in s[5:] if c.isdigit()])
            if num:
                return f"CF{int(num)}"
        return s

    def establish_connection(self, master):
        self.master = master
        return True

    def connect_to_environment(self):
        if not self.__class__._tf_ready or self.__class__._node is None:
            raise RuntimeError("Call Crazyswarm2Strategy.configure_ros(node) before starting CentralControl.")

        node = self.__class__._node
        ns = f"/{self.ros_name}"

        self._cli_takeoff = node.create_client(Takeoff, f"{ns}/takeoff")
        self._cli_land = node.create_client(Land, f"{ns}/land")
        self._cli_goto = node.create_client(GoTo, f"{ns}/go_to")

        for cli, name in [(self._cli_takeoff, "takeoff"), (self._cli_land, "land"), (self._cli_goto, "go_to")]:
            if not cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"Service {ns}/{name} not available.")
        return True

    def close_connection(self):
        if self._cli_land is None:
            return
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = _duration_msg(3.0)
        fut = self._cli_land.call_async(req)
        self._pending.append(fut)
        self._airborne = False

    def get_other_uav_positions(self):
        other = self.master.uavs.copy()
        for i, u in enumerate(other):
            if u.ID == self.parent.ID:
                other.pop(i)
                break
        return other

    def get_self_position(self):
        """Return x,y in TF world; z as negative-up to keep compatibility; theta radians (estimated)."""
        yaw = float(self._est_yaw) if self._est_yaw is not None else 0.0
        tf = self.__class__._tf_store.get_for(self.ros_name) if self.__class__._tf_store else None
        if tf is not None:
            _, (xw, yw, zw), _ = tf
            self._est_xw, self._est_yw, self._est_zw = float(xw), float(yw), float(zw)
            return float(xw), float(yw), -float(zw), yaw
        return float(self.parent.x), float(self.parent.y), float(self.parent.z), yaw

    @staticmethod
    def _sat(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def move_by_heading(self, vx, vy, z_target, duration, angle_in_radians):
        """
        Stable step-based GoTo:
          - Convert (vx,vy) to a bounded (dx,dy) step
          - Choose duration from CRUISE_V with clamps
          - Hold altitude at target_height
          - Default yaw=0 (no spin)
        """
        if self._cli_goto is None or self._cli_takeoff is None:
            return None

        # interpret z_target:
        # if z_target >= 0 -> ROS height already
        # if z_target < 0  -> AirSim negative-up
        zt = float(z_target)
        target_height = zt if zt >= 0.0 else abs(zt)

        # takeoff once
        if (not self._airborne) and target_height > 0.05:
            treq = Takeoff.Request()
            treq.group_mask = 0
            treq.height = float(target_height)
            treq.duration = _duration_msg(float(self.TAKEOFF_T))
            self._airborne = True
            fut = self._cli_takeoff.call_async(treq)
            self._pending.append(fut)
            return fut

        # --- compute bounded step ---
        dt_hint = max(0.05, float(duration))
        dx = float(vx) * dt_hint
        dy = float(vy) * dt_hint

        step = math.hypot(dx, dy)
        if step > 1e-9:
            step_len = self._sat(step, self.STEP_MIN, self.STEP_MAX)
            scale = step_len / step
            dx *= scale
            dy *= scale
        else:
            dx = 0.0
            dy = 0.0
            step_len = 0.0

        # duration from cruise speed
        dur_s = self._sat(step_len / max(self.CRUISE_V, 1e-6), self.DUR_MIN, self.DUR_MAX)

        # --- vertical hold (gentle) ---
        tf = self.__class__._tf_store.get_for(self.ros_name) if self.__class__._tf_store else None
        current_h = float(self._est_zw)
        if tf is not None:
            _, (_, _, zw), _ = tf
            current_h = float(zw)
            self._est_zw = current_h

        err_h = target_height - current_h
        max_dz = float(self.MAX_CLIMB_MPS) * dur_s
        dz = max(-max_dz, min(max_dz, err_h))

        # --- yaw control (stable default: none) ---
        if not self.USE_YAW:
            yaw_deg = 0.0
        else:
            # face direction of motion (degrees)
            if abs(dx) + abs(dy) > 1e-9:
                yaw_deg = float(math.degrees(math.atan2(dy, dx)))
            else:
                yaw_deg = float(math.degrees(angle_in_radians))

        req = GoTo.Request()
        req.group_mask = 0
        req.relative = True
        req.goal = Point(x=float(dx), y=float(dy), z=float(dz))
        req.yaw = float(yaw_deg)  # GoTo expects DEGREES
        req.duration = _duration_msg(float(dur_s))

        # update estimates
        self._est_xw += dx
        self._est_yw += dy
        self._est_zw += dz
        self._est_yaw = float(angle_in_radians)

        fut = self._cli_goto.call_async(req)
        self._pending.append(fut)
        if len(self._pending) > 300:
            self._pending = [f for f in self._pending if not f.done()]
        return fut