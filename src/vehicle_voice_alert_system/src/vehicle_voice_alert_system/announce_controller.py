# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

import os
from os import path
from dataclasses import dataclass
from simpleaudio import WaveObject
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.time import Time
from pulsectl import Pulse

from autoware_adapi_v1_msgs.msg import (
    RouteState,
    MrmState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
)
from std_msgs.msg import Float32
from tier4_hmi_msgs.srv import SetVolume
from tier4_external_api_msgs.msg import ResponseStatus


# The higher the value, the higher the priority
PRIORITY_DICT = {
    "emergency": 4,
    "departure": 4,
    "stop": 4,
    "restart_engage": 4,
    "going_to_arrive": 4,
    "obstacle_stop": 3,
    "in_emergency": 3,
    "temporary_stop": 2,
    "turning_left": 1,
    "turning_right": 1,
}

# 参照元: https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/planning/msg/PlanningBehavior.msg
STOP_ANNOUNCE_BEHAVIORS = [
    "avoidance",
    "crosswalk",
    "goal-planner",
    "intersection",
    "lane-change",
    "merge",
    "no-drivable-lane",
    "no-stopping-area",
    "rear-check",
    "route-obstacle",
    "sidewalk",
    "start-planner",
    "stop-sign",
    "surrounding-obstacle",
    "traffic-signal",
    "user-defined-attention-area",
    "virtual-traffic-light",
]

CURRENT_VOLUME_PATH = "/opt/autoware/volume.txt"

@dataclass
class TimeoutClass:
    stop_reason: Time
    turn_signal: Time
    in_emergency: Time
    driving_bgm: Time
    accept_start: Time


class AnnounceControllerProperty:
    def __init__(
        self,
        node,
        ros_service_interface,
        parameter_interface,
        autoware_interface,
    ):
        super(AnnounceControllerProperty, self).__init__()
        self._node = node
        self._ros_service_interface = ros_service_interface
        self._parameter = parameter_interface.parameter
        self._mute_parameter = parameter_interface.mute_parameter
        self._autoware = autoware_interface
        self._timeout = TimeoutClass(
            node.get_clock().now(),
            node.get_clock().now(),
            node.get_clock().now(),
            node.get_clock().now(),
            node.get_clock().now(),
        )
        self._engage_trigger_time = self._node.get_clock().now()
        self._in_emergency_state = False
        self._prev_motion_state = MotionState.UNKNOWN
        self._current_announce = ""
        self._pending_announce_list = []
        self._wav_object = None
        self._music_object = None
        self._in_stop_status = False
        self._in_driving_state = False
        self._announce_arriving = False
        self._skip_announce = False
        self._stop_announce_executed = False
        self._announce_engage = False
        self._in_slow_stop_state = False

        self._package_path = (
            get_package_share_directory("vehicle_voice_alert_system") + "/resource/sound"
        )

        self._running_bgm_file = self.get_filepath("running_music")
        self._node.create_timer(1.0, self.check_playing_callback)
        self._node.create_timer(0.5, self.turn_signal_callback)
        self._node.create_timer(0.5, self.emergency_checker_callback)
        self._node.create_timer(0.5, self.stop_reason_checker_callback)
        self._node.create_timer(0.1, self.announce_engage_when_starting)

        self._pulse = Pulse()
        if os.path.isfile(CURRENT_VOLUME_PATH):
            with open(CURRENT_VOLUME_PATH, "r") as f:
                self._sink = self._pulse.get_sink_by_name(
                    self._pulse.server_info().default_sink_name
                )
                self._pulse.volume_set_all_chans(self._sink, float(f.readline()))

        self._get_volume_pub = self._node.create_publisher(Float32, "~/get/volume", 1)
        self._node.create_timer(1.0, self.publish_volume_callback)
        self._node.create_service(SetVolume, "~/set/volume", self.set_volume)

    def set_timeout(self, timeout_attr):
        setattr(self._timeout, timeout_attr, self._node.get_clock().now())

    def reset_all_timeout(self):
        for attr in self._timeout.__dict__.keys():
            trigger_time = getattr(self._timeout, attr)
            duration = getattr(self._mute_parameter, attr)
            setattr(
                self._timeout,
                attr,
                self._node.get_clock().now() - Duration(seconds=duration),
            )

    def in_interval(self, timeout_attr):
        trigger_time = getattr(self._timeout, timeout_attr)
        duration = getattr(self._mute_parameter, timeout_attr)
        return self._node.get_clock().now() - trigger_time < Duration(seconds=duration)

    def check_in_autonomous(self):
        return self._autoware.information.operation_mode == OperationModeState.AUTONOMOUS

    def get_filepath(self, filename):
        primary_voice_folder_path = (
            self._parameter.primary_voice_folder_path + "/" + filename + ".wav"
        )
        if path.exists(primary_voice_folder_path):
            return primary_voice_folder_path
        elif not self._parameter.skip_default_voice:
            return self._package_path + "/" + filename + ".wav"
        else:
            return ""

    def process_running_music(self):
        try:
            if not self._running_bgm_file:
                return

            if self.in_interval("driving_bgm"):
                return

            if (
                self._parameter.mute_overlap_bgm
                and self._wav_object
                and self._wav_object.is_playing()
            ):
                self.set_timeout("driving_bgm")
                return

            if (
                self.check_in_autonomous()
                and not self._in_emergency_state
                and self._autoware.information.autoware_control
            ):
                if not self._announce_engage:
                    self.send_announce("departure")
                    self._announce_engage = True

                if not self._music_object or not self._music_object.is_playing():
                    sound = WaveObject.from_wave_file(self._running_bgm_file)
                    self._music_object = sound.play()

                if (
                    self._autoware.information.goal_distance
                    < self._parameter.announce_arriving_distance
                    and not self._announce_arriving
                ):
                    # announce if the goal is with the distance
                    self.send_announce("going_to_arrive")
                    self._announce_arriving = True
            elif (
                self._parameter.manual_driving_bgm
                and not self._autoware.information.autoware_control
                and not self.in_range(
                    self._autoware.information.velocity,
                    self._parameter.driving_velocity_threshold,
                )
            ):
                if not self._music_object or not self._music_object.is_playing():
                    sound = WaveObject.from_wave_file(self._running_bgm_file)
                    self._music_object = sound.play()
            else:
                if self._music_object and self._music_object.is_playing():
                    self._music_object.stop()

            if (
                self._autoware.information.route_state == RouteState.ARRIVED
                and self._autoware.information.autoware_control
                and self._in_driving_state
            ):
                # Skip announce if is in manual driving
                self.send_announce("stop")
                self._announce_arriving = False

            if self._autoware.information.route_state == RouteState.ARRIVED:
                self._skip_announce = False
                self._announce_engage = False

            self._in_driving_state = self.check_in_autonomous()
            self.set_timeout("driving_bgm")
        except Exception as e:
            self._node.get_logger().error(
                "not able to check the pending playing list: " + str(e),
                throttle_duration_sec=10,
            )

    def in_range(self, input_value, range_value):
        return -range_value <= input_value <= range_value

    def announce_engage_when_starting(self):
        try:
            if (
                self._autoware.information.localization_init_state
                == LocalizationInitializationState.UNINITIALIZED
            ):
                self._prev_motion_state = MotionState.UNKNOWN
                return

            if (
                self._autoware.information.motion_state
                in [MotionState.STARTING, MotionState.MOVING]
                and self._prev_motion_state == MotionState.STOPPED
            ):
                self._stop_announce_executed = False
                if not self._skip_announce:
                    self._skip_announce = True
                elif self._node.get_clock().now() - self._engage_trigger_time > Duration(
                    seconds=self._mute_parameter.accept_start
                ):
                    self.send_announce("departure")
                    self._engage_trigger_time = self._node.get_clock().now()

                self.reset_all_timeout()
                if self._autoware.information.motion_state == MotionState.STARTING:
                    self._service_interface.accept_start()

            # Check to see if it has not stopped waiting for start acceptance
            if self._autoware.information.motion_state != MotionState.STARTING:
                self.set_timeout("accept_start")

            # Send again when stopped in starting state for a certain period of time
            if (
                self._autoware.information.motion_state == MotionState.STARTING
                and self.in_interval("accept_start")
            ):
                self._service_interface.accept_start()

            self._prev_motion_state = self._autoware.information.motion_state
        except Exception as e:
            self._node.get_logger().error("not able to play the announce, ERROR: {}".format(str(e)))

    def check_playing_callback(self):
        try:
            self.process_running_music()
            if not self._wav_object:
                self._current_announce = ""
                return

            if not self._wav_object.is_playing():
                self._current_announce = ""
        except Exception as e:
            self._node.get_logger().error("not able to check the current playing: " + str(e))

    def play_sound(self, message):
        if (
            self._parameter.mute_overlap_bgm
            and self._music_object
            and self._music_object.is_playing()
        ):
            self._music_object.stop()

        filepath = self.get_filepath(message)
        if filepath:
            sound = WaveObject.from_wave_file(filepath)
            self._wav_object = sound.play()
        else:
            self._node.get_logger().info(
                "Didn't found the voice in the primary voice folder, and skip default voice is enabled"
            )

    def send_announce(self, message):
        if not self._autoware.information.autoware_control:
            self._node.get_logger().info("The vehicle is not control by autoware, skip announce")
            return

        priority = PRIORITY_DICT.get(message, 0)
        previous_priority = PRIORITY_DICT.get(self._current_announce, 0)

        if priority > previous_priority:
            if self._wav_object:
                self._wav_object.stop()
            self.play_sound(message)
        self._current_announce = message

    def emergency_checker_callback(self):
        if self._autoware.information.operation_mode == OperationModeState.STOP:
            in_emergency = False
        else:
            in_emergency = self._autoware.information.mrm_behavior == MrmState.EMERGENCY_STOP

        in_slow_stop = (
            self._autoware.information.mrm_behavior == MrmState.COMFORTABLE_STOP
            and self._autoware.information.motion_state == MotionState.STOPPED
        )

        if in_emergency and not self._in_emergency_state:
            self.send_announce("emergency")
        elif in_emergency and self._in_emergency_state:
            if not self.in_interval("in_emergency"):
                self.send_announce("in_emergency")
                self.set_timeout("in_emergency")
        elif in_slow_stop and self._in_slow_stop_state:
            if not self.in_interval("in_emergency"):
                self.send_announce("in_emergency")
                self.set_timeout("in_emergency")

        self._in_emergency_state = in_emergency
        self._in_slow_stop_state = in_slow_stop

    def turn_signal_callback(self):
        if self.in_interval("turn_signal"):
            return
        elif self._in_emergency_state or self._in_stop_status:
            return

        if self._autoware.information.turn_signal == 1:
            self.send_announce("turning_left")
        if self._autoware.information.turn_signal == 2:
            self.send_announce("turning_right")

        self.set_timeout("turn_signal")

    # 停止する予定を取得
    def stop_reason_checker_callback(self):
        if not self.check_in_autonomous():
            self._node.get_logger().warning(
                "The vehicle is not in driving state, do not announce",
                throttle_duration_sec=10,
            )
            return

        # skip when emergency stop
        if self._in_emergency_state:
            return

        if self._stop_announce_executed == True:
            return

        execute_stop_announce = False
        for velocity_factor in self._autoware.information.velocity_factors:
            if velocity_factor.behavior in STOP_ANNOUNCE_BEHAVIORS:
                execute_stop_announce = True
                break

        # 音声の通知
        if execute_stop_announce == True and self._autoware.information.motion_state == MotionState.STOPPED:
            if self.in_interval("stop_reason"):
                return

            self.announce_stop_reason("temporary_stop")
            self._stop_announce_executed = True
        else:
            self._in_stop_status = False

    def announce_stop_reason(self, file):
        self._in_stop_status = True
        self.send_announce(file)
        self.set_timeout("stop_reason")

    def publish_volume_callback(self):
        self._sink = self._pulse.get_sink_by_name(self._pulse.server_info().default_sink_name)
        self._get_volume_pub.publish(Float32(data=self._sink.volume.value_flat))

    def set_volume(self, request, response):
        try:
            self._sink = self._pulse.get_sink_by_name(self._pulse.server_info().default_sink_name)
            self._pulse.volume_set_all_chans(self._sink, request.volume)
            with open(CURRENT_VOLUME_PATH, "w") as f:
                f.write(f"{self._sink.volume.value_flat}\n")
            response.status.code = ResponseStatus.SUCCESS
        except Exception:
            response.status.code = ResponseStatus.ERROR
        return response
