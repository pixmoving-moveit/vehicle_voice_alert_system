# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.duration import Duration
from dataclasses import dataclass
from autoware_adapi_v1_msgs.msg import (
    RouteState,
    MrmState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
    VelocityFactorArray,
)
from tier4_debug_msgs.msg import Float64Stamped

from pix_robobus_driver_msgs.msg import (
    VcuReport1,         # 车辆信息反馈
    BcmMessage4,        # 安全带、座椅检测反馈
    VehicleDoorReport,  # 开关门反馈

)

from ros2_electron_bridge_msgs.msg import (
    V2dCurrentStationInfo, # 车辆当前站点、下一个站点、预计到达时间（/min）
)

from autoware_auto_system_msgs.msg import AutowareState
from autoware_auto_vehicle_msgs.msg import (
    VelocityReport, # 速度信息反馈
    TurnIndicatorsReport, # 转向灯反馈
)

@dataclass
class AutowareInformation:
    stop_reasons: list
    velocity_factors: list
    autoware_control: bool = False
    operation_mode: int = 0
    mrm_behavior: int = 0
    route_state: int = 0
    turn_signal: int = 0
    velocity: float = 0.0
    motion_state: int = 0
    localization_init_state: int = 0
    
    #------- pix add -------#
    vehicle_door: bool = None       # 车门状态
    alarm_triggered: bool = False   # 报警触发
    alert_message: str = ""
    alarm_flag: bool = False
    autoware_state: int = False
    _current_station: str = ""  # 当前站
    _next_station: str = ""     # 下一站
    _estimate_time: float = 0.0 # 预计到站时间    
    _target_distance: int = 1000   # 目标距离
    # ------- pixmoving end -------#


class AutowareInterface:
    def __init__(self, node):
        self._node = node
        self.information = AutowareInformation([], [])

        sub_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.SYSTEM_DEFAULT,
        )
        api_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        node.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self.sub_operation_mode_callback,
            api_qos,
        )
        node.create_subscription(
            RouteState,
            "/api/routing/state",
            self.sub_routing_state_callback,
            api_qos,
        )
        node.create_subscription(
            MrmState,
            "/api/fail_safe/mrm_state",
            self.sub_mrm_callback,
            sub_qos,
        )
        node.create_subscription(
            VelocityFactorArray,
            "/api/planning/velocity_factors",
            self.sub_velocity_factor_callback,
            sub_qos,
        )
        node.create_subscription(
            MotionState, "/api/motion/state", self.sub_motion_state_callback, api_qos
        )
        node.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            self.sub_localization_initialization_state_callback,
            api_qos,
        )
        
        #---------- pixmoving add --------#
        node.create_subscription(
            AutowareState,
            "/autoware/state",
            self.sub_autoware_state_callback, # 订阅autoware状态
            sub_qos,
        )
        
        node.create_subscription(
            TurnIndicatorsReport,
            "/vehicle/status/turn_indicators_status",
            self.sub_vehicle_turn_indicators_callback, # 订阅转向灯状态
            sub_qos,
        )
        
        node.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.sub_vehicle_velocity_callback, # 订阅车辆速度
            sub_qos,
        )
        
        node.create_subscription(
            BcmMessage4,
            "/pix_robobus/bcm_message4",
            self.sub_vehicle_seat_belt_callback, # 安全带、座椅
            sub_qos,
        )
        
        node.create_subscription(
            VehicleDoorReport,
            "/pix_robobus/vehicle_door_report",
            self.sub_door_open_and_close_callback,      # 订阅开关门反馈
            sub_qos,
        )
        
        node.create_subscription(
            V2dCurrentStationInfo,
            "/app/display/all_station_info",
            self.sub_current_station_callback,    # 车辆站点信息回调函数
            sub_qos,
        )
        
        self._autoware_connection_time = self._node.get_clock().now()
        self._node.create_timer(2, self.reset_timer)

    def reset_timer(self):
        if self._node.get_clock().now() - self._autoware_connection_time > Duration(seconds=10):
            # self.information = AutowareInformation([], [])
            # self._node.get_logger().error("Autoware disconnected", throttle_duration_sec=10)
            pass

    def sub_operation_mode_callback(self, msg):
        try:
            self.information.autoware_control = msg.is_autoware_control_enabled
            self.information.operation_mode = msg.mode
        except Exception as e:
            self._node.get_logger().error("Unable to get the operation mode, ERROR: " + str(e))

    def sub_routing_state_callback(self, msg):
        try:
            self.information.route_state = msg.state
        except Exception as e:
            self._node.get_logger().error("Unable to get the routing state, ERROR: " + str(e))

    def sub_mrm_callback(self, msg):
        try:
            self.information.mrm_behavior = msg.behavior
        except Exception as e:
            self._node.get_logger().error("Unable to get the mrm behavior, ERROR: " + str(e))

    def sub_vehicle_turn_indicators_callback(self, msg):
        try:
            self.information.turn_signal = msg.report    
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))
    
    def sub_vehicle_velocity_callback(self, msg):
        try:
            self.information.velocity = msg.longitudinal_velocity    
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))  

    def sub_autoware_state_callback(self, msg):
        try:
            # self.information.stop_reasons = msg.stop_reason.stop_reasons
            self.information.autoware_state = msg.state
            
            # if (self.information.autoware_state == AutowareState.INITIALIZING):
            #     print("初始化,请稍等")
            #     pass
            # if (self.information.autoware_state == AutowareState.WAITING_FOR_ROUTE):
            #     print("等待目标站点,请稍等")
            #     pass
            # if (self.information.autoware_state == AutowareState.PLANNING):
            #     print("路线规划中,请稍等")
            #     pass
            # if (self.information.autoware_state == AutowareState.WAITING_FOR_ENGAGE):
            #     print("已准备就绪")
            #     pass
            # if (self.information.autoware_state == AutowareState.DRIVING):
            #     print("自动驾驶中")
            #     pass
            # if (self.information.autoware_state == AutowareState.ARRIVED_GOAL):
            #     print("已到达目标站点")
            #     pass
            # if (self.information.autoware_state == AutowareState.FINALIZING):
            #     print("FINALIZING")
            #     pass
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    def sub_velocity_factor_callback(self, msg):
        try:
            self.information.velocity_factors = msg.factors
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    def sub_motion_state_callback(self, msg):
        try:
            self.information.motion_state = msg.state
        except Exception as e:
            self._node.get_logger().error("Unable to get the motion state, ERROR: " + str(e))

    def sub_localization_initialization_state_callback(self, msg):
        try:
            self.information.localization_init_state = msg.state
        except Exception as e:
            self._node.get_logger().error(
                "Unable to get the localization init state, ERROR: " + str(e)
            )

    def sub_vehicle_seat_belt_callback(self, msg):
        try:
            # 创建一个字典来存储座椅和安全带的状态
            seat_belt_status = {
                'seat1': (msg.safe_belt_sta1, msg.seat_sta1),
                'seat2': (msg.safe_belt_sta2, msg.seat_sta2),
                'seat3': (msg.safe_belt_sta3, msg.seat_sta3),
                'seat4': (msg.safe_belt_sta4, msg.seat_sta4),
                'seat5': (msg.safe_belt_sta5, msg.seat_sta5),
                'seat6': (msg.safe_belt_sta6, msg.seat_sta6)
            }
            # 如果已经触发过报警，则不再触发
            if self.information.alarm_triggered:
                return
            # 遍历座椅状态并设置相应的标志
            for seat, (safe_belt, seat_status) in seat_belt_status.items():
                self.information.alarm_flag = (safe_belt and seat_status) or (not safe_belt and not seat_status) or (not safe_belt and seat_status)
                # 如果报警标志为False，触发报警并提示哪个位置没有系安全带
                if  self.information.alarm_flag:
                    self.trigger_alarm(seat)
        except Exception as e:
                self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))  


    def trigger_alarm(self, seat):
            # 在这里添加触发报警的操作，例如发出警报声或发送通知
            # 同时，根据座位名称构建提示信息并打印到控制台
            self.information.alert_message = f"座位{seat[-1]}有人但未系安全带，触发报警！"
            print(self.information.alert_message)
            # 设置报警触发标志为True
            # self.information.alarm_flag = True  # 座椅有人、安全带系上会自动为True
            self.information.alarm_triggered = True

    def sub_door_open_and_close_callback(self,msg):
        try:   
            if(not msg.door_open_inplace):
                self.information.vehicle_door = True      # 关门语音
            elif(not msg.door_close_inplace):
                self.information.vehicle_door = False     # 开门语音
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))  


        # 车辆当前站点、下一个站点、预计到站的时间
    def sub_current_station_callback(self,msg):
        try:
            self._autoware_connection_time = self._node.get_clock().now()
            self.information._current_station = msg.current_station
            self.information._next_station = msg.next_station
            self.information._estimate_time = msg.estimate_time
            self.information._target_distance = msg.target_distance
        except Exception as e:
            self._node.get_logger().error("Unable to get the goal distance, ERROR: " + str(e))  
        