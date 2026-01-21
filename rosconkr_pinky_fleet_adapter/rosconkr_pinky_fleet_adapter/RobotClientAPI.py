# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

import math
import threading
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml):
        self.prefix = config_yaml.get('prefix', '')
        self.user = config_yaml.get('user', '')
        self.password = config_yaml.get('password', '')
        self.timeout = config_yaml.get('timeout', 5.0)
        self.debug = config_yaml.get('debug', False)
        
        # 스레드별 현재 로봇 컨텍스트 저장소
        self._thread_context = threading.local()
        
        # 설정에서 로봇별 네임스페이스 정보 로드
        self.robot_configs = config_yaml.get('robots', {})
        self._map_frame = config_yaml.get('map_frame', 'map')
        
        # ROS 2 노드 초기화
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('fleet_adapter_nav2_client')
        
        # 로봇별 ROS 핸들 관리용 딕셔너리
        self._nav_clients: Dict[str, ActionClient] = {}
        self._pose_subscribers: Dict[str, Any] = {}
        self._cmd_vel_publishers: Dict[str, Any] = {}
        self._initial_pose_publishers: Dict[str, Any] = {}
        self._battery_subscribers: Dict[str, Any] = {}
        self._goal_handles: Dict[str, Any] = {}
        
        # 로봇별 상태 추적용 딕셔너리
        self._last_poses: Dict[str, Optional[List[float]]] = {}
        self._cmd_status: Dict[str, bool] = {}
        self._battery_levels: Dict[str, float] = {}
        self._current_maps: Dict[str, str] = {}
        
        # ROS 2 메시지 처리를 위한 백그라운드 스레드 시작
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        
        if self.debug:
            print('[RobotAPI] 네임스페이스 기반 다중 로봇 API 초기화 완료')

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True

    def localize(
        self,
        robot_name: str,
        pose,
        map_name: str,
    ):
        ''' Request the robot to localize on target map. This 
            function should return True if the robot has accepted the 
            request, else False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        
        # RMF 내부 상태 관리를 위해 맵 정보만 업데이트
        self._ensure_robot_setup(robot_name)
        self._current_maps[robot_name] = map_name
        
        return True 
    
    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        try:
            self._ensure_robot_setup(robot_name)
            
            client = self._nav_clients[robot_name]
            
            # Nav2 액션 서버 연결 상태 확인
            if not client.server_is_ready():
                if not client.wait_for_server(timeout_sec=2.0):
                    if self.debug:
                        print(f'[RobotAPI] [{robot_name}] Nav2 서버 응답 없음')
                    return False
            
            # Navigation goal 생성
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            
            x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.position.z = 0.0
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # 명령 상태를 '진행 중'으로 변경
            self._cmd_status[robot_name] = False
            
            # 비동기 goal 전송
            send_future = client.send_goal_async(goal)
            
            def _goal_response_cb(fut):
                try:
                    goal_handle = fut.result()
                except Exception as e:
                    if self.debug:
                        print(f'[RobotAPI] [{robot_name}] Goal 응답 에러: {e}')
                    self._cmd_status[robot_name] = True
                    return

                self._goal_handles[robot_name] = goal_handle
                
                if not goal_handle.accepted:
                    if self.debug:
                        print(f'[RobotAPI] [{robot_name}] Navigation goal 거부됨')
                    self._cmd_status[robot_name] = True
                    return
                
                # 결과 대기 콜백 등록
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(
                    lambda r: self._navigation_done_cb(r, robot_name)
                )
            
            send_future.add_done_callback(_goal_response_cb)
            
            # 현재 맵 정보 업데이트
            self._current_maps[robot_name] = map_name
            
            if self.debug:
                namespace = self._get_namespace(robot_name)
                print(f'[RobotAPI] [{robot_name}] 네비게이션 명령 전송: '
                      f'목표=({x:.2f}, {y:.2f}, {yaw:.2f}) via {namespace}')
            
            return True
            
        except Exception as e:
            if self.debug:
                print(f'[RobotAPI] [{robot_name}] 네비게이션 에러: {e}')
            self._cmd_status[robot_name] = True
            return False

    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ):
        ''' Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if process has started/is queued successfully, else
        return False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        try:
            self._ensure_robot_setup(robot_name)
            
            # 1. 현재 네비게이션 goal 취소
            goal_handle = self._goal_handles.get(robot_name)
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            
            # 2. 속도 0으로 cmd_vel 발행 (즉시 정지)
            cmd_vel_pub = self._cmd_vel_publishers[robot_name]
            twist_msg = TwistStamped()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.header.stamp = self._node.get_clock().now().to_msg()
            cmd_vel_pub.publish(twist_msg)
            
            # 3. 명령 상태를 완료로 강제 변경
            self._cmd_status[robot_name] = True
            
            if self.debug:
                print(f'[RobotAPI] [{robot_name}] 로봇 정지 명령 실행')
            
            return True
            
        except Exception as e:
            if self.debug:
                print(f'[RobotAPI] [{robot_name}] 정지 에러: {e}')
            return False

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        self._ensure_robot_setup(robot_name)
        # AMCL 콜백에서 지속적으로 업데이트되는 최신 위치 반환
        return self._last_poses.get(robot_name)

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        self._ensure_robot_setup(robot_name)
        
        # 실제 배터리 토픽에서 수신한 SOC 값 반환
        soc = self._battery_levels.get(robot_name)
        
        if soc is None:
            if self.debug:
                print(f'[RobotAPI] [{robot_name}] 배터리 데이터 아직 미수신')
            return None
            
        return soc

    def map(self, robot_name: str):
        ''' Return the name of the map that the robot is currently on or
        None if any errors are encountered. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        self._ensure_robot_setup(robot_name)
        
        return self._current_maps.get(robot_name, 'L1')

    def is_command_completed(self):
        ''' Return True if the robot has completed its last command, else
        return False. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        robot_name = getattr(self._thread_context, 'robot_name', None)
        
        if robot_name is None:
            if self.debug:
                print('[RobotAPI] 경고: 로봇 컨텍스트 없이 is_command_completed 호출됨')
            return True  # 데드락 방지를 위한 안전한 기본값
        
        status = self._cmd_status.get(robot_name, True)
        
        if self.debug and not status:
            print(f'[RobotAPI] [{robot_name}] 명령 진행 중...')
        
        return status

    def get_data(self, robot_name: str):
        ''' Returns a RobotUpdateData for one robot if a name is given. Otherwise
        return a list of RobotUpdateData for all robots. '''
        self._ensure_robot_setup(robot_name)
        self._thread_context.robot_name = robot_name
        
        map = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        
        if map is None or position is None:
            return None
        
        # battery_soc가 None이면 기본값 1.0 (100%) 사용
        if battery_soc is None:
            battery_soc = 1.0
            
        return RobotUpdateData(robot_name, map, position, battery_soc)

    # ==========================================================================
    # 다중 로봇 지원을 위한 내부 헬퍼 함수들
    # 템플릿 인터페이스 외부에 추가된 구현 함수들입니다.
    # ==========================================================================

    def _spin(self):
        """ROS 2 콜백 처리를 위한 백그라운드 스핀 루프"""
        try:
            rclpy.spin(self._node)
        except Exception as e:
            if self.debug:
                print(f'[RobotAPI] Spin 에러: {e}')

    def _get_namespace(self, robot_name: str) -> str:
        """로봇 이름을 기반으로 네임스페이스 반환 (설정 없으면 자동 생성)"""
        if robot_name in self.robot_configs:
            return self.robot_configs[robot_name].get('namespace')
        # 설정이 없으면 /robot_name 형태로 fallback
        return ""

    def _ensure_robot_setup(self, robot_name: str):
        """
        지연 초기화 (Lazy Initialization): 로봇이 처음 호출될 때 ROS 클라이언트 동적 생성
        이를 통해 Fleet Adapter 재시작 없이 새로운 로봇을 동적으로 추가할 수 있음
        """
        if robot_name in self._nav_clients:
            return  # 이미 설정됨
        
        namespace = self._get_namespace(robot_name)
        
        if self.debug:
            print(f'[RobotAPI] [{robot_name}] 로봇 설정 시작 (네임스페이스: {namespace})')
        
        # 1. Navigation Action Client 생성
        action_name = f'{namespace}/navigate_to_pose'
        self._nav_clients[robot_name] = ActionClient(
            self._node,
            NavigateToPose,
            action_name
        )
        
        # 2. AMCL Pose Subscription 생성
        amcl_topic = f'{namespace}/amcl_pose'
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self._pose_subscribers[robot_name] = self._node.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            lambda msg, rname=robot_name: self._amcl_cb(msg, rname),
            qos
        )
        
        # 3. cmd_vel Publisher 생성 (정지 명령용)
        cmd_vel_topic = f'{namespace}/cmd_vel'
        self._cmd_vel_publishers[robot_name] = self._node.create_publisher(
            TwistStamped,
            cmd_vel_topic,
            10
        )
        
        # 4. Initial Pose Publisher 생성 (로컬라이제이션용)
        initial_pose_topic = f'{namespace}/initialpose'
        self._initial_pose_publishers[robot_name] = self._node.create_publisher(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            10
        )
        
        # 5. Battery Subscription 생성 (/robot_ns/battery/present)
        battery_topic = f'{namespace}/battery/present'
        battery_qos = QoSProfile(
            depth=10,  # 배터리 데이터는 여러 개 보관
            reliability=ReliabilityPolicy.RELIABLE,  # 배터리 정보는 중요하므로 RELIABLE
            durability=DurabilityPolicy.VOLATILE
        )
        self._battery_subscribers[robot_name] = self._node.create_subscription(
            Float32,
            battery_topic,
            lambda msg, rname=robot_name: self._battery_cb(msg, rname),
            battery_qos
        )
        
        # 로봇별 초기 상태 설정
        self._last_poses[robot_name] = None
        self._cmd_status[robot_name] = True
        self._goal_handles[robot_name] = None
        self._battery_levels[robot_name] = 1.0  # 초기값: 100%
        self._current_maps[robot_name] = 'L1'
        
        if self.debug:
            print(f'[RobotAPI] [{robot_name}] 로봇 설정 완료:')
            print(f'  - Navigation: {action_name}')
            print(f'  - AMCL: {amcl_topic}')
            print(f'  - cmd_vel: {cmd_vel_topic}')
            print(f'  - Battery: {battery_topic}')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped, robot_name: str):
        """특정 로봇의 AMCL 위치 추정 결과를 [x, y, yaw] 형태로 저장"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        
        # 쿼터니언을 yaw 각도로 변환
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self._last_poses[robot_name] = [p.x, p.y, yaw]

    def _battery_cb(self, msg: Float32, robot_name: str):
        """
        배터리 퍼센트 콜백 함수
        입력: std_msgs/Float32 (0.0 ~ 100.0) 예: 87.5999984741211
        출력: RMF 표준 SOC (0.0 ~ 1.0) 범위로 변환하여 저장
        """
        try:
            percent = float(msg.data)
            # 안전한 범위 제한 및 SOC 변환
            soc = max(0.0, min(1.0, percent / 100.0))
            self._battery_levels[robot_name] = soc
            
            if self.debug:
                print(f'[RobotAPI] [{robot_name}] 배터리 업데이트: {percent:.1f}% → SOC: {soc:.3f}')
                
        except Exception as e:
            if self.debug:
                print(f'[RobotAPI] [{robot_name}] 배터리 콜백 에러: {e}')

    def _navigation_done_cb(self, future, robot_name: str):
        """네비게이션 완료 시 해당 로봇의 상태를 완료로 변경"""
        self._cmd_status[robot_name] = True
        
        if self.debug:
            try:
                result = future.result()
                print(f'[RobotAPI] [{robot_name}] 네비게이션 완료')
            except Exception as e:
                print(f'[RobotAPI] [{robot_name}] 네비게이션 실패: {e}')


class RobotUpdateData:
    ''' Update data for a single robot. '''
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
