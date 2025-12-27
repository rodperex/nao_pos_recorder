#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, Trigger, SetBool
import math
import os
from datetime import datetime


class PosRecorderNode(Node):
    """
    Node to record NAO robot joint positions and save them as .pos files
    """
    
    # Joint order for .pos files (V6 format)
    JOINT_ORDER = [
        'HeadYaw', 'HeadPitch',
        'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
        'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
        'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
        'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
        'LHand', 'RHand'
    ]
    
    # Joint names abbreviations for .pos file header
    JOINT_ABBREV = [
        'HY', 'HP',
        'LSP', 'LSR', 'LEY', 'LER', 'LWY',
        'LHYP', 'LHR', 'LHP', 'LKP', 'LAP', 'LAR',
        'RHR', 'RHP', 'RKP', 'RAP', 'RAR',
        'RSP', 'RSR', 'REY', 'RER', 'RWY',
        'LH', 'RH', 'DUR'
    ]
    
    # Arm joints
    ARM_JOINTS = [
        'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
        'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
        'LHand', 'RHand'
    ]
    
    # Leg joints
    LEG_JOINTS = [
        'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
        'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'
    ]
    
    def __init__(self):
        super().__init__('pos_recorder_node')
        
        # Parameters
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('stiffness_topic', '/effectors/joint_stiffnesses')
        self.declare_parameter('output_dir', os.path.expanduser('~/nao_recordings'))
        self.declare_parameter('default_duration', 2000)  # milliseconds
        self.declare_parameter('movement_type', 'arms')  # 'arms', 'legs', or 'all'
        self.declare_parameter('total_duration', 0)  # 0 means use default_duration per keyframe
        
        # Safety parameters
        self.declare_parameter('max_velocity', 120.0)  # degrees per second
        self.declare_parameter('min_duration', 2000)  # minimum milliseconds per keyframe
        self.declare_parameter('enable_safety_checks', True)
        self.declare_parameter('auto_adjust_duration', True)  # automatically increase duration for safety
        
        joint_states_topic = self.get_parameter('joint_states_topic').value
        stiffness_topic = self.get_parameter('stiffness_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.default_duration = self.get_parameter('default_duration').value
        self.movement_type = self.get_parameter('movement_type').value.lower()
        self.max_velocity = self.get_parameter('max_velocity').value
        self.min_duration = self.get_parameter('min_duration').value
        self.enable_safety_checks = self.get_parameter('enable_safety_checks').value
        self.auto_adjust_duration = self.get_parameter('auto_adjust_duration').value
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # State
        self.current_joint_states = {}
        self.recorded_keyframes = []
        self.last_keyframe_time = None
        self.total_duration = None  # Will be set when saving
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10
        )
        
        # Publisher for joint stiffness
        self.stiffness_pub = self.create_publisher(
            JointState,
            stiffness_topic,
            10
        )
        
        # Services
        self.record_srv = self.create_service(
            Empty,
            'record_keyframe',
            self.record_keyframe_callback
        )
        
        self.save_srv = self.create_service(
            Trigger,
            'save_pos_file',
            self.save_pos_file_callback
        )
        
        self.clear_srv = self.create_service(
            Empty,
            'clear_keyframes',
            self.clear_keyframes_callback
        )
        
        self.get_logger().info('NAO Position Recorder Node started')
        self.get_logger().info(f'Movement type: {self.movement_type}')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        
        # Set joint stiffness based on movement type
        self.set_joint_stiffness()
        
        self.get_logger().info('Services available:')
        self.get_logger().info('  - /record_keyframe: Record current position')
        self.get_logger().info('  - /save_pos_file: Save recorded positions to .pos file')
        self.get_logger().info('  - /clear_keyframes: Clear all recorded positions')
        self.get_logger().info('')
        self.get_logger().info('To set total duration, use: ros2 param set /pos_recorder_node total_duration <milliseconds>')
        
    def joint_state_callback(self, msg):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
    
    def rad_to_deg(self, rad):
        """Convert radians to degrees and round to integer"""
        return round(math.degrees(rad))
    
    def calculate_required_duration(self, prev_keyframe, curr_keyframe):
        """Calculate minimum safe duration for movement between keyframes"""
        if not self.enable_safety_checks:
            return self.default_duration
        
        max_movement = 0.0
        
        for joint_name in self.JOINT_ORDER:
            prev_val = prev_keyframe.get(joint_name)
            curr_val = curr_keyframe.get(joint_name)
            
            # Skip if either value is None (not commanded)
            if prev_val is None or curr_val is None:
                continue
            
            # Calculate angular movement
            movement = abs(curr_val - prev_val)
            if movement > max_movement:
                max_movement = movement
        
        # Calculate required duration based on max velocity (deg/s)
        # duration (ms) = movement (deg) / velocity (deg/s) * 1000 (ms/s)
        required_duration = (max_movement / self.max_velocity) * 1000
        
        # Ensure minimum duration
        required_duration = max(required_duration, self.min_duration)
        
        return int(required_duration)
    
    def validate_movement_safety(self, keyframes, duration_per_keyframe):
        """Validate movement safety and provide warnings"""
        if not self.enable_safety_checks or len(keyframes) < 2:
            return True, []
        
        warnings = []
        is_safe = True
        
        for i in range(1, len(keyframes)):
            prev_keyframe = keyframes[i-1]
            curr_keyframe = keyframes[i]
            
            large_movements = []
            
            for joint_name, abbrev in zip(self.JOINT_ORDER, self.JOINT_ABBREV[:25]):
                prev_val = prev_keyframe.get(joint_name)
                curr_val = curr_keyframe.get(joint_name)
                
                if prev_val is None or curr_val is None:
                    continue
                
                movement = abs(curr_val - prev_val)
                velocity = (movement / duration_per_keyframe) * 1000  # deg/s
                
                if velocity > self.max_velocity:
                    large_movements.append((abbrev, movement, velocity))
                    is_safe = False
            
            if large_movements:
                msg = f"Keyframe {i}: Unsafe velocities detected:"
                for abbrev, movement, velocity in large_movements:
                    msg += f"\n  - {abbrev}: {movement:.1f}° @ {velocity:.1f}°/s (max: {self.max_velocity}°/s)"
                warnings.append(msg)
        
        return is_safe, warnings
    
    def set_joint_stiffness(self):
        """Set joint stiffness based on movement type"""
        stiffness_msg = JointState()
        stiffness_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.movement_type == 'arms':
            # Set arm joints to zero stiffness (easy to move)
            # Legs are not commanded and will remain in their current position
            stiffness_msg.name = self.ARM_JOINTS
            stiffness_msg.effort = [0.0] * len(self.ARM_JOINTS)
            self.get_logger().info('Setting arm joints stiffness to 0.0 (relaxed for manual movement)')
            self.get_logger().info('Leg joints remain unchanged and will hold their current position')
            
        elif self.movement_type == 'legs':
            # Set leg joints to zero stiffness (easy to move)
            # Arms are not commanded and will remain in their current position
            stiffness_msg.name = self.LEG_JOINTS
            stiffness_msg.effort = [0.0] * len(self.LEG_JOINTS)
            self.get_logger().info('Setting leg joints stiffness to 0.0 (relaxed for manual movement)')
            self.get_logger().info('Arm joints remain unchanged and will hold their current position')
            
        elif self.movement_type == 'all':
            # Set all joints (arms + legs) to zero stiffness (easy to move)
            all_joints = self.ARM_JOINTS + self.LEG_JOINTS
            stiffness_msg.name = all_joints
            stiffness_msg.effort = [0.0] * len(all_joints)
            self.get_logger().info('Setting ALL joints (arms + legs) stiffness to 0.0 (relaxed for manual movement)')
            self.get_logger().warn('CAUTION: All leg joints are relaxed - ensure robot is properly supported!')
            
        else:
            self.get_logger().warn(f'Invalid movement_type: {self.movement_type}. Use "arms", "legs", or "all"')
            return
        
        # Publish stiffness command (only for the selected joint group)
        self.stiffness_pub.publish(stiffness_msg)
        self.get_logger().info(f'Stiffness command published for {self.movement_type}')
    
    def record_keyframe_callback(self, request, response):
        """Record current robot position as a keyframe"""
        if not self.current_joint_states:
            self.get_logger().warn('No joint states received yet. Cannot record keyframe.')
            return response
        
        # Create keyframe with current joint positions (no duration yet)
        keyframe = {}
        for joint_name in self.JOINT_ORDER:
            if joint_name in self.current_joint_states:
                # Convert from radians to degrees
                keyframe[joint_name] = self.rad_to_deg(self.current_joint_states[joint_name])
            else:
                # Use '-' for missing joints
                keyframe[joint_name] = None
        
        self.recorded_keyframes.append(keyframe)
        
        self.get_logger().info(f'Recorded keyframe #{len(self.recorded_keyframes)}')
        return response
    
    def save_pos_file_callback(self, request, response):
        """Save recorded keyframes to a .pos file"""
        if not self.recorded_keyframes:
            response.success = False
            response.message = 'No keyframes recorded. Use /record_keyframe first.'
            self.get_logger().warn(response.message)
            return response
        
        num_keyframes = len(self.recorded_keyframes)
        
        # Get total_duration from parameter
        total_duration_param = self.get_parameter('total_duration').value
        if total_duration_param > 0:
            self.total_duration = total_duration_param
        
        # Calculate durations for each keyframe with safety checks
        if self.auto_adjust_duration and self.enable_safety_checks and num_keyframes > 1:
            # Calculate safe duration for each transition
            total_safe_duration = 0
            for i in range(1, num_keyframes):
                required = self.calculate_required_duration(self.recorded_keyframes[i-1], self.recorded_keyframes[i])
                total_safe_duration += required
            
            if self.total_duration is not None:
                # Use the larger of specified and safe duration
                if total_safe_duration > self.total_duration:
                    self.get_logger().warn(f'Specified duration {self.total_duration}ms is unsafe. Adjusting to {total_safe_duration}ms')
                    self.total_duration = total_safe_duration
            else:
                # Use calculated safe duration
                self.total_duration = total_safe_duration
                self.get_logger().info(f'Auto-adjusted duration to {self.total_duration}ms for safety')
        
        # Calculate durations for each keyframe
        if self.total_duration is not None:
            # Divide total duration evenly among keyframes
            duration_per_keyframe = self.total_duration // num_keyframes
            self.get_logger().info(f'Interpolating {num_keyframes} keyframes over {self.total_duration}ms (~{duration_per_keyframe}ms each)')
        else:
            # Use default duration for all keyframes
            duration_per_keyframe = max(self.default_duration, self.min_duration)
            self.get_logger().info(f'Using duration of {duration_per_keyframe}ms per keyframe')
        
        # Safety validation
        is_safe, warnings = self.validate_movement_safety(self.recorded_keyframes, duration_per_keyframe)
        
        if not is_safe:
            self.get_logger().error('SAFETY WARNING: Movement contains unsafe velocities!')
            for warning in warnings:
                self.get_logger().error(warning)
            self.get_logger().error(f'Consider increasing duration or recording more intermediate keyframes.')
            self.get_logger().error(f'To disable safety checks: ros2 param set /pos_recorder_node enable_safety_checks false')
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'recorded_{timestamp}.pos'
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            with open(filepath, 'w') as f:
                # Write header with proper spacing for alignment
                header = '  ' + '  '.join([f'{abbr:<4s}' for abbr in self.JOINT_ABBREV])
                f.write(header + '\n')
                
                # Write each keyframe
                for keyframe in self.recorded_keyframes:
                    line = '! '
                    
                    # Write joint values
                    for joint_name in self.JOINT_ORDER:
                        value = keyframe.get(joint_name)
                        if value is None:
                            line += '-    '
                        else:
                            line += f'{value:<5d} '
                    
                    # Write duration
                    line += f'{duration_per_keyframe:<5d}'
                    f.write(line + '\n')
            
            response.success = True
            if self.total_duration:
                response.message = f'Saved {num_keyframes} keyframes to {filepath} (total duration: {self.total_duration}ms)'
            else:
                response.message = f'Saved {num_keyframes} keyframes to {filepath} (default duration per keyframe)'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to save file: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def clear_keyframes_callback(self, request, response):
        """Clear all recorded keyframes"""
        num_cleared = len(self.recorded_keyframes)
        self.recorded_keyframes.clear()
        self.last_keyframe_time = None
        self.total_duration = None
        self.get_logger().info(f'Cleared {num_cleared} keyframes')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PosRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
