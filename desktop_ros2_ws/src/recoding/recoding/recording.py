# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# from geometry_msgs.msg import Twist
# import csv
# import os
# import glob
# import threading
# import time
# import sys
# import select
# import termios
# import tty


# class RecordingNode(Node):
#     def __init__(self):
#         super().__init__('recording_node')
        
#         # State variables
#         self.is_recording = False
#         self.is_playing = False
#         self.recording_data = []
#         self.playback_thread = None
        
#         # Publishers and Subscribers
#         self.control_publisher = self.create_publisher(Float32, '/control_signal', 10)
#         self.angular_publisher = self.create_publisher(Float32, '/angular_target', 10)
        
#         self.control_subscriber = self.create_subscription(
#             Float32, '/control_signal', self.control_callback, 10)
#         self.angular_subscriber = self.create_subscription(
#             Float32, '/angular_target', self.angular_callback, 10)
        
#         # Timer for recording at 100Hz
#         self.recording_timer = self.create_timer(0.01, self.recording_timer_callback)
#         self.recording_timer.cancel()  # Start disabled
        
#         # Data storage for current recording
#         self.current_control_data = None
#         self.current_angular_data = None
        
#         self.get_logger().info("Recording Node started. Press 'r' to record, 'p' to playback, 'q' to quit")
        
#     def control_callback(self, msg):
#         """Callback for control signal messages"""
#         self.current_control_data = msg
        
#     def angular_callback(self, msg):
#         """Callback for angular target messages"""
#         self.current_angular_data = msg
        
#     def recording_timer_callback(self):
#         """Timer callback for recording data at 100Hz"""
#         if self.is_recording:
#             timestamp = time.time()
#             control_data = self.current_control_data.data if self.current_control_data else []
#             angular_data = [
#                 self.current_angular_data.linear.x if self.current_angular_data else 0.0,
#                 self.current_angular_data.linear.y if self.current_angular_data else 0.0,
#                 self.current_angular_data.linear.z if self.current_angular_data else 0.0,
#                 self.current_angular_data.angular.x if self.current_angular_data else 0.0,
#                 self.current_angular_data.angular.y if self.current_angular_data else 0.0,
#                 self.current_angular_data.angular.z if self.current_angular_data else 0.0
#             ]
            
#             self.recording_data.append([timestamp] + control_data + angular_data)
    
#     def start_recording(self):
#         """Start recording data"""
#         if not self.is_recording:
#             self.is_recording = True
#             self.recording_data = []
#             self.recording_timer.reset()
#             self.get_logger().info("▶️ Started RECORDING")
#         else:
#             self.stop_recording()
    
#     def stop_recording(self):
#         """Stop recording and save to CSV"""
#         if self.is_recording:
#             self.is_recording = False
#             self.recording_timer.cancel()
            
#             if self.recording_data:
#                 # Save to CSV file
#                 filename = f"recording_{int(time.time())}.csv"
#                 with open(filename, 'w', newline='') as csvfile:
#                     writer = csv.writer(csvfile)
#                     # Write header
#                     writer.writerow(['timestamp', 'control_data', 'angular_x', 'angular_y', 'angular_z', 'angular_roll', 'angular_pitch', 'angular_yaw'])
#                     # Write data
#                     for row in self.recording_data:
#                         writer.writerow(row)
                
#                 self.get_logger().info(f"⏹️ Stopped recording. Saved {len(self.recording_data)} data points to {filename}")
#             else:
#                 self.get_logger().info("⏹️ Stopped recording. No data recorded.")
    
#     def start_playback(self):
#         """Start playback of the most recent CSV file"""
#         if not self.is_playing:
#             # Find the most recent CSV file
#             csv_files = glob.glob("recording_*.csv")
#             if not csv_files:
#                 self.get_logger().error("No recording files found!")
#                 return
            
#             latest_file = max(csv_files, key=os.path.getctime)
#             self.get_logger().info(f"▶️ Started PLAYBACK from {latest_file}")
            
#             # Start playback in a separate thread
#             self.playback_thread = threading.Thread(target=self.playback_worker, args=(latest_file,))
#             self.playback_thread.daemon = True
#             self.playback_thread.start()
#         else:
#             self.stop_playback()
    
#     def stop_playback(self):
#         """Stop playback"""
#         if self.is_playing:
#             self.is_playing = False
#             self.get_logger().info("⏹️ Stopped PLAYBACK")
    
#     def playback_worker(self, filename):
#         """Worker thread for playback"""
#         self.is_playing = True
        
#         try:
#             with open(filename, 'r') as csvfile:
#                 reader = csv.reader(csvfile)
#                 header = next(reader)  # Skip header
                
#                 start_time = time.time()
                
#                 for row in reader:
#                     if not self.is_playing:
#                         break
                    
#                     # Parse the row data
#                     timestamp = float(row[0])
#                     control_data = eval(row[1]) if row[1] else []
#                     angular_data = [float(x) for x in row[2:8]]
                    
#                     # Create messages
#                     control_msg = Float64MultiArray()
#                     control_msg.data = control_data
                    
#                     angular_msg = Twist()
#                     angular_msg.linear.x = angular_data[0]
#                     angular_msg.linear.y = angular_data[1]
#                     angular_msg.linear.z = angular_data[2]
#                     angular_msg.angular.x = angular_data[3]
#                     angular_msg.angular.y = angular_data[4]
#                     angular_msg.angular.z = angular_data[5]
                    
#                     # Publish messages
#                     self.control_publisher.publish(control_msg)
#                     self.angular_publisher.publish(angular_msg)
                    
#                     # Wait for next iteration (100Hz = 0.01s)
#                     time.sleep(0.01)
                
#                 self.get_logger().info("Playback completed - end of file reached")
                
#         except Exception as e:
#             self.get_logger().error(f"Playback error: {str(e)}")
#         finally:
#             self.is_playing = False


# def get_key():
#     """Get a single keypress from the terminal"""
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch


# def main(args=None):
#     rclpy.init(args=args)
    
#     node = RecordingNode()
    
#     try:
#         while rclpy.ok():
#             # Check for keypress
#             if select.select([sys.stdin], [], [], 0.1)[0]:
#                 key = get_key().lower()
                
#                 if key == 'r':
#                     node.start_recording()
#                 elif key == 'p':
#                     node.start_playback()
#                 elif key == 'q':
#                     break
            
#             # Spin the node
#             rclpy.spin_once(node, timeout_sec=0.01)
            
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Cleanup
#         if node.is_recording:
#             node.stop_recording()
#         if node.is_playing:
#             node.stop_playback()
        
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import csv
import os
import glob
import threading
import sys
import termios
import tty
from datetime import datetime

# A dictionary to store the latest message from each topic
latest_messages = {
    '/control_signal': None,
    'angular_target': None  # Topic name corrected to match publisher
}

class DataRecorderPlayerNode(Node):
    """
    A ROS 2 node that can record and playback data from specified topics.
    - Press 'r' to start/stop recording. Data is saved to a timestamped CSV file.
    - Press 'p' to start/stop playing back the most recent CSV file.
    - Press 'q' to quit.
    """
    def __init__(self):
        super().__init__('recorder_player_node')

        # --- State Management ---
        self.state = 'IDLE'  # Can be 'IDLE', 'RECORDING', or 'PLAYING'
        self.recorded_data = []
        self.playback_data = []
        self.playback_index = 0
        self.latest_csv_file = None

        # --- Publishers (Using Float32 to match pd_controller.py) ---
        self.control_pub = self.create_publisher(Float32, '/control_signal', 10)
        self.angular_pub = self.create_publisher(Float32, 'angular_target', 10)

        # --- Subscribers (will be created/destroyed dynamically) ---
        self.control_sub = None
        self.angular_sub = None

        # --- Timers (for 100Hz operation) ---
        self.recording_timer = None
        self.playback_timer = None

        # --- Keyboard Input Handling ---
        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

        self.print_menu()

    def print_menu(self):
        """Prints the main menu to the console."""
        self.get_logger().info("\n" + "="*50 +
                             "\n[IDLE] Ready. Press a key to continue:" +
                             "\n  'r' - Start / Stop Recording" +
                             "\n  'p' - Start / Stop Playback" +
                             "\n  'q' - Quit" +
                             "\n" + "="*50)

    def keyboard_listener(self):
        """Listens for single key presses."""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key == 'r':
                    if self.state == 'IDLE':
                        self.start_recording()
                    elif self.state == 'RECORDING':
                        self.stop_recording()
                elif key == 'p':
                    if self.state == 'IDLE':
                        self.start_playback()
                    elif self.state == 'PLAYING':
                        self.stop_playback()
                elif key == 'q':
                    self.get_logger().info("'q' pressed. Shutting down...")
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # --- Recording Logic ---
    def start_recording(self):
        self.state = 'RECORDING'
        self.recorded_data = []
        # Create subscribers with the correct Float32 type
        self.control_sub = self.create_subscription(Float32, '/control_signal', self.control_callback, 10)
        self.angular_sub = self.create_subscription(Float32, 'angular_target', self.angular_callback, 10)
        self.recording_timer = self.create_timer(0.05, self.sample_data_point)
        self.get_logger().info("▶️ RECORDING started. Press 'r' again to stop.")

    def stop_recording(self):
        if self.recording_timer:
            self.recording_timer.destroy()
        if self.control_sub:
            self.destroy_subscription(self.control_sub)
        if self.angular_sub:
            self.destroy_subscription(self.angular_sub)

        self.save_to_csv()
        self.state = 'IDLE'
        self.print_menu()

    def control_callback(self, msg):
        # FIX: Store the float value from .data, not the whole message object
        latest_messages['/control_signal'] = msg.data

    def angular_callback(self, msg):
        # FIX: Store the float value from .data, not the whole message object
        latest_messages['angular_target'] = msg.data

    def sample_data_point(self):
        """Called by a 100Hz timer to record the most recent data."""
        # FIX: Directly use the stored float values. No more .linear.x access.
        if latest_messages['/control_signal'] is not None and latest_messages['angular_target'] is not None:
            timestamp = self.get_clock().now().nanoseconds
            control_signal = latest_messages['/control_signal']
            angular_target = latest_messages['angular_target']
            self.recorded_data.append([timestamp, control_signal, angular_target])

    def save_to_csv(self):
        if not self.recorded_data:
            self.get_logger().warn("No data recorded. Nothing to save.")
            return

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"recording_{timestamp_str}.csv"
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # FIX: Simplified header for simple float data
            writer.writerow(['timestamp_ns', 'control_signal', 'angular_target'])
            writer.writerows(self.recorded_data)
        
        self.get_logger().info(f"⏹️ Recording stopped. Data saved to '{filename}'.")

    # --- Playback Logic ---
    def start_playback(self):
        list_of_files = glob.glob('t_left.csv')
        if not list_of_files:
            self.get_logger().error("No 'recording_*.csv' files found to play back.")
            self.print_menu()
            return
        self.latest_csv_file = max(list_of_files, key=os.path.getctime)
        
        self.get_logger().info(f"Reading data from '{self.latest_csv_file}'...")
        with open(self.latest_csv_file, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader) # Skip header
            self.playback_data = [row for row in reader]

        if not self.playback_data:
            self.get_logger().error(f"CSV file '{self.latest_csv_file}' is empty.")
            self.state = 'IDLE'
            self.print_menu()
            return

        self.state = 'PLAYING'
        self.playback_index = 0
        self.playback_timer = self.create_timer(0.05, self.playback_step)
        self.get_logger().info("▶️ PLAYBACK started. Press 'p' again to stop.")

    def stop_playback(self):
        if self.playback_timer:
            self.playback_timer.destroy()
        self.state = 'IDLE'
        self.get_logger().info("⏹️ Playback stopped.")
        self.print_menu()

    def playback_step(self):
        """Called by a 100Hz timer to publish one line of data."""
        if self.playback_index >= len(self.playback_data):
            self.get_logger().info("✅ Playback finished.")
            self.stop_playback()
            return

        row = self.playback_data[self.playback_index]
        try:
            # FIX: Read simple float values directly from CSV
            control_val = float(row[1])
            angular_val = float(row[2])

            # FIX: Publish as Float32 messages
            control_msg = Float32()
            control_msg.data = control_val
            self.control_pub.publish(control_msg)

            angular_msg = Float32()
            angular_msg.data = angular_val
            self.angular_pub.publish(angular_msg)
            
            self.playback_index += 1
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error processing row {self.playback_index}: {row}. Stopping playback. Error: {e}")
            self.stop_playback()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
