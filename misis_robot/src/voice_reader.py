import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import logging
import uvicorn
import threading

from fastapi import APIRouter, UploadFile
from fastapi import FastAPI

app = FastAPI(title='main app', openapi_url=f"/api/v1/openapi.json")
root_router = APIRouter()

class VoiceReader(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        #self.node = rclpy.create_node('voice_reader')
        #self.node.get_logger().info('Created node')
        #self.reader = self.node.create_subscription(
        #    msg_type=String,
        #    topic='/hello_ros2',
        #    callback=self.voice_callback,
        #    qos_profile=10)
        self.vel_pub = self.create_publisher(msg_type=Twist, topic='/tricycle_controller/cmd_vel', qos_profile=10)

        @app.get("/moving")
        async def text_ask_text(direction: str):
            print('Получено: ' + direction)
            t = 1
            vel_cmd = Twist()
            match direction:
                case "forward":
                    vel_cmd.linear.x = 10.0
                    vel_cmd.angular.z = 0.0
                    t = 100000
                case "backward":
                    vel_cmd.linear.x = 0.0
                    vel_cmd.angular.z = 0.0
                case "right":
                    vel_cmd.linear.x = 0.0
                    vel_cmd.angular.z = -4.0
                    t = 100000
                case "left":
                    vel_cmd.linear.x = 0.0
                    vel_cmd.angular.z = 4.0
                case "stop":
                    vel_cmd.linear.x = 0.0
                    vel_cmd.angular.z = 0.0
            for _ in range(t):
                self.vel_pub.publish(vel_cmd)
            return 'OK'


def main(args=None):
    rclpy.init(args=args)

    voiceReader = VoiceReader()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(voiceReader)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    uvicorn.run(app, host="192.168.1.25", port=8000, log_level="debug")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
