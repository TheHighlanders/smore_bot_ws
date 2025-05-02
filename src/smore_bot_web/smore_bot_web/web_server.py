import rclpy
from rclpy.node import Node
import os
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server')
        
        package_dir = os.path.dirname(os.path.realpath(__file__))
        web_dir = os.path.join(package_dir, 'web')
        
        if not os.path.exists(web_dir):
            self.get_logger().error(f"Web directory not found at {web_dir}")
            self.get_logger().error("Did you create the web directory and add the index.html file?")
            self.get_logger().error("Try: mkdir -p ~/smore_bot_ws/src/smore_bot_web/smore_bot_web/web")
            sys.exit(1)
            
        # Change to that directory so SimpleHTTPRequestHandler serves from there
        os.chdir(web_dir)
        
        server_address = ('', 8080)
        self.httpd = HTTPServer(server_address, SimpleHTTPRequestHandler)
        
        self.web_thread = threading.Thread(target=self.run_server)
        self.web_thread.daemon = True
        self.web_thread.start()
        
        self.get_logger().info('Web server started on port 8080')
        self.get_logger().info('Connect to http://localhost:8080 to view the dashboard')
        self.get_logger().info('Make sure rosbridge_websocket is running on port 9090!')
    
    def run_server(self):
        self.httpd.serve_forever()
        
    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()