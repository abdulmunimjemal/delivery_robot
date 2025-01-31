# app/routes/robot.py
from flask import Blueprint, jsonify
from flask_socketio import SocketIO
from database import db_session
from app.models import Order
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import threading
import time

robot_bp = Blueprint('robot', __name__, url_prefix='/robot')

class RobotController:
    def __init__(self, socketio):
        self.socketio = socketio
        self.navigator = None
        self.status = "idle"
        self.logs = []
        self.table_coordinates = {
            1: (-0.74, 2.00),
            2: (3.00, 2.65),
            3: (3.00, -4.12),
            4: (-0.40, -3.26)
        }

        self.base_coordinates = (-5.20, -3.38)
        self.current_order = None
        self.ros_thread = threading.Thread(target=self.init_ros)
        self.ros_thread.start()

    def init_ros(self):
        rclpy.init()
        self.navigator = BasicNavigator()
        self.set_initial_pose()
        executor = MultiThreadedExecutor()
        executor.add_node(self.navigator)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

    def set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.position.x, initial_pose.pose.position.y = self.base_coordinates
        initial_pose.pose.orientation.w = 0.99
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def log_action(self, message):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self.logs.insert(0, f"[{timestamp}] {message}")
        self.socketio.emit('robot_log', {'log': self.logs[0]}, namespace='/robot')

    def update_status(self, new_status):
        self.status = new_status
        self.socketio.emit('robot_status', {
            'status': self.status,
            'current_order': self.current_order,
            'battery': 85
        }, namespace='/robot')

    def navigate_to_base(self):
        return self.navigate_to_position(self.base_coordinates)

    def navigate_to_table(self, table_number):
        if table_number not in self.table_coordinates:
            return False
        return self.navigate_to_position(self.table_coordinates[table_number])

    def navigate_to_position(self, coordinates):
        x, y = coordinates
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x, goal_pose.pose.position.y = x, y
        goal_pose.pose.orientation.w = 0.99
        self.navigator.goToPose(goal_pose)
        return True

    def delivery_cycle(self, order):
        try:
            session = db_session()
            db_order = session.query(Order).get(order['id'])
            if not db_order or db_order.status != 'ready':
                self.log_action(f"Order {order['id']} not available for delivery")
                return False

            db_order.status = 'in_delivery'
            session.commit()
            self.current_order = order
            self.update_status(f"Moving to Table {order['table_number']}")
            self.log_action(f"Starting delivery for Order {order['id']}")

            if self.navigate_to_table(order['table_number']):
                while not self.navigator.isTaskComplete():
                    time.sleep(1)
                
                if self.navigator.getResult() == TaskResult.SUCCEEDED:
                    self.log_action(f"Reached Table {order['table_number']}")
                    self.update_status("Delivering items")
                    time.sleep(3)

                    db_order.status = 'delivered'
                    session.commit()
                    self.update_status("Returning to base")
                    self.navigate_to_base()

                    while not self.navigator.isTaskComplete():
                        time.sleep(1)
                    
                    self.log_action("Returned to base station" if self.navigator.getResult() == TaskResult.SUCCEEDED else "Base return failed")
            else:
                self.log_action("Navigation initialization failed")
            
            self.update_status("idle")
            self.current_order = None
            return True

        except Exception as e:
            self.log_action(f"Delivery error: {str(e)}")
            if 'id' in order:
                db_order = session.query(Order).get(order['id'])
                if db_order and db_order.status == 'in_delivery':
                    db_order.status = 'ready'
                    session.commit()
            return False

robot_controller = None

def init_robot_controller(socketio):
    global robot_controller
    robot_controller = RobotController(socketio)

@robot_bp.route('/tasks')
def get_robot_status():
    return jsonify({
        'status': robot_controller.status,
        'current_order': robot_controller.current_order,
        'logs': robot_controller.logs[:10],
        'battery': 85
    })

@robot_bp.route('/tasks/process', methods=['POST'])
def process_next_order():
    session = db_session()
    order = session.query(Order).filter_by(status='ready').first()
    print("Processing: Order", order)
    print("All Orders: ", [order.status for order in session.query(Order).all()])
    if order:
        session.commit()
        order_data = {
            'id': order.id,
            'table_number': order.table_number,
            'items': [item.menu_item.name for item in order.items]
        }
        robot_controller.socketio.start_background_task(robot_controller.delivery_cycle, order_data)
    return jsonify({'success': bool(order)})

@robot_bp.route('/tasks/<int:order_id>/pickup', methods=['POST'])
def pickup_order(order_id):
    session = db_session()
    order = session.query(Order).get(order_id)
    if not order or order.status != 'ready':
        return jsonify({'error': 'Invalid order for pickup'}), 400
    order.status = 'in_delivery'
    session.commit()
    return jsonify({'message': f'Order {order_id} picked up for delivery'})

@robot_bp.route('/tasks/<int:order_id>/deliver', methods=['POST'])
def complete_delivery(order_id):
    session = db_session()
    order = session.query(Order).get(order_id)
    if not order or order.status != 'in_delivery':
        return jsonify({'error': 'Invalid delivery completion'}), 400
    order.status = 'delivered'
    session.commit()
    return jsonify({'message': f'Order {order_id} delivered successfully'})
