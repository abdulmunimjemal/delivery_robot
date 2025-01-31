from flask import Flask
from flask_socketio import SocketIO
from config import Config
from database import init_db

socketio = SocketIO()

def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)
    
    init_db()
    
    # Initialize SocketIO with the app
    socketio.init_app(app, async_mode='threading')
    
    from app.routes.kitchen import kitchen_bp
    from app.routes.user import user_bp
    from app.routes.robot import robot_bp, init_robot_controller
    from app.routes.analytics import analytics_bp
    
    app.register_blueprint(kitchen_bp)
    app.register_blueprint(user_bp)
    app.register_blueprint(robot_bp)
    app.register_blueprint(analytics_bp)
    
    # Initialize robot controller after app is created
    with app.app_context():
        init_robot_controller(socketio)
    
    @app.route('/')
    def home():
        return "Welcome to the Restaurant System! Go to /kitchen or /user"
    
    return app