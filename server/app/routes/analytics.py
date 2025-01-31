from flask import Blueprint, jsonify
from database import db_session
from app.models import Order, OrderItem
from datetime import datetime

analytics_bp = Blueprint('analytics', __name__, url_prefix='/analytics')

@analytics_bp.route('/sales', methods=['GET'])
def get_sales_data():
    # Daily sales
    daily_sales = db_session.execute("""
        SELECT DATE(created_at) as day, SUM(total) 
        FROM orders 
        WHERE status = 'delivered'
        GROUP BY day
    """).fetchall()
    
    # Popular items
    popular_items = db_session.execute("""
        SELECT m.name, SUM(oi.quantity) as total 
        FROM order_items oi
        JOIN menu m ON oi.item_id = m.id
        GROUP BY m.name
        ORDER BY total DESC
        LIMIT 5
    """).fetchall()
    
    return jsonify({
        'daily_sales': {str(day): float(total) for day, total in daily_sales},
        'popular_items': [{'item': item, 'total': total} for item, total in popular_items]
    })