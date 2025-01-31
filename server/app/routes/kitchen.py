from flask import Blueprint, jsonify, request, render_template
from database import db_session
from app.models import Order
import requests

kitchen_bp = Blueprint('kitchen', __name__, url_prefix='/kitchen')

@kitchen_bp.route('/')
def kitchen_dashboard():
    return render_template('kitchen.html')


@kitchen_bp.route('/orders', methods=['GET'])
def get_active_orders():
    orders = Order.query.filter(Order.status.in_(['pending', 'confirmed', 'preparing'])).all()
    return jsonify([{
        'id': order.id,
        'table': order.table_number,
        'status': order.status,
        'total': order.total,
        'items': [{
            'name': item.menu_item.name,
            'quantity': item.quantity
        } for item in order.items],
        'created_at': order.created_at.isoformat()
    } for order in orders])

@kitchen_bp.route('/orders/<int:order_id>/confirm', methods=['POST'])
def confirm_order(order_id):
    order = Order.query.get(order_id)
    if not order:
        return jsonify({'error': 'Order not found'}), 404
    
    if order.status != 'pending':
        return jsonify({'error': 'Order not in pending state'}), 400
    
    order.status = 'confirmed'
    db_session.commit()
    return jsonify({'message': f'Order {order_id} confirmed'})

@kitchen_bp.route('/orders/<int:order_id>/complete', methods=['POST'])
def complete_order(order_id):
    order = Order.query.get(order_id)
    if not order:
        return jsonify({'error': 'Order not found'}), 404
    
    valid_transitions = {
        'preparing': 'ready',
        'confirmed': 'preparing',
        'pending': 'confirmed'
    }
    
    if order.status not in valid_transitions:
        return jsonify({'error': 'Invalid order status for completion'}), 400
    
    order.status = valid_transitions[order.status]
    db_session.commit()
    return jsonify({'message': f'Order {order_id} status updated to {order.status}'})

@kitchen_bp.route('/orders/<int:order_id>/cancel', methods=['POST'])
def cancel_order(order_id):
    order = Order.query.get(order_id)
    if not order:
        return jsonify({'error': 'Order not found'}), 404
    
    if order.status not in ['pending', 'confirmed', 'preparing']:
        return jsonify({'error': 'Cannot cancel order in current state'}), 400
    
    order.status = 'cancelled'
    db_session.commit()
    return jsonify({'message': f'Order {order_id} has been cancelled'})

@kitchen_bp.route('/orders/<int:order_id>/ready', methods=['POST'])
def mark_order_ready(order_id):
    session = db_session()
    try:
        order = session.query(Order).get(order_id)
        if not order:
            return jsonify({'error': 'Order not found'}), 404
        
        if order.status != 'preparing':
            return jsonify({'error': 'Order not in preparing state'}), 400
        
        order.status = 'ready'
        session.commit()

        response = requests.post('http://localhost:5000/robot/tasks/process', json={'order_id': order_id})
        
        if response.status_code != 200:
            return jsonify({'error': 'Failed to process robot delivery task', 'details': response.text}), response.status_code

        return jsonify({'message': f'Order {order_id} marked as ready for delivery'})
    finally:
        pass