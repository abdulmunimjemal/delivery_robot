from flask import Blueprint, jsonify, request, render_template
from database import db_session
from app.models import Menu, Order, OrderItem

user_bp = Blueprint('user', __name__, url_prefix='/user')

@user_bp.route('/')
def user_interface():
    return render_template('user.html')

@user_bp.route('/menu', methods=['GET'])
def get_menu():
    menu_items = Menu.query.all()
    return jsonify([{
        'id': item.id,
        'name': item.name,
        'price': item.price,
        'category': item.category,
        'preparation_time': item.preparation_time
    } for item in menu_items])


@user_bp.route('/tables/<int:table_number>/orders')
def get_table_orders(table_number):
    orders = db_session.query(Order).filter(
        Order.table_number == table_number,
        Order.status.in_(['pending', 'confirmed', 'preparing', 'ready', 'in_delivery'])
    ).all()
    
    result = [{
        'id': o.id,
        'status': o.status,
        'total': o.total,
        'items': [{
            'name': item.menu_item.name,
            'quantity': item.quantity
        } for item in o.items]
    } for o in orders]
    
    return jsonify(result)

@user_bp.route('/orders', methods=['POST'])
def create_order():
    data = request.get_json()
    
    if not data or 'table' not in data or 'items' not in data:
        return jsonify({'error': 'Invalid request format'}), 400
    
    try:
        total = 0
        order_items = []
        
        # Validate items
        for item in data['items']:
            menu_item = Menu.query.get(item.get('id'))
            if not menu_item:
                return jsonify({'error': f'Invalid menu item ID: {item.get("id")}'}), 400
            if item.get('quantity', 0) < 1:
                return jsonify({'error': 'Invalid quantity'}), 400
            
            total += menu_item.price * item['quantity']
            order_items.append({
                'item_id': menu_item.id,
                'quantity': item['quantity']
            })
        
        # Create order
        new_order = Order(
            table_number=data['table'],
            total=total,
            status='pending'
        )
        db_session.add(new_order)
        db_session.flush()
        
        # Add order items
        for item in order_items:
            db_session.add(OrderItem(
                order_id=new_order.id,
                item_id=item['item_id'],
                quantity=item['quantity']
            ))
        
        db_session.commit()
        return jsonify({
            'message': 'Order created',
            'order_id': new_order.id,
            'total': total
        }), 201
    
    except Exception as e:
        db_session.rollback()
        return jsonify({'error': str(e)}), 500