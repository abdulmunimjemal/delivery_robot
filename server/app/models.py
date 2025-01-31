from database import Base
from sqlalchemy import Column, Integer, String, Float, DateTime, ForeignKey
from sqlalchemy.orm import relationship
from datetime import datetime

class Menu(Base):
    __tablename__ = 'menu'
    id = Column(Integer, primary_key=True)
    name = Column(String(100), unique=True, nullable=False)
    price = Column(Float, nullable=False)
    category = Column(String(50))
    preparation_time = Column(Integer)  # in minutes
    description = Column(String(200))
    allergens = Column(String(100))

class Order(Base):
    __tablename__ = 'orders'
    id = Column(Integer, primary_key=True)
    table_number = Column(Integer, nullable=False)
    status = Column(String(20), default='pending', nullable=False)
    total = Column(Float, nullable=False)
    created_at = Column(DateTime, default=datetime.now)
    items = relationship('OrderItem', backref='order')

class OrderItem(Base):
    __tablename__ = 'order_items'
    id = Column(Integer, primary_key=True)
    order_id = Column(Integer, ForeignKey('orders.id'), nullable=False)
    item_id = Column(Integer, ForeignKey('menu.id'), nullable=False)
    quantity = Column(Integer, nullable=False)
    menu_item = relationship('Menu', backref='order_items')