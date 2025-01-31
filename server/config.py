import os

class Config:
    SECRET_KEY = os.environ.get('SECRET_KEY') or 'supersecretkey123'
    SQLALCHEMY_DATABASE_URI = 'sqlite:///delivery.db'
    SQLALCHEMY_TRACK_MODIFICATIONS = False
    ITEMS_PER_PAGE = 10