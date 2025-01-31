from app import create_app
from database_seeder import seed_database

app = create_app()

if __name__ == '__main__':
    seed_database()
    app.run(host='0.0.0.0', port=5000, debug=True)