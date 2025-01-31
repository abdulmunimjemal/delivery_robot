from database import db_session, init_db
from app.models import Menu

def seed_database():
    init_db()
    
    if db_session.query(Menu).count() == 0:
        menu_items = [
            Menu(
                name="Margherita Pizza",
                price=12.99,
                category="Pizza",
                preparation_time=15,
                description="Classic tomato and mozzarella",
                allergens="Dairy, Gluten"
            ),
            Menu(
                name="Pepperoni Pizza",
                price=14.99,
                category="Pizza",
                preparation_time=18,
                description="Spicy pepperoni with extra cheese",
                allergens="Dairy, Gluten"
            ),
            Menu(
                name="Sparkling Water",
                price=3.50,
                category="Drinks",
                preparation_time=2,
                description="Natural mineral water",
                allergens=None
            )
        ]
        
        db_session.bulk_save_objects(menu_items)
        db_session.commit()
        print("Database seeded successfully!")
    else:
        print("Database already contains data - skipping seeding")

if __name__ == '__main__':
    seed_database()