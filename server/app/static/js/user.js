document.addEventListener('DOMContentLoaded', () => {
    let cart = [];
    let selectedTable = null;
    const menuContainer = document.getElementById('menu-items');
    const cartContainer = document.getElementById('cart-items');
    const cartTotal = document.getElementById('cart-total');
    const placeOrderBtn = document.getElementById('place-order');
    const orderMessage = document.getElementById('order-message');
    const existingOrdersContainer = document.getElementById('existing-orders-list');

    // Table Selection Handler
    document.querySelectorAll('.table-btn').forEach(button => {
        button.addEventListener('click', (e) => {
            selectedTable = e.target.dataset.table;
            document.getElementById('table-selection').style.display = 'none';
            document.getElementById('main-interface').style.display = 'block';
            document.getElementById('selected-table').textContent = selectedTable;
            loadExistingOrders();
            loadMenu();
        });
    });

    // Load Menu Items
    async function loadMenu() {
        try {
            const response = await fetch('/user/menu');
            if (!response.ok) throw new Error('Menu loading failed');
            
            const menuItems = await response.json();
            renderMenu(menuItems);
        } catch (error) {
            showMessage(`Menu Error: ${error.message}`, 'error');
        }
    }

    function renderMenu(menuItems) {
        menuContainer.innerHTML = menuItems.map(item => `
            <div class="menu-item">
                <h3>${item.name}</h3>
                <p class="price">${item.price.toFixed(2)} ETB</p>
                ${item.description ? `<p class="description">${item.description}</p>` : ''}
                <button class="add-to-cart" data-id="${item.id}">Add to Cart</button>
            </div>
        `).join('');
    }

    // Load Existing Orders
    async function loadExistingOrders() {
        try {
            const response = await fetch(`/user/tables/${selectedTable}/orders`);
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            
            const orders = await response.json();
            renderExistingOrders(orders);
        } catch (error) {
            existingOrdersContainer.innerHTML = `
                <div class="error">
                    Failed to load orders: ${error.message}
                    <button onclick="loadExistingOrders()">Retry</button>
                </div>
            `;
        }
    }

    function renderExistingOrders(orders) {
        existingOrdersContainer.innerHTML = orders.map(order => `
            <div class="existing-order">
                <div class="order-header">
                    <h3>Order #${order.id}</h3>
                    <span class="order-status status-${order.status}">
                        ${order.status.toUpperCase()}
                    </span>
                </div>
                <div class="order-details">
                    <p class="order-total">Total: ${order.total.toFixed(2)} ETB</p>
                    <div class="order-items">
                        ${order.items.map(item => `
                            <div class="order-item">
                                <span class="item-quantity">${item.quantity}x</span>
                                <span class="item-name">${item.name}</span>
                            </div>
                        `).join('')}
                    </div>
                </div>
            </div>
        `).join('');
    }

    // Cart Management
    menuContainer.addEventListener('click', (e) => {
        if (e.target.classList.contains('add-to-cart')) {
            const itemId = parseInt(e.target.dataset.id);
            const menuItem = e.target.closest('.menu-item');
            const itemName = menuItem.querySelector('h3').textContent;
            const price = parseFloat(menuItem.querySelector('.price').textContent.replace(' ETB', ''));
            
            addToCart(itemId, itemName, price);
        }
    });

    function addToCart(itemId, name, price) {
        const existingItem = cart.find(item => item.id === itemId);
        if (existingItem) {
            existingItem.quantity++;
        } else {
            cart.push({ id: itemId, name, price, quantity: 1 });
        }
        updateCartDisplay();
    }

    cartContainer.addEventListener('click', (e) => {
        if (e.target.classList.contains('remove-item')) {
            const itemId = parseInt(e.target.dataset.id);
            cart = cart.filter(item => item.id !== itemId);
            updateCartDisplay();
        }
    });

    function updateCartDisplay() {
        cartContainer.innerHTML = cart.map(item => `
            <div class="cart-item">
                <div>
                    <h4>${item.name}</h4>
                    <p>${item.price.toFixed(2)} ETB × ${item.quantity}</p>
                </div>
                <div class="cart-item-controls">
                    <span>${(item.price * item.quantity).toFixed(2)} ETB</span>
                    <button class="remove-item" data-id="${item.id}">Remove</button>
                </div>
            </div>
        `).join('');

        const total = cart.reduce((sum, item) => sum + (item.price * item.quantity), 0);
        cartTotal.textContent = total.toFixed(2);
        placeOrderBtn.disabled = cart.length === 0;
    }

    // Order Submission
    placeOrderBtn.addEventListener('click', async () => {
        orderMessage.style.display = 'none';
        
        if (!selectedTable) {
            showMessage('Please select a table first!', 'error');
            return;
        }

        if (cart.length === 0) {
            showMessage('Your cart is empty!', 'error');
            return;
        }

        const orderData = {
            table: parseInt(selectedTable),
            items: cart.map(item => ({
                id: item.id,
                quantity: item.quantity
            }))
        };

        try {
            placeOrderBtn.disabled = true;
            placeOrderBtn.textContent = 'Placing Order...';

            const response = await fetch('/user/orders', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(orderData)
            });

            if (!response.ok) {
                const errorData = await response.json();
                throw new Error(errorData.error || 'Order failed');
            }

            const data = await response.json();
            showMessage(`Order placed! Total: ${data.total.toFixed(2)} ETB`, 'success');
            
            cart = [];
            updateCartDisplay();
            loadExistingOrders();
        } catch (error) {
            showMessage(error.message, 'error');
        } finally {
            placeOrderBtn.disabled = false;
            placeOrderBtn.textContent = 'Place Order';
        }
    });

    function showMessage(text, type) {
        orderMessage.textContent = text;
        orderMessage.className = `${type}-message`;
        orderMessage.style.display = 'block';
        setTimeout(() => {
            orderMessage.style.display = 'none';
        }, 5000);
    }
});