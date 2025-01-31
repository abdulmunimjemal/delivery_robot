document.addEventListener('DOMContentLoaded', () => {
    let currentOrders = [];
    const container = document.getElementById('orders-container');
    const stats = {
        pending: document.getElementById('stat-pending'),
        preparing: document.getElementById('stat-preparing'),
        ready: document.getElementById('stat-ready')
    };

    const updateOrders = async () => {
        try {
            const response = await fetch('/kitchen/orders');
            if (!response.ok) throw new Error('Network error');
            
            const orders = await response.json();
            currentOrders = orders;
            renderOrders();
            updateStats();
        } catch (error) {
            showError(`Error: ${error.message}`);
        }
    };

    const renderOrders = () => {
        container.innerHTML = currentOrders.map(order => `
            <div class="order-card ${order.status}">
                <div class="order-header">
                    <h3>Order #${order.id} <span class="table-number">(Table ${order.table})</span></h3>
                    <div class="order-meta">
                        <span class="status-badge ${order.status}">${order.status.toUpperCase()}</span>
                        <span class="timer"><i class="fas fa-clock"></i> ${timeSince(order.created_at)}</span>
                    </div>
                </div>
                
                <div class="order-body">
                    <div class="items-list">
                        ${order.items.map(item => `
                            <div class="item">
                                <span class="quantity">${item.quantity}x</span>
                                <span class="name">${item.name}</span>
                            </div>
                        `).join('')}
                    </div>
                    
                    <div class="order-actions">
                        ${order.status === 'pending' ? `
                            <button class="btn confirm" onclick="handleAction(${order.id}, 'confirm')">
                                <i class="fas fa-check"></i> Confirm
                            </button>
                        ` : ''}
                        
                        ${order.status === 'confirmed' ? `
                            <button class="btn prepare" onclick="handleAction(${order.id}, 'prepare')">
                                <i class="fas fa-blender"></i> Start Preparing
                            </button>
                        ` : ''}


                        ${order.status === 'preparing' ? `
                            <button class="btn btn-ready" onclick="handleAction(${order.id}, 'ready')">
                                <i class="fas fa-check-double"></i> Ready
                            </button>
                        ` : ''}
                        
                        <button class="btn cancel" onclick="handleAction(${order.id}, 'cancel')">
                            <i class="fas fa-times"></i> Cancel
                        </button>
                    </div>
                </div>
            </div>
        `).join('');
    };

    const updateStats = () => {
        stats.pending.textContent = currentOrders.filter(o => o.status === 'pending').length;
        stats.preparing.textContent = currentOrders.filter(o => o.status === 'preparing').length;
        stats.ready.textContent = currentOrders.filter(o => o.status === 'ready').length;
    };

    const showError = (message) => {
        container.innerHTML = `
            <div class="error">
                <i class="fas fa-exclamation-triangle"></i> ${message}
                <button onclick="updateOrders()">Retry</button>
            </div>
        `;
    };

    // Initial load
    updateOrders();
    setInterval(updateOrders, 10000);
});

async function handleAction(orderId, action) {
    try {
        let endpoint;
        switch(action) {
            case 'confirm':
                endpoint = 'confirm';
                break;
            case 'prepare':
                endpoint = 'complete';  // Same endpoint but progresses status
                break;
            case 'ready':
                endpoint = 'ready';
                break;
            case 'cancel':
                endpoint = 'cancel';
                break;
            default:
                throw new Error('Invalid action');
        }

        const response = await fetch(`/kitchen/orders/${orderId}/${endpoint}`, {
            method: 'POST'
        });

        if (!response.ok) throw new Error('Action failed');
        
        const result = await response.json();
        // console.log(result.message);
        await showToast(result.message, "success");
    } catch (error) {
        console.error(`Error handling ${action}:`, error);
        // alert(`Action failed: ${error.message}`);
        await showToast(`Action failed: ${error.message}`, "error");
    }
}

// Time formatting helper
function timeSince(date) {
    const seconds = Math.floor((new Date() - date) / 1000);
    const intervals = {
        year: 31536000,
        month: 2592000,
        day: 86400,
        hour: 3600,
        minute: 60
    };

    for (const [unit, secondsInUnit] of Object.entries(intervals)) {
        const count = Math.floor(seconds / secondsInUnit);
        if (count > 0) {
            return `${count} ${unit}${count !== 1 ? 's' : ''} ago`;
        }
    }
    return 'Just now';
}

async function handleOrderAction(orderId, action) {
    try {
        const endpoint = {
            confirm: 'confirm',
            cancel: 'cancel',
            ready: 'ready',
            preparing: 'confirm' // Same endpoint but status changes
        }[action];

        const response = await fetch(`/kitchen/orders/${orderId}/${endpoint}`, {
            method: 'POST'
        });

        if (!response.ok) throw new Error('Action failed');
        
        const result = await response.json();
        await showToast(result.message, "success");
        
    } catch (error) {
        console.error(`Error ${action} order:`, error);
        // alert(`Failed to ${action} order: ${error.message}`);
        await showToast(`Failed to ${action} order: ${error.message}`, "error");
    }
}


async function showToast(message, type = "success") {
    return new Promise((resolve) => {
        Toastify({
            text: message,
            duration: 1000, // Toast lasts for 5 seconds
            gravity: "top", // Appears at the top
            position: "right",
            backgroundColor: type === "success" ? "linear-gradient(to right, #00b09b, #96c93d)" 
                                                : "linear-gradient(to right, #ff416c, #ff4b2b)",
            stopOnFocus: true, // Stop timeout on hover
            className: "cool-toast",
            style: {
                borderRadius: "12px",
                padding: "12px 18px",
                fontSize: "16px",
                fontWeight: "bold",
                color: "#fff",
                boxShadow: "0px 4px 10px rgba(0, 0, 0, 0.2)",
            },
            callback: () => resolve() // Resolve when toast disappears
        }).showToast();
    });
}
