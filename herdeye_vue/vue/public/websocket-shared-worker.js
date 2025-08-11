// SharedWorker to manage shared WebSocket connection
let socket;
let connections = [];

// Establish WebSocket connection
function connect() {
  // Replace with your WebSocket server URL
  const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsHost = window.location.host;
  socket = new WebSocket(`${wsProtocol}//${wsHost}/ws`);

  socket.onopen = () => {
    console.log('Shared WebSocket connection established');
    broadcastMessage({ type: 'CONNECTED', data: 'WebSocket connection established' });
  };

  socket.onmessage = (event) => {
    try {
      const message = JSON.parse(event.data);
      broadcastMessage(message);
    } catch (error) {
      console.error('Error parsing WebSocket message:', error);
    }
  };

  socket.onerror = (error) => {
    console.error('WebSocket error:', error);
    broadcastMessage({ type: 'ERROR', data: error.message });
  };

  socket.onclose = (event) => {
    console.log('WebSocket connection closed. Reconnecting...');
    broadcastMessage({ type: 'DISCONNECTED', data: 'Connection closed' });
    // Try to reconnect after 3 seconds
    setTimeout(connect, 3000);
  };
}

// Broadcast message to all connected clients
function broadcastMessage(message) {
  connections.forEach(port => {
    if (port) {
      port.postMessage(message);
    }
  });
}

// Handle connections from different browser tabs
self.onconnect = (event) => {
  const port = event.ports[0];
  connections.push(port);

  // Initialize connection if not already connected
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    connect();
  } else {
    // Notify new client about existing connection
    port.postMessage({ type: 'ALREADY_CONNECTED', data: 'Using existing WebSocket connection' });
  }

  // Handle messages from client
  port.onmessage = (event) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(event.data));
    } else {
      port.postMessage({ type: 'ERROR', data: 'WebSocket connection not established' });
    }
  };

  // Remove port when connection is closed
  port.onclose = () => {
    connections = connections.filter(p => p !== port);
    // Close WebSocket if no clients left
    if (connections.length === 0 && socket) {
      socket.close();
      socket = null;
      console.log('All clients disconnected, closing WebSocket');
    }
  };
};