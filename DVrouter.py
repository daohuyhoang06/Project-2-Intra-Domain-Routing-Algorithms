import json
from router import Router

class DVrouter(Router):
    """Distance vector routing protocol implementation."""

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        # Forwarding table: dest -> (cost, port)
        self.forwarding_table = {}
        # Distance vector: dest -> cost
        self.distance_vector = {}
        # Neighbors: port -> (addr, cost)
        self.neighbors = {}
        # Distance vectors from neighbors: neighbor_addr -> {dest: cost}
        self.neighbor_vectors = {}

        # Initialize own distance vector with cost 0 to self
        self.distance_vector[self.addr] = 0
        self.forwarding_table[self.addr] = (0, None)

    def send_distance_vector(self):
        """Send this router's distance vector to all neighbors."""
        for port in self.neighbors:
            pkt = self.create_routing_packet(self.distance_vector)
            self.send(port, pkt)

    def create_routing_packet(self, vector):
        """Create a routing packet carrying the distance vector."""
        from packet import Packet
        vector_str = json.dumps(vector)  # Convert dict to JSON string
        return Packet(Packet.ROUTING, self.addr, None, content=vector_str)

    def update_forwarding_table(self):
        """Recalculate the forwarding table based on neighbor vectors."""
        new_distance_vector = {self.addr: 0}
        new_forwarding_table = {self.addr: (0, None)}

        # Consider all known destinations from neighbors and self
        all_dests = set()
        all_dests.add(self.addr)
        for vec in self.neighbor_vectors.values():
            all_dests.update(vec.keys())

        for dest in all_dests:
            if dest == self.addr:
                continue
            best_cost = float('inf')
            best_port = None
            for port, (neighbor, cost_to_neighbor) in self.neighbors.items():
                neighbor_vector = self.neighbor_vectors.get(neighbor, {})
                cost_via_neighbor = cost_to_neighbor + neighbor_vector.get(dest, float('inf'))
                if cost_via_neighbor < best_cost:
                    best_cost = cost_via_neighbor
                    best_port = port
            if best_cost < float('inf'):
                new_distance_vector[dest] = best_cost
                new_forwarding_table[dest] = (best_cost, best_port)

        changed = new_distance_vector != self.distance_vector
        self.distance_vector = new_distance_vector
        self.forwarding_table = new_forwarding_table
        return changed

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            # Normal data packet - forward according to forwarding table
            if packet.dst_addr in self.forwarding_table:
                _, next_port = self.forwarding_table[packet.dst_addr]
                if next_port is not None:
                    self.send(next_port, packet)
        elif packet.is_routing:
            # Routing packet - contains distance vector from neighbor
            vector_str = packet.content
            vector = json.loads(vector_str)  # Convert JSON string back to dict
            self.neighbor_vectors[packet.src_addr] = vector
            if self.update_forwarding_table():
                self.send_distance_vector()

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link creation."""
        self.neighbors[port] = (endpoint, cost)
        # Initial distance vector for new neighbor: cost 0 to itself
        self.neighbor_vectors[endpoint] = {endpoint: 0}
        if self.update_forwarding_table():
            self.send_distance_vector()

    def handle_remove_link(self, port):
        """Handle link removal."""
        if port in self.neighbors:
            neighbor, _ = self.neighbors.pop(port)
            self.neighbor_vectors.pop(neighbor, None)
            if self.update_forwarding_table():
                self.send_distance_vector()

    def handle_time(self, time_ms):
        """Periodic tasks, send distance vector every heartbeat_time ms."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.send_distance_vector()

    def __repr__(self):
        """String representation for debugging."""
        return f"DVrouter(addr={self.addr}, dv={self.distance_vector})"
