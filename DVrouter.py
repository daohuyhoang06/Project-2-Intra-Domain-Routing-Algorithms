####################################################
# DVrouter.py
# Name: 23020667
# HUID: Nguyễn Văn Hoàng
#####################################################


from router import Router
from collections import defaultdict
import copy

INFINITY = 999999

class DVrouter(Router):
    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)
        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        # Distance vector: {destination: cost}
        self.dv = {}
        # Forwarding table: {destination: (port, cost)}
        self.forwarding_table = {}
        # Neighbor info: {port: (neighbor_addr, cost)}
        self.neighbors = {}
        # Distance vector từ các hàng xóm: {neighbor_addr: their_vector}
        self.neighbor_dv = {}

    def send_distance_vector(self):
        for port, (neighbor, _) in self.neighbors.items():
            poisoned_dv = {}
            for dest, cost in self.dv.items():
                # Poisoned reverse: nếu đang gửi đến neighbor mà route đến dest đi qua nó thì báo ∞
                if dest in self.forwarding_table and self.forwarding_table[dest][0] == port:
                    poisoned_dv[dest] = INFINITY
                else:
                    poisoned_dv[dest] = cost
            pkt = self.make_routing_packet(poisoned_dv)
            self.send(port, pkt)

    def make_routing_packet(self, vector):
        pkt = self.create_packet()
        pkt.is_traceroute = False
        pkt.dst_addr = None
        pkt.src_addr = self.addr
        pkt.vector = vector
        return pkt

    def handle_packet(self, port, packet):
        if packet.is_traceroute:
            dst = packet.dst_addr
            if dst in self.forwarding_table:
                out_port = self.forwarding_table[dst][0]
                self.send(out_port, packet)
        else:
            # Cập nhật DV nhận từ hàng xóm
            neighbor = self.neighbors.get(port, (None,))[0]
            if neighbor is None:
                return
            vector = packet.vector
            self.neighbor_dv[neighbor] = vector

            changed = False
            for dest in vector:
                if dest == self.addr:
                    continue
                new_cost = self.neighbors[port][1] + vector[dest]
                # Cập nhật nếu tốt hơn
                if dest not in self.dv or new_cost < self.dv[dest]:
                    self.dv[dest] = new_cost
                    self.forwarding_table[dest] = (port, new_cost)
                    changed = True

            if changed:
                self.send_distance_vector()

    def handle_new_link(self, port, endpoint, cost):
        self.neighbors[port] = (endpoint, cost)
        self.dv[endpoint] = cost
        self.forwarding_table[endpoint] = (port, cost)
        self.send_distance_vector()

    def handle_remove_link(self, port):
        if port in self.neighbors:
            lost_neighbor = self.neighbors[port][0]
            del self.neighbors[port]
            self.neighbor_dv.pop(lost_neighbor, None)

        # Xóa các route dùng port đó
        to_delete = [dest for dest in self.forwarding_table if self.forwarding_table[dest][0] == port]
        for dest in to_delete:
            self.dv[dest] = INFINITY
            del self.forwarding_table[dest]

        self.send_distance_vector()

    def handle_time(self, time_ms):
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.send_distance_vector()

    def __repr__(self):
        return f"DVrouter(addr={self.addr})"
