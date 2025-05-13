####################################################
# LSrouter.py
# Name: 23020666
# HUID: Đào Huy Hoàng
#####################################################


import json
import heapq
from router import Router


class LSrouter(Router):
    """Link state routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the `Router` base class for docstrings of the methods to
    override.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        # TODO
        self.link_state = {}  # Trạng thái liên kết cục bộ: {hàng xóm: chi phí}
        self.neighbor_ports = {}  # {neighbor: port}
        self.link_state_db = {}  # Cơ sở dữ liệu trạng thái: {router: (sequence, state)}
        self.sequence_num = 0  # Số thứ tự ban đầu
        self.forwarding_table = {}  # Bảng chuyển tiếp: {đích: cổng}
        pass

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        # TODO
        if packet.is_traceroute:
            # Xử lý gói tin dữ liệu (traceroute)
            if packet.dst_addr in self.forwarding_table:
                self.send(self.forwarding_table[packet.dst_addr], packet)
        else:
            # Xử lý gói tin định tuyến (Packet.ROUTING)
            data = json.loads(packet.content)
            src_addr = packet.src_addr
            seq = data['sequence']
            state = data['state']

            # Cập nhật nếu sequence number lớn hơn
            if src_addr not in self.link_state_db or self.link_state_db[src_addr][0] < seq:
                self.link_state_db[src_addr] = (seq, state)
                # Phát quảng bá đến các hàng xóm (trừ cổng nhận)
                for p in self.links:
                    if p != port:
                        self.send(p, packet)
                # Cập nhật bảng chuyển tiếp
                self._update_routing_table()

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # TODO
        #   update local data structures and forwarding table
        #   broadcast the new link state of this router to all neighbors
        self.link_state[endpoint] = cost
        self.neighbor_ports[endpoint] = port
        self.sequence_num += 1
        self._broadcast_link_state()
        self._update_routing_table()
        pass

    def handle_remove_link(self, port):
        """Handle removed link."""
        # TODO
        #   update local data structures and forwarding table
        #   broadcast the new link state of this router to all neighbors
        endpoint = None
        for neighbor, p in self.neighbor_ports.items():
            if p == port:
                endpoint = neighbor
                break

        if endpoint:
            del self.link_state[endpoint]
            del self.neighbor_ports[endpoint]
            self.sequence_num += 1
            self._broadcast_link_state()
            self._update_routing_table()
        pass

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            # TODO
            #   broadcast the link state of this router to all neighbors
            self._broadcast_link_state()
            pass

    def _broadcast_link_state(self):
        packet_content = json.dumps({
            'sequence': self.sequence_num,
            'state': self.link_state
        })

        from packet import Packet  # import ở đây để tránh import vòng
        pkt = Packet(Packet.ROUTING, self.addr, None, packet_content)

        for port in self.links:
            self.send(port, pkt)

        self.link_state_db[self.addr] = (self.sequence_num, self.link_state.copy())

    def _update_routing_table(self):
        # Dijkstra's algorithm
        graph = {}
        for router, (_, neighbors) in self.link_state_db.items():
            graph[router] = neighbors

        dist = {self.addr: 0}
        prev = {}
        visited = set()
        heap = [(0, self.addr)]

        while heap:
            d, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)

            for v in graph.get(u, {}):
                alt = d + graph[u][v]
                if v not in dist or alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(heap, (alt, v))

        self.forwarding_table = {}
        for dest in dist:
            if dest == self.addr:
                continue
            next_hop = dest
            while prev.get(next_hop) != self.addr:
                next_hop = prev[next_hop]
            if next_hop in self.neighbor_ports:
                self.forwarding_table[dest] = self.neighbor_ports[next_hop]



    def __repr__(self):
        """Representation for debugging in the network visualizer."""
        # TODO
        #   NOTE This method is for your own convenience and will not be graded
        return f"LSrouter(addr={self.addr})"

