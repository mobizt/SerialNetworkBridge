import serial
import time
import socket
import threading
import struct
import queue
import ssl
from cobs import cobs
import websocket  # pip install websocket-client

# ==========================================
#  CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM3'   # Update this!
BAUD_RATE = 115200

# ==========================================
#  PROTOCOL CONSTANTS
# ==========================================
# Global
GLOBAL_SLOT_ID         = 0xFF
CMD_C_SET_DEBUG        = 0x05
CMD_C_PING_HOST        = 0x06
CMD_C_REBOOT_HOST      = 0x07

# TCP
CMD_C_CONNECT_HOST     = 0x10
CMD_C_WRITE            = 0x11
CMD_C_STOP             = 0x13
CMD_C_IS_CONNECTED     = 0x14
CMD_C_DATA_ACK         = 0x15
CMD_C_START_TLS        = 0x16
CMD_C_SET_CA_CERT      = 0x17
CMD_C_POLL_DATA        = 0x18

# UDP
CMD_C_UDP_BEGIN        = 0x20
CMD_C_UDP_END          = 0x21
CMD_C_UDP_BEGIN_PACKET = 0x22
CMD_C_UDP_WRITE_DATA   = 0x23
CMD_C_UDP_END_PACKET   = 0x24
CMD_C_UDP_PARSE_PACKET = 0x25

# WebSocket
CMD_C_WS_CONNECT       = 0x30
CMD_C_WS_SEND_FRAME    = 0x31
CMD_C_WS_DISCONNECT    = 0x32
CMD_C_WS_LOOP          = 0x33

# Responses
CMD_H_ACK              = 0x80
CMD_H_NAK              = 0x81
CMD_H_PING_RESPONSE    = 0x86
CMD_H_HOST_RESET       = 0x87
CMD_H_CONNECTED_STATUS = 0x94
CMD_H_DATA_PAYLOAD     = 0x95
CMD_H_POLL_RESPONSE    = 0x96
CMD_H_UDP_PACKET_INFO  = 0xA0
CMD_H_UDP_DATA_PAYLOAD = 0xA1
CMD_H_WS_EVENT         = 0xB0

# WS Types
WS_EVENT_DISCONNECTED  = 0x00
WS_EVENT_CONNECTED     = 0x01
WS_FRAME_TEXT          = 0x02
WS_FRAME_BINARY        = 0x03
WS_FRAME_PONG          = 0x04
WS_EVENT_ERROR         = 0xFF

MAX_SLOTS = 4
FRAME_DELIMITER = b'\x00'

# ==========================================
#  CRC16 (Modbus)
# ==========================================
def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

# ==========================================
#  HOST IMPLEMENTATION
# ==========================================
class PySerialNetworkHost:
    def __init__(self, port, baud_rate):
        self.ser = serial.Serial(port, baud_rate, timeout=0.01)
        self.lock = threading.Lock()
        self.running = True
        
        # TCP State
        self.tcp_slots = [None] * MAX_SLOTS
        self.tcp_ca_certs = [None] * MAX_SLOTS 
        # Events for Flow Control (Stop-and-Wait)
        self.tcp_ack_events = [threading.Event() for _ in range(MAX_SLOTS)]
        
        # UDP State
        self.udp_sockets = [None] * MAX_SLOTS
        self.udp_rx_queues = [queue.Queue() for _ in range(MAX_SLOTS)]
        self.udp_tx_buffers = [None] * MAX_SLOTS
        
        # WebSocket State
        self.ws_clients = [None] * MAX_SLOTS

        self.session_id = int(time.time() * 1000) & 0xFFFF 
        if self.session_id == 0: self.session_id = 1
        
        print(f"Bridge started on {port} @ {baud_rate}. Session: {self.session_id:04X}")
        self.notify_boot()

    def notify_boot(self):
        payload = struct.pack('>H', self.session_id)
        self.send_packet(CMD_H_HOST_RESET, GLOBAL_SLOT_ID, payload)

    def send_packet(self, cmd, slot, payload=b''):
        raw_packet = bytes([cmd, slot]) + payload
        crc = calculate_crc16(raw_packet)
        crc_bytes = struct.pack('<H', crc)
        encoded = cobs.encode(raw_packet + crc_bytes) + FRAME_DELIMITER
        
        with self.lock:
            self.ser.write(encoded)
            # self.ser.flush() 

    # --------------------------------------------------------------------------
    # TCP Logic
    # --------------------------------------------------------------------------
    def tcp_rx_thread(self, slot_id, sock):
        """
        Reads from internet, sends to Arduino, and WAITS for Arduino ACK.
        This mimics the 'WAIT_FOR_ACK' state in SerialNetworkHost.h
        """
        while self.tcp_slots[slot_id] == sock and self.running:
            try:
                # Read chunks suited for Arduino buffer
                data = sock.recv(64) 
                if not data: break
                
                # 1. Clear previous ACK
                self.tcp_ack_events[slot_id].clear()
                
                # 2. Send Data
                self.send_packet(CMD_H_DATA_PAYLOAD, slot_id, data)
                
                # 3. Wait for ACK (Flow Control)
                # Arduino Client sends CMD_C_DATA_ACK after processing
                ack_received = self.tcp_ack_events[slot_id].wait(timeout=2.0)
                
                if not ack_received:
                    print(f"Slot {slot_id} Timeout waiting for ACK. Dropping packet.")
                    # In a full retry system, we would resend here.
                    
            except socket.timeout: continue
            except OSError: break
            except Exception as e:
                print(f"TCP RX Error: {e}")
                break
        
        self.close_tcp(slot_id)

    def close_tcp(self, slot):
        if self.tcp_slots[slot]:
            try: self.tcp_slots[slot].close()
            except: pass
            self.tcp_slots[slot] = None
            self.send_packet(CMD_H_CONNECTED_STATUS, slot, b'\x00')
            print(f"TCP Slot {slot} Closed")

    def wrap_socket_ssl(self, sock, hostname, ca_cert=None):
        context = ssl.create_default_context()
        if ca_cert:
            try:
                context.load_verify_locations(cafile=ca_cert)
            except Exception as e:
                print(f"SSL Cert Error: {e}")
        else:
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        return context.wrap_socket(sock, server_hostname=hostname)

    # --------------------------------------------------------------------------
    # UDP Logic
    # --------------------------------------------------------------------------
    def udp_rx_thread(self, slot_id, sock):
        while self.udp_sockets[slot_id] == sock and self.running:
            try:
                data, addr = sock.recvfrom(512)
                self.udp_rx_queues[slot_id].put((data, addr[0], addr[1]))
            except socket.timeout: continue
            except: break

    # --------------------------------------------------------------------------
    # WebSocket Logic
    # --------------------------------------------------------------------------
    def start_ws(self, slot, host, port, path, use_ssl):
        scheme = "wss" if use_ssl else "ws"
        url = f"{scheme}://{host}:{port}{path}"
        
        def on_message(ws, message):
            if isinstance(message, str):
                self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_FRAME_TEXT]) + message.encode('utf-8'))
            else:
                self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_FRAME_BINARY]) + message)

        def on_error(ws, error):
            print(f"WS Error {slot}: {error}")
            self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_EVENT_ERROR]))

        def on_close(ws, close_status_code, close_msg):
            print(f"WS Closed {slot}")
            self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_EVENT_DISCONNECTED]))
            self.ws_clients[slot] = None

        def on_open(ws):
            print(f"WS Open {slot} -> {url}")
            self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_EVENT_CONNECTED]))

        ws = websocket.WebSocketApp(url,
                                    on_open=on_open,
                                    on_message=on_message,
                                    on_error=on_error,
                                    on_close=on_close)
        
        self.ws_clients[slot] = ws
        wst = threading.Thread(target=ws.run_forever)
        wst.daemon = True
        wst.start()

    # --------------------------------------------------------------------------
    # Command Processor
    # --------------------------------------------------------------------------
    def process_command(self, packet):
        if len(packet) < 2: return
        cmd, slot = packet[0], packet[1]
        payload = packet[2:]
        success = False

        # --- GLOBAL ---
        if cmd == CMD_C_PING_HOST:
            self.send_packet(CMD_H_PING_RESPONSE, GLOBAL_SLOT_ID)
            return
            
        elif cmd == CMD_C_SET_DEBUG:
            print(f"Debug Level Set: {payload[0]}")
            self.send_packet(CMD_H_ACK, GLOBAL_SLOT_ID)
            return

        # --- TCP ---
        if cmd == CMD_C_CONNECT_HOST:
            try:
                use_ssl = bool(payload[0])
                port = (payload[1] << 8) | payload[2]
                hlen = payload[3]
                host = payload[4:4+hlen].decode('utf-8')
                
                print(f"TCP Connect ({'SSL' if use_ssl else 'Plain'}) -> {host}:{port}")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                
                if use_ssl:
                    s = self.wrap_socket_ssl(s, host, self.tcp_ca_certs[slot])
                
                s.connect((host, port))
                s.settimeout(0.1) 
                
                self.tcp_slots[slot] = s
                t = threading.Thread(target=self.tcp_rx_thread, args=(slot, s))
                t.daemon = True
                t.start()
                
                success = True
                self.tcp_ca_certs[slot] = None 
                self.send_packet(CMD_H_CONNECTED_STATUS, slot, b'\x01')
            except Exception as e:
                print(f"Connect Failed: {e}")

        elif cmd == CMD_C_WRITE:
            if self.tcp_slots[slot]:
                try: 
                    self.tcp_slots[slot].sendall(payload)
                    success = True
                except: pass

        elif cmd == CMD_C_STOP:
            self.close_tcp(slot)
            success = True

        elif cmd == CMD_C_START_TLS:
            if self.tcp_slots[slot]:
                try:
                    print("Upgrading to SSL (STARTTLS)...")
                    ctx = ssl.create_default_context()
                    ctx.check_hostname = False
                    ctx.verify_mode = ssl.CERT_NONE
                    self.tcp_slots[slot] = ctx.wrap_socket(self.tcp_slots[slot], server_side=False)
                    success = True
                except Exception as e:
                    print(f"STARTTLS Failed: {e}")

        elif cmd == CMD_C_SET_CA_CERT:
            try:
                fname = payload.decode('utf-8')
                self.tcp_ca_certs[slot] = fname
                success = True
            except: pass

        elif cmd == CMD_C_POLL_DATA:
            # Check Session ID for Reset
            if len(payload) >= 2:
                client_sid = (payload[0] << 8) | payload[1]
                if client_sid != 0 and client_sid != self.session_id:
                     print(f"Session Mismatch (Client:{client_sid:04X} Host:{self.session_id:04X}). Resetting.")
                     self.notify_boot()
                     return

            # Send Poll Response
            is_conn = 1 if self.tcp_slots[slot] else 0
            # Payload: [Conn][LenH][LenL]
            resp = bytes([is_conn, 0, 0]) 
            self.send_packet(CMD_H_POLL_RESPONSE, slot, resp)
            return

        elif cmd == CMD_C_DATA_ACK:
            # Client confirms receipt of data. Unblock RX thread.
            self.tcp_ack_events[slot].set()
            return # No ACK needed for an ACK

        elif cmd == CMD_C_IS_CONNECTED:
            status = b'\x01' if self.tcp_slots[slot] else b'\x00'
            self.send_packet(CMD_H_CONNECTED_STATUS, slot, status)
            return

        # --- UDP ---
        elif cmd == CMD_C_UDP_BEGIN:
            try:
                port = (payload[0] << 8) | payload[1]
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind(('0.0.0.0', port))
                s.settimeout(0.1)
                self.udp_sockets[slot] = s
                with self.udp_rx_queues[slot].mutex: self.udp_rx_queues[slot].queue.clear()
                
                t = threading.Thread(target=self.udp_rx_thread, args=(slot, s))
                t.daemon = True
                t.start()
                success = True
            except Exception as e: print(f"UDP Begin: {e}")

        elif cmd == CMD_C_UDP_BEGIN_PACKET:
            # Payload: [PortH][PortL][Type][...Addr...]
            try:
                port = (payload[0] << 8) | payload[1]
                ptype = payload[2]
                
                if ptype == 0: # IP
                    addr = f"{payload[3]}.{payload[4]}.{payload[5]}.{payload[6]}"
                elif ptype == 1: # Hostname
                    hlen = payload[3]
                    addr = payload[4:4+hlen].decode('utf-8')
                
                self.udp_tx_buffers[slot] = {'addr': (addr, port), 'data': b''}
                success = True
            except: pass

        elif cmd == CMD_C_UDP_WRITE_DATA:
            if self.udp_tx_buffers[slot]:
                self.udp_tx_buffers[slot]['data'] += payload
                success = True

        elif cmd == CMD_C_UDP_END_PACKET:
            if self.udp_sockets[slot] and self.udp_tx_buffers[slot]:
                try:
                    tx = self.udp_tx_buffers[slot]
                    self.udp_sockets[slot].sendto(tx['data'], tx['addr'])
                    success = True
                except Exception as e: print(f"UDP Send: {e}")
            self.udp_tx_buffers[slot] = None

        elif cmd == CMD_C_UDP_PARSE_PACKET:
            try:
                data, ip_str, port = self.udp_rx_queues[slot].get_nowait()
                ip_parts = [int(x) for x in ip_str.split('.')]
                size = len(data)
                # INFO: [IP0..3][PortH][PortL][SizeH][SizeL]
                info = bytes(ip_parts) + struct.pack('>HH', port, size)
                
                self.send_packet(CMD_H_UDP_PACKET_INFO, slot, info)
                self.send_packet(CMD_H_UDP_DATA_PAYLOAD, slot, data)
                success = True
            except queue.Empty:
                success = True

        elif cmd == CMD_C_UDP_END:
            if self.udp_sockets[slot]:
                self.udp_sockets[slot].close()
                self.udp_sockets[slot] = None
            success = True

        # --- WEBSOCKET ---
        elif cmd == CMD_C_WS_CONNECT:
            try:
                # [SSL][PortH][PortL][HLen][Host][PLen][Path]
                use_ssl = bool(payload[0])
                port = (payload[1] << 8) | payload[2]
                hl = payload[3]
                host = payload[4:4+hl].decode('utf-8')
                pl = payload[4+hl]
                path = payload[5+hl:5+hl+pl].decode('utf-8')
                
                self.start_ws(slot, host, port, path, use_ssl)
                success = True
            except Exception as e: print(f"WS Connect: {e}")

        elif cmd == CMD_C_WS_SEND_FRAME:
            if self.ws_clients[slot] and len(payload) > 0:
                ftype = payload[0]
                data = payload[1:]
                op = websocket.ABNF.OPCODE_TEXT if ftype == WS_FRAME_TEXT else websocket.ABNF.OPCODE_BINARY
                try:
                    self.ws_clients[slot].send(data, opcode=op)
                    success = True
                except: pass
                
        elif cmd == CMD_C_WS_DISCONNECT:
            if self.ws_clients[slot]:
                self.ws_clients[slot].close()
            success = True

        elif cmd == CMD_C_WS_LOOP:
            success = True

        # --- FINAL ACK ---
        # Some commands don't need a Generic ACK because they have specific responses 
        # (POLL_RESPONSE, CONNECTED_STATUS, DATA_ACK logic).
        # We ACK everything else to satisfy Client "awaitAckNak".
        if cmd not in [CMD_C_POLL_DATA, CMD_C_IS_CONNECTED, CMD_C_DATA_ACK]:
             self.send_packet(CMD_H_ACK if success else CMD_H_NAK, slot)

    def run(self):
        buffer = b''
        while self.running:
            try:
                if self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting)
                    buffer += chunk
                
                if FRAME_DELIMITER in buffer:
                    frames = buffer.split(FRAME_DELIMITER)
                    buffer = frames.pop() 
                    for frame in frames:
                        if len(frame) > 0:
                            try:
                                decoded = cobs.decode(frame)
                                if len(decoded) > 2:
                                    data = decoded[:-2]
                                    recv_crc = struct.unpack('<H', decoded[-2:])[0]
                                    if recv_crc == calculate_crc16(data):
                                        self.process_command(data)
                            except: pass
                time.sleep(0.001)
            except KeyboardInterrupt: self.running = False
            except Exception as e: 
                print(f"Main Loop Error: {e}")
                self.running = False
        self.ser.close()

if __name__ == "__main__":
    host = PySerialNetworkHost(SERIAL_PORT, BAUD_RATE)
    host.run()