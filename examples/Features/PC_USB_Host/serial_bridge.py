import serial
import time
import socket
import threading
import struct
import queue
import ssl
import sys
import os
import platform
import subprocess
from cobs import cobs
import websocket  # pip install websocket-client

# ==========================================
#  CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM3'   # Update this to match your port
BAUD_RATE = 115200

# ==========================================
#  PROTOCOL CONSTANTS
# ==========================================
# Global
GLOBAL_SLOT_ID         = 0xFF
CMD_C_SET_WIFI         = 0x01
CMD_C_CONNECT_NET      = 0x02
CMD_C_DISCONNECT_NET   = 0x03
CMD_C_IS_NET_CONNECTED = 0x04
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
        # Retry logic for serial port
        self.ser = None
        for i in range(5):
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=0.01)
                break
            except Exception as e:
                print(f"[INIT] Failed to open {port} (Attempt {i+1}/5): {e}")
                time.sleep(1)
        
        if not self.ser:
            print(f"[CRITICAL] Could not open serial port {port}. Exiting.")
            sys.exit(1)

        self.lock = threading.Lock()
        self.running = True
        
        # TCP State
        self.tcp_slots = [None] * MAX_SLOTS
        self.tcp_ca_certs = [None] * MAX_SLOTS 
        self.tcp_ack_events = [threading.Event() for _ in range(MAX_SLOTS)]
        
        # UDP State
        self.udp_sockets = [None] * MAX_SLOTS
        self.udp_rx_queues = [queue.Queue() for _ in range(MAX_SLOTS)]
        self.udp_tx_buffers = [None] * MAX_SLOTS
        
        # WebSocket State
        self.ws_clients = [None] * MAX_SLOTS

        self.session_id = int(time.time() * 1000) & 0xFFFF 
        if self.session_id == 0: self.session_id = 1
        
        # Store WiFi credentials for connect/disconnect usage
        self.wifi_ssid = ""
        self.wifi_pass = ""

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

    # --------------------------------------------------------------------------
    # TCP Logic
    # --------------------------------------------------------------------------
    def tcp_rx_thread(self, slot_id, sock):
        while self.tcp_slots[slot_id] == sock and self.running:
            try:
                data = sock.recv(64) 
                if not data: break
                
                self.tcp_ack_events[slot_id].clear()
                self.send_packet(CMD_H_DATA_PAYLOAD, slot_id, data)
                ack_received = self.tcp_ack_events[slot_id].wait(timeout=2.0)
                
                if not ack_received:
                    print(f"Slot {slot_id} Timeout waiting for ACK. Dropping packet.")
                    
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
        
        run_kwargs = {}
        if use_ssl:
            run_kwargs = {"sslopt": {"cert_reqs": ssl.CERT_NONE}}

        wst = threading.Thread(target=ws.run_forever, kwargs=run_kwargs)
        wst.daemon = True
        wst.start()

    # --------------------------------------------------------------------------
    # System Command Utilities (Integrated from wifi_management.py)
    # --------------------------------------------------------------------------
    def check_internet(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

    def is_raspberry_pi(self):
        """Detect if running on Raspberry Pi hardware."""
        if platform.system() == "Linux":
            try:
                with open("/proc/cpuinfo", "r") as f:
                    cpuinfo = f.read().lower()
                    if "raspberry pi" in cpuinfo or "bcm" in cpuinfo:
                        return True
            except Exception:
                pass
        return False

    def get_wifi_interface_linux(self):
        """Auto-detect WiFi interface on Linux using nmcli, fallback to wlan0."""
        try:
            result = subprocess.check_output("nmcli device status", shell=True).decode()
            for line in result.splitlines():
                if "wifi" in line.lower():
                    return line.split()[0]  # first column is interface name
        except Exception:
            pass
        return "wlan0"

    def get_wifi_interface_mac(self):
        """Auto-detect WiFi interface on macOS using networksetup, fallback to en0."""
        try:
            result = subprocess.check_output("networksetup -listallhardwareports", shell=True).decode()
            for block in result.split("\n\n"):
                if "Wi-Fi" in block:
                    for line in block.splitlines():
                        if line.startswith("Device:"):
                            return line.split(":")[1].strip()
        except Exception:
            pass
        return "en0"

    def system_set_wifi(self, ssid, password):
        sys_platform = platform.system()
        print(f"[SYSTEM] Configuring WiFi for {sys_platform}...")
        
        if sys_platform == "Windows":
            xml_template = f"""<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
    <name>{ssid}</name>
    <SSIDConfig><SSID><name>{ssid}</name></SSID></SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>auto</connectionMode>
    <MSM><security>
        <authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption>
        <sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>{password}</keyMaterial></sharedKey>
    </security></MSM>
</WLANProfile>"""
            try:
                with open("wifi.xml", "w") as f:
                    f.write(xml_template)
                os.system("netsh wlan disconnect")
                os.system('netsh wlan add profile filename="wifi.xml"')
                os.system(f'netsh wlan connect name="{ssid}"')
                if os.path.exists("wifi.xml"):
                    os.remove("wifi.xml")
                return True
            except Exception as e:
                print(f"[ERROR] Windows WiFi Config Failed: {e}")
                return False

        elif sys_platform == "Linux":
            iface = self.get_wifi_interface_linux()
            print(f"[SYSTEM] Using Interface: {iface}")
            
            # Check if Raspberry Pi (might need wpa_cli instead of nmcli)
            if self.is_raspberry_pi():
                try:
                    # Prefer nmcli first
                    os.system(f"nmcli device disconnect {iface}")
                    if password:
                        ret = os.system(f"nmcli device wifi connect '{ssid}' password '{password}'")
                    else:
                        ret = os.system(f"nmcli device wifi connect '{ssid}'")
                    if ret == 0: return True
                except:
                    pass # Fallback to wpa_cli below

                # Fallback for RPi Lite (wpa_cli)
                try:
                    print("[SYSTEM] nmcli failed, trying wpa_cli...")
                    os.system(f"wpa_cli -i {iface} disconnect")
                    os.system(f"wpa_cli -i {iface} add_network")
                    os.system(f"wpa_cli -i {iface} set_network 0 ssid '\"{ssid}\"'")
                    if password:
                        os.system(f"wpa_cli -i {iface} set_network 0 psk '\"{password}\"'")
                    os.system(f"wpa_cli -i {iface} enable_network 0")
                    return True
                except Exception as e:
                    print(f"[ERROR] RPi WiFi Config Failed: {e}")
                    return False
            else:
                # Standard Linux (Ubuntu/Fedora etc)
                try:
                    os.system(f"nmcli device disconnect {iface}")
                    if password:
                        ret = os.system(f"nmcli device wifi connect '{ssid}' password '{password}'")
                    else:
                        ret = os.system(f"nmcli device wifi connect '{ssid}'")
                    return ret == 0
                except:
                    return False

        elif sys_platform == "Darwin": # macOS
            iface = self.get_wifi_interface_mac()
            print(f"[SYSTEM] Using Interface: {iface}")
            try:
                os.system(f"networksetup -setairportpower {iface} off")
                time.sleep(1)
                os.system(f"networksetup -setairportpower {iface} on")
                if password:
                    ret = os.system(f"networksetup -setairportnetwork {iface} '{ssid}' '{password}'")
                else:
                    ret = os.system(f"networksetup -setairportnetwork {iface} '{ssid}'")
                return ret == 0
            except:
                return False
        
        return False

    def system_disconnect_wifi(self):
        sys_platform = platform.system()
        print(f"[SYSTEM] Disconnecting WiFi for {sys_platform}...")
        
        if sys_platform == "Windows":
            os.system("netsh wlan disconnect")
        elif sys_platform == "Linux":
            iface = self.get_wifi_interface_linux()
            if self.is_raspberry_pi():
                try:
                    os.system(f"nmcli device disconnect {iface}")
                except:
                    os.system(f"wpa_cli -i {iface} disconnect")
            else:
                os.system(f"nmcli device disconnect {iface}")
        elif sys_platform == "Darwin": # macOS
            iface = self.get_wifi_interface_mac()
            os.system(f"networksetup -setairportpower {iface} off")
        return True

    def system_reboot(self):
        print("[SYSTEM] Rebooting OS...")
        sys_platform = platform.system()
        if sys_platform == "Windows":
            os.system("shutdown /r /t 0")
        else:
            os.system("sudo reboot")

    # --------------------------------------------------------------------------
    # Command Processor
    # --------------------------------------------------------------------------
    def process_command(self, packet):
        if len(packet) < 2: return
        cmd, slot = packet[0], packet[1]
        payload = packet[2:]
        success = False

        # --- GLOBAL COMMANDS ---
        if cmd == CMD_C_PING_HOST:
            self.send_packet(CMD_H_PING_RESPONSE, GLOBAL_SLOT_ID)
            return
            
        elif cmd == CMD_C_SET_DEBUG:
            print(f"[GLOBAL] Debug Level Set: {payload[0]}")
            self.send_packet(CMD_H_ACK, GLOBAL_SLOT_ID)
            return

        elif cmd == CMD_C_REBOOT_HOST:
            print("\n" + "="*40)
            print("!!! SYSTEM REBOOT REQUESTED BY CLIENT !!!")
            print("Host Computer will reboot in 10 seconds...")
            print("="*40 + "\n")
            self.send_packet(CMD_H_ACK, GLOBAL_SLOT_ID)
            
            def perform_reboot():
                time.sleep(10)
                if self.ser: self.ser.close()
                self.system_reboot()

            threading.Thread(target=perform_reboot, daemon=True).start()
            return

        elif cmd == CMD_C_SET_WIFI:
            try:
                sl = payload[0]
                ssid = payload[1:1+sl].decode('utf-8')
                pl = payload[1+sl]
                password = payload[2+sl:2+sl+pl].decode('utf-8')
                
                print(f"[GLOBAL] Set WiFi Request: {ssid}")
                
                self.wifi_ssid = ssid
                self.wifi_pass = password
                
                success = True
            except: 
                print("[GLOBAL] Failed to parse Set WiFi command")
                success = False
            self.send_packet(CMD_H_ACK if success else CMD_H_NAK, GLOBAL_SLOT_ID)
            return

        elif cmd == CMD_C_CONNECT_NET:
            print(f"[GLOBAL] Connecting to WiFi: {self.wifi_ssid}...")
            if self.wifi_ssid:
                threading.Thread(target=self.system_set_wifi, args=(self.wifi_ssid, self.wifi_pass), daemon=True).start()
                success = True
            else:
                print("[GLOBAL] No SSID configured.")
                success = False
            self.send_packet(CMD_H_ACK if success else CMD_H_NAK, GLOBAL_SLOT_ID)
            return

        elif cmd == CMD_C_DISCONNECT_NET:
            print("[GLOBAL] Disconnecting Network...")
            threading.Thread(target=self.system_disconnect_wifi, daemon=True).start()
            self.send_packet(CMD_H_ACK, GLOBAL_SLOT_ID)
            return

        elif cmd == CMD_C_IS_NET_CONNECTED:
            is_connected = self.check_internet()
            print(f"[GLOBAL] Network Check: {'Online' if is_connected else 'Offline'}")
            self.send_packet(CMD_H_ACK if is_connected else CMD_H_NAK, GLOBAL_SLOT_ID)
            return

        # --- TCP ---
        if cmd == CMD_C_CONNECT_HOST:
            try:
                if self.tcp_slots[slot]:
                    print(f"[TCP] Slot {slot} busy. Closing previous connection...")
                    self.close_tcp(slot)

                use_ssl = bool(payload[0])
                port = (payload[1] << 8) | payload[2]
                hlen = payload[3]
                host = payload[4:4+hlen].decode('utf-8')
                
                print(f"[TCP] Connect ({'SSL' if use_ssl else 'Plain'}) -> {host}:{port}")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(10.0) 
                
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
            if len(payload) >= 2:
                client_sid = (payload[0] << 8) | payload[1]
                if client_sid != 0 and client_sid != self.session_id:
                     print(f"Session Mismatch (Client:{client_sid:04X} Host:{self.session_id:04X}). Resetting.")
                     self.notify_boot()
                     return

            is_conn = 1 if self.tcp_slots[slot] else 0
            resp = bytes([is_conn, 0, 0]) 
            self.send_packet(CMD_H_POLL_RESPONSE, slot, resp)
            return

        elif cmd == CMD_C_DATA_ACK:
            self.tcp_ack_events[slot].set()
            return 

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
                            except Exception as e:
                                print(f"Decoder Warning: {e}")
                                pass
                time.sleep(0.001)
            except KeyboardInterrupt: self.running = False
            except Exception as e: 
                print(f"Main Loop Error: {e}")
                self.running = False
        if self.ser: self.ser.close()

if __name__ == "__main__":
    host = PySerialNetworkHost(SERIAL_PORT, BAUD_RATE)
    host.run()