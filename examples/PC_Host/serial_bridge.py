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
import pywifi
from pywifi import const
from colorama import init, Fore, Back, Style

init()

# ==========================================
#  CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM3'   # Update this to match your port
BAUD_RATE = 115200     # Must match Arduino

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
CMD_C_DEBUG_INFO       = 0x08

# TCP
CMD_C_CONNECT_HOST     = 0x10
CMD_C_WRITE            = 0x11
CMD_C_STOP             = 0x13
CMD_C_IS_CONNECTED     = 0x14
CMD_C_DATA_ACK         = 0x15
CMD_C_START_TLS        = 0x16
CMD_C_SET_CA_CERT      = 0x17
CMD_C_POLL_DATA        = 0x18
CMD_C_SET_BUF_SIZE     = 0xC3

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

# Flags
CMD_C_GET_FLAG         = 0xC0
CMD_C_SET_FLAG         = 0xC1
CMD_H_FLAG_RESPONSE    = 0xC2

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

# SSL Flag Bitmasks
SSL_STATUS_BIT         = 0x01
SSL_PLAIN_START_BIT    = 0x02
SSL_INSECURE_BIT       = 0x04

# WS Types
WS_EVENT_DISCONNECTED  = 0x00
WS_EVENT_CONNECTED     = 0x01
WS_FRAME_TEXT          = 0x02
WS_FRAME_BINARY        = 0x03
WS_FRAME_PONG          = 0x04
WS_EVENT_ERROR         = 0xFF

MAX_SLOTS = 4
FRAME_DELIMITER = b'\x00'
SERIAL_TCP_DATA_PAYLOAD_SIZE = 32    
SERIAL_TCP_HOST_TX_BUFFER_SIZE = 2048 
SERIAL_TCP_DATA_PACKET_TIMEOUT = 2.0  

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

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def wifi_exists(target_ssid):
    try:
        wifi = pywifi.PyWiFi()
        iface = wifi.interfaces()[0]
        iface.scan()
        time.sleep(2)
        results = iface.scan_results()
        for network in results:
            if network.ssid == target_ssid:
                return True
    except: pass
    return False

# ==========================================
#  Classes
# ==========================================

class DynamicQueue:
    def __init__(self, limit=SERIAL_TCP_HOST_TX_BUFFER_SIZE):
        self.queue = bytearray()
        self.limit = limit
        self.lock = threading.Lock()

    def available(self):
        return len(self.queue)

    def space(self):
        return self.limit - len(self.queue)

    def enqueue(self, data: bytes):
        with self.lock:
            space = self.limit - len(self.queue)
            if space <= 0:
                return 0
            to_add = data[:space]
            self.queue.extend(to_add)
            return len(to_add)

    def dequeue(self, count):
        with self.lock:
            if count > len(self.queue):
                count = len(self.queue)
            data = self.queue[:count]
            del self.queue[:count]
            return data
            
    def clear(self):
        with self.lock:
            self.queue.clear()

class PySerialNetworkHost:
    import serial
    import serial.tools.list_ports

    def print_color(self, text, color=Fore.YELLOW):
        print(color +  text + Style.RESET_ALL, end='', flush=True)

    def list_ports(self):        
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.print_color(" No serial ports found.\r\n",Fore.RED)
            return []
        self.print_color(" Available serial ports:\r\n\r\n",Fore.CYAN)
        for i, port in enumerate(ports, start=1):
            self.print_color(f" {i}: {port.device} ({port.description})\r\n",Fore.CYAN)
        return ports

    def choose_port(self):
        ports = self.list_ports()
        if not ports: return None
        while True:
            try:
                choice = int(input(Fore.CYAN + "\r\n Please select a port number: " + Style.RESET_ALL))
                if 1 <= choice <= len(ports):
                    return ports[choice - 1].device
                else:
                    self.print_color(" Invalid choice. Please try again.\r\n",Fore.RED)
                    clear_screen()
                    ports = self.list_ports()
            except ValueError:
                self.print_color(" Please enter a valid number.\r\n",Fore.RED)
                clear_screen()
                ports = self.list_ports()

    def __init__(self, port, baud_rate):
        self.ser = None
        retry  = 30
        clear_screen()

        for i in range(retry):
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=0.01)
                break
            except Exception as e:
                self.print_color(f"\r\n [INIT] Could not open {port}\r\n",Fore.RED)
                print()
                port = self.choose_port()
        if not self.ser:
            self.print_color(f"\r\n [CRITICAL] Could not open {port}. Exiting.\r\n",Fore.RED)
            sys.exit(1)

        self.lock = threading.Lock()
        self.running = True
        
        # Connection State
        self.tcp_slots = [None] * MAX_SLOTS
        self.tcp_hostnames = [None] * MAX_SLOTS # Store hostnames for STARTTLS
        self.tcp_ca_certs = [None] * MAX_SLOTS 
        self.udp_sockets = [None] * MAX_SLOTS
        self.udp_rx_queues = [queue.Queue() for _ in range(MAX_SLOTS)]
        self.udp_tx_buffers = [None] * MAX_SLOTS
        self.ws_clients = [None] * MAX_SLOTS
        
        # Flag Storage
        self.slot_flags = [0] * MAX_SLOTS
        # Buffer Size Storage (rx, tx)
        self.slot_buf_sizes = [None] * MAX_SLOTS
        
        # TX Queue & Flow Control State
        self.tx_queues = [DynamicQueue() for _ in range(MAX_SLOTS)]
        self.tx_states = ["IDLE"] * MAX_SLOTS 
        self.last_data_send_time = [0] * MAX_SLOTS
        self.last_data_packet = [None] * MAX_SLOTS 

        self.session_id = int(time.time() * 1000) & 0xFFFF 
        if self.session_id == 0: self.session_id = 1
        
        self.wifi_ssid = ""
        self.wifi_pass = ""
        clear_screen()
        self.print_color(f"\r\n PC Host Serial Bridge",Fore.LIGHTYELLOW_EX)
        self.print_color(f" started on {port} @ {baud_rate}. Session: {self.session_id:04X}\r\n\r\n",Fore.CYAN)
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

    def process_data_queues(self):
        for i in range(MAX_SLOTS):
            if self.tx_states[i] == "WAIT_FOR_ACK":
                if time.time() - self.last_data_send_time[i] > SERIAL_TCP_DATA_PACKET_TIMEOUT:
                    self.print_color(f" [FLOW] Slot {i} ACK Timeout. Retrying...\r\n",Fore.YELLOW)
                    if self.last_data_packet[i]:
                        cmd, slot, payload = self.last_data_packet[i]
                        self.send_packet(cmd, slot, payload)
                        self.last_data_send_time[i] = time.time()
                continue

            if self.tx_states[i] == "IDLE" and self.tx_queues[i].available() > 0:
                chunk = self.tx_queues[i].dequeue(SERIAL_TCP_DATA_PAYLOAD_SIZE)
                if chunk:
                    self.last_data_packet[i] = (CMD_H_DATA_PAYLOAD, i, chunk)
                    self.send_packet(CMD_H_DATA_PAYLOAD, i, chunk)
                    self.tx_states[i] = "WAIT_FOR_ACK"
                    self.last_data_send_time[i] = time.time()

    def tcp_rx_thread(self, slot_id, sock):
        while self.tcp_slots[slot_id] == sock and self.running:
            try:
                data = sock.recv(1024) 
                if not data: break
                remaining = data
                while len(remaining) > 0 and self.running and self.tcp_slots[slot_id] == sock:
                    sent_count = self.tx_queues[slot_id].enqueue(remaining)
                    if sent_count < len(remaining):
                        time.sleep(0.005) 
                        remaining = remaining[sent_count:]
                    else:
                        remaining = b''
            except socket.timeout: continue
            except OSError: break
            except Exception as e:
                self.print_color(f"\r\n [TCP] RX Error: {e}\r\n",Fore.RED)
                break
        self.close_tcp(slot_id)

    def close_tcp(self, slot):
        if self.tcp_slots[slot]:
            try: self.tcp_slots[slot].close()
            except: pass
            self.tcp_slots[slot] = None
            self.tcp_hostnames[slot] = None # Clear hostname on close
            timeout_wait = time.time()
            while self.tx_queues[slot].available() > 0 and self.running and (time.time() - timeout_wait < 5.0):
                time.sleep(0.05)
            self.tx_queues[slot].clear()
            self.tx_states[slot] = "IDLE"
            self.send_packet(CMD_H_CONNECTED_STATUS, slot, b'\x00')
            self.print_color(f"\r\n [TCP] Slot {slot} Closed\r\n",Fore.CYAN)

    def set_socket_buffers(self, sock, slot_id):
        if self.slot_buf_sizes[slot_id]:
            rx, tx = self.slot_buf_sizes[slot_id]
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rx)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, tx)
                # self.print_color(f" [TCP] Slot {slot_id} Buffers set: RX={rx}, TX={tx}\r\n", Fore.CYAN)
            except Exception as e:
                self.print_color(f" [TCP] Failed to set buffers: {e}\r\n", Fore.RED)

    # [UPDATED] wrap_socket_ssl to respect INSECURE flag and use stored hostname
    def wrap_socket_ssl(self, sock, hostname, slot_id):
        context = ssl.create_default_context()
        
        # Check stored flag for Insecure Bit
        flag = self.slot_flags[slot_id]
        if (flag & SSL_INSECURE_BIT):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
            self.print_color(f" [SSL] Slot {slot_id} Insecure Mode Enabled\r\n", Fore.YELLOW)
        else:
            # Standard Verification
            ca_cert = self.tcp_ca_certs[slot_id]
            if ca_cert:
                try:
                    context.load_verify_locations(cafile=ca_cert)
                except Exception as e:
                    self.print_color(f" SSL Cert Error: {e}\r\n",Fore.RED)
            else:
                # Default behavior (Strict verify using system CAs)
                pass 

        # [FIX] Ensure server_hostname is provided if verification is enabled
        # If hostname is None but verification is ON, SSLContext will raise an error.
        # However, create_default_context() defaults check_hostname=True.
        if hostname:
            return context.wrap_socket(sock, server_hostname=hostname)
        else:
            # If no hostname, we must disable check_hostname or it will fail
            if context.check_hostname:
                self.print_color(f" [SSL] Warning: No hostname for SNI/Check. Disabling check_hostname.\r\n", Fore.YELLOW)
                context.check_hostname = False 
            return context.wrap_socket(sock)

    def udp_rx_thread(self, slot_id, sock):
        while self.udp_sockets[slot_id] == sock and self.running:
            try:
                data, addr = sock.recvfrom(512)
                self.udp_rx_queues[slot_id].put((data, addr[0], addr[1]))
            except socket.timeout: continue
            except: break

    def start_ws(self, slot, host, port, path, use_ssl):
        scheme = "wss" if use_ssl else "ws"
        url = f"{scheme}://{host}:{port}{path}"
        def on_message(ws, message):
            if isinstance(message, str):
                self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_FRAME_TEXT]) + message.encode('utf-8'))
            else:
                self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_FRAME_BINARY]) + message)
        def on_error(ws, error):
            self.print_color(f" WS Error {slot}: {error}\r\n",Fore.RED)
            self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_EVENT_ERROR]))
        def on_close(ws, close_status_code, close_msg):
            self.print_color(f" WS Closed {slot}\r\n",Fore.CYAN)
            self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_EVENT_DISCONNECTED]))
            self.ws_clients[slot] = None
        def on_open(ws):
            self.print_color(f" WS Open {slot} -> {url}\r\n",Fore.CYAN)
            self.send_packet(CMD_H_WS_EVENT, slot, bytes([WS_EVENT_CONNECTED]))

        # Check Insecure Flag for WS
        sslopt = {}
        if use_ssl:
            flag = self.slot_flags[slot]
            if (flag & SSL_INSECURE_BIT):
                sslopt = {"cert_reqs": ssl.CERT_NONE}
        
        ws = websocket.WebSocketApp(url, on_open=on_open, on_message=on_message, on_error=on_error, on_close=on_close)
        self.ws_clients[slot] = ws
        wst = threading.Thread(target=ws.run_forever, kwargs={"sslopt": sslopt})
        wst.daemon = True
        wst.start()

    def check_internet(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError: return False

    # ... (Wifi utility functions remain the same: is_raspberry_pi, get_wifi_interface_linux, etc.)
    # Ensure they use the suppressed output versions discussed earlier.
    def get_wifi_interface_linux(self):
        try:
            res = subprocess.check_output("nmcli device status", shell=True).decode()
            for line in res.splitlines():
                if "wifi" in line.lower(): return line.split()[0]
        except: pass
        return "wlan0"

    def get_wifi_interface_mac(self):
        try:
            res = subprocess.check_output("networksetup -listallhardwareports", shell=True).decode()
            for block in res.split("\n\n"):
                if "Wi-Fi" in block:
                    for line in block.splitlines():
                        if line.startswith("Device:"): return line.split(":")[1].strip()
        except: pass
        return "en0"

    def system_set_wifi(self, ssid, password):
        sys_platform = platform.system()
        self.print_color(f" [SYSTEM] Configuring WiFi for {sys_platform}...\r\n",Fore.CYAN)
        if not wifi_exists(ssid):
            self.print_color(" [SYSTEM] WiFi AP \"" + ssid + "\" was not found.\r\n",Fore.YELLOW)
            return False
        
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
                with open("wifi.xml", "w") as f: f.write(xml_template)
                os.system("netsh wlan disconnect > NUL 2>&1")
                os.system('netsh wlan add profile filename="wifi.xml" > NUL 2>&1')
                os.system(f'netsh wlan connect name="{ssid}" > NUL 2>&1')
                if os.path.exists("wifi.xml"): os.remove("wifi.xml")
                return True
            except Exception as e:
                self.print_color(f" [ERROR] Windows WiFi: {e}\r\n",Fore.RED)
                return False
        elif sys_platform == "Linux":
            iface = self.get_wifi_interface_linux()
            try:
                os.system(f"nmcli device disconnect {iface} > /dev/null 2>&1")
                cmd = f"nmcli device wifi connect '{ssid}'"
                if password: cmd += f" password '{password}'"
                cmd += " > /dev/null 2>&1"
                if os.system(cmd) == 0: return True
            except: pass
            if self.is_raspberry_pi():
                try:
                    os.system(f"wpa_cli -i {iface} disconnect > /dev/null 2>&1")
                    os.system(f"wpa_cli -i {iface} add_network > /dev/null 2>&1")
                    os.system(f"wpa_cli -i {iface} set_network 0 ssid '\"{ssid}\"' > /dev/null 2>&1")
                    if password: os.system(f"wpa_cli -i {iface} set_network 0 psk '\"{password}\"' > /dev/null 2>&1")
                    os.system(f"wpa_cli -i {iface} enable_network 0 > /dev/null 2>&1")
                    return True
                except: return False
            return False
        elif sys_platform == "Darwin":
            iface = self.get_wifi_interface_mac()
            try:
                os.system(f"networksetup -setairportpower {iface} off > /dev/null 2>&1")
                time.sleep(1)
                os.system(f"networksetup -setairportpower {iface} on > /dev/null 2>&1")
                cmd = f"networksetup -setairportnetwork {iface} '{ssid}'"
                if password: cmd += f" '{password}'"
                cmd += " > /dev/null 2>&1"
                return os.system(cmd) == 0
            except: return False
        return False

    def system_disconnect_wifi(self):
        sys_platform = platform.system()
        import logging
        logging.getLogger('comtypes').setLevel(logging.CRITICAL)
        if sys_platform == "Windows": os.system("netsh wlan disconnect > NUL 2>&1")
        elif sys_platform == "Linux":
            iface = self.get_wifi_interface_linux()
            if os.system(f"nmcli device disconnect {iface} > /dev/null 2>&1") != 0 and self.is_raspberry_pi():
                os.system(f"wpa_cli -i {iface} disconnect > /dev/null 2>&1")
        elif sys_platform == "Darwin":
            iface = self.get_wifi_interface_mac()
            os.system(f"networksetup -setairportpower {iface} off > /dev/null 2>&1")
        return True

    def system_reboot(self):
        time.sleep(0.3)
        for i in range(20, 0, -1): # waits 20 seconds
            time.sleep(1)
            self.print_color(f" [SYSTEM] Rebooting OS in {i} seconds...\r",Fore.YELLOW)
        self.print_color(f"\n [SYSTEM] Rebooting OS now...\r\n",Fore.CYAN)
        time.sleep(2)
        if platform.system() == "Windows": os.system("shutdown /r /t 0 >nul 2>&1")
        else: os.system("sudo reboot")

    def process_command(self, packet):
        if len(packet) < 2: return
        cmd, slot = packet[0], packet[1]
        payload = packet[2:]
        success = False

        if cmd == CMD_C_PING_HOST:
            self.send_packet(CMD_H_PING_RESPONSE, GLOBAL_SLOT_ID)
            return
        elif cmd == CMD_C_SET_DEBUG:
            self.print_color(f" [GLOBAL] Debug Level: {payload[0]}\r\n",Fore.CYAN)
            self.send_packet(CMD_H_ACK, GLOBAL_SLOT_ID)
            return
        elif cmd == CMD_C_REBOOT_HOST:
            self.print_color("\n !!! REBOOT REQUESTED !!!\r\n",Fore.YELLOW)
            self.send_packet(CMD_H_ACK, GLOBAL_SLOT_ID)
            def perform_reboot():
                self.system_reboot()
            threading.Thread(target=perform_reboot, daemon=True).start()
            return
        elif cmd == CMD_C_DEBUG_INFO:
            try:
                text = payload.decode('utf-8', errors='replace')
                if "Error" in text:
                    self.print_color(text, Fore.RED)
                elif "Warning" in text:
                    self.print_color(text, Fore.YELLOW)
                else:
                    self.print_color(text, Fore.GREEN)
            except: pass
            return
        elif cmd == CMD_C_SET_WIFI:
            try:
                sl = payload[0]
                ssid = payload[1:1+sl].decode('utf-8')
                pl = payload[1+sl]
                password = payload[2+sl:2+sl+pl].decode('utf-8')
                self.print_color(f" [GLOBAL] Set WiFi: {ssid}\r\n",Fore.CYAN)
                self.wifi_ssid = ssid
                self.wifi_pass = password
                success = True
            except: success = False
            self.send_packet(CMD_H_ACK if success else CMD_H_NAK, GLOBAL_SLOT_ID)
            return
        elif cmd == CMD_C_CONNECT_NET:
            self.print_color(f" [GLOBAL] Connect WiFi: {self.wifi_ssid}\r\n",Fore.CYAN)
            if self.wifi_ssid:
                threading.Thread(target=self.system_set_wifi, args=(self.wifi_ssid, self.wifi_pass), daemon=True).start()
                success = True
            else: success = False
            self.send_packet(CMD_H_ACK if success else CMD_H_NAK, GLOBAL_SLOT_ID)
            return
        elif cmd == CMD_C_DISCONNECT_NET:
            self.print_color(" [GLOBAL] Disconnect WiFi\r\n",Fore.CYAN)
            threading.Thread(target=self.system_disconnect_wifi, daemon=True).start()
            self.send_packet(CMD_H_ACK, GLOBAL_SLOT_ID)
            return
        elif cmd == CMD_C_IS_NET_CONNECTED:
            is_connected = self.check_internet()
            self.send_packet(CMD_H_ACK if is_connected else CMD_H_NAK, GLOBAL_SLOT_ID)
            return

        # [NEW] FLAG HANDLERS
        elif cmd == CMD_C_SET_FLAG:
            if len(payload) >= 1:
                flag_val = payload[0]
                self.slot_flags[slot] = flag_val
                # print(f"Slot {slot} flag set to {flag_val:02X}")
                success = True
            else: success = False

        elif cmd == CMD_C_GET_FLAG:
            # 1. Get stored flag
            flag = self.slot_flags[slot]
            
            # 2. Update Bit 0 (SSL Status) dynamically
            is_secure = False
            if self.tcp_slots[slot]:
                if isinstance(self.tcp_slots[slot], ssl.SSLSocket):
                    is_secure = True
            
            if is_secure:
                flag |= SSL_STATUS_BIT
            else:
                flag &= ~SSL_STATUS_BIT
            
            # 3. Send Response
            self.send_packet(CMD_H_FLAG_RESPONSE, slot, bytes([flag]))
            return # No ACK needed for GET

        # [NEW] SET BUFFER SIZES
        elif cmd == CMD_C_SET_BUF_SIZE:
            if len(payload) >= 4:
                rx = (payload[0] << 8) | payload[1]
                tx = (payload[2] << 8) | payload[3]
                self.slot_buf_sizes[slot] = (rx, tx)
                # Apply immediately if connected
                if self.tcp_slots[slot]:
                    self.set_socket_buffers(self.tcp_slots[slot], slot)
                success = True
            else: success = False

        # TCP
        elif cmd == CMD_C_CONNECT_HOST:
            try:
                if self.tcp_slots[slot]:
                    self.print_color(f"\r\n [TCP] Slot {slot} busy. Resetting...\r\n",Fore.YELLOW)
                    self.close_tcp(slot)

                # Packet says use_ssl?
                req_ssl = bool(payload[0])
                port = (payload[1] << 8) | payload[2]
                hlen = payload[3]
                host = payload[4:4+hlen].decode('utf-8')
                
                # Check PLAIN_START flag
                flag = self.slot_flags[slot]
                if (flag & SSL_PLAIN_START_BIT):
                    req_ssl = False # Force plain text initially
                
                # [NEW] Store hostname for later STARTTLS use
                self.tcp_hostnames[slot] = host

                self.print_color(f"\r\n [TCP] Connect ({'SSL' if req_ssl else 'Plain'}) -> {host}:{port}\r\n",Fore.CYAN)
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
                # Apply buffers
                self.set_socket_buffers(s, slot)

                s.settimeout(10.0) 
                if req_ssl:
                    # Pass slot_id to wrap_socket_ssl so it can check Insecure flag
                    s = self.wrap_socket_ssl(s, host, slot)
                s.connect((host, port))
                s.settimeout(0.1) 
                
                self.tcp_slots[slot] = s
                self.tx_queues[slot].clear()
                self.tx_states[slot] = "IDLE"
                
                t = threading.Thread(target=self.tcp_rx_thread, args=(slot, s))
                t.daemon = True
                t.start()
                
                success = True
                self.tcp_ca_certs[slot] = None 
                self.send_packet(CMD_H_CONNECTED_STATUS, slot, b'\x01')
            except Exception as e:
                self.print_color(f" Connect Failed: {e}\r\n",Fore.RED)

        elif cmd == CMD_C_WRITE:
            if self.tcp_slots[slot]:
                try: self.tcp_slots[slot].sendall(payload)
                except: pass
            self.send_packet(CMD_H_ACK, slot)

        elif cmd == CMD_C_STOP:
            self.close_tcp(slot)
            success = True

        elif cmd == CMD_C_DATA_ACK:
            self.tx_states[slot] = "IDLE"
            return 

        # STARTTLS (Updated for SSL Flags)
        elif cmd == CMD_C_START_TLS:
            if self.tcp_slots[slot]:
                try:
                    self.print_color(f" Upgrading Slot {slot} to SSL (STARTTLS)...\r\n",Fore.CYAN)
                    sock = self.tcp_slots[slot]
                    self.tcp_slots[slot] = None # Detach to stop old thread
                    time.sleep(0.2)
                    
                    sock.settimeout(10.0)
                    
                    # [FIX] Retrieve hostname for validation
                    host_for_tls = self.tcp_hostnames[slot]

                    # Pass slot to check Insecure flag
                    ssl_sock = self.wrap_socket_ssl(sock, host_for_tls, slot) 
                    
                    # Re-apply buffers to SSL socket (just in case implementation needs it)
                    self.set_socket_buffers(ssl_sock, slot)

                    ssl_sock.settimeout(0.1)
                    
                    self.tcp_slots[slot] = ssl_sock
                    t = threading.Thread(target=self.tcp_rx_thread, args=(slot, ssl_sock))
                    t.daemon = True
                    t.start()
                    
                    success = True
                    self.print_color(" STARTTLS Upgrade Successful\r\n",Fore.CYAN)
                except Exception as e: 
                    self.print_color(f" STARTTLS Failed: {e}\r\n",Fore.RED)
                    if self.tcp_slots[slot] is None:
                        self.tcp_slots[slot] = sock

        # UDP / WS (Existing logic omitted for brevity, keeping structure)
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
             except Exception as e: self.print_color(f" UDP Begin: {e}\r\n",Fore.RED)
        elif cmd == CMD_C_UDP_BEGIN_PACKET:
            try:
                port = (payload[0] << 8) | payload[1]
                ptype = payload[2]
                if ptype == 0: addr = f"{payload[3]}.{payload[4]}.{payload[5]}.{payload[6]}"
                elif ptype == 1:
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
                except Exception as e: self.print_color(f" UDP Send: {e}\r\n",Fore.RED)
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
            except queue.Empty: success = True
        elif cmd == CMD_C_UDP_END:
            if self.udp_sockets[slot]:
                self.udp_sockets[slot].close()
                self.udp_sockets[slot] = None
            
            # [FIX] Also close the TCP client if it's connected on this slot
            # This ensures both UDP and TCP resources are released when stop() is called on the client.
            if self.tcp_slots[slot]:
                self.close_tcp(slot)
                
            success = True
        
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
            except Exception as e: self.print_color(f" WS Connect: {e}\r\n",Fore.RED)
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
        elif cmd == CMD_C_WS_LOOP: success = True

        elif cmd == CMD_C_SET_CA_CERT:
            try:
                fname = payload.decode('utf-8')
                self.tcp_ca_certs[slot] = fname
                success = True
            except: pass

        if cmd not in [CMD_C_POLL_DATA, CMD_C_IS_CONNECTED, CMD_C_DATA_ACK]:
             self.send_packet(CMD_H_ACK if success else CMD_H_NAK, slot)

    def run(self):
        import traceback
        buffer = b''
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
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
                                self.print_color(f" Decoder Warning: {e}\r\n",Fore.YELLOW)
                                pass
                
                self.process_data_queues()
                time.sleep(0.001)
            except KeyboardInterrupt: self.running = False
            except Exception as e: 
                self.print_color(f" Loop Error: {e}\r\n",Fore.RED)
                self.running = False
        if self.ser: self.ser.close()

if __name__ == "__main__":
    host = PySerialNetworkHost(SERIAL_PORT, BAUD_RATE)
    host.run()