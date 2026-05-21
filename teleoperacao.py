import socket
import threading
import time
import struct
import json
import tkinter as tk
from tkinter import messagebox, scrolledtext
import re

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import collections

# ==============================================================================
# CLASSE BACKEND 1 - DobotAPI
# ==============================================================================
class DobotAPI:
    def __init__(self, ip):
        self.ip = ip
        self.DASHBOARD_PORT = 29999
        self.FEEDBACK_PORT = 30004

        self.dashboard_socket = None
        self.feedback_socket = None

        self.feedback_thread = None
        self.keep_alive_thread = None
        self.is_running = False

        self.feedback_data = {"TCPPose": [0.0] * 6, "JointAngle": [0.0] * 6}
        self.data_lock = threading.Lock()

    def connect(self):
        try:
            self.dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard_socket.settimeout(3)
            self.dashboard_socket.connect((self.ip, self.DASHBOARD_PORT))

            self.feedback_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.feedback_socket.settimeout(1)
            self.feedback_socket.connect((self.ip, self.FEEDBACK_PORT))

            try:
                response, _ = self._send_command("RequestControl()")
            except Exception as e:
                if "-10000" in str(e): response = "0,{},Firmware antigo;"
                else: raise e

            self.is_running = True
            self.feedback_thread = threading.Thread(target=self._read_feedback_loop, daemon=True)
            self.feedback_thread.start()
            self.keep_alive_thread = threading.Thread(target=self._keep_alive_loop, daemon=True)
            self.keep_alive_thread.start()

            return True, response

        except socket.timeout:
            self._cleanup_sockets()
            raise Exception(f"Timeout: Não foi possível conectar a {self.ip}")
        except Exception as e:
            self._cleanup_sockets()
            raise Exception(f"Erro ao conectar: {e}")

    def disconnect(self):
        self.is_running = False
        if self.keep_alive_thread and self.keep_alive_thread.is_alive(): self.keep_alive_thread.join(timeout=1)
        if self.feedback_thread and self.feedback_thread.is_alive(): self.feedback_thread.join(timeout=1)
        try:
            if self.dashboard_socket: self._send_command("DisableRobot()")
        except: pass
        self._cleanup_sockets()

    def _cleanup_sockets(self):
        if self.dashboard_socket:
            self.dashboard_socket.close()
            self.dashboard_socket = None
        if self.feedback_socket:
            self.feedback_socket.close()
            self.feedback_socket = None

    def _send_command(self, command, log=True):
        if not self.dashboard_socket: raise Exception("Não conectado")
        try:
            self.dashboard_socket.sendall(command.encode('ascii'))
            response = self.dashboard_socket.recv(1024).decode('ascii')
            if log: print(f"CMD: {command} | RES: {response}")
            error_id, result_id = self._parse_response(response)
            if error_id != 0: raise Exception(f"Erro {error_id} para '{command}'. Resposta: {response}")
            return response, result_id
        except socket.timeout:
            self.disconnect()
            raise Exception("Timeout na porta Dashboard")
        except Exception as e: raise e

    def _parse_response(self, response):
        if not response: raise Exception("Resposta vazia")
        error_match = re.match(r"(-?\d+),", response)
        if not error_match: raise Exception(f"Formato inválido: {response}")
        error_id = int(error_match.group(1))
        result_id = None
        result_match = re.search(r"\{(\d+)\}", response)
        if result_match: result_id = int(result_match.group(1))
        return error_id, result_id

    def _send_silent(self, command):
        if not self.is_running or not self.dashboard_socket: return
        try: self._send_command(command, log=False)
        except: pass

    def _read_feedback_loop(self):
        while self.is_running:
            try:
                data = self.feedback_socket.recv(1440)
                if not data or len(data) < 1440: continue
                tcp_pose = struct.unpack_from('<6d', data, 624)
                joint_angle = struct.unpack_from('<6d', data, 432)
                with self.data_lock:
                    self.feedback_data["TCPPose"] = tcp_pose
                    self.feedback_data["JointAngle"] = joint_angle
            except socket.timeout: continue
            except Exception as e:
                if self.is_running: print(f"Erro feedback: {e}")
                break

    def _keep_alive_loop(self):
        while self.is_running:
            for _ in range(50):
                if not self.is_running: break
                time.sleep(1)
            if not self.is_running: break
            self._send_silent("RobotMode()")

    def get_feedback(self):
        with self.data_lock: return self.feedback_data.copy()

    def EnableRobot(self): return self._send_command("EnableRobot()")
    def DisableRobot(self): return self._send_command("DisableRobot()")
    def ClearError(self): 
        self._send_command("ClearError()")
        return self._send_command("Continue()")
    def RobotMode(self): return self._send_command("RobotMode()")
    def SpeedFactor(self, ratio): return self._send_command(f"SpeedFactor({ratio})")
    def MovJ(self, x, y, z, rx, ry, rz): return self._send_command(f"MovJ(pose={{{x},{y},{z},{rx},{ry},{rz}}})")
    def MovL(self, x, y, z, rx, ry, rz): return self._send_command(f"MovL(pose={{{x},{y},{z},{rx},{ry},{rz}}})")
    def JointMovJ(self, j1, j2, j3, j4, j5, j6): return self._send_command(f"MovJ(joint={{{j1},{j2},{j3},{j4},{j5},{j6}}})")
    def RelMovLUser(self, x, y, z, rx, ry, rz): return self._send_command(f"RelMovLUser({x}, {y}, {z}, {rx}, {ry}, {rz})")
    def Halt(self): return self._send_command("Halt()")
    def Pause(self): return self._send_command("Pause()")
    def ConfigToolOffset(self, tool_id, x, y, z, rx, ry, rz):
        self._send_command(f"SetTool({tool_id}, {{{x}, {y}, {z}, {rx}, {ry}, {rz}}})")
        return self._send_command(f"Tool({tool_id})")
    def StartDrag(self): return self._send_command("StartDrag()")
    def StopDrag(self): return self._send_command("StopDrag()")
    def Stop(self): return self._send_command("Stop()")
    def EmergencyStop(self): return self._send_command("EmergencyStop()")
    def SendDirectCommand(self, cmd): return self._send_command(cmd)


# ==============================================================================
# CLASSE BACKEND 2 - ESP32 UDP
# ==============================================================================
class ESP32Client:
    def __init__(self, max_points=200):
        self.sock = None
        self.thread = None
        self.is_connected = False 
        self.max_points = max_points
        self.t_data = collections.deque(maxlen=self.max_points)
        self.p_data = collections.deque(maxlen=self.max_points)
        # latest_data: t=sequencial(ou tempo local), p=peso, a=seq, va=voltagem
        self.latest_data = {"t": 0.0, "p": 0.0, "a": 0, "va": 0.0}
        self.data_lock = threading.Lock()
        self.start_time = time.time()

    def connect(self, ip, port=8080):
        if self.is_connected: return
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('', port))
            self.sock.settimeout(1.0)
            
            self.is_connected = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print(f"UDP Server iniciado na porta {port}. Aguardando ESP32...")
        except Exception as e:
            print(f"Erro ao iniciar UDP Server: {e}")
            self.is_connected = False

    def disconnect(self):
        self.is_connected = False
        if self.sock:
            self.sock.close()
            self.sock = None

    def _listen_loop(self):
        while self.is_connected:
            try:
                data, addr = self.sock.recvfrom(1024)
                if len(data) >= 12:
                    seq, peso, volt = struct.unpack('<Iff', data[:12])
                    
                    elapsed = time.time() - self.start_time
                    
                    with self.data_lock:
                        self.latest_data = {"t": elapsed, "p": peso, "a": seq, "va": volt}
                        self.t_data.append(elapsed)
                        self.p_data.append(peso)
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_connected: print(f"Erro no loop UDP: {e}")
                break

    def get_data(self):
        with self.data_lock: 
            return self.latest_data.copy(), list(self.t_data), list(self.p_data)


# ==============================================================================
# CLASSE FRONTEND - DobotControlApp 
# ==============================================================================
class DobotControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Teleoperação DoboT CR10A")
        self.root.geometry("1450x900") 

        self.api = None
        self.esp_api = ESP32Client(max_points=200)
        self.is_dragging = False 
        self.current_tool_feedback = "Padrão de Fábrica"

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_gui_loops()

    def _build_ui(self):
        main_paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ====== LADO ESQUERDO: CONTROLE DO ROBÔ ======
        left_frame = tk.Frame(main_paned)
        main_paned.add(left_frame, minsize=650)

        top_frame = tk.Frame(left_frame, pady=5)
        top_frame.pack(side=tk.TOP, fill="x")
        tk.Label(top_frame, text="IP CR10A:").pack(side=tk.LEFT, padx=5)
        self.ip_entry = tk.Entry(top_frame, width=15)
        self.ip_entry.insert(0, "192.168.5.1")
        self.ip_entry.pack(side=tk.LEFT, padx=5)
        self.connect_button = tk.Button(top_frame, text="Conectar", command=self.connect_robot)
        self.connect_button.pack(side=tk.LEFT, padx=5)
        self.disconnect_button = tk.Button(top_frame, text="Desconectar", command=self.disconnect_robot, state=tk.DISABLED)
        self.disconnect_button.pack(side=tk.LEFT, padx=5)
        self.status_label = tk.Label(top_frame, text="● Desconectado", fg="red")
        self.status_label.pack(side=tk.LEFT, padx=10)

        control_frame = tk.Frame(left_frame, pady=5)
        control_frame.pack(fill="x")
        self.enable_button = tk.Button(control_frame, text="Habilitar", command=self.enable_robot, state=tk.DISABLED)
        self.enable_button.pack(side=tk.LEFT, padx=5)
        self.disable_button = tk.Button(control_frame, text="Desabilitar", command=self.disable_robot, state=tk.DISABLED)
        self.disable_button.pack(side=tk.LEFT, padx=5)
        self.clear_error_button = tk.Button(control_frame, text="Limpar Erros", command=self.clear_errors, state=tk.DISABLED)
        self.clear_error_button.pack(side=tk.LEFT, padx=5)

        safety_frame = tk.Frame(left_frame, pady=5)
        safety_frame.pack(fill="x")
        self.drag_button = tk.Button(safety_frame, text="Ativar Drag", command=self.toggle_drag, state=tk.DISABLED, bg="lightgray")
        self.drag_button.pack(side=tk.LEFT, padx=5)
        self.stop_button = tk.Button(safety_frame, text="Parar (Stop)", command=self.stop_robot, state=tk.DISABLED, bg="orange")
        self.stop_button.pack(side=tk.LEFT, padx=15)
        self.estop_button = tk.Button(safety_frame, text="EMERGÊNCIA", command=self.emergency_stop, state=tk.DISABLED, bg="red", fg="white")
        self.estop_button.pack(side=tk.LEFT, padx=5)

        speed_frame = tk.Frame(left_frame, pady=5)
        speed_frame.pack(fill="x")
        tk.Label(speed_frame, text="Velocidade (%):").pack(side=tk.LEFT, padx=5)
        self.speed_entry = tk.Entry(speed_frame, width=6)
        self.speed_entry.insert(0, "50")
        self.speed_entry.pack(side=tk.LEFT, padx=5)
        self.speed_button = tk.Button(speed_frame, text="Definir", command=self.set_speed, state=tk.DISABLED)
        self.speed_button.pack(side=tk.LEFT, padx=5)

        direct_frame = tk.LabelFrame(left_frame, text="Comando Direto", pady=5, padx=5)
        direct_frame.pack(fill="x", pady=5)
        self.direct_command_entry = tk.Entry(direct_frame, width=50)
        self.direct_command_entry.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
        self.send_direct_button = tk.Button(direct_frame, text="Enviar", command=self.send_direct_command, state=tk.DISABLED)
        self.send_direct_button.pack(side=tk.LEFT, padx=5)

        tool_frame = tk.LabelFrame(left_frame, text="Tool Offset (TCP)", pady=5, padx=5)
        tool_frame.pack(fill="x", pady=5)
        tk.Label(tool_frame, text="ID:").grid(row=0, column=0)
        self.tool_id_entry = tk.Entry(tool_frame, width=5)
        self.tool_id_entry.grid(row=0, column=1, padx=5)
        self.tool_entries = {}
        t_labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for i, label in enumerate(t_labels):
            tk.Label(tool_frame, text=label).grid(row=0, column=2 + (i*2), padx=2)
            entry = tk.Entry(tool_frame, width=5)
            entry.grid(row=0, column=2 + (i*2) + 1, padx=2)
            self.tool_entries[label] = entry
        self.set_tool_button = tk.Button(tool_frame, text="Set", command=self.set_tool_offset, state=tk.DISABLED)
        self.set_tool_button.grid(row=0, column=15, padx=5)

        move_frame = tk.LabelFrame(left_frame, text="Movimento Cartesiano", pady=5, padx=5)
        move_frame.pack(fill="x", pady=5)
        self.pose_entries = {}
        for i, label in enumerate(t_labels):
            tk.Label(move_frame, text=label).grid(row=0, column=i*2, padx=2)
            entry = tk.Entry(move_frame, width=7)
            entry.grid(row=0, column=i*2 + 1, padx=2)
            self.pose_entries[label] = entry
        self.send_movj_button = tk.Button(move_frame, text="MovJ", command=self.send_movj, state=tk.DISABLED)
        self.send_movj_button.grid(row=0, column=12, padx=2)
        self.send_movl_button = tk.Button(move_frame, text="MovL", command=self.send_movl, state=tk.DISABLED)
        self.send_movl_button.grid(row=0, column=13, padx=2)

        rel_frame = tk.LabelFrame(left_frame, text="Movimento Relativo", pady=5, padx=5)
        rel_frame.pack(fill="x", pady=5)
        self.rel_entries = {}
        for i, label in enumerate(["dX", "dY", "dZ", "dRx", "dRy", "dRz"]):
            tk.Label(rel_frame, text=label).grid(row=0, column=i*2, padx=2)
            entry = tk.Entry(rel_frame, width=7)
            entry.insert(0, "0.0")
            entry.grid(row=0, column=i*2 + 1, padx=2)
            self.rel_entries[label] = entry
        self.send_rel_button = tk.Button(rel_frame, text="Mover Relativo", command=self.send_relative_move, state=tk.DISABLED)
        self.send_rel_button.grid(row=0, column=12, padx=10)

        joint_frame = tk.LabelFrame(left_frame, text="Juntas (Graus)", pady=5, padx=5)
        joint_frame.pack(fill="x", pady=5)
        self.joint_entries = {}
        for i, label in enumerate(["J1", "J2", "J3", "J4", "J5", "J6"]):
            tk.Label(joint_frame, text=label).grid(row=0, column=i*2, padx=2)
            entry = tk.Entry(joint_frame, width=7)
            entry.grid(row=0, column=i*2 + 1, padx=2)
            self.joint_entries[label] = entry
        self.send_joint_button = tk.Button(joint_frame, text="Set Joints", command=self.send_joint_movj, state=tk.DISABLED)
        self.send_joint_button.grid(row=0, column=12, padx=2)

        feedback_frame = tk.LabelFrame(left_frame, text="Monitoramento do Robô", pady=5, padx=5)
        feedback_frame.pack(fill="x", pady=5)
        self.tool_feedback_label = tk.Label(feedback_frame, text="Tool: Padrão", font=("Courier", 10), fg="purple")
        self.tool_feedback_label.pack(anchor="w")
        self.tcp_feedback_label = tk.Label(feedback_frame, text="TCP : N/A", font=("Courier", 10))
        self.tcp_feedback_label.pack(anchor="w")
        self.joint_feedback_label = tk.Label(feedback_frame, text="Juntas: N/A", font=("Courier", 10), fg="blue")
        self.joint_feedback_label.pack(anchor="w")

        log_frame = tk.LabelFrame(left_frame, text="Log", pady=5, padx=5)
        log_frame.pack(fill="both", expand=True, pady=5)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=5, state=tk.DISABLED)
        self.log_text.pack(fill="both", expand=True)

        # ====== LADO DIREITO: CÉLULA DE CARGA ESP32 ======
        right_frame = tk.Frame(main_paned)
        main_paned.add(right_frame, minsize=550)

        esp_top_frame = tk.Frame(right_frame, pady=5)
        esp_top_frame.pack(side=tk.TOP, fill="x")
        tk.Label(esp_top_frame, text="Porta UDP Local:").pack(side=tk.LEFT, padx=5)
        self.esp_port_entry = tk.Entry(esp_top_frame, width=8)
        self.esp_port_entry.insert(0, "8080")
        self.esp_port_entry.pack(side=tk.LEFT, padx=5)
        self.esp_connect_btn = tk.Button(esp_top_frame, text="Iniciar", command=self.connect_esp)
        self.esp_connect_btn.pack(side=tk.LEFT, padx=5)
        self.esp_disconnect_btn = tk.Button(esp_top_frame, text="Parar", command=self.disconnect_esp, state=tk.DISABLED)
        self.esp_disconnect_btn.pack(side=tk.LEFT, padx=5)
        self.esp_status_label = tk.Label(esp_top_frame, text="● Offline", fg="red")
        self.esp_status_label.pack(side=tk.LEFT, padx=10)

        cards_frame = tk.Frame(right_frame, pady=10)
        cards_frame.pack(fill="x")
        self.lbl_t = self._create_card(cards_frame, "Tempo", "0s")
        self.lbl_p = self._create_card(cards_frame, "Peso", "0kg")
        self.lbl_a = self._create_card(cards_frame, "Sequencial", "0")
        self.lbl_va = self._create_card(cards_frame, "Vadc", "0V")

        graph_frame = tk.LabelFrame(right_frame, text="Gráfico de Peso", padx=5, pady=5)
        graph_frame.pack(fill="both", expand=True, padx=5, pady=5)
        self.fig, self.ax = plt.subplots(figsize=(6, 4), dpi=100)
        self.line, = self.ax.plot([], [], 'b-', linewidth=2, label="Peso (kg)")
        self.ax.set_ylim(-0.5, 5) # Ajuste conforme sua carga
        self.ax.set_xlabel("Tempo Decorrido (s)")
        self.ax.set_ylabel("Peso (kg)")
        self.ax.grid(True, linestyle='--', alpha=0.6)
        self.ax.legend(loc="upper left")
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _create_card(self, parent, title, initial_val):
        frame = tk.Frame(parent, relief=tk.RIDGE, borderwidth=2, padx=10, pady=5)
        frame.pack(side=tk.LEFT, expand=True, fill="x", padx=5)
        tk.Label(frame, text=title, font=("Arial", 10)).pack()
        lbl_val = tk.Label(frame, text=initial_val, font=("Arial", 14, "bold"), fg="#007bff")
        lbl_val.pack()
        return lbl_val

    def update_gui_loops(self):
        # Update Robô
        if self.api and self.api.is_running:
            try:
                feedback = self.api.get_feedback()
                tcp = feedback.get("TCPPose", [0.0]*6)
                jnt = feedback.get("JointAngle", [0.0]*6)
                self.tcp_feedback_label.config(text=f"TCP : X:{tcp[0]:.1f} Y:{tcp[1]:.1f} Z:{tcp[2]:.1f} Rx:{tcp[3]:.1f} Ry:{tcp[4]:.1f} Rz:{tcp[5]:.1f}")
                self.joint_feedback_label.config(text=f"Juntas: J1:{jnt[0]:.1f} J2:{jnt[1]:.1f} J3:{jnt[2]:.1f} J4:{jnt[3]:.1f} J5:{jnt[4]:.1f} J6:{jnt[5]:.1f}")
                
                focused = self.root.focus_get()
                for i, label in enumerate(["X", "Y", "Z", "Rx", "Ry", "Rz"]):
                    if focused != self.pose_entries[label]:
                        self.pose_entries[label].delete(0, tk.END)
                        self.pose_entries[label].insert(0, f"{tcp[i]:.2f}")
                for i, label in enumerate(["J1", "J2", "J3", "J4", "J5", "J6"]):
                    if focused != self.joint_entries[label]:
                        self.joint_entries[label].delete(0, tk.END)
                        self.joint_entries[label].insert(0, f"{jnt[i]:.2f}")
            except: pass

        # Update ESP32 (UDP)
        if self.esp_api.is_connected:
            self.esp_status_label.config(text="● Escutando", fg="green")
            latest, t_hist, p_hist = self.esp_api.get_data()
            self.lbl_t.config(text=f"{latest['t']:.1f}s")
            self.lbl_p.config(text=f"{latest['p']:.3f}kg")
            self.lbl_a.config(text=f"{latest['a']}")
            self.lbl_va.config(text=f"{latest['va']:.3f}V")

            if len(t_hist) > 1:
                self.line.set_data(t_hist, p_hist)
                self.ax.set_xlim(min(t_hist), max(t_hist) + 0.5)
                # Auto-scale Y se o peso sair do range inicial
                if latest['p'] > self.ax.get_ylim()[1]: self.ax.set_ylim(-0.5, latest['p'] + 1)
                self.canvas.draw_idle()
        else:
            self.esp_status_label.config(text="● Offline", fg="red")

        self.root.after(50, self.update_gui_loops)

    def connect_esp(self):
        try:
            port = int(self.esp_port_entry.get())
            self.esp_api.connect("", port)
            self.esp_connect_btn.config(state=tk.DISABLED)
            self.esp_disconnect_btn.config(state=tk.NORMAL)
        except ValueError:
            messagebox.showerror("Erro", "Porta inválida")

    def disconnect_esp(self):
        self.esp_api.disconnect()
        self.esp_connect_btn.config(state=tk.NORMAL)
        self.esp_disconnect_btn.config(state=tk.DISABLED)

    def connect_robot(self):
        ip = self.ip_entry.get()
        self.api = DobotAPI(ip)
        try:
            self.api.connect()
            self.status_label.config(text="● Conectado", fg="green")
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            self._enable_control_buttons()
        except Exception as e:
            messagebox.showerror("Erro", str(e))

    def disconnect_robot(self):
        if self.api:
            self.api.disconnect()
            self.api = None
            self.status_label.config(text="● Desconectado", fg="red")
            self.connect_button.config(state=tk.NORMAL)
            self.disconnect_button.config(state=tk.DISABLED)
            self._disable_control_buttons()

    def _enable_control_buttons(self):
        btns = [self.enable_button, self.disable_button, self.clear_error_button, 
                self.send_movj_button, self.send_movl_button, self.send_rel_button,
                self.drag_button, self.send_joint_button, self.set_tool_button, 
                self.stop_button, self.estop_button, self.speed_button, self.send_direct_button]
        for b in btns: b.config(state=tk.NORMAL)

    def _disable_control_buttons(self):
        btns = [self.enable_button, self.disable_button, self.clear_error_button, 
                self.send_movj_button, self.send_movl_button, self.send_rel_button,
                self.send_joint_button, self.set_tool_button, self.stop_button, 
                self.estop_button, self.speed_button, self.send_direct_button]
        for b in btns: b.config(state=tk.DISABLED)

    def send_api_command(self, func, *args):
        if not self.api: return
        try:
            res, _ = func(*args)
            self.log_message(f"{func.__name__} -> {res}")
        except Exception as e: self.log_message(f"Erro: {e}")

    def log_message(self, msg):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def enable_robot(self): self.send_api_command(self.api.EnableRobot)
    def disable_robot(self): self.send_api_command(self.api.DisableRobot)
    def clear_errors(self): self.send_api_command(self.api.ClearError)
    def stop_robot(self): self.send_api_command(self.api.Stop)
    def emergency_stop(self): self.send_api_command(self.api.EmergencyStop)
    def set_speed(self): self.send_api_command(self.api.SpeedFactor, int(self.speed_entry.get()))
    def send_direct_command(self): self.send_api_command(self.api.SendDirectCommand, self.direct_command_entry.get())
    
    def toggle_drag(self):
        if not self.is_dragging:
            self.send_api_command(self.api.StartDrag)
            self.drag_button.config(text="OFF Drag", bg="yellow")
            self.is_dragging = True
        else:
            self.send_api_command(self.api.StopDrag)
            self.drag_button.config(text="ON Drag", bg="lightgray")
            self.is_dragging = False

    def set_tool_offset(self):
        tid = int(self.tool_id_entry.get())
        v = [float(self.tool_entries[k].get()) for k in ["X", "Y", "Z", "Rx", "Ry", "Rz"]]
        self.send_api_command(self.api.ConfigToolOffset, tid, *v)

    def send_movj(self):
        v = [float(self.pose_entries[k].get()) for k in ["X", "Y", "Z", "Rx", "Ry", "Rz"]]
        self.send_api_command(self.api.MovJ, *v)

    def send_movl(self):
        v = [float(self.pose_entries[k].get()) for k in ["X", "Y", "Z", "Rx", "Ry", "Rz"]]
        self.send_api_command(self.api.MovL, *v)

    def send_joint_movj(self):
        v = [float(self.joint_entries[k].get()) for k in ["J1", "J2", "J3", "J4", "J5", "J6"]]
        self.send_api_command(self.api.JointMovJ, *v)

    def send_relative_move(self):
        d = [float(self.rel_entries[k].get()) for k in ["dX", "dY", "dZ", "dRx", "dRy", "dRz"]]
        curr = self.api.get_feedback().get("TCPPose", [0.0]*6)
        target = [curr[i] + d[i] for i in range(6)]
        self.send_api_command(self.api.MovL, *target)

    def on_closing(self):
        if self.api: self.api.disconnect()
        self.esp_api.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DobotControlApp(root)
    root.mainloop()