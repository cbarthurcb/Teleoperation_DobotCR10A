import socket
import threading
import time
import struct
import tkinter as tk
from tkinter import messagebox

# ==============================================================================
# CLASSE - Controlador PI
# ==============================================================================
class PIController:
    def __init__(self, kp, ki, dt, max_output=2.0):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.max_output = max_output
        self.setpoint = 0.0
        self.integral = 0.0
        self.deadzone = 0.1  # 100 gramas de tolerância

    def update(self, current_value):
        error = self.setpoint - current_value

        # 1. Zona morta (ADCESP)
        if abs(error) < self.deadzone:
            return 0.0, error

        # 2. Reset da integral na mudança de sinal do erro
        # Quando o robô cruza o setpoint, a integral acumulada na direção
        # anterior luta contra a correção — zerá-la permite resposta imediata.
        if (error > 0 and self.integral < 0) or (error < 0 and self.integral > 0):
            self.integral = 0.0

        P = self.kp * error

        # 3. Anti-windup condicional
        # Só acumula integral se a saída provisória ainda não está saturada.
        output_provisorio = P + self.ki * self.integral
        if abs(output_provisorio) < self.max_output:
            self.integral += error * self.dt
            self.integral = max(min(self.integral, 5.0), -5.0)

        I = self.ki * self.integral
        output = P + I

        # 4. Saturação de saída
        output = max(min(output, self.max_output), -self.max_output)

        return output, error

    def reset(self):
        self.integral = 0.0

# ==============================================================================
# BACKEND - Comunicação
# ==============================================================================
class DobotAPI:
    def __init__(self, ip):
        self.ip = ip
        self.dashboard_socket = None
        self.is_running = False
        self.feedback_data = {"TCPPose": [0.0] * 6}
        self.lock = threading.Lock()

    def connect(self):
        try:
            self.dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard_socket.settimeout(2)
            self.dashboard_socket.connect((self.ip, 29999))

            fb_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            fb_sock.connect((self.ip, 30004))

            self.dashboard_socket.sendall("RequestControl()".encode('ascii'))
            self.is_running = True

            def fb_loop():
                while self.is_running:
                    try:
                        data = fb_sock.recv(1440)
                        if len(data) >= 1440:
                            tcp = struct.unpack_from('<6d', data, 624)
                            with self.lock:
                                self.feedback_data["TCPPose"] = tcp
                    except:
                        continue

            threading.Thread(target=fb_loop, daemon=True).start()
            return True
        except:
            return False

    def send_rel_z(self, z_step):
        # +Z sobe (aumenta carga), -Z desce (alivia)
        cmd = f"RelMovLUser(0,0,{z_step:.4f},0,0,0)"
        try:
            self.dashboard_socket.sendall(cmd.encode('ascii'))
        except:
            pass

    def stop(self):
        # Para o robô e limpa a fila de comandos pendentes
        try:
            self.dashboard_socket.sendall("Stop()".encode('ascii'))
            time.sleep(0.1)  
        except:
            pass

    def disconnect(self):
        self.is_running = False
        if self.dashboard_socket:
            self.dashboard_socket.close()


class ESP32Client:
    def __init__(self):
        self.latest_p = 0.0
        self.is_connected = False

    def connect(self, port=8080):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)
        sock.bind(('', port))
        self.is_connected = True

        def listen():
            while self.is_connected:
                try:
                    data, _ = sock.recvfrom(1024)
                    if len(data) >= 12:
                        _, p, _ = struct.unpack('<Iff', data[:12])
                        self.latest_p = p
                except:
                    continue

        threading.Thread(target=listen, daemon=True).start()

    def disconnect(self):
        self.is_connected = False

# ==============================================================================
# INTERFACE GRÁFICA
# ==============================================================================
class DobotControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Controle de Força - Dobot CR10A")
        self.root.geometry("620x780")

        self.api = None
        self.esp = ESP32Client()
        self.pi = PIController(kp=1.5, ki=0.1, dt=0.08)
        self.is_running = False

        self.contact_threshold = 0.05
        self.approach_step = 0.5
        self.stable_time = 2.0  # segundos dentro da deadzone para encerrar

        self._ultimo_dt = 0.0
        self._fase = "parado"
        self._t_estavel = 0.0   # tempo acumulado dentro da deadzone

        self._build_ui()
        self.update_ui()

    def _param_row(self, parent, row, label, var, step, fmt, min_val=0.0, max_val=999.0):
        tk.Label(parent, text=label, anchor="w", width=22).grid(
            row=row, column=0, sticky="w", pady=3
        )

        def decrement():
            novo = max(min_val, round(var.get() - step, 10))
            var.set(float(f"{novo:{fmt}}"))

        def increment():
            novo = min(max_val, round(var.get() + step, 10))
            var.set(float(f"{novo:{fmt}}"))

        tk.Button(parent, text="−", width=2, command=decrement,
                  font=("Arial", 11, "bold")).grid(row=row, column=1, padx=(0, 2))

        tk.Entry(parent, textvariable=var, width=8, justify="center",
                 font=("Arial", 11)).grid(row=row, column=2, padx=2)

        tk.Button(parent, text="+", width=2, command=increment,
                  font=("Arial", 11, "bold")).grid(row=row, column=3, padx=(2, 0))

    def _build_ui(self):
        # --- Conexão ---
        top = tk.Frame(self.root, pady=10)
        top.pack()
        tk.Label(top, text="IP Robô:").pack(side="left")
        self.ent_ip = tk.Entry(top, width=14)
        self.ent_ip.insert(0, "192.168.5.1")
        self.ent_ip.pack(side="left", padx=5)
        tk.Button(top, text="CONECTAR", command=self.connect,
                  bg="#2255aa", fg="white", font=("Arial", 10, "bold")).pack(side="left")

        # --- Parâmetros PI ---
        pi_f = tk.LabelFrame(self.root, text="Parâmetros PI", padx=15, pady=10)
        pi_f.pack(fill="x", padx=15, pady=5)

        self.var_sp = tk.DoubleVar(value=1.0)
        self.var_kp = tk.DoubleVar(value=1.5)
        self.var_ki = tk.DoubleVar(value=0.1)

        self._param_row(pi_f, 0, "Setpoint — alvo (kg):",    self.var_sp,
                        step=0.1,  fmt=".2f", min_val=0.0, max_val=50.0)
        self._param_row(pi_f, 1, "Kp — ganho proporcional:", self.var_kp,
                        step=0.1,  fmt=".2f", min_val=0.0, max_val=20.0)
        self._param_row(pi_f, 2, "Ki — ganho integral:",     self.var_ki,
                        step=0.01, fmt=".3f", min_val=0.0, max_val=5.0)

        # --- Parâmetros de Aproximação ---
        apx_f = tk.LabelFrame(self.root, text="Fase de Aproximação", padx=15, pady=10)
        apx_f.pack(fill="x", padx=15, pady=5)

        self.var_thresh  = tk.DoubleVar(value=0.05)
        self.var_apstep  = tk.DoubleVar(value=0.5)
        self.var_sttime  = tk.DoubleVar(value=2.0)

        self._param_row(apx_f, 0, "Threshold contato (kg):", self.var_thresh,
                        step=0.01, fmt=".3f", min_val=0.0,  max_val=2.0)
        self._param_row(apx_f, 1, "Passo aproximação (mm):", self.var_apstep,
                        step=0.1,  fmt=".2f", min_val=0.05, max_val=5.0)
        self._param_row(apx_f, 2, "Tempo estável (s):",      self.var_sttime,
                        step=0.5,  fmt=".1f", min_val=0.5,  max_val=30.0)

        # --- Botão principal ---
        self.btn_run = tk.Button(
            self.root, text="ATIVAR MALHA FECHADA",
            bg="green", fg="white", font=("Arial", 12, "bold"),
            height=2, command=self.toggle
        )
        self.btn_run.pack(fill="x", padx=15, pady=10)

        # --- Displays de leitura ---
        self.lbl_p = tk.Label(self.root, text="0.000 kg",
                              font=("Arial", 42, "bold"), fg="#1155cc")
        self.lbl_p.pack()

        self.lbl_z = tk.Label(self.root, text="Z: 0.00 mm", font=("Arial", 20))
        self.lbl_z.pack()

        # --- Diagnóstico ---
        diag_f = tk.Frame(self.root)
        diag_f.pack(pady=5)

        self.lbl_fase = tk.Label(diag_f, text="Fase: —",
                                 font=("Arial", 12, "bold"), fg="gray", width=22)
        self.lbl_fase.pack(side="left", padx=10)

        self.lbl_dt = tk.Label(diag_f, text="dt: — ms",
                               font=("Arial", 11), fg="gray", width=14)
        self.lbl_dt.pack(side="left")

        self.lbl_integral = tk.Label(diag_f, text="∫: 0.000",
                                     font=("Arial", 11), fg="gray", width=14)
        self.lbl_integral.pack(side="left")

    # --------------------------------------------------------------------------

    def connect(self):
        self.api = DobotAPI(self.ent_ip.get())
        if self.api.connect():
            self.esp.connect()
            self.api.dashboard_socket.sendall("EnableRobot()".encode('ascii'))
            messagebox.showinfo("Sucesso", "Sistema Conectado.")

    def toggle(self):
        if not self.is_running:
            if not self.api:
                return
            self.pi.reset()
            self.is_running = True
            self._fase = "aproximacao"
            self._t_estavel = 0.0
            self.btn_run.config(text="PARAR CONTROLE", bg="red")
            threading.Thread(target=self._control_loop, daemon=True).start()
        else:
            self.is_running = False
            self._fase = "parado"
            self.api.stop()
            self.btn_run.config(text="ATIVAR MALHA FECHADA", bg="green")

    def _control_loop(self):
        t_anterior = time.time()
        contato_detectado = False  # trava de fase — nunca volta para aproximação
        sinal_anterior = 0         # rastreia a direção do último comando enviado

        while self.is_running:
            t0 = time.time()

            # dt real
            dt_real = max(0.001, min(t0 - t_anterior, 0.5))
            t_anterior = t0
            self.pi.dt = dt_real

            # Lê parâmetros em tempo real
            try:
                self.pi.kp             = self.var_kp.get()
                self.pi.ki             = self.var_ki.get()
                self.pi.setpoint       = self.var_sp.get()
                self.contact_threshold = self.var_thresh.get()
                self.approach_step     = self.var_apstep.get()
                self.stable_time       = self.var_sttime.get()
            except Exception:
                pass

            peso_atual = self.esp.latest_p

            # Transição de fase — ocorre uma única vez por sessão
            if not contato_detectado and peso_atual >= self.contact_threshold:
                contato_detectado = True
                # Para o robô e limpa a fila de comandos acumulados
                # durante a aproximação antes do PI assumir
                self.api.stop()
                self.pi.reset()
                sinal_anterior = 0

            if not contato_detectado:
                # Fase de aproximação: passo fixo, integral zerada
                self._fase = "aproximacao"
                self.pi.reset()
                self.api.send_rel_z(self.approach_step)

            else:
                # Fase de controle PI
                self._fase = "controle"
                acao_z, _ = self.pi.update(peso_atual)

                if abs(acao_z) > 0:
                    sinal_atual = 1 if acao_z > 0 else -1

                    # Mudança de direção — limpa a fila antes de inverter
                    # Isso garante que o robô responde imediatamente à inversão
                    # do PI sem executar comandos antigos da direção oposta
                    if sinal_anterior != 0 and sinal_atual != sinal_anterior:
                        self.api.stop()

                    self.api.send_rel_z(acao_z)
                    sinal_anterior = sinal_atual
                    self._t_estavel = 0.0  # qualquer movimento reseta o contador
                else:
                    # Saída zero = dentro da deadzone — acumula tempo estável
                    self._t_estavel += dt_real
                    self._fase = "estabilizando"

                    if self._t_estavel >= self.stable_time:
                        # Sistema estabilizado — encerra o controle automaticamente
                        self._fase = "estabilizado"
                        self.is_running = False
                        self.root.after(0, self._on_estabilizado)

            self._ultimo_dt = dt_real

            time.sleep(max(0, 0.08 - (time.time() - t0)))

    def _on_estabilizado(self):
        # Chamado pela thread de controle via root.after — seguro para UI
        self._fase = "estabilizado"
        self.btn_run.config(text="ATIVAR MALHA FECHADA", bg="green")

    def update_ui(self):
        self.lbl_p.config(text=f"{self.esp.latest_p:.3f} kg")

        if self.api and self.api.is_running:
            with self.api.lock:
                z = self.api.feedback_data["TCPPose"][2]
            self.lbl_z.config(text=f"Z: {z:.2f} mm")

        cores = {
            "aproximacao":   "orange",
            "controle":      "green",
            "estabilizando": "blue",
            "estabilizado":  "purple",
            "parado":        "gray",
        }

        if self.is_running or self._fase == "estabilizado":
            # Mostra progresso de estabilização no label
            if self._fase == "estabilizando":
                prog = min(self._t_estavel / max(self.stable_time, 0.1), 1.0)
                texto = f"Fase: ESTABILIZANDO {prog*100:.0f}%"
            else:
                texto = f"Fase: {self._fase.upper()}"

            self.lbl_fase.config(text=texto, fg=cores.get(self._fase, "gray"))
            self.lbl_dt.config(text=f"dt: {self._ultimo_dt * 1000:.1f} ms")
            self.lbl_integral.config(text=f"∫: {self.pi.integral:.3f}")
        else:
            self.lbl_fase.config(text="Fase: —", fg="gray")
            self.lbl_dt.config(text="dt: — ms")
            self.lbl_integral.config(text="∫: —")

        self.root.after(100, self.update_ui)

    def on_closing(self):
        self.is_running = False
        if self.api:
            self.api.stop()
            self.api.disconnect()
        self.esp.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = DobotControlApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()