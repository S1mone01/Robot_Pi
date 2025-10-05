import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import signal
import os
import psutil
import threading
import time


class LauncherNode(Node):
    def __init__(self):
        super().__init__("launcher_node")
        
        # Sottoscrizione al topic
        self.sub = self.create_subscription(
            String, 
            "/robot_launcher", 
            self.callback, 
            10
        )
        
        # Publisher per feedback di stato
        self.status_pub = self.create_publisher(
            String, 
            "/launcher_status", 
            10
        )
        
        # Comandi disponibili
        self.commands = {
            "Start": "ros2 launch create_bringup create_2.py camera:=true navigation:=true foxglove:=false map:=map_palermo.yaml",
            "Start + ia": "ros2 launch create_bringup create_2.py camera:=true navigation:=true foxglove:=false rosbridge:=true map:=map_palermo.yaml",
            "limit area": "ros2 launch create_bringup zone_maker.py",
            "slow area": "ros2 launch create_bringup zone_maker.py node:=speed slow_value:=40",
            "docking": "ros2 launch create_bringup docking.py",
            "undocking": "ros2 launch create_bringup undocking.py",
            "localization": "ros2 launch create_bringup localization.py",
            "Start Mapping": "ros2 launch create_bringup create_2M.py",
            "Save Map": "ros2 run nav2_map_server map_saver_cli -f tmp",
            "Explorer": "ros2 run custom_explorer explorer",
            "Joy": "ros2 launch create_bringup joy_teleop.launch.py"
        }
        
        # Dizionario per tracciare i processi attivi con i loro nomi
        self.active_processes = {}
        self.process_lock = threading.Lock()
        self.foxglove_process = None
        
        # Avvia Foxglove Bridge automaticamente
        self.start_foxglove_bridge()
        
        # Timer per monitorare lo stato dei processi
        self.monitor_timer = self.create_timer(5.0, self.monitor_processes)
        
        self.get_logger().info("LauncherNode inizializzato e pronto")

    def start_foxglove_bridge(self):
        """Avvia Foxglove Bridge in modo separato"""
        try:
            cmd = "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 host:=0.0.0.0"
            self.get_logger().info("🦊 Avvio Foxglove Bridge...")
            
            # Controlla se è già in esecuzione
            if self.is_foxglove_running():
                self.get_logger().info("Foxglove Bridge già in esecuzione")
                return
            
            self.foxglove_process = subprocess.Popen(
                cmd, 
                shell=True,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.publish_status("Foxglove Bridge avviato")
            
        except Exception as e:
            self.get_logger().error(f"Errore nell'avvio di Foxglove Bridge: {str(e)}")

    def is_foxglove_running(self):
        """Controlla se Foxglove Bridge è già in esecuzione"""
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                cmdline = ' '.join(proc.info['cmdline'] or [])
                if 'foxglove_bridge' in cmdline.lower():
                    return True
        except Exception:
            pass
        return False

    def callback(self, msg: String):
        """Callback per gestire i comandi ricevuti con funzionalità toggle"""
        cmd = msg.data.strip()
        
        self.get_logger().info(f"📨 Ricevuto comando: '{cmd}'")
        
        if cmd == "Stop":
            self.stop_all_processes()
        elif cmd == "Status":
            self.report_status()
        elif cmd == "Restart Foxglove":
            self.restart_foxglove_bridge()
        elif cmd in self.commands:
            # Funzionalità toggle: se è attivo lo ferma, altrimenti lo avvia
            self.toggle_process(cmd)
        else:
            self.get_logger().warn(f"⚠️ Comando sconosciuto: '{cmd}'")
            self.publish_status(f"Comando sconosciuto: {cmd}")

    def toggle_process(self, cmd_name):
        """Toggle del processo: avvia se inattivo, termina se attivo"""
        with self.process_lock:
            # Verifica se il processo è già attivo
            if cmd_name in self.active_processes:
                process = self.active_processes[cmd_name]
                if process.poll() is None:  # Processo ancora in esecuzione
                    self.get_logger().info(f"🔄 Processo '{cmd_name}' attivo - procedo con terminazione")
                    self.stop_single_process(cmd_name)
                    return
                else:
                    # Processo terminato, rimuovilo e avviane uno nuovo
                    self.get_logger().info(f"Processo '{cmd_name}' era terminato - riavvio")
                    del self.active_processes[cmd_name]
            
            # Avvia il processo
            self.start_process(cmd_name)

    def start_process(self, cmd_name):
        """Avvia un processo specifico"""
        try:
            self.get_logger().info(f"🚀 Avvio comando: {cmd_name}")
            
            process = subprocess.Popen(
                self.commands[cmd_name],
                shell=True,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.active_processes[cmd_name] = process
            self.publish_status(f"✅ Processo '{cmd_name}' avviato (PID: {process.pid})")
            
            # Avvia un thread per monitorare il processo
            monitor_thread = threading.Thread(
                target=self.monitor_single_process,
                args=(cmd_name, process),
                daemon=True
            )
            monitor_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"❌ Errore nell'avvio del comando '{cmd_name}': {str(e)}")
            self.publish_status(f"Errore avvio '{cmd_name}': {str(e)}")

    def stop_single_process(self, cmd_name):
        """Arresta un singolo processo con terminazione graceful (come Ctrl+C)"""
        try:
            process = self.active_processes.get(cmd_name)
            if not process or process.poll() is not None:
                self.get_logger().warn(f"Processo '{cmd_name}' non trovato o già terminato")
                if cmd_name in self.active_processes:
                    del self.active_processes[cmd_name]
                return
            
            self.get_logger().info(f"🛑 Arresto processo: {cmd_name} (PID: {process.pid})")
            
            try:
                pgid = os.getpgid(process.pid)
                
                # Primo tentativo: SIGINT (equivalente a Ctrl+C)
                self.get_logger().info(f"Invio SIGINT a '{cmd_name}'...")
                os.killpg(pgid, signal.SIGINT)
                
                # Attendi 5 secondi per terminazione graceful
                try:
                    process.wait(timeout=5)
                    self.get_logger().info(f"✅ Processo '{cmd_name}' terminato con SIGINT")
                    self.publish_status(f"🛑 Processo '{cmd_name}' terminato")
                except subprocess.TimeoutExpired:
                    # Secondo tentativo: SIGTERM
                    self.get_logger().warn(f"Timeout SIGINT, invio SIGTERM a '{cmd_name}'...")
                    os.killpg(pgid, signal.SIGTERM)
                    
                    try:
                        process.wait(timeout=5)
                        self.get_logger().info(f"✅ Processo '{cmd_name}' terminato con SIGTERM")
                        self.publish_status(f"🛑 Processo '{cmd_name}' terminato (SIGTERM)")
                    except subprocess.TimeoutExpired:
                        # Ultimo tentativo: SIGKILL
                        self.get_logger().warn(f"⚠️ Force kill del processo '{cmd_name}'")
                        os.killpg(pgid, signal.SIGKILL)
                        process.wait()
                        self.publish_status(f"🛑 Processo '{cmd_name}' forzato (SIGKILL)")
                
            except ProcessLookupError:
                self.get_logger().warn(f"Processo '{cmd_name}' già terminato")
            
            # Rimuovi dalla lista dei processi attivi
            if cmd_name in self.active_processes:
                del self.active_processes[cmd_name]
                
        except Exception as e:
            self.get_logger().error(f"❌ Errore nell'arresto del processo '{cmd_name}': {str(e)}")
            # Rimuovi comunque dalla lista
            if cmd_name in self.active_processes:
                del self.active_processes[cmd_name]

    def monitor_single_process(self, cmd_name, process):
        """Monitora un singolo processo in un thread separato"""
        try:
            stdout, stderr = process.communicate()
            return_code = process.returncode
            
            with self.process_lock:
                if return_code == 0 or return_code == -2:  # -2 è spesso SIGINT
                    self.get_logger().info(f"✅ Processo '{cmd_name}' terminato normalmente (code: {return_code})")
                elif return_code == -15:  # SIGTERM
                    self.get_logger().info(f"✅ Processo '{cmd_name}' terminato via SIGTERM")
                elif return_code is not None:
                    self.get_logger().warn(f"⚠️ Processo '{cmd_name}' terminato con codice {return_code}")
                    if stderr:
                        error_msg = stderr.decode()[:500]  # Primi 500 caratteri
                        self.get_logger().error(f"Stderr: {error_msg}")
                
                # Rimuovi il processo dalla lista se ancora presente
                if cmd_name in self.active_processes:
                    del self.active_processes[cmd_name]
                    self.publish_status(f"Processo '{cmd_name}' terminato")
                    
        except Exception as e:
            self.get_logger().error(f"Errore nel monitoraggio del processo '{cmd_name}': {str(e)}")
            with self.process_lock:
                if cmd_name in self.active_processes:
                    del self.active_processes[cmd_name]

    def monitor_processes(self):
        """Timer callback per monitorare periodicamente i processi"""
        with self.process_lock:
            dead_processes = []
            
            for cmd_name, process in self.active_processes.items():
                if process.poll() is not None:
                    dead_processes.append(cmd_name)
            
            # Rimuovi i processi terminati
            for cmd_name in dead_processes:
                self.get_logger().info(f"🔄 Processo '{cmd_name}' rimosso dalla lista (terminato)")
                del self.active_processes[cmd_name]

    def stop_all_processes(self):
        """Arresta tutti i processi attivi (escluso Foxglove Bridge)"""
        self.get_logger().info("🔴 Arresto di tutti i processi attivi (Foxglove Bridge escluso)...")
        
        with self.process_lock:
            process_list = list(self.active_processes.keys())
        
        stopped_count = 0
        for cmd_name in process_list:
            self.stop_single_process(cmd_name)
            stopped_count += 1
        
        self.publish_status(f"Arrestati {stopped_count} processi")
        self.get_logger().info(f"✅ Arrestati {stopped_count} processi")

    def restart_foxglove_bridge(self):
        """Riavvia Foxglove Bridge"""
        self.get_logger().info("🔄 Riavvio Foxglove Bridge...")
        
        # Arresta il processo corrente se esiste
        if self.foxglove_process and self.foxglove_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.foxglove_process.pid), signal.SIGINT)
                self.foxglove_process.wait(timeout=3)
            except Exception as e:
                self.get_logger().warn(f"Errore nell'arresto di Foxglove: {str(e)}")
                try:
                    os.killpg(os.getpgid(self.foxglove_process.pid), signal.SIGKILL)
                except Exception:
                    pass
        
        # Riavvia
        time.sleep(1)
        self.start_foxglove_bridge()

    def report_status(self):
        """Riporta lo stato corrente"""
        with self.process_lock:
            active_count = len(self.active_processes)
            active_names = list(self.active_processes.keys())
        
        foxglove_status = "attivo" if (self.foxglove_process and self.foxglove_process.poll() is None) else "inattivo"
        
        status_msg = f"Processi attivi: {active_count}, Foxglove: {foxglove_status}"
        if active_count > 0:
            status_msg += f" [{', '.join(active_names)}]"
        
        self.get_logger().info(f"📊 Status: {status_msg}")
        self.publish_status(status_msg)

    def publish_status(self, message):
        """Pubblica un messaggio di stato"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)

    def cleanup(self):
        """Pulizia prima della chiusura del nodo"""
        self.get_logger().info("🧹 Pulizia del nodo...")
        self.stop_all_processes()
        
        # Arresta anche Foxglove se necessario
        if self.foxglove_process and self.foxglove_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.foxglove_process.pid), signal.SIGINT)
                self.foxglove_process.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.foxglove_process.pid), signal.SIGKILL)
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = LauncherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interruzione ricevuta")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
