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
            "Explorer": "ros2 run custom_explorer explorer"
        }
        
        # Dizionario per tracciare i processi attivi con i loro nomi
        self.active_processes = {}
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
                preexec_fn=os.setsid,  # Crea un nuovo group per gestire meglio il processo
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
        """Callback per gestire i comandi ricevuti"""
        cmd = msg.data.strip()
        
        self.get_logger().info(f"📨 Ricevuto comando: '{cmd}'")
        
        if cmd == "Stop":
            self.stop_all_processes()
        elif cmd == "Status":
            self.report_status()
        elif cmd == "Restart Foxglove":
            self.restart_foxglove_bridge()
        elif cmd in self.commands:
            self.start_process(cmd)
        else:
            self.get_logger().warn(f"⚠️ Comando sconosciuto: '{cmd}'")
            self.publish_status(f"Comando sconosciuto: {cmd}")

    def start_process(self, cmd_name):
        """Avvia un processo specifico"""
        try:
            # Verifica se il processo è già attivo
            if cmd_name in self.active_processes:
                if self.active_processes[cmd_name].poll() is None:
                    self.get_logger().warn(f"Il processo '{cmd_name}' è già in esecuzione")
                    return
                else:
                    # Il processo è terminato, rimuovilo dalla lista
                    del self.active_processes[cmd_name]
            
            self.get_logger().info(f"🚀 Avvio comando: {cmd_name}")
            
            process = subprocess.Popen(
                self.commands[cmd_name],
                shell=True,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.active_processes[cmd_name] = process
            self.publish_status(f"Processo '{cmd_name}' avviato (PID: {process.pid})")
            
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

    def monitor_single_process(self, cmd_name, process):
        """Monitora un singolo processo in un thread separato"""
        try:
            stdout, stderr = process.communicate()
            return_code = process.returncode
            
            if return_code != 0:
                self.get_logger().warn(f"⚠️ Processo '{cmd_name}' terminato con codice {return_code}")
                if stderr:
                    self.get_logger().error(f"Stderr: {stderr.decode()}")
            else:
                self.get_logger().info(f"✅ Processo '{cmd_name}' terminato normalmente")
                
        except Exception as e:
            self.get_logger().error(f"Errore nel monitoraggio del processo '{cmd_name}': {str(e)}")
        finally:
            # Rimuovi il processo dalla lista se ancora presente
            if cmd_name in self.active_processes:
                del self.active_processes[cmd_name]

    def monitor_processes(self):
        """Timer callback per monitorare periodicamente i processi"""
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
        
        stopped_count = 0
        
        for cmd_name, process in list(self.active_processes.items()):
            try:
                if process.poll() is None:  # Il processo è ancora attivo
                    self.get_logger().info(f"Arresto processo: {cmd_name}")
                    
                    # Invia SIGTERM prima
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    
                    # Aspetta 5 secondi per la terminazione graceful
                    try:
                        process.wait(timeout=5)
                        self.get_logger().info(f"✅ Processo '{cmd_name}' terminato correttamente")
                    except subprocess.TimeoutExpired:
                        # Force kill se non risponde
                        self.get_logger().warn(f"⚠️ Force kill del processo '{cmd_name}'")
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        process.wait()
                    
                    stopped_count += 1
                    
            except Exception as e:
                self.get_logger().error(f"❌ Errore nell'arresto del processo '{cmd_name}': {str(e)}")
        
        self.active_processes.clear()
        self.publish_status(f"Arrestati {stopped_count} processi")
        self.get_logger().info(f"✅ Arrestati {stopped_count} processi")

    def restart_foxglove_bridge(self):
        """Riavvia Foxglove Bridge"""
        self.get_logger().info("🔄 Riavvio Foxglove Bridge...")
        
        # Arresta il processo corrente se esiste
        if self.foxglove_process and self.foxglove_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.foxglove_process.pid), signal.SIGTERM)
                self.foxglove_process.wait(timeout=3)
            except Exception as e:
                self.get_logger().warn(f"Errore nell'arresto di Foxglove: {str(e)}")
        
        # Riavvia
        time.sleep(1)
        self.start_foxglove_bridge()

    def report_status(self):
        """Riporta lo stato corrente"""
        active_count = len(self.active_processes)
        foxglove_status = "attivo" if (self.foxglove_process and self.foxglove_process.poll() is None) else "inattivo"
        
        status_msg = f"Processi attivi: {active_count}, Foxglove: {foxglove_status}"
        if active_count > 0:
            status_msg += f" [{', '.join(self.active_processes.keys())}]"
        
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
                os.killpg(os.getpgid(self.foxglove_process.pid), signal.SIGTERM)
                self.foxglove_process.wait(timeout=3)
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
