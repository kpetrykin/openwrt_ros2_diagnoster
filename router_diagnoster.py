import rclpy
from rclpy.node import Node
import diagnostic_msgs
import diagnostic_updater
import random
import time

from router_monitor import RouterMonitor

class RouterDiagnoster(Node):
    def __init__(self, router_monitor) -> None:
        super().__init__('router_diagnoster')

        self._router_monitor = router_monitor
        
        self.declare_parameter('cpu_critical_level', 0.9)
        self._cpu_critical_level = self.get_parameter('cpu_critical_level')

    def diagnose_cpu(self, status_updater):
        diag_status =  diagnostic_msgs.msg.DiagnosticStatus.ERROR
        diag_message = 'Unknown error'
        
        try:
            cpu_data = self._router_monitor.get_cpu()
            
            diag_status =  diagnostic_msgs.msg.DiagnosticStatus.OK
            diag_message = 'CPU is OK'
            
            status_updater.add('1 min average load', str(cpu_data[0]))
            
            status_updater.add('5 min average load', str(cpu_data[1]))
            if float(cpu_data[1]) > self._cpu_critical_level.value:
                diag_status =  diagnostic_msgs.msg.DiagnosticStatus.WARN
                diag_message = '5 min average cpu load is too high!'
                
            status_updater.add('15 min average load', str(cpu_data[2]))
            if float(cpu_data[2]) > self._cpu_critical_level.value:
                diag_status =  diagnostic_msgs.msg.DiagnosticStatus.ERROR
                diag_message = '15 min average cpu load is too high!'
            
            
        except Exception as e:
            diag_status =  diagnostic_msgs.msg.DiagnosticStatus.ERROR
            diag_message = f'Error getting CPU data: {e}'
            
        status_updater.summary(
           diag_status,
           diag_message)
        
        return status_updater
        
def main(args=None):
    rclpy.init(args=args)
    
    rm = RouterMonitor(host="http://192.168.1.1/ubus", username="root", password="123")
    rd = RouterDiagnoster(rm)
    
    updater = diagnostic_updater.Updater(rd)
    updater.setHardwareID('Kroks_router')
    
    updater.add('/router/cpu', rd.diagnose_cpu)
    
    rclpy.spin(rd)
    rd.destroy_node()
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()