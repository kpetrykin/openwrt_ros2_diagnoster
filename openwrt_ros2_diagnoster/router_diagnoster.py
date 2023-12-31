import rclpy
from rclpy.node import Node
import diagnostic_msgs
import diagnostic_updater
import random
import time

from openwrt_ros2_diagnoster.openwrt_data_getter import OpenWRTDataGetter


class RouterDiagnoster(Node):
    def __init__(self, router_data_getter):
        super().__init__('router_diagnoster')

        self._router_data_getter = router_data_getter

        self.declare_parameter('router_host', '192.168.1.1')
        self._router_host = self.get_parameter('router_host')

        self.declare_parameter('router_username', 'root')
        self._router_username = self.get_parameter('router_username')

        self.declare_parameter('router_password', '123')
        self._router_password = self.get_parameter('router_password')

        self._router_data_getter.connect(self._router_host.value,
                                         self._router_username.value,
                                         self._router_password.value)

        self.declare_parameter('cpu_critical_level', 1.0)
        self._cpu_critical_level = self.get_parameter('cpu_critical_level')

        self.declare_parameter('free_memory_critical_level', 5.0)
        self._free_memory_critical_level = self.get_parameter(
            'free_memory_critical_level')
        
        self.declare_parameter('reliability_critical_level', 20.0)
        self._reliability_critical_level = self.get_parameter('reliability_critical_level')

    # Decorator for diagnose methods
    def diagnose_method(diag_name):
        def decorate(func):
            def inner(self, status_updater):
                diag_status = diagnostic_msgs.msg.DiagnosticStatus.OK
                diag_message = f'{diag_name} is OK'
                try:
                    status_updater, diag_status, diag_message = func(
                        self, status_updater, diag_status, diag_message)
                except Exception as e:
                    diag_status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
                    diag_message = f'Error getting {diag_name} data: {e}'

                status_updater.summary(
                    diag_status,
                    diag_message)

                return status_updater
            return inner
        return decorate

    @diagnose_method('CPU')
    def diagnose_cpu(self, status_updater, diag_status, diag_message):
        cpu_data = self._router_data_getter.get_cpu()

        status_updater.add('1 min average load', str(cpu_data[0]))

        status_updater.add('5 min average load', str(cpu_data[1]))
        if float(cpu_data[1]) > self._cpu_critical_level.value:
            diag_status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            diag_message = '5 min average cpu load is too high!'

        status_updater.add('15 min average load', str(cpu_data[2]))
        if float(cpu_data[2]) > self._cpu_critical_level.value:
            diag_status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            diag_message = '15 min average cpu load is too high!'

        return status_updater, diag_status, diag_message

    @diagnose_method('Memory')
    def diagnose_memory(self, status_updater, diag_status, diag_message):
        memory_data = self._router_data_getter.get_memory()
        status_updater.add('Free memory, %', str(memory_data))

        if float(memory_data) < self._free_memory_critical_level.value:
            diag_status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            diag_message = 'Low level of free memory!'

        return status_updater, diag_status, diag_message

    @diagnose_method('Modem1')
    def diagnose_modem1(self, status_updater, diag_status, diag_message):
        return self._diagnose_modem('modem1', status_updater,
                                    diag_status, diag_message)

    @diagnose_method('Modem2')
    def diagnose_modem2(self, status_updater, diag_status, diag_message):
        return self._diagnose_modem('modem2', status_updater,
                                    diag_status, diag_message)

    def _diagnose_modem(self, modem_name, status_updater, diag_status, diag_message):
        modem_data = self._router_data_getter.get_modem(modem_name)
        for k, v in modem_data.items():
            if k == 'network_type' and v in ['umts', 'evdo']:
                diag_status = diagnostic_msgs.msg.DiagnosticStatus.WARN
                diag_message = 'Celluar network is 3G'
            if k == 'network_type' and v in ['cdma1x', 'gsm']:
                diag_status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
                diag_message = 'Celluar network is 2G!'
            if k == 'online' and v == False and diag_status != diagnostic_msgs.msg.DiagnosticStatus.ERROR:
                diag_status = diagnostic_msgs.msg.DiagnosticStatus.WARN
                diag_message = f'{modem_name} is offline!'
            if k == 'connected_to_network' and v == False:
                diag_status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
                diag_message = f'{modem_name} is not connected to celluar network!'
            if k == 'reliability' and v is not None and diag_status != diagnostic_msgs.msg.DiagnosticStatus.ERROR:
                if float(v) <= -self._reliability_critical_level.value:
                    diag_status = diagnostic_msgs.msg.DiagnosticStatus.WARN
                    diag_message = 'Internet signal reliability low!'
            
            status_updater.add(k, str(v))

        return status_updater, diag_status, diag_message

    @diagnose_method('Interfaces')
    def diagnose_interfaces(self, status_updater, diag_status, diag_message):
        diag_status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
        diag_message = 'No one WAN interface is online!'
        interfaces_data = self._router_data_getter.get_interfaces()
        for interface in interfaces_data:
            if interface['interface'] not in ['lan', 'loopback']:
                status_updater.add(
                    interface['interface'], str(interface['up']))
                if interface['up']:
                    diag_status = diagnostic_msgs.msg.DiagnosticStatus.OK
                    diag_message = 'OK, we have at least one WAN interface'

        return status_updater, diag_status, diag_message
    
    def _diagnose_lan(self, index, status_updater, diag_status, diag_message):
        link_up = self._router_data_getter.check_interface_link_up(index)
        status_updater.add('Has link', str(link_up))
        
        if link_up:
            diag_status = diagnostic_msgs.msg.DiagnosticStatus.OK
            diag_message = f'LAN{index} OK'
        else:
            diag_status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            diag_message = f'LAN{index} has no link!'
        
        return status_updater, diag_status, diag_message
    
    @diagnose_method('LAN1')
    def diagnose_lan1(self, status_updater, diag_status, diag_message):
        return self._diagnose_lan(1, status_updater, diag_status, diag_message)
    
    @diagnose_method('LAN2')
    def diagnose_lan2(self, status_updater, diag_status, diag_message):
        return self._diagnose_lan(2, status_updater, diag_status, diag_message)
    
    @diagnose_method('LAN3')
    def diagnose_lan3(self, status_updater, diag_status, diag_message):
        return self._diagnose_lan(3, status_updater, diag_status, diag_message)


def main(args=None):
    rclpy.init(args=args)

    owdg = OpenWRTDataGetter()
    rd = RouterDiagnoster(owdg)

    updater = diagnostic_updater.Updater(rd, 5)
    updater.setHardwareID('kroks_router')

    updater.add('/router/devices/cpu', rd.diagnose_cpu)
    updater.add('/router/devices/memory', rd.diagnose_memory)
    updater.add('/router/devices/interfaces', rd.diagnose_interfaces)
    updater.add('/router/devices/lan/lan1', rd.diagnose_lan1)
    updater.add('/router/devices/lan/lan2', rd.diagnose_lan2)
    updater.add('/router/devices/lan/lan3', rd.diagnose_lan3)
    updater.add('/router/celluar_network1', rd.diagnose_modem1)
    updater.add('/router/celluar_network2', rd.diagnose_modem2)
    
    # TODO: 2g - error, 3g - warn, 4g, 5g - OK
    # TODO: RTT
    # TODO: interfaces split to modems

    rclpy.spin(rd)
    rd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
