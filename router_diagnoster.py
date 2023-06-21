import rclpy
from rclpy.node import Node
import diagnostic_updater as du
import random

from router_status_getter import RouterStatusGetter

class RouterDiagnoster(Node):
    def __init__(self):
        super().__init__('router_diagnoster')
        
        self._diag_status = du.DiagnosticStatus()
        self._diag_status.level = du.DiagnosticStatus.OK
        self._diag_status.name = 'Router status'
        self._diag_status.message = 'Router is OK'
        self._diag_status.hardware_id = 'KROKS Router'
        self._diag_status.values = []
        
        self._diag_pub = self.create_publisher(
            du.DiagnosticArray, '/diagnostics', 10)
        
        self.current_msg = None
        self.pubtimer = self.create_timer(0.1, self._publish_diag)
        
    def _publish_diag(self):
        new_msg = du.DiagnosticArray()
        
        if len(self._diag_status.values) == 0:
            new_kv = du.KeyValue()
            new_kv.key = 'rand_stable'
            new_kv.value = str(random.randint(0, 100))
            self._diag_status.values.append(new_kv)
        else:
            self._diag_status.values[0].value = str(random.randint(0, 100))
        
        new_msg.status.append(self._diag_status)
        self._diag_pub.publish(new_msg)
        
def main(args=None):
    rclpy.init(args=args)
    try:
        rd = RouterDiagnoster()
        rclpy.spin(rd)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
        
if __name__ == "__main__":
    main()