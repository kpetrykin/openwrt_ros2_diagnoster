from typing import List
from openwrt.ubus import Ubus
import pprint
pp = pprint.PrettyPrinter(indent=4)

class RouterStatusGetter:
    def __init__(self, host: str, username: str, password: str) -> None:
        try:
            self._ubus = Ubus(host=host, username=username, password=password)
            self._ubus.connect()
        except Exception as e:
            print('Error during initialization: ', e)
        
    def get_router_status(self) -> List:
        return self._get_modems()
    
    def _get_modems(self) -> List:
        modems = self._ubus.api_call("call", "kroks.dev.modem", "object", {})
        if modems is None:
            print('Failed to get modems')
            return None
        else:
            for i, modem in modems.items():
                print(f"*********************{i}***************************")
                signal = None
                try:
                    signal = modem['storage']['signal']
                except Exception:
                    pass
                
                net_check = None
                try:
                    net_check = modem['kroks.net.check']
                except Exception:
                    pass
                
                links = None
                try:
                    links = modem['storage']['generic']['link-modulation']
                except Exception:
                    pass
                
                
                pp.pprint([signal, net_check, links])
                
    def _get_cpu(self) -> List:
        r = self._ubus.api_call("call", "system", "info", {})
        if r is not None:
            load = []
            for l in r['load']:
                load.append(f'{float(l) / 65536.0:.2}')
                
            pp.pprint(load)
        else:
            print('Failed to get cpu info')
            return None
    
    def _get_memory(self) -> List:
        r = self._ubus.api_call("call", "system", "info", {})
        load = r['load'][0] / 65536.0
        print(load)
        pp.pprint(r)

if __name__ == "__main__":
    rsg = RouterStatusGetter(host="http://192.168.1.1/ubus", username="root", password="123")
    rsg.get_router_status()
    rsg._get_cpu()
    rsg._get_memory()
    