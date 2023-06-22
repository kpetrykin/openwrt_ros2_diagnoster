from typing import List
from openwrt.ubus import Ubus
import pprint
import time
pp = pprint.PrettyPrinter(indent=4)


class RouterMonitor:
    def __init__(self, host: str, username: str, password: str) -> None:
        self._host = host
        self._username = username
        self._password = password
        self._ubus = None

        self._connect()

    def _connect(self):
        connect_result = None
        while connect_result is None:
            try:
                self._ubus = Ubus(
                    host=self._host, username=self._username, password=self._password)
                connect_result = self._ubus.connect()
            except Exception as e:
                print('Error during connection: ', e)

    def get_router_status(self) -> List:
        return self.get_modems()

    def get_modems(self) -> List:
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
                
    def _api_call(self, *args, **kwargs):
        try:
            return self._ubus.api_call(*args, **kwargs)
        except Exception as e:
            print('API call failed: ', e)
            print('Reconnecting...')
            self._connect()

    def get_cpu(self) -> List:
        r = self._api_call("call", "system", "info", {})
        if r is not None:
            load = []
            for l in r['load']:
                load.append(f'{float(l) / 65536.0:.2}')

            pp.pprint(load)

            return load
        else:
            print('Failed to get cpu info')
            raise ConnectionError("Connection to router lost")

    def get_memory(self) -> List:
        r = self._ubus.api_call("call", "system", "info", {})
        load = r['load'][0] / 65536.0
        print(load)
        pp.pprint(r)


if __name__ == "__main__":
    rsg = RouterMonitor(host="http://192.168.1.1/ubus",
                        username="root", password="123")
    rsg.get_router_status()
    rsg.get_cpu()
    rsg.get_memory()
