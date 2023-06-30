from typing import List, Dict
from openwrt.ubus import Ubus
import pprint
import time
pp = pprint.PrettyPrinter(indent=4)


class OpenWRTDataGetter:
    def __init__(self) -> None:
        self._ubus = None

    def connect(self, host: str, username: str, password: str) -> None:
        self._host = host
        self._username = username
        self._password = password
        connect_result = None
        while connect_result is None:
            try:
                self._ubus = Ubus(
                    host=self._host, username=self._username, password=self._password)
                connect_result = self._ubus.connect()
            except Exception as e:
                print('Error during connection: ', e)

    def _api_call(self, *args, **kwargs):
        try:
            return self._ubus.api_call(*args, **kwargs)
        except Exception as e:
            print('API call failed: ', e)
            print('Reconnecting...')
            self.connect(host=self._host, username=self._username,
                         password=self._password)

    def get_modem(self, modem_index) -> Dict:
        r = self._api_call("call", "kroks.dev.modem", "object", {})
        if r is not None:
            # pp.pprint(r['modem1'])
            modem = {}
            modem['connected_to_network'] = r[modem_index]['kroks.net.check']['status']
            modem['reliability'] = r[modem_index]['kroks.net.check']['reliability']

            modem['online'] = False
            rtt_avg = 0
            rtt_counter = 0
            for check in r[modem_index]['kroks.net.check']['check']:
                if check['status'] == True:
                    modem['online'] = True
                if 'rtt_avg' in check:
                    print('rtt_avg: ', check['rtt_avg'])
                    rtt_avg += float(check['rtt_avg'])
                    rtt_counter += 1

            if rtt_counter != 0:
                modem['RTT'] = rtt_avg / rtt_counter
            else:
                modem['RTT'] = 0

            modem['RSSI'] = None

            try:
                for k, v in r[modem_index]['storage']['signal'].items():
                    if 'rssi' in v:
                        if v['rssi'] != '--':
                            modem['RSSI'] = v['rssi']
                            break
            except:
                pass

            return modem
        else:
            print(f'Failed to get {modem_index} info')
            raise ConnectionError("Connection to router lost")

    def get_cpu(self) -> List:
        r = self._api_call("call", "system", "info", {})
        if r is not None:
            load = []
            for l in r['load']:
                load.append(f'{float(l) / 65536.0:.2}')

            # pp.pprint(load)

            return load
        else:
            print('Failed to get cpu info')
            raise ConnectionError("Connection to router lost")

    def get_memory(self) -> List:
        r = self._ubus.api_call("call", "system", "info", {})
        if r is not None:
            return float(r['memory']['free']) / float(r['memory']['total']) * 100
        else:
            print('Failed to get mmeory info')
            raise ConnectionError("Connection to router lost")

    def get_interfaces(self) -> List:
        r = self._ubus.api_call("call", "network.interface", "dump", {})
        if r is not None:
            return r['interface']
        else:
            print('Failed to get interfaces info')
            raise ConnectionError("Connection to router lost")


if __name__ == "__main__":
    rsg = OpenWRTDataGetter(host="http://192.168.1.1/ubus",
                            username="root", password="123")
    rsg.get_cpu()
    rsg.get_memory()
