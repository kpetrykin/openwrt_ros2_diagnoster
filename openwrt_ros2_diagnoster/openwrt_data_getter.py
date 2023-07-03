from typing import List, Dict
from openwrt.ubus import Ubus
import pprint
from openwrt_luci_rpc import OpenWrtLuciRPC

pp = pprint.PrettyPrinter(indent=4)


class OpenWRTDataGetter:
    def __init__(self) -> None:
        self._ubus = None
        self._luci = None

    def connect(self, host: str, username: str, password: str) -> None:
        self._host = host
        self._username = username
        self._password = password
        self._connect_ubus()
        self._connect_luci()

    def _connect_ubus(self) -> None:
        connect_ubus_result = None
        while connect_ubus_result is None:
            try:
                self._ubus = Ubus(
                    host='http://' + self._host + '/ubus', username=self._username, password=self._password)
                connect_ubus_result = self._ubus.connect()
            except Exception as e:
                print('Error during connection to UBUS: ', e)

    def _connect_luci(self) -> None:
        connect_luci_result = None
        while connect_luci_result is None:
            try:
                self._luci = OpenWrtLuciRPC(
                    self._host, self._username, self._password, False, False)
                connect_luci_result = True
            except Exception as e:
                print('Error during connection to LUCI: ', e)

    def _ubus_api_call(self, *args, **kwargs):
        try:
            return self._ubus.api_call(*args, **kwargs)
        except Exception as e:
            print('UBUS API call failed: ', e)
            print('Reconnecting...')
            self._connect_ubus()

    def _luci_api_call(self, *args, **kwargs):
        try:
            return self._luci._call_json_rpc(*args, **kwargs)
        except Exception as e:
            print('LUCI API call failed: ', e)
            print('Reconnecting...')
            self._connect_luci()

    def get_modem(self, modem_index) -> Dict:
        r = self._ubus_api_call("call", "kroks.dev.modem", "object", {})
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
                    # print('rtt_avg: ', check['rtt_avg'])
                    rtt_avg += float(check['rtt_avg'])
                    rtt_counter += 1

            if rtt_counter != 0:
                modem['RTT'] = rtt_avg / rtt_counter
            else:
                modem['RTT'] = 0

            modem['RSSI'] = None
            modem['network_type'] = None

            try:
                for k, v in r[modem_index]['storage']['signal'].items():
                    if 'rssi' in v:
                        if v['rssi'] != '--':
                            modem['RSSI'] = v['rssi']
                            modem['network_type'] = k
                            break
                    if k == '5g':
                        if v['rsrp'] != '--':
                            modem['network_type'] = k
                        
            except:
                pass

            return modem
        else:
            print(f'Failed to get {modem_index} info')
            raise ConnectionError("Connection to router lost")

    def get_cpu(self) -> List:
        r = self._ubus_api_call("call", "system", "info", {})
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
        r = self._ubus_api_call("call", "system", "info", {})
        if r is not None:
            return float(r['memory']['free']) / float(r['memory']['total']) * 100
        else:
            print('Failed to get mmeory info')
            raise ConnectionError("Connection to router lost")

    def get_interfaces(self) -> List:
        r = self._ubus_api_call("call", "network.interface", "dump", {})
        if r is not None:
            return r['interface']
        else:
            print('Failed to get interfaces info')
            raise ConnectionError("Connection to router lost")

    def check_interface_link_up(self, index: int) -> bool:
        # Weird true LAN ports order
        if index == 1:
            index = 3
        else:
            if index == 3:
                index = 1
        
        result = self._luci_api_call(
            f'http://{self._host}/cgi-bin/luci/rpc/sys',
            'exec',
            f'swconfig dev rt305x port {index} show')
        
        # print(result)
        
        is_up = 'link:up' in result.split()
        
        return is_up


if __name__ == "__main__":
    rsg = OpenWRTDataGetter(host="192.168.1.1",
                            username="root", password="123")
    rsg.get_cpu()
    rsg.get_memory()
