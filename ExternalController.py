"""
An external control interface via UDP protocol to the KUKA controller.
Developed based on KUKA Sunrise.OS Med 2.6 V4, Chapter 12

Quick Start:

- Please check if the local machine's IP matches the one configured in the sunrise project.
- Check your firewall settings if REQUEST TIMEOUT.
- Run `app_start` to start the default application.
- Restart the application by `app_restart` after finishing execution.
- Run `get_state` to get the controller state. (actually the state is returned everytime
    a command is sent.)
- Error: INCORRECT_DATA_PACKET_COUNTER
    As you might reset the controller or this script, the data packet counter is
    not always aligned. You can mannually reset `KUKA_UDP`'s `packet_sent` to 
    `seq_kuka_recv` which is the third value in the printed state header.

TODO:
- Test With App_Enable supported option.
"""
import enum
import socket
import time

class KUKA_ERROR_CODE(enum.Enum):
    # Note: If more than one fault occurs simultaneously, the fault with the 
    # highest priority is transferred. A fault with the ID -3, for example, 
    # has a higher priority than a fault with the ID -4.

    NO_ERROR_int_trig = -1
    NO_ERROR = 0
    INCORRECT_CLIENT_IP = -1 # doesn't match the IP configured
    INCORRECT_MESSAGE_STRUCTURE = -2
    INCORRECT_DATA_PACKET_COUNTER = -3
    INCORRECT_TIME_STAMP = -4
    INCORRECT_SIGNAL_NAME = -5 
    INCORRECT_SIGNAL_VALUE = -6
    TIMEOUT_ERROR = -7

class KUKA_INPUT_SIGNAL(enum.Enum):
    APP_START = 1
    APP_ENABLE = 2
    GET_STATE = 3


import threading

class ExternalController:
    DEFAULT_KUKA_IP_ADDRESS = "172.31.1.147" # Sunrise Project Station Setup
    KUKA_EXTERNAL_CONTROL_PORT = 30300  # Sunrise Project Settings

    # packet
    # coding: UTF-8
    # separator = ';' 

    # inputs
    # 1 | Timestamp;
    # 2 | Data packet counter sent;
    # 3 | Input Signal Name;
    # 4 | Input Signal Value; 
    CMD_APP_START = 'App_Start'
    CMD_APP_ENABLE = 'App_Enable'
    CMD_GET_STATE = 'Get_State'
    SIGNAL_NAME_MAP = {
        KUKA_INPUT_SIGNAL.APP_START: CMD_APP_START,
        KUKA_INPUT_SIGNAL.APP_ENABLE: CMD_APP_ENABLE,
        KUKA_INPUT_SIGNAL.GET_STATE: CMD_GET_STATE
    }
    CMD_VALUE_TRUE = 'true'
    CMD_VALUE_FALSE = 'false'

    # outputs
    # 1 | Timestamp; 
    # 2 | Data packet counter sent;
    # 3 | Data packet counter received;
    # 4 | Error ID;
    # 5 | AutExt_Active; (AUT mode is active)
    # 6 | AutExt_AppReadyToStart;
    # 7 | DeafaultApp_Error;
    # 8 | Station_Error;
    # 9 | Current state of the default application; (see below)
    # 10| App_Start;
    # 11| App_Enable 

    APP_STATE_IDLE = 'IDLE' # selected
    APP_STATE_RUNNING = 'RUNNING'
    APP_STATE_MOTIONPAUSED = 'MOTIONPAUSED'
    APP_STATE_REPOSITIONG = 'REPOSITIONG'
    APP_STATE_ERROR = 'ERROR'
    APP_STATE_STARTING = 'STARTING'
    APP_STATE_STOPPING = 'STOPPING'

    def __init__(self, 
                 initial_packet_seq = 0,
                 with_app_enable_supported = False,
                 kuka_ip = DEFAULT_KUKA_IP_ADDRESS,
                 verbose = False):
        """
        @param with_app_enable_supported: 
        With App_Enable Signal Evaluated: The robot application paused if 
         it receives "App_Enable;false" or,
         it didn't receive "App_Enable;true" in 100ms.
        """
        self.verbose = verbose

        self.with_app_enable_supported = with_app_enable_supported
        if self.with_app_enable_supported:
            self.app_enable_set = False
            self.app_enable_heartbeat_thread = None

        self.packet_sent = initial_packet_seq

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.settimeout(0.1) # 100ms
        self.kuka_address = (kuka_ip, ExternalController.KUKA_EXTERNAL_CONTROL_PORT)

    def __app_enable_heartbeat(self):
        while self.app_enable_set:
            self.app_enable()
    
    def _app_enable_heartbeat_start(self):
        if self.app_enable_heartbeat_thread is None or not self.app_enable_heartbeat_thread.is_alive():
            self.app_enable_set = True
            self.app_enable_heartbeat_thread = threading.Thread(target=self.__app_enable_heartbeat)
            self.app_enable_heartbeat_thread.daemon = True
            self.app_enable_heartbeat_thread.start()
    
    def _app_enable_heartbeat_cancel(self):
        if self.app_enable_heartbeat_thread and self.app_enable_heartbeat_thread.is_alive():
            self.app_enable_set = False
            self.app_enable_heartbeat_thread.join()

    @staticmethod
    def local_ip_check(desired_IP = "172.31.1.5"):
        print(f"Checking IP address, IP configured in the project: {desired_IP}")        
        from netifaces import interfaces, ifaddresses, AF_INET
        found = False
        for ifaceName in interfaces():
            addresses = [i['addr'] 
                         for i in ifaddresses(ifaceName).setdefault(
                             AF_INET, [{'addr':'No IP addr'}] )]
            if any(x == desired_IP for x in addresses):
                found = True
            print(f"{ifaceName}: {', '.join(addresses)}")

        if not found:
            print(f"! We are using {desired_IP} !") # TODO: Found a way to synchronize it all over the whole project...
        return found

    def __get_packet_num(self, readonly = False):
        if not readonly:
            self.packet_sent += 1
        return self.packet_sent
    
    @staticmethod
    def __get_timestamp() -> str:
        return str(int(time.time() * 1000))
    
    def __send(self, msg: str):
        if self.verbose:
            print(f"Sending: {msg}")
        # TODO: error code of 'sendto'
        self.client_socket.sendto(msg.encode("utf-8"), self.kuka_address)
    
    def __compose_cmd(self, input_signal: KUKA_INPUT_SIGNAL, 
                      value: bool = True) -> str:
        return ";".join([
            self.__get_timestamp(),
            str(self.__get_packet_num()),
            self.SIGNAL_NAME_MAP[input_signal],
            self.CMD_VALUE_TRUE if value else self.CMD_VALUE_FALSE
        ])
    
    def __recv(self) -> bool:
        # In the following cases, the robot controller sends status messages 
        # to the clients that are configured as recipients of status messages 
        # in the project settings:
        # • Following receipt of the control message from an external client
        # • Following the change in state of an output signal
        try:
            data, server = self.client_socket.recvfrom(1024)
            if self.verbose:
                print("Recv:", data.decode())

            data = data.decode().split(';')
            # TODO: error catch
            ts = int(data[0]) / 1000
            seq_kuka_sent = int(data[1])
            seq_kuka_recv = int(data[2])
            error_id = int(data[3])
            aut_activate = data[4] == 'true'
            aut_ready = data[5] == 'true'
            app_error = data[6] == 'true'
            station_error = data[7] == 'true'
            app_state = data[8]
            signal_app_start = data[9] == 'true'
            signal_app_enable = data[10] == 'true'
            header = (f"[{ts:.3f}, {seq_kuka_sent}, {seq_kuka_recv}]")
            if error_id < 0:
                print(header, f"Error: {KUKA_ERROR_CODE(error_id).name}")
            if not aut_activate:
                print(header, "AUT mode is not activated!") # switch the key and change the mode
            if not aut_ready:
                print(header, "AUT mode is not ready!") # switch the key back
            if app_error:
                print(header, "APP error occurs!")
                # TODO return this message and restart the APP
            if station_error:
                print(header, "Station Error!")
            print(header, "APP State:", app_state)
            print(header, f"app_start: {signal_app_start}\tapp_enable: {signal_app_enable}")
            
            return True
        except socket.timeout:
            print('REQUEST TIMED OUT')
            return False

    def get_state(self):
        self.__send(
            self.__compose_cmd(
                KUKA_INPUT_SIGNAL.GET_STATE
            )
        )
        self.__recv()

    def app_start(self, stop=False):
        if self.with_app_enable_supported:
            self._app_enable_heartbeat_start()

        self.__send(
            self.__compose_cmd(
                KUKA_INPUT_SIGNAL.APP_START,
                not stop
            )
        )
        self.__recv()
    
    def app_stop(self):
        if not self.with_app_enable_supported:
            print("Can't stop the app via UDP without App_Enable Support")
            print("Please use the SmartPAD to stop")
            return
        
        self.__send(
            self.__compose_cmd(
                KUKA_INPUT_SIGNAL.APP_ENABLE,
                False
            )
        )
        self.__recv()
        self._app_enable_heartbeat_cancel()
    
    def app_enable(self, show_reply = False):
        self.__send(
            self.__compose_cmd(
                KUKA_INPUT_SIGNAL.APP_ENABLE
            )
        )
        if show_reply:
            self.__recv()
    
    def app_restart(self):
        self.app_start(stop=True)
        self.app_start()

    def resume(self):
        """
        The application was paused.
        Send two rising edges to first enable repositioning and then enable resuming.
        """
        self.app_restart()
        # TODO: wait until repositioning is finished
        self.app_restart()


if __name__ == '__main__':
    kuka = ExternalController(initial_packet_seq=1, verbose=True)
    if kuka.local_ip_check():
        kuka.get_state()