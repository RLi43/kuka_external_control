import socket

DEFAULT_KUKA_IP_ADDRESS = "172.31.1.147"
UDP_SEVER_PORT = 30001

class KUKA_Params:
    """
    [Command Type],[Data];
    [Command Type],[Secondary Command Type],[Axis],[Data];
    [Command Type],[Axis],[Data1],[Data2],[Data3],...;
    """
    # Separators
    SEPARATOR_SECTION = ','
    SEPARATOR_COMMAND = ';'

    # Command Type Code
    CODE_CTRL_CART = 'C'
    CODE_CTRL_JNT = 'J'
    CODE_MOTION = 'M'
    CODE_STATUS = 'S'
    CODE_TOOL = 'T'

    # Secondary Command Type
    CODE_DAMPING = 'D'
    CODE_STIFFNESS = 'S'

    # Command axis code
    CODE_DOF_X = 'X'
    CODE_DOF_Y = 'Y'
    CODE_DOF_Z = 'Z'
    CODE_DOF_A = 'A'
    CODE_DOF_B = 'B'
    CODE_DOF_C = 'C'
    CODE_DOF_TRANSL = 'T'
    CODE_DOF_ROT = 'R'
    CODE_DOF_NULLSPACE = 'N'
    CODE_DOF_ALL = 'E'


class UDPClient:
    def __init__(self, 
                 kuka_ip = DEFAULT_KUKA_IP_ADDRESS,
                 kuka_port = UDP_SEVER_PORT,
                 socket_timeout = 0.1, # 100ms
                 verbose = False):
        self.verbose = verbose

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(socket_timeout)
        self.kuka_address = (kuka_ip, kuka_port)

        self.remote_server = None

    def __send(self, msg: str):
        if self.verbose:
            print(f"Sending: {msg}")
        # TODO: error code of 'sendto'
        self.client_socket.sendto(msg.encode("utf-8"), self.kuka_address)

    def __recv(self) -> bool:
        try:
            data, server = self.socket.recvfrom(1024)
            if self.remote_server is None:
                self.remote_server = server
                print("Remote: ", self.remote_server)
            if self.verbose:
                print("Recv:", data.decode())

            # TODO: read and update status

            return True
        except socket.timeout:
            print('REQUEST TIMED OUT')
            return False
        
    def _send(self, msg):
        self.__send(msg)
        self.__recv()

if __name__ == "__main__":
    import time

    k = UDPClient()
    for i in range(3):
        k._send("hello")
        time.sleep(5)
    
    k._send("EXIT")
