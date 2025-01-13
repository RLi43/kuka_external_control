import socket
from enum import Enum 
from typing import Annotated, Literal, Optional

DEFAULT_KUKA_IP_ADDRESS = "172.31.1.147"
UDP_SEVER_PORT = 30001

class KUKA_Params:
    """
    [Command Direction][Command Type],[Data];
    [Command Direction][Command Type],[Secondary Command Type],[Axis],[Data];
    [Command Direction][Command Type],[Axis],[Data1],[Data2],[Data3],...;

    R,[Data1],[Data2],...;
    F;
    """
    # Separators
    SEPARATOR_SECTION = ','
    SEPARATOR_COMMAND = ';'

    # Command Direction
    CODE_GET = 'G'
    CODE_SET = 'S'
    CODE_RESPONSE = 'R'

    # Command Type
    CODE_CTRL_POS = 'P'
    CODE_CTRL_CART = 'C'
    CODE_CTRL_JNT = 'J'
    CODE_MOTION = 'M'
    CODE_STATUS = 'S'
    CODE_TOOL = 'T'
    CODE_EXIT = 'E'
    CODE_SHOW = 'W'

    CODE_QUERY_FAILED = 'F'
    CODE_QUERY_SUCCESS = 'S'

    # Secondary Command Type
    CODE_DAMPING: str = 'D'
    CODE_STIFFNESS: str = 'S'

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

    # Status
    CODE_EXTERNAL_FORCE = 'F'
    CODE_MOTION_ACTIVITY = 'A'

    class CONTROLLER_MODE(Enum):
        POSITION_CONTROL = 0
        JOINT_IMPEDANCE_CONTROL = 1
        CARTESIAN_IMPEDANCE_CONTROL = 2
    
    class CARTESIAN_AXIS(Enum):
        DOF_X = 0
        DOF_Y = 1
        DOF_Z = 2
        DOF_A = 3
        DOF_B = 4
        DOF_C = 5
        DOF_NULLSPACE = 6
        DOF_TRANSL = 7
        DOF_ROT = 8
        DOF_ALL = 9

    Available_Tools = ("Flange", "HandTool", "Empty")

    def __init__(self, controller = CONTROLLER_MODE.CARTESIAN_IMPEDANCE_CONTROL):
        self.controller = controller
        self.joint_dampings = [0.7] * 7
        self.joint_stiffnesses = [1000.0] * 7
        self.cartesian_dampings = [0.7] * 7
        self.cartesian_stiffnesses = [2000.0] * 3 + [200.0] * 3 + [0.0]
        self.additional_forces = [0.0] * 6
        self.current_tool = 0


class UDPClient:
    def __init__(self, 
                 kuka_ip = DEFAULT_KUKA_IP_ADDRESS,
                 kuka_port = UDP_SEVER_PORT,
                 controller = KUKA_Params.CONTROLLER_MODE.CARTESIAN_IMPEDANCE_CONTROL,
                 socket_timeout = 0.1, # 100ms
                 verbose = False):
        self.verbose = verbose
        self.params = KUKA_Params(controller)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(socket_timeout)
        self.kuka_address = (kuka_ip, kuka_port)

        self.response = None

        self.remote_server = None

    def __send(self, msg: str):
        if self.verbose:
            print(f"Sending: {msg}")
        # TODO: error code of 'sendto'
        self.socket.sendto(msg.encode("utf-8"), self.kuka_address)

    def __recv(self) -> bool:
        try:
            data, server = self.socket.recvfrom(1024)
            if self.remote_server is None:
                self.remote_server = server
                print("Remote: ", self.remote_server)

            self.response = data.decode()
            if self.verbose:
                print("Recv:", self.response)
            if self.response[-1] != KUKA_Params.SEPARATOR_COMMAND:
                print("Invalid Message Ending!")
                self.response = None
                return True
            self.response = self.response[:-1]
            self.response = self.response.split(KUKA_Params.SEPARATOR_SECTION)
            if self.response[0] != KUKA_Params.CODE_RESPONSE:
                print("Reply doesn't start as expected!")
                self.response = None
                return True
            self.response = self.response[1:]

            return True
        except socket.timeout:
            print('REQUEST TIMED OUT')
            return False
        
    def _send(self, msg):
        self.__send(msg)
        self.__recv()

    def _get_from_server(self, query, expected_length) -> bool:
        self._send(query)
        if self.response is None:
            return False
        
        if self.response[0] == KUKA_Params.CODE_QUERY_FAILED or len(self.response) != expected_length:
            print("Server Failed to read")
            return False
        
        return True
    
    def _set_to_server(self, query):
        self._send(query)

        if self.response is None:
            return False
        
        if self.response[0] != KUKA_Params.CODE_QUERY_SUCCESS:
            print("Server Failed to set")
            return False
        
        return True

    # Joint Impedance
    def get_joint_params(self, 
                         param: Literal[KUKA_Params.CODE_DAMPING, KUKA_Params.CODE_STIFFNESS]
                         ) -> Optional[Annotated[list[float], 7]]:
        query = KUKA_Params.SEPARATOR_SECTION.join([
            KUKA_Params.CODE_GET,
            KUKA_Params.CODE_CTRL_JNT,
            param,
            KUKA_Params.CODE_DOF_ALL
            ]) + KUKA_Params.SEPARATOR_COMMAND
        
        if self._get_from_server(query, 7):
            return [float(x) for x in self.response]
    
    def get_joint_param(self, 
                        param: Literal[KUKA_Params.CODE_DAMPING, KUKA_Params.CODE_STIFFNESS],
                        axis: KUKA_Params.CARTESIAN_AXIS
                        ) -> Optional[float]:
        query = KUKA_Params.SEPARATOR_SECTION.join([
            KUKA_Params.CODE_GET,
            KUKA_Params.CODE_CTRL_JNT,
            param,
            axis
            ]) + KUKA_Params.SEPARATOR_COMMAND

        if self._get_from_server(query, 1):
            return float(self.response[0])
    
    def set_joint_params(self,
                         param: Literal[KUKA_Params.CODE_DAMPING, KUKA_Params.CODE_STIFFNESS],
                         values: Annotated[list[float], 7]
                         ) -> bool:
        query = KUKA_Params.SEPARATOR_SECTION.join([
            KUKA_Params.CODE_SET,
            KUKA_Params.CODE_CTRL_JNT,
            param,
            KUKA_Params.CARTESIAN_AXIS.DOF_ALL,
            *[f"{x:.3f}" for x in values]
            ]) + KUKA_Params.SEPARATOR_COMMAND
        
        return self._set_to_server(query)
    
    def set_joint_param(self,
                         param: Literal[KUKA_Params.CODE_DAMPING, KUKA_Params.CODE_STIFFNESS],
                         axis: KUKA_Params.CARTESIAN_AXIS,
                         value: float
                        ) -> bool:
        query = KUKA_Params.SEPARATOR_SECTION.join([
            KUKA_Params.CODE_SET,
            KUKA_Params.CODE_CTRL_JNT,
            param,
            axis,
            f"{value:.3f}"
            ]) + KUKA_Params.SEPARATOR_COMMAND
        
        return self._set_to_server(query)
    
    # TODO: Cartesian Params

    # TODO: Command Activity
    
    def get_external_forces(self) -> Optional[Annotated[list[float], 6]]:
        query = KUKA_Params.SEPARATOR_SECTION.join([
            KUKA_Params.CODE_GET,
            KUKA_Params.CODE_STATUS,
            KUKA_Params.CODE_EXTERNAL_FORCE
        ]) + KUKA_Params.SEPARATOR_COMMAND

        if self._get_from_server(query, 6):
            return [float(x) for x in self.response]
    
    # TODO: Tool
        
    def cmd_exit(self) -> bool:
        query = KUKA_Params.SEPARATOR_SECTION.join([
            KUKA_Params.CODE_SET,
            KUKA_Params.CODE_EXIT
        ]) + KUKA_Params.SEPARATOR_COMMAND
        
        return self._set_to_server(query)
    
    def cmd_show(self) -> bool:
        query = KUKA_Params.SEPARATOR_SECTION.join([
            KUKA_Params.CODE_SET,
            KUKA_Params.CODE_SHOW
        ]) + KUKA_Params.SEPARATOR_COMMAND
        
        return self._set_to_server(query)   




if __name__ == "__main__":
    import numpy as np

    k = UDPClient(verbose=True)
    ex_force = k.get_external_forces()
    print("External Force:", ex_force, "norm=", np.linalg.norm(ex_force[:3]))
    k.cmd_show()
