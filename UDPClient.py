import socket
from enum import Enum
from typing import Annotated, Optional

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
    SEPARATOR_SECTION = ","
    SEPARATOR_COMMAND = ";"

    # Command Direction
    CODE_GET = "G"
    CODE_SET = "S"
    CODE_RESPONSE = "R"

    # Command Type
    CODE_CTRL_POS = "P"
    CODE_CTRL_CART = "C"
    CODE_CTRL_JNT = "J"
    CODE_MOTION = "M"
    CODE_STATUS = "S"
    CODE_TOOL = "T"
    CODE_EXIT = "E"
    CODE_SHOW = "W"

    CODE_QUERY_FAILED = "F"
    CODE_QUERY_SUCCESS = "S"

    # Status
    CODE_EXTERNAL_FORCE = "F"
    CODE_MOTION_ACTIVITY = "A"

    class CTRL_MODE(Enum):
        POSITION_CONTROL = "P"
        JNT_IMP = "J"
        CART_IMP = "C"

    class CART_AXIS(Enum):
        X = "X"
        Y = "Y"
        Z = "Z"
        A = "A"
        B = "B"
        C = "C"
        NULLSPACE = "N"
        TRANSL = "T"
        ROT = "R"
        ALL = "E"

    class PARAM_TYPE(Enum):
        DAMPING = "D"
        STIFFNESS = "S"
        ADDITIONAL_FORCE = "A"

    Available_Tools = ("Flange", "HandTool", "Empty")

    def __init__(self, controller=CTRL_MODE.CART_IMP):
        self.controller = controller
        self.joint_dampings = [0.7] * 7
        self.joint_stiffnesses = [1000.0] * 7
        self.cartesian_dampings = [0.7] * 7
        self.cartesian_stiffnesses = [2000.0] * 3 + [200.0] * 3 + [0.0]
        self.additional_forces = [0.0] * 6
        self.current_tool = 0


class UDPClient:
    def __init__(
        self,
        kuka_ip=DEFAULT_KUKA_IP_ADDRESS,
        kuka_port=UDP_SEVER_PORT,
        controller=KUKA_Params.CTRL_MODE.CART_IMP,
        socket_timeout=0.1,  # 100ms
        verbose=False,
    ):
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
            print("REQUEST TIMED OUT")
            return False

    def _send(self, msg):
        self.__send(msg)
        self.__recv()

    def _get_from_server(self, query, expected_length) -> bool:
        self._send(query)
        if self.response is None:
            return False

        if (
            self.response[0] == KUKA_Params.CODE_QUERY_FAILED
            or len(self.response) != expected_length
        ):
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

    def _get_ctrl_params(
        self,
        ctrl_type: KUKA_Params.CTRL_MODE,
        param: KUKA_Params.PARAM_TYPE,
    ) -> Optional[Annotated[list[float], 7]]:
        """
        fetch parameters for all axis
        """
        query = (
            KUKA_Params.SEPARATOR_SECTION.join(
                [
                    KUKA_Params.CODE_GET,
                    ctrl_type.value,
                    param.value,
                    KUKA_Params.CART_AXIS.ALL.value,
                ]
            )
            + KUKA_Params.SEPARATOR_COMMAND
        )

        if self._get_from_server(query, 7):
            return [float(x) for x in self.response]

    def _get_ctrl_param(
        self,
        ctrl_type: KUKA_Params.CTRL_MODE,
        param: KUKA_Params.PARAM_TYPE,
        axis: KUKA_Params.CART_AXIS,
    ) -> Optional[float]:
        """
        fetch parameters for one axis
        """
        query = (
            KUKA_Params.SEPARATOR_SECTION.join(
                [KUKA_Params.CODE_GET, ctrl_type.value, param.value, axis.value]
            )
            + KUKA_Params.SEPARATOR_COMMAND
        )

        if self._get_from_server(query, 1):
            return float(self.response[0])

    # maybe todo: fetch TRANSL or ROT

    def _set_ctrl_params(
        self,
        ctrl_type: KUKA_Params.CTRL_MODE,
        param: KUKA_Params.PARAM_TYPE,
        values: Annotated[list[float], 7],
    ) -> bool:
        query = (
            KUKA_Params.SEPARATOR_SECTION.join(
                [
                    KUKA_Params.CODE_SET,
                    ctrl_type.value,
                    param.value,
                    KUKA_Params.CART_AXIS.ALL.value,
                    *[f"{x:.3f}" for x in values],
                ]
            )
            + KUKA_Params.SEPARATOR_COMMAND
        )

        return self._set_to_server(query)

    def _set_ctrl_param(
        self,
        ctrl_type: KUKA_Params.CTRL_MODE,
        param: KUKA_Params.PARAM_TYPE,
        axis: KUKA_Params.CART_AXIS,
        value: float,
    ) -> bool:
        query = (
            KUKA_Params.SEPARATOR_SECTION.join(
                [
                    KUKA_Params.CODE_SET,
                    ctrl_type.value,
                    param.value,
                    axis.value,
                    f"{value:.3f}",
                ]
            )
            + KUKA_Params.SEPARATOR_COMMAND
        )

        return self._set_to_server(query)

    # Joint Impedance
    def get_joint_params(
        self,
        param: KUKA_Params.PARAM_TYPE,
    ) -> Optional[Annotated[list[float], 7]]:
        return self._get_ctrl_params(KUKA_Params.CTRL_MODE.JNT_IMP, param)

    def get_joint_param(
        self,
        param: KUKA_Params.PARAM_TYPE,
        axis: KUKA_Params.CART_AXIS,
    ) -> Optional[float]:
        return self._get_ctrl_param(KUKA_Params.CTRL_MODE.JNT_IMP, param, axis)

    def set_joint_params(
        self,
        param: KUKA_Params.PARAM_TYPE,
        values: Annotated[list[float], 7],
    ) -> bool:
        return self._set_ctrl_params(KUKA_Params.CTRL_MODE.JNT_IMP, param, values)

    def set_joint_param(
        self,
        param: KUKA_Params.PARAM_TYPE,
        axis: KUKA_Params.CART_AXIS,
        value: float,
    ) -> bool:
        return self._set_ctrl_param(KUKA_Params.CTRL_MODE.JNT_IMP, param, axis, value)

    # Cartesian Params
    def get_cartesian_params(
        self,
        param: KUKA_Params.PARAM_TYPE,
    ) -> Optional[Annotated[list[float], 7]]:
        return self._get_ctrl_params(KUKA_Params.CTRL_MODE.CART_IMP, param)

    def get_cartesian_param(
        self,
        param: KUKA_Params.PARAM_TYPE,
        axis: KUKA_Params.CART_AXIS,
    ) -> Optional[float]:
        return self._get_ctrl_param(KUKA_Params.CTRL_MODE.CART_IMP, param, axis)

    def set_cartesian_params(
        self,
        param: KUKA_Params.PARAM_TYPE,
        values: Annotated[list[float], 7],
    ) -> bool:
        return self._set_ctrl_params(KUKA_Params.CTRL_MODE.CART_IMP, param, values)

    def set_cartesian_param(
        self,
        param: KUKA_Params.PARAM_TYPE,
        axis: KUKA_Params.CART_AXIS,
        value: float,
    ) -> bool:
        return self._set_ctrl_param(KUKA_Params.CTRL_MODE.CART_IMP, param, axis, value)

    # TODO: Command Activity

    def get_external_forces(self) -> Optional[Annotated[list[float], 6]]:
        query = (
            KUKA_Params.SEPARATOR_SECTION.join(
                [
                    KUKA_Params.CODE_GET,
                    KUKA_Params.CODE_STATUS,
                    KUKA_Params.CODE_EXTERNAL_FORCE,
                ]
            )
            + KUKA_Params.SEPARATOR_COMMAND
        )

        if self._get_from_server(query, 6):
            return [float(x) for x in self.response]

    # TODO: Tool

    def cmd_exit(self) -> bool:
        query = (
            KUKA_Params.SEPARATOR_SECTION.join(
                [KUKA_Params.CODE_SET, KUKA_Params.CODE_EXIT]
            )
            + KUKA_Params.SEPARATOR_COMMAND
        )

        return self._set_to_server(query)

    def cmd_show(self) -> bool:
        query = (
            KUKA_Params.SEPARATOR_SECTION.join(
                [KUKA_Params.CODE_SET, KUKA_Params.CODE_SHOW]
            )
            + KUKA_Params.SEPARATOR_COMMAND
        )

        return self._set_to_server(query)


def external_force_publisher(k: UDPClient):
    # UDP Server for visualization
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t = 0
    while True:
        ex_force = k.get_external_forces()
        data = {"timestamp": time.time(), "force": ex_force}
        json_data = json.dumps(data)
        sock.sendto(json_data.encode(), ("127.0.0.1", 12345))
        t += 1
        time.sleep(0.1)  # 10 Hz


if __name__ == "__main__":
    import numpy as np
    import time
    import json

    k = UDPClient(verbose=True)
    # k.cmd_show()
    k.get_cartesian_params(KUKA_Params.PARAM_TYPE.STIFFNESS)
    # k.set_cartesian_param(KUKA_Params.PARAM_TYPE.STIFFNESS, KUKA_Params.CART_AXIS.Z, 100)
    k.set_cartesian_params(KUKA_Params.PARAM_TYPE.STIFFNESS, [2000.0, 100.0, 2000.0, 200.0, 200.0, 200.0, 0.0])
    # k.set_cartesian_param(KUKA_Params.PARAM_TYPE.DAMPING, KUKA_Params.CART_AXIS.Z, 0.1)

    # external_force_publisher(k)
    # k.cmd_exit()
