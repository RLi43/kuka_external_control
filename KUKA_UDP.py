"""
KUKA UDP Client to
- Activate cabinet robot application
- Set/Get parameters that are not available via FRI (Need UDP Server running on the Cabinet)
"""

from ExternalController import ExternalController
from UDPClient import UDPClient

class KUKA_UDP:
    def __init__(self, verbose):
        self.verbose = verbose
        self.ex_ctrl: ExternalController = ExternalController(
            verbose=self.verbose)
        self.udp_client: UDPClient = UDPClient()
