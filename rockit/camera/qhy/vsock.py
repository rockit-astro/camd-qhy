#
# This file is part of the Robotic Observatory Control Kit (rockit)
#
# rockit is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# rockit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with rockit.  If not, see <http://www.gnu.org/licenses/>.

"""Helpers for sending data from a virtual machine to its host using AF_VSOCK"""

from ctypes import byref, c_uint8, c_uint16, c_uint, create_string_buffer, sizeof, Structure, WinDLL
import platform
import socket

AF_VSOCK = 40
VMADDR_CID_HOST = 2
SOCK_STREAM = 1

class SockAddrVm(Structure):
    _fields_ = [
        ("svm_family", c_uint16),
        ("svm_reserved1", c_uint16),
        ("svm_port", c_uint),
        ("svm_cid", c_uint),
        ("svm_zero", c_uint8 * 4)
    ]

class VSockClientHelper:
    def __init__(self, host_port):
        self.host_port = host_port

        # Python on Windows does not support AF_VSOCK,
        # so we must use winsock directly
        if platform.system() == 'Windows':
            wsa_data = create_string_buffer(512)
            self._winsock = WinDLL('Ws2_32.dll')
            self._winsock.WSAStartup(0x0202, wsa_data)
            self._winsock_socket = -1
        else:
            self._winsock = None
            self._socket = None

    def sendall(self, data):
        if self._winsock:
            if self._winsock_socket == -1:
                self._winsock_socket = self._winsock.socket(AF_VSOCK, SOCK_STREAM, 0)
                print(f'socket is {self._winsock_socket}')
                if self._winsock_socket == -1:
                    return False

                addr = SockAddrVm()
                addr.svm_family = AF_VSOCK
                addr.svm_reserved1 = 0
                addr.svm_port = self.host_port
                addr.svm_cid = VMADDR_CID_HOST
                if self._winsock.connect(self._winsock_socket, byref(addr), sizeof(addr)) != 0:
                    self._winsock_socket = -1
                    return False

            sent = 0
            total = len(data)
            while sent < total:
                ret = self._winsock.send(self._winsock_socket, data[sent:], total - sent)
                if ret < 0:
                    error_code = self._winsock.WSAGetLastError()
                    print(f'send failed with code {error_code}')

                    self._winsock.closesocket(self._winsock_socket)
                    self._winsock_socket = -1
                    return False
                sent += ret
            print(f'sent {sent}')
            return True
        else:
            try:
                if not self._socket:
                    self._socket = socket.socket(socket.AF_VSOCK, socket.SOCK_STREAM, 0)
                    self._socket.connect((socket.VMADDR_CID_HOST, self.host_port))

                self._socket.sendall(data, 0)
                return True
            except:
                self._socket = None
                return False

    def __del__(self):
        if self._winsock:
            if self._winsock_socket != -1:
                self._winsock.closesocket(self._winsock_socket)
            self._winsock.WSACleanup()
        else:
            if self._socket:
                self._socket.close()
