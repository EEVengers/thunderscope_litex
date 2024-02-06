#!/usr/bin/env python3

#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2021-2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# SCPI Server proof of concept, tested with ./glscopeclient --debug myscope:enjoy-digital:lan:127.0.0.1

import os
import sys
import math
import time
import socket
import argparse
import threading

# SCPI Server --------------------------------------------------------------------------------------

class SCPIServer:
    def __init__(self, bind_ip="localhost", channels=1, sample_rate=1e9, sample_depth=16384,
        control_port  = 5025,
        control_only  = False,
        waveform_port = 50101):
        self.bind_ip       = bind_ip
        self.channels      = channels
        self.sample_rate   = sample_rate
        self.sample_depth  = sample_depth
        self.control_port  = control_port
        self.control_only  = control_only
        self.waveform_port = waveform_port

    def open(self):
        print(f"Opening Server {self.bind_ip}:c{self.control_port:d}:w{self.waveform_port:d}...")
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.control_sock.bind((self.bind_ip, self.control_port))
        self.control_sock.listen(1)

        if not self.control_only:
            self.waveform_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.waveform_sock.bind((self.bind_ip, self.waveform_port))
            self.waveform_sock.listen(1)

    def close(self):
        print("Closing Server...")
        self.control_sock.close()
        del self.control_sock
        if not self.control_only:
            self.waveform_sock.close()
            del self.waveform_sock

    def _control_thread(self):
        while True:
            client, addr = self.control_sock.accept()
            print(f"Control: Connected with {addr[0]}:{str(addr[1])}")
            try:
                while True:
                    data = client.recv(1024).decode("UTF-8")
                    #print(data)
                    if "IDN?" in data:
                        client.send(bytes(f"Enjoy-Digital,GEN00{self.channels},0001,0.1\n", "UTF-8"))
                    if "GAIN?" in data:
                        client.send(bytes("0\n", "UTF-8"))
                    if "OFFS?" in data:
                        client.send(bytes("0\n", "UTF-8"))
                    if "SAMP:TIM?" in data:
                        client.send(bytes(f"{int(1e15/self.sample_rate):d}\n", "UTF-8"))
                    if "SAMP:DEPTH?" in data:
                        client.send(bytes(f"{int(self.sample_depth):d}\n", "UTF-8"))

            finally:
                print("Control: Disconnect")
                client.close()

    def _waveform_thread(self, pattern=True):
        while True:
            client, addr = self.waveform_sock.accept()
            print(f"Waveform: Connected with {addr[0]}:{str(addr[1])}")
            os.system(f"sudo mkdir -p /tmpfs")
            os.system(f"sudo mount -t tmpfs -o size=1024m tmpfs /tmpfs")
            try:
                n = 0
                while True:
                    if pattern:
                        periods = 16
                        length  = int(self.sample_depth/self.channels)
                        shift   = 1024
                        data    = [int(128+128*math.sin((n*shift + periods*i*2*3.1415)/length)) for i in range(length)]
                        #print(len(data))
                        client.send(bytes(data))
                        n += 1
                    else:
                        # FIXME: Proof of concept; do it differently and move sample processing/remapping to GLScopeClient?
                        os.system(f"../software/user/litepcie_test record /tmpfs/waveform.bin {int(self.sample_depth*8):d}")
                        with open("/tmpfs/waveform.bin", "rb") as f:
                            data = list(f.read())
                            #print(f"{data[0]:02x} {data[1]:02x} {data[2]:02x} {data[3]:02x}")
                            samples = []
                            for n in range(self.sample_depth):
                                assert self.channels in [1]
                                for c in range(self.channels):
                                    # 8:1 decimation (ramp).
                                    if self.channels == 1:
                                        sample = data[8*n]
                                    sample = sample & 0xff
                                    # 2's complement to decimal.
                                    if sample & 0x80:
                                        sample &= 0x7f
                                        sample = 128 - sample
                                    # Offset/Clamp.
                                    sample = (sample + 128)
                                    sample = min(sample, 255)
                                    sample = max(sample,   0)
                                    samples.append(sample)
                            samples = bytes(samples)
                            client.send(samples)
                        time.sleep(0.1)
            finally:
                os.system(f"sudo umount /tmpfs")
                print("Waveform: Disconnect")
                client.close()
                self.close()

    def start(self):
        self.control_thread = threading.Thread(target=self._control_thread)
        self.control_thread.setDaemon(True)
        self.control_thread.start()

        if not self.control_only:
            self.waveform_thread = threading.Thread(target=self._waveform_thread)
            self.waveform_thread.setDaemon(True)
            self.waveform_thread.start()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SCPI Server test.")
    parser.add_argument("--bind-ip",       default="localhost", help="Host bind address.")
    parser.add_argument("--channels",      default=1,           help="Number of Channels.")
    parser.add_argument("--sample-rate",   default=30.72e6,     help="Samplerate (per channel).")
    parser.add_argument("--sample-depth",  default=16384,       help="Sampledepth (per channel).")
    parser.add_argument("--control-port",  default=5025,        help="Host bind Control port.")
    parser.add_argument("--control-only",  action="store_true", help="Only enable Control port.")
    parser.add_argument("--waveform-port", default=50101,       help="Host bind Waveform port.")
    args = parser.parse_args()

    server = SCPIServer(
        bind_ip       = args.bind_ip,
        channels      = int(args.channels),
        sample_rate   = int(args.sample_rate),
        sample_depth  = int(args.sample_depth),
        control_port  = int(args.control_port),
        control_only  = args.control_only,
        waveform_port = int(args.waveform_port)
    )
    server.open()
    server.start()
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
