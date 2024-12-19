```
                ________                __
               /_  __/ /  __ _____  ___/ /__ _______ _______  ___  ___
                / / / _ \/ // / _ \/ _  / -_) __(_-</ __/ _ \/ _ \/ -_)
               /_/ /_//_/\_,_/_//_/\_,_/\__/_/ /___/\__/\___/ .__/\__/
                                                           /_/
                         FPGA gateware for Thunderscope hardware.
                                Powered by Migen & LiteX
```

![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)


LiteX based FPGA gateware for Thunderscope.
===========================================

This repo aims to provide a LiteX based gateware for Thunderscope hardware.

<p align="center"><img src="https://user-images.githubusercontent.com/1450143/179495534-4c54973b-9203-4893-9eaa-d9177413e9bf.png" width="800"></p>

This repo is for now a WIP.

[> Prerequisites
----------------
- Python3, Vivado WebPACK
- Either a Vivado-compatible JTAG cable (native or XVCD), or OpenFPGALoader.

[> Installing LiteX
-------------------
```sh
$ wget https://raw.githubusercontent.com/enjoy-digital/litex/master/litex_setup.py
$ chmod +x litex_setup.py
$ sudo ./litex_setup.py init install
```

> **Note:** The LiteI2C core used in this design is not currently in the base LiteX install.  You may need to clone the [LiteI2C repo](https://github.com/litex-hub/litei2c) to the directory where LiteX has downloaded, and rerun the `./litex_setup.py install` command.

[> Build and Load the bitstream
--------------------------------
```sh
$ ./thunderscope --variant=a200t --driver --build --load
```

| Variant Arg | Target                   |
| :---------: | :----------------------: |
| `a200t`     | Trenz TE0712 200T Module |
| `a100t`     | Trenz TE0712 100T Module |
| `a50t`      | Custom Rev2 A50T Module  |

[> Open LiteX server
--------------------
Over JTAGBone (on local machine):
```sh
$ litex_server --jtag --jtag-config=openocd_xc7_ft232.cfg
```
Over PCIeBone (on local or remote machine):
```sh
$ sudo litex_server --pcie --pcie-bar=0x:00.0 (--host=192.168.1.X if on remote machine)
```

[> Compile/Mount LitePCIe Driver
--------------------------------
```sh
$ cd software/kernel
$ make
$ sudo ./init.sh
$ cd ../user
$ make
```

[> Run test scripts
-------------------
```sh
$ cd test
$ ./i2c_test --scan (--host=192.168.1.X if remotely)
$ ./test_adc.py --channels=1 --mode=ramp --afe-coupling=DC --afe-attenuation=10X --pga-preamp=10 --pga-atten=10 --pga-bw=full --pga-offset=128
$ ./test_glscopeclient.py
$ glscopeclient --debug myscope:enjoy-digital:lan:127.0.0.1
```

[> Flash Firmware Layout
------------------------

The Trenz A100T and A200T modules include a 256Mb SPI Flash chip, the A50T/A35T builds use a 32Mb SPI Flash.  The larger chips' bitstream does not fit into the flash used on the smaller design, therefore we have two Flash partition tables.

A35T/A50T (0x80_0000):

| Address Range          | Content                   |
| :--------------------: | :-----------------        |
| 0x000000 - 0x27FFFF    | Factory Bitstream*        |
| 0x280000 - 0x3FFFFF    | Factory Calibration Data* |
| 0x400000 - 0x67FFFF    | Primary Bitstream         |
| 0x680000 - 0x7FFFFF    | Available for User Data   |

A100T/A200T (0x400_0000):

| Address Range          | Content                   |
| :--------------------: | :-----------------        |
| 0x0000000 - 0x0FFFFFF  | Factory Bitstream*        |
| 0x1000000 - 0x1FFFFFF  | Factory Calibration Data* |
| 0x2000000 - 0x2FFFFFF  | Primary Bitstream         |
| 0x3000000 - 0x3FFFFFF  | Available for User Data   |


\* Write Protected
