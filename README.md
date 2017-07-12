# SPI master IP core

Implementation of SPI master in VHDL. IP core contains simple SPI master with variable clock, data size and 3 slave-select lines. Folder /API contains C library allowing to use SPI functionality from xyllinux running on ARM cores of Zynq chip. For use in bare metal applications comment out Xil_In & Xil_Out functions from API/axispi.c and use ones provided by Xilinx SDK.


Features:

* Variable clock from 4.6kHz to 445kHz through 32bit clock divider (base clock 50MHz)
* 3 slave-select outputs
* Variable data length from 8 to 32 bits
* Fixed polarity: Data is set on falling CLK edge and read on rising

Guidelines for using:
1. Clone repo and add it to you IP core repository
2. Add axi_spi_v1.0 to your block design
3. Connect signals to pins of your Zynq chip
4. Open Address editor and find out base address of peripheral
5. Generate bitstream and setup your application
6. Use API and (if neccessarry) update base address to match yours


NOTE: When running bare metal, you can access registers directly, without mmap() just beware of offset 4 per register.
