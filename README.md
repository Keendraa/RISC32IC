# RISC32IC
This open source project is part of the finished Maria Tomàs' Degree in Computer Engineering dissertation (2021/22), Escola d'Enginyeria, UAB, Spain.
The objective of the project is to implement the C instruction subset and make the required modifications to a 32-bit RISC-V 5-stage pipeline in order processor.
The files distribution in each folder may contain:

src: Source Verilog files of the synthesizable HW models.
tb: Testbench Verilog files used to simulate the HW models.
data: Folder containing the programs and data operands used to load the instruction and data memories of the core.
sim: Output simulation files end here.
lib: Contains the branched multifunction IP modules.
syn: Files used at the synthesis tool.

Authors of previous project used as starting point of this project are:

Pau Casacoverta Orta for implementing a base core RV32I compliant with the base instruction set fixed by the ISA standard (https://github.com/4a1c0/RV32i-Verilog).
Raimon Casanova Mohr for tutoring and adding pipeline structure to the core, facilitating the addition of expansions and increasing the data througput.
The open source PULPino's HW infraestructure authors (https://github.com/pulp-platform/pulpino), as this project uses their platform (excluding their core) at the synthesis phase on FPGA chip to use the various I/O interfaces like SPI, UART, GPIO, between others in the future.
Francisco Javier Fuentes Diaz for implementing the M and F extensions (https://github.com/FFD-UAB/RISC-V-Instruction-sets-M-and-F).

Future work:
The test environment needs to be automated to perform a more thorough verification, this way programs could be run with randomly generated instructions.