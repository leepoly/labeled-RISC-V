_Quick and dirty instructions:_

Checkout The Code
-----------------

    $ git submodule update --init --recursive


Building The Toolchain
----------------------

To build RISC-V ISA simulator, frontend server, proxy kernel and newlib based GNU toolchain:

    $ export RISCV=/path/to/riscv/toolchain/installation
    $ cd riscv-tools
    $ ./build.sh


Building The Project
--------------------

To build the C simulator:

    $ cd emulator
    $ make

To build the VCS simulator:

    $ cd vsim
    $ make

in either case, you can run a set of assembly tests or simple benchmarks:

    $ make run-asm-tests
    $ make run-vecasm-tests
    $ make run-vecasm-timer-tests
    $ make run-bmarks-test

To build a C simulator that is capable of VCD waveform generation:

    $ cd emulator
    $ make emulator-debug

(note that you must have run `make emulator` at least once before
running `make emulator-debug`)

And to run the assembly tests on the C simulator and generate waveforms:

    $ make run-asm-tests-debug
    $ make run-vecasm-tests-debug
    $ make run-vecasm-timer-tests-debug
    $ make run-bmarks-test-debug

To get FPGA-synthesizable verilog (output will be in `fsim/generated-src`):

    $ cd fsim
    $ make


Updating To A Newer Version Of Chisel
-------------------------------------

To grab a newer version of chisel:

    $ git submodule update --init
    $ cd chisel
    $ git pull origin master