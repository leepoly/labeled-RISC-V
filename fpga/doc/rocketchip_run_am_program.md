# 怎样在RISC-V PARD平台上运行AM应用

## AM是什么

AM是一套跨平台的裸机底层代码，配套的运行库以及常用的裸机程序。在RISC-V PARD开发中，我们使用AM程序来完成如下测测试：

* 硬件功能测试
* 性能测试
* 系统正确性的回归测试

## 配置以及运行AM

### 配置

将gitlab上的nexus-am的`tracebuffer-demo`分支clone下来，按照AM的ReadMe，把AM_HOME环境变量设置好。

### 运行程序

在RISC-V PARD平台上运行程序，你需要将编译出的bin文件拷贝到fpga/build目录下，并重命名为linux.bin，并将mem.bin软链接指向linux.bin。为什么要这么搞，我也不知道，详询诲喆，但是this works。之后再`make run-emu -j4`，程序串口的输出会出现在serial@6000***文件中。

在nexus-am/apps目录下是一系列裸机应用程序，它们的项目组织以及编译流程基本都遵循AM的框架（见ReadMe）。要在PARD平台上运行AM程序，你只需要编译出bin，然后照着上面的步骤将bin文件移到fpga/build目录下，然后直接跑就可以了。编译以及拷贝bin，新建软链接的过程建议自己写个bash脚本自动化。

例如对于hello，我的脚本内容如下：
``` bash
make ARCH=riscv64-rocket
cp ~/riscv-pard/sw/nexus-am/apps/hello/build/hello-riscv64-rocket.bin ~/riscv-pard/labeled-RISC-V/fpga/build/linux.bin
ln -sf ~/riscv-pard/labeled-RISC-V/fpga/build/linux.bin ~/riscv-pard/labeled-RISC-V/fpga/build/mem.bin
```

对于microbench，microbench编译时有一个编译选项`INPUT`，可以选择测试模式`TEST`以及跑分模式`REF`。一般在仿真时就跑小规模的测试模式。我的脚本内容如下：
``` bash
make ARCH=riscv64-rocket INPUT=TEST
rm -rf ~/riscv-pard/labeled-RISC-V/fpga/build/linux.bin ~/riscv-pard/labeled-RISC-V/fpga/build/mem.bin ~/riscv-pard/labeled-RISC-V/fpga/build/bin.txt
cp build/microbench-riscv64-rocket.bin ~/riscv-pard/labeled-RISC-V/fpga/build/linux.bin
ln -sf ~/riscv-pard/labeled-RISC-V/fpga/build/linux.bin ~/riscv-pard/labeled-RISC-V/fpga/build/mem.bin
```
