# UltraSound
## Quartus 端
`grvd.v`里面是Quartus的verilog代码，Quartus工程文件夹里面还有一堆奇奇怪怪的文件我暂时先没有放上来，Qsys的在`soc_system.qsys`里。但我不清楚是否可以直接用Quartus在本地打开
## C端
- `hps_0.h`为Quartus工程生成的头文件。使用Quartus自带的shell运行`generate_hps_qsys_header.sh`即可。其中记录了我们定义的各种寄存器、队列的相对地址信息（同Qsys中）
- `main.c`中负责计算延迟和输送信号，目前重新起了一个进程计算和输送延迟信号，具体能不能跑还没试过。用自带的shell跑Makefile文件就可以得到可执行文件（需要放到hps里执行）
