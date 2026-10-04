# CH32V307_LibXR_Template

CH32V307 的 LibXR 模板工程 / LibXR template project for the CH32V307

## 1. 板子与平台 / Board and Platform

模板使用 WCH CH32V307VC（RISC-V，带单精度浮点，系统时钟 144 MHz，`Link.ld` 按 256 KB Flash、64 KB RAM 配置），系统为 FreeRTOS，外设由 LibXR 的 `ch` 驱动提供。`User/main.c` 设置中断分组，创建运行 `app_main()` 的 FreeRTOS 任务并启动调度器，LibXR 应用代码位于 `User/app_main.cpp`。LibXR 是 `libxr/` 下的 Git 子模块，地址为 `https://github.com/xrobot-org/libxr.git`，本仓库记录的子模块提交固定所用的 LibXR 版本。

```text
User/app_main.cpp         LibXR 应用代码 app_main()
User/main.c               启动 FreeRTOS，创建运行 app_main() 的任务
Core/                     WCH 内核与系统文件、FreeRTOSConfig.h
startup_ch32v30x_D8C.S    启动文件
Peripheral/               WCH 外设库
FreeRTOS/                 FreeRTOS 内核与 RISC-V 移植
Link.ld                   链接脚本
wch-riscv.cfg             OpenOCD 配置（WCH-LinkE，SDI）
libxr/                    LibXR 子模块
```

The template uses the WCH CH32V307VC (RISC-V with single-precision floating point, 144 MHz system clock; `Link.ld` is set up for 256 KB Flash and 64 KB RAM) and runs FreeRTOS; the peripherals are provided by the LibXR `ch` driver. `User/main.c` sets the interrupt priority grouping, creates the FreeRTOS task that runs `app_main()` and starts the scheduler. The LibXR application code is in `User/app_main.cpp`. LibXR is the Git submodule `libxr/` at `https://github.com/xrobot-org/libxr.git`, and the submodule commit recorded in this repository pins the LibXR version in use.

## 2. 示例程序 / Example Application

示例程序 `User/app_main.cpp` 演示 GPIO、I2C、UART、双 USB 与终端：创建 `LibXR::CH32Timebase`，调用 `LibXR::PlatformInit(3, 8192)`。PB4 上的 LED 由 `LibXR::Timer` 任务每 1000 ms 翻转一次；PB3 的按键产生下降沿中断（EXTI3），每次触发翻转 PA15 上的 LED。初始化 I2C1（PB6 / PB7，400 kHz）和 USART2（PA2 / PA3，115200 8N1）。USB OTG FS 与 OTG HS 各提供一个 CDC 串口，`LibXR::STDIO` 绑定到 OTG HS 的 CDC，其上运行 `LibXR::RamFS` 与 `LibXR::Terminal`。

The example application `User/app_main.cpp` shows GPIO, I2C, UART, dual USB and a terminal: it creates `LibXR::CH32Timebase` and calls `LibXR::PlatformInit(3, 8192)`. A `LibXR::Timer` task toggles the LED on PB4 every 1000 ms; the key on PB3 raises a falling-edge interrupt (EXTI3) that toggles the LED on PA15 on each press. I2C1 (PB6 / PB7, 400 kHz) and USART2 (PA2 / PA3, 115200 8N1) are initialized. USB OTG FS and OTG HS each provide one CDC serial port; `LibXR::STDIO` is bound to the OTG HS CDC, on which `LibXR::RamFS` and `LibXR::Terminal` run.

## 3. 构建 / Build

构建使用 CMake 和 WCH RISC-V GCC 15.2 或更高版本，CMake 在版本低于 15.2 时报错。工具链为 `riscv32-wch-elf-gcc`，位于 `PATH` 中；位于其他位置时用 `-DCOMPILER_PREFIX=<前缀>` 指定。镜像 `ghcr.io/xrobot-org/docker-image-ch32-riscv:main` 提供该工具链、CMake 和 OpenOCD。

```bash
git clone --recursive https://github.com/xrobot-org/CH32V307_LibXR_Template.git
cd CH32V307_LibXR_Template
docker run --rm -v "$PWD:/work" -w /work ghcr.io/xrobot-org/docker-image-ch32-riscv:main \
  bash -c 'cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j"$(nproc)"'
```

产物为 `build/CH32V307VC.elf`、`build/CH32V307VC.hex` 和 `build/CH32V307VC.bin`。已克隆但未带子模块时，运行 `git submodule update --init` 获取 LibXR。

`.github/workflows/build.yml` 在上述镜像中递归检出子模块并构建。其中的 `libxr-master` 作业每天运行一次，把 `libxr/` 更新到 LibXR 的 `master` 后构建，用于提前发现兼容性变化，作业失败不影响工作流结果。

Building uses CMake and the WCH RISC-V GCC 15.2 or newer; CMake stops with an error for an older version. The toolchain is `riscv32-wch-elf-gcc` on `PATH`, or located with `-DCOMPILER_PREFIX=<prefix>`. The image `ghcr.io/xrobot-org/docker-image-ch32-riscv:main` provides the toolchain, CMake and OpenOCD.

The output is `build/CH32V307VC.elf`, `build/CH32V307VC.hex` and `build/CH32V307VC.bin`. For a clone made without submodules, `git submodule update --init` fetches LibXR.

`.github/workflows/build.yml` checks out the submodules recursively and builds in the image above. Its `libxr-master` job runs daily, updates `libxr/` to the LibXR `master` branch and builds, which reveals compatibility changes early; a failure of that job does not fail the workflow.

## 4. 烧录与运行 / Flash and Run

`wch-riscv.cfg` 是 OpenOCD 配置，使用 WCH-LinkE（`wlinke` 适配器，SDI 接口，6 MHz），需要 WCH 版 OpenOCD（镜像中已包含）。`openocd -f wch-riscv.cfg` 启动调试服务。VS Code 中 `.vscode/launch.json` 的 `Launch CH32V307` 通过 Cortex-Debug 使用该配置，下载并调试 `build/CH32V307VC.elf`，`gdbPath` 为 `riscv32-wch-elf-gdb`。

运行后 PB4 上的 LED 每 1000 ms 翻转一次，OTG FS 与 OTG HS 各枚举出一个 CDC 串口，OTG HS 的串口上提供 LibXR 终端。

`wch-riscv.cfg` is an OpenOCD configuration for the WCH-LinkE (`wlinke` adapter, SDI interface, 6 MHz) and needs the WCH build of OpenOCD, which the image includes. `openocd -f wch-riscv.cfg` starts the debug server. In VS Code, `Launch CH32V307` in `.vscode/launch.json` uses this configuration through Cortex-Debug to download and debug `build/CH32V307VC.elf`, with `riscv32-wch-elf-gdb` as `gdbPath`.

At run time the LED on PB4 toggles every 1000 ms, USB OTG FS and OTG HS each enumerate one CDC serial port, and the LibXR terminal is available on the OTG HS serial port.

## 许可 / License

本仓库以 Apache-2.0 发布，见 [LICENSE](LICENSE)；`Core/`、`Peripheral/` 中的 WCH 代码、启动文件和 `FreeRTOS/` 保留各自文件头中的版权与许可声明。

This repository is released under Apache-2.0, see [LICENSE](LICENSE); the WCH code in `Core/` and `Peripheral/`, the startup file and the code in `FreeRTOS/` keep the copyright and license notices in their file headers.
