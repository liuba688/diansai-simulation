# MSPM0G3507 小车 CCS 构建

本目录用于使用 CCS 自带的 TI ArmClang 编译正式小车固件。源代码仍位于：

- `../code/`
- `../user/`
- `../../libraries/`

CCS 构建直接使用本机安装的 MSPM0 SDK，不会再复制 SDK；仓库原有的
`libraries/sdk/ti_config/` 只保留当前载板的 SysConfig 生成配置。

## 已验证环境

- CCS：安装于 `D:\ccs`
- TI ArmClang：`ti-cgt-armllvm_5.1.1.LTS`
- MSPM0 SDK：`D:\ccs\mspm0_sdk_2_11_00_07`
- 目标芯片：MSPM0G3507

## 一键编译

在 PowerShell 7 中运行：

```powershell
cd D:\diansai\following_new\project\MSPM0G3507_Car\project\ccs
.\build.ps1 -Rebuild
```

若安装路径不同：

```powershell
.\build.ps1 `
  -CcsRoot E:\ti\ccs `
  -SdkRoot E:\ti\mspm0_sdk_2_11_00_07 `
  -CompilerVersion ti-cgt-armllvm_5.1.1.LTS `
  -Rebuild
```

成功后生成：

```text
build/mspm0g3507_ball_car.out
build/mspm0g3507_ball_car.hex
build/mspm0g3507_ball_car.map
```

`.out` 可直接交给 CCS 下载调试，`.hex` 可用于支持 Intel HEX 的烧录工具。

## CCS 中使用

优先使用本目录中的：

```text
mspm0g3507_ball_car.projectspec
```

通过 CCS 的 ProjectSpec 导入功能创建 `mspm0g3507_ball_car` 工程。该方式已在本机通过完整构建。
`.c` 文件链接到仓库源码；头文件在导入时复制到 CCS 工作区，因此修改头文件后需要重新导入
ProjectSpec。命令行 `build.ps1` 始终直接读取仓库源码。

也可以把本目录作为“Makefile Project with Existing Code”导入。构建命令使用：

```text
D:\ccs\ccs\utils\bin\gmake.exe
```

默认目标为 `all`。清理目标为 `clean`。若 CCS 或 SDK 不在默认位置，在工程的 Make Build Variables 中覆盖：

```text
CCS_ROOT
SDK_ROOT
COMPILER_ROOT
```

链接脚本按 MSPM0G3507 的 128 KiB Flash、32 KiB SRAM 配置，栈空间为 4 KiB。

## 2026-07-27 构建结果

```text
CCS ProjectSpec：0 errors，5 warnings
Flash：25,264 bytes（约 19.3%）
SRAM：7,007 bytes（约 21.4%，含 4 KiB 栈）
```

5 个警告来自仓库原有逐飞底层库，不来自新增寻球代码。
