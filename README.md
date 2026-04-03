# b2z1_ros2 使用说明

## 1. 这个仓库做什么

本仓库用于在 ROS2 下运行 Unitree 相关示例，主要分两部分：

- `cyclonedds_ws/`：消息与接口包（`unitree_go`、`unitree_hg`、`unitree_api`）
- `example/`：示例节点包（`unitree_ros2_example`）

当前与 B2 相关的两个示例：
- `b2_stand_example`：官方标准 B2 示例（仅 B2，不含 Z1）
- `b2z1_stand_example`：B2+Z1 联动示例（你当前希望使用的版本）

说明：
- `WITH_Z1_SDK` 编译开关已绑定在 `b2z1_stand_example`，不会影响官方 `b2_stand_example`。

## 2. 目录放置要求（重点）

`z1_sdk` 需要和 `b2z1_ros2` 同级放置。

推荐目录结构：

```text
/home/vi/lxy/b2+z1/
├── b2z1_ros2/
│   ├── cyclonedds_ws/
│   └── example/
└── z1_sdk/
    ├── include/
    └── lib/
```

原因：
- `example/src/CMakeLists.txt` 中使用了相对路径 `../../../z1_sdk` 查找 SDK。
- 会检查 `z1_sdk/include` 和 `z1_sdk/lib` 是否存在来决定是否开启 `WITH_Z1_SDK`。

## 3. 是否需要编译 `z1_sdk`

结论：通常不需要单独编译 `z1_sdk`。

- 本工程只依赖 `z1_sdk/include` 头文件和 `z1_sdk/lib/libZ1_SDK_<架构>.so` 动态库。
- 只要 `lib` 目录里有对应架构库（例如 `libZ1_SDK_x86_64.so`），就可以直接编译本仓库。
- `z1_sdk` 目录里的 `CMakeLists.txt` 主要是编译它自己的示例程序，不会重新产出核心闭源 SDK 库。

## 4. 编译流程（推荐顺序）

### 步骤 1：编译接口消息工作区

```bash
cd /home/vi/lxy/b2+z1/b2z1_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select unitree_go unitree_hg unitree_api
```

### 步骤 2：编译示例工作区

```bash
cd /home/vi/lxy/b2+z1/b2z1_ros2/example
source /opt/ros/humble/setup.bash
source /home/vi/lxy/b2+z1/b2z1_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select unitree_ros2_example --cmake-clean-cache
```

说明：
- 使用 `--cmake-clean-cache` 可以确保 `WITH_Z1_SDK` 开关状态按当前目录真实情况重新判断。

## 5. 运行前环境加载

可选方式 A（默认网卡）：

```bash
source /home/vi/lxy/b2+z1/b2z1_ros2/setup_default.sh
```

可选方式 B（指定物理网卡 enp3s0）：

```bash
source /home/vi/lxy/b2+z1/b2z1_ros2/setup.sh
```

可选方式 C（本地回环 lo，常用于本机测试）：

```bash
source /home/vi/lxy/b2+z1/b2z1_ros2/setup_local.sh
```

然后再加载示例包：

```bash
source /home/vi/lxy/b2+z1/b2z1_ros2/example/install/setup.bash
```

## 6. 运行示例

运行官方 B2 示例（仅 B2）：

```bash
ros2 run unitree_ros2_example b2_stand_example
```

运行 B2+Z1 联动示例（推荐）：

```bash
ros2 run unitree_ros2_example b2z1_stand_example
```

## 7. 如何确认 `WITH_Z1_SDK` 已开启

### 方法 1：看编译日志

在 `colcon build` 输出中出现：

- `Found z1_sdk at ...`

### 方法 2：看编译标志

```bash
grep "WITH_Z1_SDK" /home/vi/lxy/b2+z1/b2z1_ros2/example/build/unitree_ros2_example/CMakeFiles/b2z1_stand_example.dir/flags.make
```

出现 `-DWITH_Z1_SDK=1` 即表示已开启。

## 8. 常见问题

### Q1：换电脑后如何保证 Z1 功能可用？

1. 保证目录结构不变（`z1_sdk` 与 `b2z1_ros2` 同级）。
2. 保证 `z1_sdk/lib` 下有当前机器架构的库（`x86_64` 或 `aarch64`）。
3. 重新执行第 4 节编译命令（带 `--cmake-clean-cache`）。

### Q2：为什么编译成功但没有 Z1 控制？

通常是 `z1_sdk` 路径不对或 `lib` 缺失，导致自动降级为不定义 `WITH_Z1_SDK`。

### Q3：`b2z1_stand_example` 是否可以直接运行？

可以，已加入 CMake 可执行目标：

```bash
ros2 run unitree_ros2_example b2z1_stand_example
```
