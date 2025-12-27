MuJoCo MPC 3D 仪表盘可视化系统

一、项目信息
姓名：郭浩
学号：232011191
班级：计科2305班
完成日期：2025年12月

二、项目简介
本项目基于 MuJoCo MPC 物理仿真框架，实现了一个 3D 汽车仪表盘可视化系统。
系统主要用于展示车辆关键运行参数，包括转速和速度信息。
通过 3D 转速表、数字速度显示，使车辆运行状态更加直观清晰。

三、功能特性
1. 3D 转速表显示，范围为 0–8000 RPM
2. 数字方式显示车辆速度，当前速度为 10 KM/H
3. 仪表盘采用立体视觉效果设计，具有良好的可视化效果
4. 系统可与 MuJoCo MPC 仿真框架进行数据对接

四、开发环境
操作系统：Ubuntu 24.04 LTS
编译器：GCC 11.3.0
物理引擎：MuJoCo MPC
图形接口：OpenGL
构建工具：CMake
项目路径：~/mujoco_projects/mujoco_mpc

五、项目编译与运行
1. 创建 build 目录并进入
2. 使用 cmake 进行项目配置
3. 使用 make 命令完成编译
4. 运行生成的 mjpc 可执行文件
```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装必要依赖
sudo apt install -y build-essential cmake git \
    libgl1-mesa-dev libglfw3-dev libglew-dev \
    libeigen3-dev libopenblas-dev
```

```bash
# 克隆代码（如果你已有项目，跳过此步）
git clone https://github.com/google-deepmind/mujoco_mpc.git
cd mujoco_mpc

# 编译项目
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

3. 运行程序

```bash
# 运行带仪表盘的汽车仿真（simplecar任务）
./bin/mjpc --task=SimpleCar

# 或者指定自定义场景文件
./bin/mjpc --mjcf=../mjpc/tasks/car/car_simple.xml
```
## 🗂️ 项目结构

```text 
mujoco_projects/mujoco_mpc/
├── build/                 # 编译输出目录
│   └── bin/
│       └── mjpc         # 可执行文件
├── mjpc/
│   ├── dashboard.cc      # 仪表盘模块
│   ├── dashboard_data.h
│   ├── dashboard.h
│   ├── tasks/
│   │   └── simple_car/          # 汽车场景文件
│   │       ├── car_model.xml
│   │       └── task.xml
│   ├── app.cc           # 主程序入口（已集成仪表盘）
│   ├── simulate.cc      # 仿真逻辑（已集成仪表盘）
│   └── render.cc        # 渲染逻辑
└── docs/
    ├── report.md        # 详细技术报告
    ├── screenshots/     # 运行截图
    └── demo.mp4         # 演示视频
```

六、仪表盘组件说明
1. 转速表
   转速范围：0–8000 RPM
   当前显示转速：8000 RPM
   指针指向满刻度位置

2. 速度显示
   显示方式：数字显示
   单位：KM/H
   当前速度：10 KM/H

七、参数设定说明
为便于展示仪表盘效果，系统中采用固定参数值：
RPM = 8000
Speed = 10 KM/H
后续可替换为仿真系统实时数据，实现动态显示。

八、实验结果
实验结果表明，该 3D 仪表盘能够清晰显示车辆转速与速度信息。
仪表盘显示稳定，指针动画平滑，整体效果符合设计预期。

九、实验总结
通过本次项目设计，加深了对 MuJoCo MPC 仿真框架和 3D 可视化技术的理解。
实验完成了车辆转速表和速度显示模块的设计，为后续功能扩展奠定了基础。
