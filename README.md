# DataReplay

一个基于 Qt Widgets 的数据回放与可视化工具，用于解析设备导出的数据文件，并以 2D 曲线和 OpenGL 视图展示加速度、角速度、姿态、噪声和速度等信息。

## 功能概览

- 数据文件解析与回放
- 加速度、角速度、姿态 2D 曲线显示
- 噪声与速度曲线显示
- 加速度 3D 可视化
- 姿态与轨道 3D 模拟显示
- 支持里程校正表加载

## 开发环境

- Qt 5.14.2
- C++
- Qt Widgets
- QOpenGLWidget
- QCustomPlot
- QXlsx 源码集成

## 项目结构

- `main.cpp`：程序入口
- `mainwindow.cpp` / `mainwindow.h`：主界面、回放控制、图表刷新
- `datafileparser.cpp` / `datafileparser.h`：数据解析与里程校正加载
- `accelerationvisualizationwidget.cpp` / `accelerationvisualizationwidget.h`：加速度 3D 可视化
- `posesimulationwidget.cpp` / `posesimulationwidget.h`：姿态与轨道 3D 模拟
- `qcustomplot.cpp` / `qcustomplot.h`：QCustomPlot 源码
- `third_party/QXlsx/`：QXlsx 第三方源码

## 使用方式

1. 使用 Qt Creator 打开 `DataReplay.pro`
2. 配置 Qt 5.14.2 对应的 Kit
3. 构建并运行程序
4. 在主界面中打开数据文件进行回放和查看
5. 如有需要，可额外加载里程校正文件

## 数据说明

- 项目主要处理设备导出的原始数据
- 解析结果包含时间、IMU、姿态、噪声、速度及相对位移等信息
- 部分输入文件虽然使用 `.txt` 扩展名，但实际内容可能为二进制帧流

## 说明

- 仓库中已排除本地开发配置、构建产物和 `.trae` 目录
- 当前仓库以源码集成为主，便于直接在 Qt Creator 中打开和维护
