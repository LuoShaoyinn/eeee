# Camera1 鱼眼去畸变与 Cubie 光流

本项目用于 Camera1 USB 鱼眼相机的标定、实时去畸变、PC 端光流调试，以及 Cubie A7S 上的去畸变光流服务。

## 当前有效配置

- 相机：Camera1（Windows OpenCV 索引 `1`；Cubie 设备 `/dev/video0`）
- 标定板：`12 × 9` 个方格，即 `11 × 8` 个**内角点**
- 方格边长：`20 mm`（`0.020 m`）
- 标定和运行分辨率：`1280 × 720`
- 模型：OpenCV `fisheye`
- 标定 RMS 重投影误差：约 `0.200 px`

最终应使用以下一组参数文件：

- `camera1_fisheye_1280x720_rectilinear_f400.npz`
- `camera1_fisheye_1280x720_rectilinear_f400.yaml`

文件包含鱼眼内参 `K`、畸变系数 `D`，以及已验证的矩形输出投影 `rectified_K`：

```text
rectified_K = [[400,   0, 640],
               [  0, 400, 360],
               [  0,   0,   1]]
```

该镜头视场超过普通矩形透视投影可完整表示的范围。不要使用自动“保留全部鱼眼视场”的去畸变输出，否则可能出现放射状拉伸条纹。当前方案以 400 px 输出焦距校正中心有效区域，并裁掉最外圈视场。

## PC 端使用

PC Python 依赖位于 `.venv-pc`，当前使用 OpenCV 4.11。

### 实时去畸变预览

```powershell
.\.venv-pc\Scripts\python.exe .\tools\fisheye_calibrate.py preview `
  --camera 1 --width 1280 --height 720 --backend dshow `
  --calibration .\camera1_fisheye_1280x720_rectilinear_f400.npz
```

按 `q` 或 `Esc` 关闭窗口。

### 光流调试

`tools/optical_flow_debug.py` 默认使用 Camera1、1280×720 和最终去畸变参数。光流和显示均在去畸变图像上运行。

```powershell
# 仅预览
.\.venv-pc\Scripts\python.exe .\tools\optical_flow_debug.py --mode view

# 光流；给定相机离地高度后会自动采用保存的 400 px 焦距输出米/秒
.\.venv-pc\Scripts\python.exe .\tools\optical_flow_debug.py --height-m 0.20
```

### 采集去畸变数据集

`tools/dataset_capture.py` 会在保存前去畸变，默认输出目录为 `dataset/images/rectified`。

```powershell
.\.venv-pc\Scripts\python.exe .\tools\dataset_capture.py --source local
```

### PC 端曝光

各 PC 相机工具提供 `--exposure` 参数；数值越负通常越暗，例如 `--exposure -6`。不同 Windows/UVC 驱动的取值范围可能不同，若驱动不接受会保留原曝光模式。

## 重新标定

采集时必须让棋盘格完整可见，并覆盖中心、四边和四角；仅在画面中央采集会得到中心区域 RMS 很低、但边缘无法正确去畸变的错误结果。

```powershell
# 采集：绿色角点完整识别后按空格保存，q 退出
.\.venv-pc\Scripts\python.exe .\tools\fisheye_calibrate.py capture `
  --camera 1 --board-cols 11 --board-rows 8 --width 1280 --height 720 `
  --backend dshow --images .\calibration_images_camera1_new

# 计算参数
.\.venv-pc\Scripts\python.exe .\tools\fisheye_calibrate.py calibrate `
  --board-cols 11 --board-rows 8 --square-size-m 0.020 `
  --images .\calibration_images_camera1_new --output .\camera1_fisheye_new.npz
```

重新标定后，应重新生成包含 `rectified_K`（400 px 输出焦距）的最终参数组，并部署到 Cubie。

## Cubie A7S

### SSH 连接

```bash
ssh radxa@192.168.19.105
```

本机已生成专用于部署的 Ed25519 密钥，并已将其公钥加入 Cubie 用户 `radxa` 的 `authorized_keys`。私钥位于本机 SSH 目录，不应提交、复制或写入本文档。

### 当前服务

服务名为 `cubie-optical-flow.service`，其行为如下：

- 使用 `/dev/video0` 的 `1280 × 720` MJPEG、30 FPS 模式；
- 从 `/home/radxa/optical_flow/camera1_fisheye_1280x720_rectilinear_f400.yaml` 读取 `K`、`D` 和 `rectified_K`；
- 在光流、视频和 `live_preview.jpg` 前应用鱼眼去畸变；
- `raw_preview.jpg` 保留原始鱼眼画面；
- 当前使用 V4L2 自动曝光（`auto_exposure=3`）；
- 原有黄色圆柱、红色方块的颜色识别算法已移除，只保留光流逻辑。

常用维护命令：

```bash
sudo systemctl status cubie-optical-flow
sudo systemctl restart cubie-optical-flow
journalctl -u cubie-optical-flow -f
v4l2-ctl -d /dev/video0 --get-ctrl=auto_exposure,exposure_time_absolute
```

### 部署

部署工具会上传 C++ 源码、标定 YAML 和 systemd 服务文件，在 Cubie 本机编译后再替换二进制并重启服务：

```powershell
.\.venv-pc\Scripts\python.exe .\tools\deploy_cubie.py --deploy
```

首次配置或检查相机时：

```powershell
.\.venv-pc\Scripts\python.exe .\tools\deploy_cubie.py --install-key --inspect
```

部署命令会在需要时提示输入 Cubie 的 SSH/sudo 密码；不要把密码写入脚本、README 或 Git。

## 性能说明

Cubie 在 1280×720 全分辨率上执行稠密 Farneback 光流时，当前实测约 1.9 FPS。若需要提高 PC 或 Cubie 的帧率，建议按以下优先级优化：

1. 保持 1280×720 去畸变，但将灰度图缩小到 50% 后计算光流，位移再按比例还原；计算量约为原来的四分之一。
2. 减少调试箭头绘制密度（例如从 20 px 提高到 40–60 px），或关闭箭头绘制。
3. 只在去畸变后的有效中心 ROI 计算光流，避开鱼眼边缘与黑边。
4. 将 Farneback 参数调为较快配置，例如 `levels=2`、`iterations=2`、`winsize=15`；会牺牲部分大位移和抗噪能力。
5. 将取帧、计算和显示拆分线程，降低 GUI/写文件对取帧的影响。

## 主要文件

- `tools/fisheye_calibrate.py`：棋盘格采集、鱼眼标定和实时去畸变预览。
- `tools/camera_rectify.py`：PC 工具共享的去畸变映射加载逻辑。
- `tools/optical_flow_debug.py`：PC 端去畸变光流调试。
- `tools/dataset_capture.py`：PC 端去畸变数据集采集。
- `tools/cubie_optical_flow.cpp`：Cubie 去畸变光流服务源码。
- `tools/cubie-optical-flow.service`：Cubie systemd 服务配置。
- `tools/deploy_cubie.py`：Cubie 密钥配置、相机检查和部署工具。
