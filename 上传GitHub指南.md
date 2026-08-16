# 上传 GitHub 指南（选择性开源）

本仓库：`stm32-ugv-mine-transport`（井下轨道运输车 STM32F103RCT6 固件）
远程已配置：`https://github.com/2396344866/stm32-ugv-mine-transport.git`（本地 `origin` 已指向）

---

## 一、哪些**不**随仓发布（已自动排除，无需手动处理）

本仓库已在 `.gitignore` 中排除**受许可证约束、禁止再分发**的第三方库，push 时这些文件**根本不会上传**：

| 排除项 | 原因 | 状态 |
|---|---|---|
| `Motion_driver/`（InvenSense / TDK eMPL & MotionDriver 全套） | InvenSense 许可**禁止再分发** DMP 固件 | ✅ 已被 `.gitignore` 排除 |
| `User/Modules/mpu6050/` 中的 eMPL 文件（`inv_mpu.c/.h`、`inv_mpu_dmp_motion_driver.c/.h`、`dmpKey.h`、`dmpmap.h`、`libmpllib.lib`） | 同上，专有 DMP 固件 | ✅ 已被 `.gitignore` 排除 |

> 验证：`git check-ignore User/Modules/mpu6050/inv_mpu.c` 有输出即表示已被拦住，不会入库。

**会随仓发布的**（均可开源）：
- 作者自研代码：`User/app`、`User/Modules/mpu6050` 中用户封装部分（`MPU6050.c/.h` 等）、`board_config.h`、`syscalls.c`、固件本体。
- 第三方**开放可再分发**组件：FreeRTOS 10.5.1、STM32 StdPeriph、CMSIS（各自遵循原始许可证）。
- 休眠的 Qorvo **DW3000** 官方驱动（`User/Modules/dw1000/`，9 个文件）：属于 Qorvo 公开 API，可再分发；目录名沿用 `dw1000` 但实为 DW3000 驱动、当前未激活，保留无害且与"BU03 基于 DW3000"口径一致。

---

## 二、推送步骤（在本机终端执行）

本环境无法交互输入凭证，请在**你自己电脑**的终端里操作：

```bash
cd "C:\Users\123\Desktop\RCT6"
git status                 # 确认只有预期改动（无 eMPL/密钥）
git push -u origin main    # 推 1 个未发布的干净 commit（fast-forward，无需 force）
```

### 凭证说明（二选一）
- **HTTPS + Personal Access Token（推荐新手）**
  1. GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic) → Generate new token。
  2. 勾选 `repo`（完整仓库控制权限）。
  3. 推送时用户名填 GitHub 账号，**密码栏粘贴 PAT**（不是账号密码）。
  4. 想免密：用 `git credential-manager` 缓存，或把 remote 改为 `https://<PAT>@github.com/2396344866/stm32-ugv-mine-transport.git`。
- **SSH（一劳永逸）**
  1. `ssh-keygen -t ed25519 -C "你的邮箱"`，把 `~/.ssh/id_ed25519.pub` 内容加到 GitHub → Settings → SSH and GPG keys。
  2. 改 remote：`git remote set-url origin git@github.com:2396344866/stm32-ugv-mine-transport.git`
  3. 再 `git push -u origin main`。

---

## 三、仓库可见性

- 简历已公开链接该仓库，建议设为 **Public**（专有 eMPL 已被排除，公开无合规风险）。
- 若暂不想公开：先在 GitHub 建为 **Private**，本地照常 push；想公开时 Settings → Change visibility → Make public 即可，无需改代码。

---

## 四、克隆者须知（写在仓库 README 已说明，这里提醒）

- 仓库**不含 eMPL**。克隆后若编译涉及 MPU6050 **DMP** 的部分，会缺 `inv_mpu*` 等文件：
  - 方案 A：从 TDK / InvenSense 官方获取 MotionDriver 包，按仓库内 DMP 移植指南放置后编译；
  - 方案 B：改用自写 `MPU6050.c` 寄存器级姿态解算（不依赖 DMP）。
- 其余（GCC `Project/build.py`、Keil `stm32f103RCT6.uvprojx`）均可直接编译，无外部专有依赖。
- 休眠的 `User/Modules/dw1000/`（DW3000 驱动）未接入活动路径，不影响固件编译。

---

## 五、后续更新流程

```bash
cd "C:\Users\123\Desktop\RCT6"
# 改完代码/文档后
git add -A                    # 注意：会同时暂存未追踪文件，确认无敏感/大文件
git status                    # 二次确认暂存内容
git commit -m "简述本次改动"
git push                      # 已设 upstream 后直接 push
```

> 提示：`.gitignore` 已锁死 eMPL，正常 `git add -A` 也不会把它加进来；但若误把整个第三方包拖进未忽略路径，请先确认再提交。

---

## 六、可选：蓝图截图

`Doc/蓝图截图.png` 当前为**未追踪**文件（本地保留即可，未纳入版本控制）。若想随仓发布，执行：
```bash
git add "Doc/蓝图截图.png" && git commit -m "docs: 新增蓝图截图" && git push
```
如不需公开，保持不追踪即可。
