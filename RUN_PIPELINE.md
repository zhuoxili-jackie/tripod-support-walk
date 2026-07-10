# 轮足四足 (pcb_v2) 步态导出流水线 —— 自运行手册

从 **Crocoddyl 最优控制求解 → 23 列参考轨迹 CSV → PyBullet 动画** 的端到端操作说明。
对应驱动脚本 `examples/quadruped_gaits_fwddyn.py`(侧向 trot 步态)。

> 环境路径以本机为例:`REPO=/home/zzc/Desktop/zhuoxili-jackie/crocoddyl`

---

## 0. 流水线总览

```
quadruped_gaits_fwddyn.py                 plot.py / render_headless.py
   (SolverIntro 解 8 相侧向 trot)             (读 23 列 CSV + URDF 回放)
        │                                          │
   pcb_v2 URDF ──► solver.xs ──► 手写导出 ─► trajectory_*.csv ──► 动画
   (Pinocchio)     (最优状态)     (base 位姿+关节角)
```

- **①** 加载自定义轮足四足 `pcb_v2`,用 `crocoddyl.SolverIntro` 解侧向 trotting(`desired_velocity=[0,-0.05,0]`),8 个步态相首尾相接。
- **②** 逐相遍历 `solver.xs`,对每个状态用 Pinocchio FK 取 `Base_link` 帧位姿 + 12 个腿关节角,写 23 列 CSV(100 Hz,无时间戳列)。
- **③** 用 `plot.py`(PyBullet GUI)或 headless 渲染器读 CSV 回放。

---

## 1. ⚠️ 环境说明(务必先读)

仓库里预编译的 `build/` 是**从别的机器拷来的**(RUNPATH 指向 `/home/user/crocoddyl` + `/opt/openrobots`),其原生依赖(`libpinocchio_default.so.3.9.0`、`libeigenpy.so`、`libboost_python310.so.1.74.0`、`libipopt.so.1`)在本机**已不存在**,且只含 Python 3.10 字节码,**无法直接使用**。

因此本流水线改为在干净的 conda 环境里**重新编译到 `build_conda/`**(`build/` 保持不动,`build*/` 已被 `.gitignore` 忽略)。

### 1.1 一次性:创建 conda 环境 + 编译 fork

```bash
REPO=/home/zzc/Desktop/zhuoxili-jackie/crocoddyl
source ~/miniconda3/etc/profile.d/conda.sh

# anaconda.org 直连易断流；国内建议用清华 TUNA 镜像
M=https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge
conda create -y -n croco310 --override-channels -c $M \
  python=3.10 "pinocchio<4" eigenpy example-robot-data eigen \
  libboost-devel libboost-python-devel \
  cxx-compiler c-compiler cmake make pkg-config \
  numpy scipy matplotlib-base

conda activate croco310
cmake -S $REPO -B $REPO/build_conda -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
  -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python \
  -DBUILD_PYTHON_INTERFACE=ON \
  -DBUILD_WITH_ODYN=OFF \
  -DBUILD_WITH_IPOPT=OFF \
  -DBUILD_TESTING=OFF -DBUILD_BENCHMARK=OFF -DBUILD_EXAMPLES=OFF
make -C $REPO/build_conda -j$(nproc)
```

关键点:
- **`pinocchio<4`** 会解析到 **3.9.0**,正好与 fork 当初链接的版本一致(pinocchio 4.x 是大版本跳变,可能编译不过)。
- **`BUILD_WITH_ODYN=OFF` 必须关**:`odyn>=0.7.0` 是私有依赖,conda 上没有。关掉后 `crocoddyl.WITH_ODYN` 仍会被导出为 `False`,驱动里的 SQP 分支会自动跳过,不影响主路径。
- 编译产物:`build_conda/bindings/python/crocoddyl/libcrocoddyl_pywrap_float{32,64}.cpython-310*.so`。

### 1.2 验证环境

```bash
conda activate croco310
PYTHONPATH=$REPO/build_conda/bindings/python python -c \
 "import crocoddyl, pinocchio, numpy; \
  print('pinocchio', pinocchio.__version__); \
  print('WITH_ODYN', crocoddyl.WITH_ODYN); \
  print('SolverIntro', hasattr(crocoddyl,'SolverIntro')); \
  print('DynamicsSolverType', [x for x in dir(crocoddyl.DynamicsSolverType) if not x.startswith('_')][:4])"
```
预期:`pinocchio 3.9.0` / `WITH_ODYN False` / `SolverIntro True` / `['FeasShoot','HybridShoot','MultiShoot','SingleShoot']`。

---

## 2. ② 跑求解 → 出 CSV

```bash
conda activate croco310
export PYTHONPATH=$REPO/build_conda/bindings/python:$REPO/examples
cd $REPO/examples            # 必须在 examples/ 下（脚本 import pcb_v2 包）
python quadruped_gaits_fwddyn.py
```

- 输出文件名由脚本内 `output_path` 决定,默认 **`trajectory_trotting_acc_f005.csv`**。
- 会打印 8 个相的 DDP 迭代日志,最后 `Done. Wrote 8 phases.`。
- 想改行走速度/方向:改脚本里的 `desired_velocity = np.array([vx, vy, omega])`(当前 `[0,-0.05,0]` = 沿 -Y 侧移)。

### 2.1 对角步态(trot)可参数化驱动 —— 后续改动请基于它

`quadruped_gaits_fwddyn.py` 是**冻结基线,勿改**(单速度/单方向硬编码)。同一个最优控制问题的干净参数化版本是 `examples/quadruped_trot_sideways.py`:

```bash
conda activate croco310
export PYTHONPATH=$REPO/build_conda/bindings/python:$REPO/examples
cd $REPO/examples
python quadruped_trot_sideways.py                    # right(-Y) 0.05 m/s,新站姿,约 7 s
python quadruped_trot_sideways.py --speed 0.20       # right 0.20
python quadruped_trot_sideways.py --direction left   # left(+Y),右移的精确 Y 镜像
python quadruped_trot_sideways.py --pose legacy      # 回到 go_neutral() 老站姿
```

- 输出名 `trajectory_trot_sideways[_left]_v<speed>.csv`(**永不自动写 `trajectory_trotting_acc_*` 冻结名**;要覆盖请显式 `--out`)。
- 其它开关:`--cycles`(默认 8)、`--step-height`(0.1)、`--step-knots`/`--support-knots`(35/5)、`--max-iters`(100)、`--mirror-pose`(auto)、`--display`。`timeStep` 是契约固定值 0.01,不开放。
- **`--pose new` 是默认(2026-07-10)**:前大腿 +20°、前小腿 −50°、后大腿 +35°、后小腿 +50°,base z 0.745913、pitch −67.21°(常量 `Q0_NEW`,与 `quadruped_tripod_lowstep.py` 同一个)。**站姿注入在 driver 里,不改 `pcb_v2/pcbWrapper.py:go_neutral`**——那个函数被另外 5 个 driver 共用。`--pose legacy` 逐字节还原老站姿。
- **`--pose legacy` 复现验收(2026-07-10)**:4 速度 × 2 方向重跑,全部 664 行 × 23 列,与 `trajectory_trotting_acc_{,f}{005,01,015,02}.csv` 逐帧偏差 ≤ **0.55–2.7 mm**(base)/ **0.24–2.6°**(关节),净 Y 位移一致到 0.2 mm。参考 CSV 是在另一台机器(不同 BLAS / 带 ODYN 的构建)上解的,**不可能逐字节相同**;偏差逐周期递减(c0 最大、c7 最小),说明两条轨迹收敛到同一步态。
- **库改动对 trot 惰性**:把 `build_conda` 的 `crocoddyl/utils/quadruped.py` 换回 `8a6a027`(三足改动之前)重跑,输出**逐位相同(max diff 0.0)**——`stateWeights`/`comWeights` 两个新形参默认 `None` 走 stock 分支,trot 从不传它们。**不要为了 trot 去回滚库**,那会打断三足 driver。
- **为什么以 trot 作改动起点**:对角双腿同摆自平衡,hip 侧向内收极小(新站姿 0.05 实测 |max| 前 2.8–3.4°、后 5.1–6.1°),不像单腿摆的 walking / tripod 那样出现膝内八(内八)。
- **LEFT = 右移的精确 Y 镜像**(2026-07-10 起):除翻方向向量外,还做两件事——① 用 `mirror_pose_y()` 镜像初始站姿(新站姿带 roll −0.058°/yaw −1.135°,不做就会从"同向偏航"的姿态起步);② 镜像交给 `SimpleQuadrupedalGaitProblem` 的**左右脚角色绑定**(`lf↔rf`、`lh↔rh`),因为库里 `createTrottingProblem` 恒先摆 `(rfFoot, lhFoot)` 对角,换绑定后左移开局摆 (FL,RR),正是右移 (FR,RL) 的镜像。验收 `python _verify_mirror.py <right.csv> <left.csv>`:实测 base ≤0.48 mm、姿态 ≤0.33 mrad、足端 ≤0.37 mm。
  > 对照:冻结的 `trajectory_trotting_acc_{005,...}` 早于此改动,是**纯翻方向**;老站姿完美 Y 对称,代价只是半周期相位差,所以当时没暴露。用 `_verify_mirror.py` 查那一对会看到 base y 差 24 mm、足端 90 mm。
- **每周期爬升 ~5.7 mm**:摆动 dz 剖面最后一个节点没回零(`2*stepHeight/stepKnots`),落地冲量(权重 1e7)钉在该点。实测每脚每周期 5.5–6.1 mm,8 周期 base z 漂 +38~45 mm。stock crocoddyl 行为,参考 CSV 里也有。
- **没有地面**:`ContactModel3D` 的 Baumgarte 增益是 `[kp=0, kd=50]`,`xref` 是死参数 → 脚停在 `FK(q0)` 放的地方。**初始姿态即地形**。当前站姿前轮在 z≈0.87、后轮在 z≈0.087(前轮踩 0.8 m 高台),摩擦锥法向对四只脚都写死世界 +Z(`Rsurf=I`, `mu=0.7`)。

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate croco310
export PYTHONPATH=/home/zzc/Desktop/zhuoxili-jackie/crocoddyl/build_conda/bindings/python:/home/zzc/Desktop/zhuoxili-jackie/crocoddyl/examples
cd /home/zzc/Desktop/zhuoxili-jackie/crocoddyl/examples
python quadruped_walking_fwddyn.py --speed 0.12
```

- --speed X:目标平均侧向速度(m/s)。
- 输出自动命名:一律写 trajectory_walking_sideways_sc_v<speed>.csv(v1.4 起**永不自动写基线名** trajectory_walking_sideways.csv,防误覆盖冻结基线;哪怕 --speed 0.05 也写 sc_v0.05.csv)。
- --direction {right,left}(可选,默认 right):侧移方向。right = −Y(现状);**left = +Y,作为右移的精确 Y 镜像**——除翻方向向量外还会镜像左右脚角色绑定(只翻方向会残留 RR ~1.2mm 冷启动抖动)。left 输出名带 `left_` 前缀(trajectory_walking_sideways_sc_left_v<speed>.csv),不覆盖右移文件。镜像验收:`python _verify_mirror.py`。
- --cadence-share A(可选,默认 0.5):步频/步长分配。0 = 退回旧「纯步长」;1 = 纯步频;0.5 = 等分(推荐)。
- --out 路径.csv(可选):强制输出文件名。

### 验证产物

```bash
CSV=trajectory_trotting_acc_f005.csv
head -1 $CSV | awk -F, '{print NF,"列"}'     # 期望 23
wc -l $CSV                                   # 期望 665 行 = 1 表头 + 664 数据
head -2 $CSV
```
- **23 列**,**664 数据行** = 8 相 × 每相 83(= 各相 `len(solver.xs)` 之和)。
- 首个数据行应为站立初值(`z≈0.7296`、腿关节 = `go_neutral`、4 个 foot 列 = 0.0)。

---

## 3. ③ CSV → 动画

关节名两套 URDF(`pcb.urdf` / `pcb_v2.urdf`)都能对上 16/16。**数据是 100 Hz,GUI 回放务必 `--fps 100`**,否则速度只有一半。

### 3.1 有显示器:PyBullet GUI

```bash
conda activate croco310
pip install pybullet          # 该环境若还没有
cd $REPO/examples
python plot.py trajectory_trotting_acc_f005.csv \
  --urdf pcb_v2/pcb_v2/urdf/pcb_v2.urdf --fps 100
```
（快捷键:空格暂停,R 回到开头。）

### 3.2 无显示器 / headless:离屏截图

用仓库里的 `examples/render_headless.py`(`p.connect(p.DIRECT)` + TinyRenderer,不需要 X):

```bash
conda activate croco310
pip install pybullet          # 该环境若还没有（Pillow 已随 matplotlib-base 装好）
cd $REPO/examples
python render_headless.py trajectory_trotting_acc_f005.csv \
  --urdf pcb_v2/pcb_v2/urdf/pcb_v2.urdf --outdir frames --nframes 8
# （日常复核用户自己跑 plot.py；不要再批量生成 PNG/GIF）
```

它会打印 `joints matched: 16/16` 和 base 的 XYZ 范围,可用来快速判断步态是否合理(如侧向 trot 时 Y 明显位移)。

---

## 4. 23 列 CSV 接口契约

| 列 | 字段 | 说明 |
|---|---|---|
| 1–3 | `root_pos_x/y/z` | Base_link 世界系平移 |
| 4–7 | `root_rot_x/y/z/w` | Base_link 四元数(**xyzw** 顺序) |
| 8–19 | 12 个腿关节 | `FL/FR/RL/RR` × `hip/thigh/calf`(取自 `q[7:]`) |
| 20–23 | `FL/FR/RL/RR_foot_joint` | 轮/足关节,URDF 里是 fixed 无 DoF → 写 `0.0` 占位 |

无时间戳列;行间隔 = `timeStep = 0.01 s`(100 Hz)。**`timeStep` 是契约固定值,任何变速方案都不得改它**(下游 `csv_to_npz.py` 按 100Hz 取 fps);要提高步频只能缩小 `stepKnots/supportKnots`(见三足支撑驱动 `quadruped_walking_fwddyn.py --cadence-share`)。

> 变速导出(三足支撑,步长×步频联合缩放):
> `python quadruped_walking_fwddyn.py --speed 0.10`(默认 `--cadence-share 0.5`)→ `trajectory_walking_sideways_sc_v0.10.csv`(行数随步频变,仍 100Hz)。
> 间距验收:`python _verify_foot_spacing.py <csv> [<csv>...]`(逐周期末 RL-RR/FL-FR 水平间距 + 单调收窄判定)。

---

## 5. 已做的代码修改(均有 `.bak` 备份)

| 文件 | 改动 | 原因 |
|---|---|---|
| `examples/quadruped_gaits_fwddyn.py` (~27–42 行) | 硬编码 URDF/mesh 路径改为正确的相对 examples 路径 | 原来靠前导 `/` + 字符串拼接歪打正着,脆弱易错 |
| `examples/quadruped_gaits_fwddyn.py` (第 6 行) | 删除 `import example_robot_data` | 导入但从未使用;conda 版无该 python 模块 |
| `examples/pcb_v2/pcbWrapper.py` (第 5、12 行) | 删除 `import meshcat.geometry` 与 `import example_robot_data` | 顶层导入但类中未用;此环境未安装 |

备份:`*.py.bak`、`trajectory_trotting_acc_f005.csv.orig_backup`。

---

## 6. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `import crocoddyl` 报 `libpinocchio_default.so.3.9.0 not found` | 你在用旧的 `build/`。改用 `build_conda/` + `conda activate croco310`。 |
| `ModuleNotFoundError: pcb_v2` | 没在 `examples/` 目录下运行,或 `PYTHONPATH` 未含 `examples/`。 |
| `bad magic number` | 用了非 3.10 的解释器导入 3.10 字节码。务必 `croco310`(Python 3.10)。 |
| conda 下载反复 `IncompleteRead`/`SSLError` | 换国内镜像(TUNA),或多试几次(缓存会累积);pip/PyPI 通常正常。 |
| 动画速度只有一半 | 数据 100 Hz,回放要 `--fps 100`。 |
| 关节对不上 | `plot.py` 的 `JOINT_NAMES` 必须与 URLF 关节名逐字一致(pcb / pcb_v2 均已对齐 16/16)。 |

---

## 7. pcb 与 pcb_v2 说明

驱动**全程用 pcb_v2**:`from pcb_v2.pcbWrapper import pcb` 导入的类虽名为 `pcb`,加载的却是 `pcb_v2` 的 URDF;`load("pcb_v2")` 又独立加载一次同一 URDF(`robot_pcb` 为主模型)。`examples/pcb`(v1)在此驱动中未被使用。
