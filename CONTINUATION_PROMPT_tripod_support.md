# 三足支撑（单腿摆动）参考步态：在「前脚踩台面 + 后脚在地」下把质心做进支撑三角 —— 继续工作文档 v1.0

> 给下一个 Claude Code 会话：本文件是 **「用 Crocoddyl 为轮足四足 pcb_v2 生成『三足支撑（一次只摆一条腿）』参考步态 CSV，并解决『单腿摆动时质心投影不在另外三支撑脚三角形内』的问题」** 的实时交接。**请用简体中文回复。**
>
> 这是 **Crocoddyl 侧（轨迹生成）** 的工作，不是 Isaac Lab / RL 侧。产物是 23 列 100Hz CSV，喂给下游 `Robot_cooperation` 的 `csv_to_npz.py` → BeyondMimic 参考动作（见 `RUN_PIPELINE.md` §4 与记忆 `crocoddyl-to-robotcoop-pipeline`）。
>
> ### ★ 本轮范围（2026-07-02 用户口径）★
> 1. **质心不在三角形里的问题**：现有「三足支撑」参考（学长版 + 我复现的 stock 版）在**摆后腿时质心落在支撑三角形外 14~22 cm**（静态会翻，见 §0.4）。这是本轮**主线**。
> 2. **导出不同策略**：把「哪条腿先动 / 什么顺序 / 走哪个方向」参数化，批量导出多份 CSV 供下游挑选/对比（见 §0.5-B）。
> 3. **三足支撑模块还能做什么**：头脑风暴 + 挑几项落地（见 §0.5-C / §五）。
>
> ### ★★ 三条最重要的前提 ★★
> 1. **本机就是生成机**：conda 环境 `croco310`（py3.10、pinocchio 3.9.0），fork 已编译在 `build_conda/`。`conda activate croco310` 后即可 solve / 导出 / 渲染，不需要别的机器。**旧 `build/` 已失效，只用 `build_conda/`。**
> 2. **求解很便宜**（≠ RL 训练那种 5.5h）：Crocoddyl 一个步态周期 FDDP 求解 ~几秒（15~137 iter）。**可以放心多试、批量扫参数。**
> 3. **验收 = 定量 + 肉眼**：`examples/_verify_tripod_com.py <csv>`（逐摆动相报「质心是否在三支撑脚三角形内 + 裕度」）是本轮**核心判据**；再配 `render_headless.py`/`plot.py` 肉眼确认「确实一次只摆一条腿、前脚在台上、后脚在地、不翻」。
>
> **工作方式**：可「改一处 → solve → `_verify` → 再改」自主迭代。**commit 到 main、单作者、不加 Co-Authored-By**（见记忆 `git-workflow-prefs`）；**改动先送审再 commit**。⚠ **用户明确偏好「尽量不改 crocoddyl 库代码」**（`bindings/python/crocoddyl/utils/quadruped.py`）——优先在**驱动脚本层**用现成 API 组装；确需改库时先说清代价并留可回退（见 §二）。

---

## 〇、当前状态速览（先读这段）

**一句话进度**：学长「没改任何代码」跑出的三足支撑 = **stock `createWalkingProblem` 侧向(−Y)走**（`trajectory_single_leg_acc_f005.csv`，1320 行）。我已**零库改**完整复现（`examples/quadruped_walking_fwddyn.py` → `trajectory_walking_sideways.csv`，与学长版 base 差 ≤0.023 m、腿关节均差 0.027 rad）。**但 `_verify` 坐实：这套 stock 参考在摆后腿(RL/RR)时质心在支撑三角外 14~22 cm（静态不稳），只因 Crocoddyl 接触力非单侧(可"拉地")+ PyBullet 回放无物理，才看起来像干净三足支撑。摆前腿(FL/FR)则质心在三角内、裕度 +0.12~0.15（安全）。** 本轮要把「摆后腿质心出界」修掉。

> **我之前的修法（`createTripodProblem`）已按用户要求回退**：它把质心逐相拉到「三支撑脚三角形内最近可达点」+ 前移沉降相 + 放软 stateWeights，做到 235/238 帧在三角内；但那版是**向前(+X)走**、机身有诡异侧偏、且**改了库**。用户否决 → 库与 `RUN_PIPELINE.md` 已 `git checkout` 回 HEAD（stock）。**那份工作完整存在 scratchpad `my_tripod_work/`**（`quadruped.py.mymod`、`my_changes.patch`、`quadruped_tripod_fwddyn.py`），要复活直接 `git apply` 那个 patch。
>
> **★ 关键洞察（省下一会话踩坑）**：用户吐槽的「机身诡异侧偏」大概率是因为我当时**向前(+X)走**——前脚在台上向前迈会把机身带歪。**学长/被接受的步态是侧向(−Y)走，−Y 位移是目的、不是漂移。** 所以本轮的质心修正应当**加在侧向(−Y)步态上**，很可能既解决质心出界、又不会再"诡异侧偏"。这是首选试法（§0.5-A 方案 1）。

### 0.1 环境 & 运行速览

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate croco310
REPO=/home/zzc/Desktop/zhuoxili-jackie/crocoddyl
export PYTHONPATH=$REPO/build_conda/bindings/python:$REPO/examples
cd $REPO/examples          # 必须在 examples/ 下（脚本要 import pcb_v2 包）
```

> ⚠ **库改传播的坑**：`build_conda/bindings/python/crocoddyl/utils/` 里**只有编译好的 sourceless `quadruped.pyc`（无 `.py`）**。改源 `bindings/python/crocoddyl/utils/quadruped.py` **不生效**，必须 `cp` 到 `build_conda/.../utils/quadruped.py`（`.py` 会盖过 `.pyc`；`python -c "import crocoddyl.utils.quadruped as m; print(m.__file__)"` 应以 `.py` 结尾）。原始 `.pyc` 备份在 `quadruped.pyc.orig_backup`。**当前库=HEAD（stock），只在你再改库时才需管这条。**

### 0.2 ★★ 本会话已运行/已确认（record）

**(A) 复现学长的三足支撑（零库改）** —— `examples/quadruped_walking_fwddyn.py`
- 用 **stock `SimpleQuadrupedalGaitProblem.createWalkingProblem`**（它本就是「一次摆一条腿、三条支撑」，内部固定顺序 rh→rf→lh→lf）+ `SolverFDDP`，**侧向 `direction=(0,-1)`**，8 周期链式求解（每周期 `x0 = solver.xs[-1]`）。
- 参数：`timeStep=0.01, stepKnots=35, supportKnots=10, stepLength=0.08, stepHeight=0.10` → **8×165 = 1320 行 × 23 列**（与学长 CSV 行数一致）。
- **摆动顺序由「脚帧名绑到哪个 role」决定**（driver 级选择，非改代码）：构造 `SimpleQuadrupedalGaitProblem(model, lf, rf, lh, rh)`，因 createWalkingProblem 按 rh→rf→lh→lf 摆，绑 `rh=FL,rf=RL,lh=FR,lf=RR` → 得学长的 **FL→RL→FR→RR**；自然绑 `lf=FL,rf=FR,lh=RL,rh=RR` → **RR→FR→RL→FL**。
- 产物 `trajectory_walking_sideways.csv`：base X −0.006..0.05、**Y 0..−0.61（侧向走 −Y）**、Z 0.72..0.78、roll −0.01..0.04、**pitch −1.11..−1.09（稳；~−63° 是前脚踩 0.8 m 台的固有站姿）**。FDDP 收敛 15~137 iter，`isFeasible=False` 但 `||ffeas||→0`（gap 已闭，FDDP 标志位习性，可忽略）。
- **与学长 `trajectory_single_leg_acc_f005.csv` 对比**：base 位置 max 差 0.023 m、四元数 0.007、腿关节 max 0.166 / 均 0.027 rad → **同一套步态**（差异仅因我猜 `stepLength=0.08`、solver 迭代数不同）。

**(B) 质心-三角形体检**（`_verify_tripod_com.py`，对**学长版**与**我的 stock 版**结果一致）：

| 摆动腿 | 质心是否在三支撑三角内 | 最差裕度(barycentric) | 读法 |
|---|---|---|---|
| 前腿 FL / FR | **全部在内**（26/26 等）| **+0.12 ~ +0.15** | ✅ 安全 |
| 后腿 RL / RR | **0 帧在内** | **−0.14 ~ −0.22** | ✗ 质心在三角外 14~22 cm |

- 合计 **200/401** 摆动帧在内。**根因**：stock `comRef = 4 脚平均`（≈x=0.07），而摆后腿的三角形质心在 **x≈0.30（很靠前，因两前脚踩在高台上）**；机器人自然质心根本没往前挪 → 摆后腿必然出界。**前腿摆动的三角形质心≈x=0.07≈自然质心 → 天然在内。**
- **为什么回放看不出翻**：Crocoddyl `ContactModel3D` 的摩擦锥是**软代价(weight 1e1)**、力可为**吸附/拉地**（非单侧），优化器廉价地无视之；PyBullet 回放又只重放关节角、无物理。→ 看着像完美三足支撑，真机/物理 sim 摆后腿会翻。**这正是用户最初任务担心的「不然会翻」。**

**(C) 我的 `createTripodProblem` 修法（已回退，存档）**：235/238 帧质心在内（前移沉降相 + 三角内最近可达点 + 软 stateWeights），但那版**向前走 + 改库 + 机身侧偏** → 用户否决、已回 HEAD。patch 在 `scratchpad/my_tripod_work/my_changes.patch`。

### 0.3 必须知道的物理事实（决定改动有没有用）

1. **初始站姿 `go_neutral` 本身就是「前脚踩台、后脚在地」**（FK 实测，无需改 q0）：脚帧 z —— **FL/FR≈0.901、RL/RR≈0.092**（各比接触面高 ~0.09 = 轮半径）；台面顶 z=0.8、地面 z=0；base z≈0.73、pitch≈−1.10 rad（~−63°，前高后低）。
2. **台子(box)只是可视化 / 参考**：`box_center=[0.95,0,0.4]`、`box_size=[1,4,0.8]`（顶 z=0.8、x∈[0.45,1.45]、y∈[−2,2]）。**它不在 URDF 里、不参与 Crocoddyl 接触**——接触是把各脚帧原点钉在其当前世界位置（`ContactModel3D`）。所以脚踩没踩台，取决于 `go_neutral` 摆的位置，不是台子实体。
3. **各脚水平坐标(x,y) @neutral**：FL(0.529, 0.202)、FR(0.529, −0.202)、RL(−0.158, 0.202)、RR(−0.158, −0.202)。**支撑多边形 = 各接触点水平投影的凸包**（重力竖直，只有 (x,y) 决定静平衡；各脚各自保持自己的 z）。摆某腿时，另外 3 脚的三角形质心：摆后腿→x≈0.30（靠前，难够到）；摆前腿→x≈0.07（≈自然质心，好达成）。**这条不对称是本问题的全部要害。**
4. **脚帧命名**：lf=`FL_foot_link`、rf=`FR_foot_link`、lh=`RL_foot_link`、rh=`RR_foot_link`（前=F、后=R；左=L、右=R）。

---

## 0.5 本轮三项任务（下一会话自主推进）

### 0.5-A ★★ 任务一（主线）：把「摆后腿时质心出界」修掉

目标：让 `_verify_tripod_com.py` 在**所有**摆动相（含 RL/RR）都报「质心在三支撑三角内、裕度 > 0（最好 ≥ +0.05）」，同时不牺牲「一次一条腿、不翻、前脚在台后脚在地」。候选杠杆（按推荐优先级，**先单刀、验收归因清楚再组合**）：

- **方案 1（首选，零库改 + 侧向）**：写驱动脚本，用 **stock `createFootstepModels` + `createModel` 在 driver 层组装**（不改库），把每个摆动相的 `comTask`/`comPos0` 从「4 脚平均」换成「**当次 3 支撑脚三角形内、离机器人自然质心最近的可达点**（三角形朝质心收缩 `marginFrac`≈0.3 留裕度）」，并在摆动**前加四脚支撑「预移相」**把质心挪过去、摆动**后加四脚「沉降相」**保持到脚落稳。**方向用侧向(−Y)**（大概率避免上次的"诡异侧偏"）。
  - ⚠ **已知障碍**：stock `createModel` 的 `stateWeights` 写死（base 姿态 500、关节 50），很硬 → 质心可能**够不到**靠前的后腿目标（我上次实测：硬权重下向前只到 x≈0.10~0.16，够不到 x≈0.19+ 的入界线）。若 driver 层只改 comTask 仍进不了三角，**只能二选一**：(i) 小改库放软 stateWeights（见方案 3，可回退）；(ii) 换硬约束解法（方案 2）。**先试 driver-only，实测够不够。**
  - 复用现成逻辑：`scratchpad/my_tripod_work/quadruped.py.mymod` 里的 `createTripodProblem` + `_closest_point_in_triangle` 是现成的「三角内最近可达点 + 预移/沉降」实现，可**照搬到 driver 脚本**（把方向改 −Y、把 stateWeights 作为 driver 变量传入或先用 stock）。
- **方案 2（最"物理正确"，可能要 SolverIntro）**：`createWalkingProblem(..., constraint=True)` 把摩擦锥从软代价变**硬不等式约束**（`ConstraintModelResidual`）→ 优化器**不能再吸附/拉地**，要抬后腿就**必须**把质心挪进三角（否则问题不可行）。→ 质心-三角自动被满足，或直接暴露「此站姿抬后腿本就静态不可行」。**需要能处理约束的 solver**：本 fork 有 `crocoddyl.SolverIntro`（trot 脚本在用）；`WITH_ODYN=False` 故 SolverOdynSQP 不可用。**这是把「质心在三角内」和「物理可实现」绑定的最干净办法，强烈建议试。** 风险：约束解更难收敛、可能报不可行（那本身就是重要结论）。
- **方案 3（改库，最后手段，留可回退）**：直接放软 `createModel` 里的 stateWeights（base 姿态 500→30、关节 50→3）让机身能前倾/侧倾把质心送进三角。这是我上次成功(235/238)的关键之一，但**改了库**、且要注意别把别的步态(trot/walking)带坏（用可选参数 `stateWeights=None` 默认保持 stock，只 tripod 传软值——patch 里就是这么做的）。**用户偏好不改库，故列为后备。**
- **方案 4（改站姿/降难度）**：若上面都难，考虑「抬后腿时把质心目标只送到刚好入界(x≈0.20)而非三角质心(x≈0.30)」（更省力、更可达），或**减小后腿抬腿高度/步长**留裕度，或（需用户同意）**降低台子高度**让 4 脚平均本就落在各三角内。

**验收（每次改完必做）**：`_verify_tripod_com.py <新csv>` 全摆动相在内 + 裕度>0；`render_headless.py` 肉眼确认一次一条腿、不翻、无诡异侧偏；base roll/pitch 范围不爆（roll 别超 ~±0.25、pitch 稳定）。

### 0.5-B ★ 任务二：导出不同策略（哪条腿先动 / 顺序 / 方向）

把「策略」参数化，批量出 CSV 供下游对比：
- **摆动顺序**：createWalkingProblem 内部固定 rh→rf→lh→lf，**顺序只由脚名绑定决定**（4 种旋转：自然=RR,FR,RL,FL；学长=FL,RL,FR,RR；等）。**若要任意顺序**（如"两后腿先、再两前腿"或 wave gait LH→LF→RH→RF），createWalkingProblem 给不了 → 需 **driver 层用 `createFootstepModels` 自己排单腿相**（正好和方案一的组装同源，一并做）。
- **方向**：侧向 `(0,±1)`（现状）/ 向前 `(1,0)` / 斜向。**注意**：向前走 + 前脚在台 → 前脚会朝台前缘(x=1.45)推进、机身易带歪（上次的教训）；侧向最稳。
- **"哪条腿先动"与稳定性**：抬某腿前质心要能移进"去掉该腿"的三角。一般规则=**先动的腿其对角/远端要有支撑、质心先朝支撑侧挪**。对本机「后腿难抬」的特性，顺序应保证**每次抬后腿前质心已尽量靠前**。建议做法：driver 里对每种顺序跑 `_verify`，用「全相最小裕度」当分数排序，导出最优那版 + 若干对比版（文件名带策略标签）。
- 交付：一个参数化驱动（如 `examples/quadruped_tripod_strategies.py`），`--order/--direction/--out` 批量出 CSV + 自动跑 `_verify` 汇总表。

### 0.5-C ★ 任务三：三足支撑模块还能做什么（头脑风暴，挑 1~2 项落地）

1. **静稳定裕度作为一等指标**：把 `_verify` 里「质心到三角各边最小距离(米)」做成随帧曲线导出/打印，量化每份参考的稳定裕度（给下游 reward/筛选用）。
2. **同时导出接触力 / ZMP**：从 `solver` 取每相接触力（`data.differential.multibody.contacts`）；除静态质心外算 ZMP（动态稳定），供 reward shaping。
3. **物理可行参考**：方案 2（constraint=True）产出的「摩擦锥硬约束」参考 = 力学上真的能站住的轨迹，价值高。
4. **台子高度 / 站姿扫描**：参数化 box 高度 & go_neutral，找「抬后腿仍静态可行」的最大台高，给硬件/场景设计定边界。
5. **速度 / 步长 / 步高扫描**：出一组不同 vy、步长的参考让 RL 学一个范围而非单点。
6. **原地转向 / 三足支撑下转弯**。
7. **把 box 放进接触模型**（现在只可视化）：让脚真踩在台面几何上，参考更自洽。
8. **鲁棒性**：扰动初值/脚位，看参考的静稳定裕度对误差的敏感度。

---

## 一、运行方法（速查）

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate croco310
export PYTHONPATH=$REPO/build_conda/bindings/python:$REPO/examples
cd $REPO/examples

# ---- 生成：stock 三足支撑（侧向，零库改；本轮基线）----
python quadruped_walking_fwddyn.py                 # -> trajectory_walking_sideways.csv (1320×23)

# ---- 验收（核心）：逐摆动相「质心在不在三支撑三角内 + 裕度」----
python _verify_tripod_com.py trajectory_walking_sideways.csv        # 我的 stock 版
python _verify_tripod_com.py trajectory_single_leg_acc_f005.csv     # 学长参考版（对照）

# ---- 动画 ----
python render_headless.py trajectory_walking_sideways.csv \
  --urdf pcb_v2/pcb_v2/urdf/pcb_v2.urdf --outdir frames_walk --nframes 60 --gif walking_sideways.gif   # 无显示器
python plot.py trajectory_walking_sideways.csv --urdf pcb_v2/pcb_v2/urdf/pcb_v2.urdf --fps 100          # 有显示器(务必 --fps 100)

# ---- 快速看某脚是否踩台 / neutral 各脚位置（一次性 FK 探针，需要时自己写）----
# 参考 0.3 的数字：FL/FR z≈0.901、RL/RR z≈0.092
```

> ⚠ 复活我回退的 `createTripodProblem`（若走方案 3）：`cd $REPO && git apply <scratchpad>/my_tripod_work/my_changes.patch`，再 `cp` 到 build_conda（见 §0.1 坑）；driver 用 `<scratchpad>/my_tripod_work/quadruped_tripod_fwddyn.py`。**记得把方向改 (0,−1)。**

---

## 二、关于「改不改库」（用户偏好：尽量别改）

- **能在 driver 层做的都在 driver 层做**：`createFootstepModels`(接受 `comPos0/feetPos0/footContacts/swingFootNames/direction`) 与 `createModel`(接受 `comTask/footContacts/swingFootTask`) 都是**现成 public-ish 方法**，够在驱动里自己排单腿相 + 自定义每相质心目标（方案 1 / 任务二都靠这个）。
- **stock `createModel`/`createFootstepModels` 没有 `stateWeights` 入参**（那是我回退掉的加法）。所以 driver-only 改不动「机身姿态/关节」的正则硬度——若质心够不到靠前目标，要么方案 2（硬约束）、要么方案 3（小改库放软，可选参数默认不动 stock，保留可回退）。
- **改库务必**：① 用默认值保持其它步态(trot/walking)行为不变；② `cp` 到 build_conda；③ 记一笔到本文件 §六 + `RUN_PIPELINE.md`；④ 送审再 commit。

---

## 三、验收 / 闭环

1. **改一处**（driver 的 comTask 策略 / 顺序 / 方向 / 或小改库）+ 更新本文件。
2. **solve**（`python <driver>.py`，几秒~1 分钟；看是否收敛、行列数对）。
3. **`_verify_tripod_com.py <csv>`**：全摆动相在三角内、裕度>0（主判据）；对照学长版。
4. **render** 肉眼：一次一条腿、前脚在台后脚在地、不翻、无诡异侧偏；base roll/pitch 不爆。
5. **记一笔到 §六**。**改动先送审再 commit（main、单作者、无 co-author）。**

**判定**：全摆动相质心在内(裕度>0) + 一次一条腿 + 不翻 + 侧向位移正常 + 与 stock 相比无明显新毛病 → 本轮主线达成，导出交付 CSV + 更新 `RUN_PIPELINE.md`。

---

## 四、关键文件 & 工具速查

| 用途 | 文件 |
|---|---|
| **stock 三足支撑驱动（侧向，零库改；本轮基线）** | `examples/quadruped_walking_fwddyn.py` → `trajectory_walking_sideways.csv` |
| 侧向 trot 驱动（上一阶段，勿覆盖）| `examples/quadruped_gaits_fwddyn.py` → `trajectory_trotting_acc_f005.csv` |
| **步态工厂（库；尽量别改）** | `bindings/python/crocoddyl/utils/quadruped.py`：`createWalkingProblem`(单腿摆动、rh→rf→lh→lf)、`createFootstepModels`(单相、driver 可直接调)、`createModel`(接受 `comTask`) |
| **质心-三角形验收（核心）** | `examples/_verify_tripod_com.py <csv>`（逐摆动相 inside/frames + 最差裕度 + 三角质心 vs 实际质心）|
| 动画（headless / GUI）| `examples/render_headless.py`（`--gif`）、`examples/plot.py`（`--fps 100`）|
| 机器人加载 + `go_neutral` 站姿 + 脚帧名 | `examples/pcb_v2/pcbWrapper.py` |
| 学长的参考 CSV（对照目标）| `examples/trajectory_single_leg_acc_f005.csv`（1320×23，FL→RL→FR→RR，侧向 −Y）|
| 环境/编译/23 列 schema/常见问题 | `RUN_PIPELINE.md` |
| **我回退的 CoM 修法（存档，可复活）** | `scratchpad/my_tripod_work/`：`quadruped.py.mymod`、`my_changes.patch`、`quadruped_tripod_fwddyn.py` |
| 相关记忆 | `memory/crocoddyl-to-robotcoop-pipeline.md`（含本轮结论 + build_conda 坑）、`user-wheeled-legged-rl.md`、`git-workflow-prefs.md` |

> scratchpad 绝对路径见会话开头「Scratchpad Directory」；本轮 = `.../8ae69989-.../scratchpad/my_tripod_work/`。

---

## 五、23 列 CSV 契约（不变，抄自 RUN_PIPELINE §4）

| 列 | 字段 |
|---|---|
| 1–3 | `root_pos_x/y/z`（Base_link 世界系平移）|
| 4–7 | `root_rot_x/y/z/w`（**xyzw**）|
| 8–19 | 12 腿关节 `FL/FR/RL/RR × hip/thigh/calf`（取 `q[7:]`）|
| 20–23 | `FL/FR/RL/RR_foot_joint`（fixed 无 DoF → 写 0.0 占位）|

100 Hz（`timeStep=0.01`），无时间戳列。base 位姿取 `Base_link` 帧，关节角取 `q[7:]`。行数 = 各相 `len(solver.xs)` 之和。

---

## 六、变更历史

- **v1.0（2026-07-02，建档）**：
  - **复现学长「零库改」三足支撑** = stock `createWalkingProblem` 侧向(−Y)走 8 周期（`quadruped_walking_fwddyn.py`，supportKnots=10/stepKnots=35/stepLength=0.08/stepHeight=0.10；摆动顺序由脚名绑定选，学长=FL,RL,FR,RR）→ `trajectory_walking_sideways.csv`（1320×23），与学长 CSV base ≤0.023 m / 腿关节均差 0.027 rad = 同一步态。
  - **`_verify` 坐实核心问题**：stock 参考摆前腿质心在三角内(+0.12~0.15)、**摆后腿质心出界 14~22 cm**（0 帧在内），200/401 帧在内。因 Crocoddyl 接触力非单侧 + 回放无物理，看着不翻、实则静态不稳。**= 本轮主线。**
  - **我之前的 `createTripodProblem`（235/238 在内）按用户要求回退**（那版向前走 + 改库 + 机身诡异侧偏被否决）；库与 RUN_PIPELINE 回 HEAD；该工作存 scratchpad `my_tripod_work/`。**洞察：诡异侧偏源于"向前走"，本轮把质心修正加到"侧向走"上很可能两全。**
  - 列出三项任务（§0.5）：质心入三角（§0.5-A：driver 组装 / constraint=True 硬约束 / 放软 stateWeights 三条杠杆）、导出多策略（§0.5-B：顺序由脚名绑定、任意顺序需 driver 排相）、模块扩展头脑风暴（§0.5-C）。
