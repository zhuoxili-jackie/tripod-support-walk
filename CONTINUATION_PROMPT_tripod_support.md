# 三足支撑（单腿摆动）参考步态：在「前脚踩台面 + 后脚在地」下把质心做进支撑三角 —— 继续工作文档 v1.0

> 给下一个 Claude Code 会话：本文件是 **「用 Crocoddyl 为轮足四足 pcb_v2 生成『三足支撑（一次只摆一条腿）』参考步态 CSV，并解决『单腿摆动时质心投影不在另外三支撑脚三角形内』的问题」** 的实时交接。**请用简体中文回复。**
>
> 这是 **Crocoddyl 侧（轨迹生成）** 的工作，不是 Isaac Lab / RL 侧。产物是 23 列 100Hz CSV，喂给下游 `Robot_cooperation` 的 `csv_to_npz.py` → BeyondMimic 参考动作（见 `RUN_PIPELINE.md` §4 与记忆 `crocoddyl-to-robotcoop-pipeline`）。
>
> ### ★ 本轮范围（2026-07-02 用户口径，同日已细化见下）★
> 1. **摆后腿时身体前后倾斜（pitch）的问题**：现有「三足支撑」参考（学长版 + 我复现的 stock 版）在**摆后腿时质心落在支撑三角形外 14~22 cm**（静态会翻，见 §0.4）。**同日细化（见 §0.2-B'）：这个问题不是「侧翻(roll，左右)」——横向分量 dy 在所有摆动相（含安全的摆前腿）都很小且稳定（±6~7cm），说明优化器本身已经做了合理的左右重心转移，「会不会侧翻」这个最初的担心可以放下。真正、且仅存在于摆后腿相的，是纵向(pitch，前后)分量：dx≈−0.22~−0.23 m，四个周期里几乎恒定不变（结构性，不是漂移）——即后腿摆动时机身有前后倾斜/绕支撑边前倾后仰的摔倒风险，不是左右倒。修正目标应聚焦 pitch 轴、只在摆后腿相生效，不要再动 roll 轴（它本来就没坏）。** 这是本轮**主线**。
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

> **同日细化（轴向拆解，2026-07-02 二次会话）**：把「质心 − 三角形形心」偏移拆成前后(dx，pitch 轴)/左右(dy，roll 轴)两个分量后发现——**dy（roll，左右）在所有摆动相都很小、很稳定（±6~7cm），且摆前腿摆后腿差不多，说明左右方向本来就没问题，「会不会侧翻」的担心可以放下**；**问题完全集中在 dx（pitch，前后）**：摆前腿 dx≈0（±1cm，天然安全），**摆后腿 dx≈−0.22~−0.23 m，四个步态周期里几乎恒定不变**（不是逐渐累积漂移，而是「前脚踩台、后脚在地」这个站姿本身决定的结构性偏差）。**所以准确的问题描述是「摆后腿时机身前后倾斜（pitch，绕左右轴前倾/后仰）」，不是笼统的「质心有问题」或「侧翻」。** 后续任何修正方案都应该**只针对 pitch 轴、只在摆后腿相生效**，不要再碰 roll 轴（它没坏——§0.5-A 里回退掉的 `createTripodProblem` 之所以「诡异侧偏」，大概率是当时误往前(+X)走带偏了 roll，而不是 roll 本身需要修正；侧向(−Y)走 + 只修 pitch，理论上不会重蹈覆辙）。

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

### 0.2-B' ★ 轴向拆解（2026-07-02 二次会话，把「出界」拆成 pitch/roll）

对 `trajectory_walking_sideways.csv` 把「质心 − 三支撑三角形形心」拆成 dx(前后=pitch轴)/dy(左右=roll轴)（body 系：X=前后，脚 x≈+0.53(前)/−0.15(后)，轴距 0.68m；Y=左右=**行走方向本身**，脚 y≈±0.20，root 从 y=0 走到 y≈−0.61）：

| 摆动腿 | dx 均值（前后/pitch） | dy 均值（左右/roll） | 结论 |
|---|---|---|---|
| FL / FR（前腿）| **≈0（+0.002~+0.010）** | ±0.070 | pitch 天然安全；roll 有正常的重心转移 |
| RL / RR（后腿）| **−0.222 ~ −0.233**（4 个周期几乎不变） | ±0.06~0.07 | **只有 pitch 出界**；roll 幅度和前腿摆动时基本一样 |

**读法**：roll(dy) 在前腿摆动（本来就稳）和后腿摆动（不稳）里数值几乎一样（都 ±6~7cm），说明**侧向(左右)重心转移是优化器自带的正常行为，不是问题**——「会不会侧翻」这个最初问题的答案是「不会，侧向本身没事」。真正、且仅在摆后腿相出现的，是 pitch(dx) 的巨大且恒定的偏移（−0.22~−0.23m，四周期几乎不变，说明是站姿几何决定的结构性问题，不是累积误差）——**准确描述是「摆后腿时机身前后倾斜（绕左右轴前倾/后仰）」，而不是笼统的「质心问题」或「侧翻」**。修正应该只加在 pitch 轴、只在摆后腿相生效，roll 轴不用动。

**(C) 我的 `createTripodProblem` 修法（已回退，存档）**：235/238 帧质心在内（前移沉降相 + 三角内最近可达点 + 软 stateWeights），但那版**向前走 + 改库 + 机身侧偏** → 用户否决、已回 HEAD。patch 在 `scratchpad/my_tripod_work/my_changes.patch`。

### 0.3 必须知道的物理事实（决定改动有没有用）

1. **初始站姿 `go_neutral` 本身就是「前脚踩台、后脚在地」**（FK 实测，无需改 q0）：脚帧 z —— **FL/FR≈0.901、RL/RR≈0.092**（各比接触面高 ~0.09 = 轮半径）；台面顶 z=0.8、地面 z=0；base z≈0.73、pitch≈−1.10 rad（~−63°，前高后低）。
2. **台子(box)只是可视化 / 参考**：`box_center=[0.95,0,0.4]`、`box_size=[1,4,0.8]`（顶 z=0.8、x∈[0.45,1.45]、y∈[−2,2]）。**它不在 URDF 里、不参与 Crocoddyl 接触**——接触是把各脚帧原点钉在其当前世界位置（`ContactModel3D`）。所以脚踩没踩台，取决于 `go_neutral` 摆的位置，不是台子实体。
3. **各脚水平坐标(x,y) @neutral**：FL(0.529, 0.202)、FR(0.529, −0.202)、RL(−0.158, 0.202)、RR(−0.158, −0.202)。**支撑多边形 = 各接触点水平投影的凸包**（重力竖直，只有 (x,y) 决定静平衡；各脚各自保持自己的 z）。摆某腿时，另外 3 脚的三角形质心：摆后腿→x≈0.30（靠前，难够到）；摆前腿→x≈0.07（≈自然质心，好达成）。**这条不对称是本问题的全部要害。**
4. **脚帧命名**：lf=`FL_foot_link`、rf=`FR_foot_link`、lh=`RL_foot_link`、rh=`RR_foot_link`（前=F、后=R；左=L、右=R）。

---

## 0.5 本轮三项任务（下一会话自主推进）

### 0.5-A ★★ 任务一（主线）：把「摆后腿时机身前后倾斜（pitch 出界）」修掉

目标：让 `_verify_tripod_com.py` 在**所有**摆动相（含 RL/RR）都报「质心在三支撑三角内、裕度 > 0（最好 ≥ +0.05）」，同时不牺牲「一次一条腿、不翻、前脚在台后脚在地」。**§0.2-B' 轴向拆解已确认：这纯粹是 pitch(前后) 轴问题（dx≈−0.22~−0.23m，仅摆后腿相），roll(左右) 轴本来就稳（±6~7cm，摆前/后腿一样），不需要修——所以修正只需把摆后腿相的 comTask 往前(+X)推，不要引入任何改变 roll/侧向行为的项，避免重蹈上次「诡异侧偏」的覆辙。** 候选杠杆（按推荐优先级，**先单刀、验收归因清楚再组合**）：

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
- **方向**：侧向 `(0,±1)`（现状）/ 向前 `(1,0)` / 斜向。**★ 侧向左右两向已落地（v1.5，`--direction {right,left}`）**：右=`(0,−1)`、左=`(0,+1)` 且**左移是右移的精确镜像**（除翻方向向量外还要镜像左右脚绑定，否则残留 RR ~1.2mm 抖动，见 §六 v1.5）。**注意**：向前走 + 前脚在台 → 前脚会朝台前缘(x=1.45)推进、机身易带歪（上次的教训）；侧向最稳。
- **"哪条腿先动"与稳定性**：抬某腿前质心要能移进"去掉该腿"的三角。一般规则=**先动的腿其对角/远端要有支撑、质心先朝支撑侧挪**。对本机「后腿难抬」的特性，顺序应保证**每次抬后腿前质心已尽量靠前**。建议做法：driver 里对每种顺序跑 `_verify`，用「全相最小裕度」当分数排序，导出最优那版 + 若干对比版（文件名带策略标签）。
- 交付：一个参数化驱动（如 `examples/quadruped_tripod_strategies.py`），`--order/--direction/--out` 批量出 CSV + 自动跑 `_verify` 汇总表。

### 0.5-D ★★ 任务四（**v1.3 已完成，2026-07-02 四次会话**）：多速度导出的「步长 × 步频」联合缩放

> **★ 已解决（v1.3）**：给 `quadruped_walking_fwddyn.py` 加 `--cadence-share`（alpha，默认 0.5），把速度比 `R=speed/0.05` 按几何方式拆给步频和步长：`cadenceFactor=R**alpha`、`strideFactor=R**(1−alpha)`，二者乘积恒 = R。**步频靠缩小 stepKnots/supportKnots（缩短 T_step_cycle）实现，timeStep 恒 0.01 不动**；stepLength 再按「实际(取整后)周期时长 × speed」反算,使平均速度精确命中。alpha=0.5 时 R=1（0.05）→ 参数与基线完全一致,基线字节不变。新 3 版 = `trajectory_walking_sideways_sc_v0.10/0.15/0.20.csv`（sc=stride×cadence,与纯步长草稿 `_v0.10.csv` 等区分）。**间距收窄大幅改善**（0.20 版 FL-FR −0.058→−0.008,RL-RR −0.028→−0.0175,均逼近基线 ~−0.01 噪声级；见下表）；步长减半(0.20:0.32→0.164m)、步频×1.95;CoM/pitch 结构不变(后腿最差裕度 −0.24 vs 基线 −0.226,未变糟)。**新增可复用工具 `examples/_verify_foot_spacing.py`**（逐周期末 RL-RR/FL-FR 水平间距 + 单调收窄判定,已用基线/草稿自证复现本节数字）。
>
> **未改库**（纯 driver 层)。**已知未解**：RL-RR 收窄 ~−0.017 几乎不随 alpha 变（alpha 0.35/0.5/0.65 实测 −0.0176/−0.0175/−0.0168）,说明它不是步长/跟踪主导、而是更结构性的东西（疑与后腿 pitch/几何有关,§0.5-A 那条老账）; 峰值摆动脚速 ∝ 平均速度（0.20 恒 ~0.91 m/s,与 split 无关,物理必然,联合缩放降不了,只能靠加大占空比,那是另一个旋钮)。
>
> | 速度 | 方案 | RL-RR cycle0→7 | FL-FR cycle0→7 |
> |---|---|---|---|
> | 0.05 基线 | — | 0.4396→0.4292 (−0.0105) | 0.4459→0.4509 (+0.0050) |
> | 0.10 | **新(联合,sc)** | 0.4438→0.4329 (−0.0109) | 0.4616→0.4609 (**−0.0008**) |
> | 0.10 | 旧(纯步长) | 0.4508→0.4405 (−0.0103) | 0.4836→0.4735 (−0.0102) |
> | 0.15 | **新(联合,sc)** | 0.4504→0.4361 (**−0.0143**) | 0.4728→0.4681 (**−0.0047**) |
> | 0.15 | 旧(纯步长) | 0.4649→0.4445 (−0.0204) | 0.5186→0.4826 (−0.0360) |
> | 0.20 | **新(联合,sc)** | 0.4560→0.4385 (**−0.0175**) | 0.4849→0.4767 (**−0.0082**) |
> | 0.20 | 旧(纯步长) | 0.4797→0.4513 (−0.0284) | 0.5523→0.4944 (−0.0580) |

**背景**：`quadruped_walking_fwddyn.py` 加了 `--speed` 参数后跑了 0.10/0.15/0.20 m/s 三版（见 §六 v1.2）。当前实现**只放大 stepLength，步频(stepKnots/supportKnots/timeStep → T_step_cycle=1.6s)完全不变**：`stepLength = speed * 1.6`（0.05→0.08m，0.20→0.32m）。

**问题**（用户在 `plot.py` 里肉眼发现 0.20 版"步子迈得很大，同时后腿距离越来越近"；我用 FK 复核确认，非主观）：单步位移从 0.08m 顶到 0.32m 后，摆动腿尖速度从 0.23 m/s 飙到 0.91 m/s（0.35s=stepKnots×timeStep 摆动窗口不变），且**左右脚水平间距逐周期收窄、速度越高收窄越狠**（0.05 基线本身有量级 ~1cm/8周期的轻微漂移，可忽略；但不是单调收窄——FL-FR 甚至还变宽了）：

| 速度 | RL-RR 距离 cycle0→cycle7 | FL-FR 距离 cycle0→cycle7 |
|---|---|---|
| 0.05（基线，正常）| 0.4396→0.4292 m（−0.010m）| 0.4459→0.4509 m（**+0.005m，变宽**）|
| 0.10 | 0.4508→0.4405 m（−0.010m）| 0.4836→0.4735 m（−0.010m）|
| 0.15 | 0.4649→0.4445 m（−0.020m）| 0.5186→0.4826 m（−0.036m）|
| 0.20 | 0.4797→0.4513 m（−0.028m）| 0.5523→0.4944 m（**−0.058m，收窄最狠**）|

**可能原因（假说，下一会话需验证/证伪，非定论）**：单步过大让 FDDP 在固定 0.35s 摆动窗口内更难精确命中落脚目标，残余误差经链式热启动（`x0 = 上一周期 solver.xs[-1]`）逐周期累积——即「只加步长、步频不变」把全部速度增量都堆给了单步幅度，超出 stepKnots=35/supportKnots=10 这套时序参数在 0.05 下已验证的工作范围。

**用户明确的设计目标**（转述原话）：
- **0.05 m/s 这版保持不动**（已验证好用，作为基线不要覆盖/回归）。
- 更高速度**不能靠"只加步长"**（当前 `--speed` 的做法：步子过大、间距收窄）；**也不能靠"只加步频"**（即只调小 stepKnots/supportKnots 让摆动更快、步长不变——用户指出这样在 CSV 固定 100Hz 帧率下会让"迈步频率过高"，观感/物理都不自然）。
- 期望**步频与步长联合上调**（类比 步幅×步频=速度）：提速时步子"适当变大 + 适当变快"，两个维度分担速度增量，而不是把增量全部塞给其中一个。

**下一会话要做（未开始，仅记录范围，不要直接复用当前 `--speed` 的纯步长实现）**：
1. 设计联合缩放：给「步长缩放」和「步频缩放」各一个自由度（例如引入 `--freq-scale`/`--length-scale`，或直接按某种权重把 `speed/0.05` 的倍数分给两者），**步频**通过调小 `stepKnots`/`supportKnots`（缩短 T_step_cycle）实现，**不要动 `timeStep=0.01`**——CSV 契约是固定 100Hz（RUN_PIPELINE §4），改 timeStep 会破坏下游 `csv_to_npz.py` 的 fps 假设。
2. 验收新增判据（除老的 `_verify_tripod_com.py` 质心/pitch 指标外）：本节「左右脚水平间距逐周期变化」要接近 0.05 基线量级、**不能单调持续收窄**（尤其别超过基线收窄速率太多）；`render_headless.py`/`plot.py` 肉眼复核步子大小、步频观感是否自然、有无腿部靠近/交叉风险。
3. `trajectory_walking_sideways_v0.10.csv` / `_v0.15.csv` / `_v0.20.csv` 三份是**纯步长缩放**的产物，**已知有此问题，先当草稿/对照用，不要当交付版本**；等联合缩放方案做出来后重新导出替换（文件名待定，注意别覆盖这三份草稿，方便前后对比）。

### 0.5-E ★★ 任务五（**v1.4 已完成，2026-07-02 五次会话**）：修掉「第 0 周期 RL 支撑脚抖动」

**现象**（用户肉眼在 0.1/0.15/0.2 上发现）：一开始 FL 摆动没问题，但 **RL 摆动落地后、在 FR 和 RR 摆动期间，已落地的 RL 支撑脚发生抖动（上下颤动）**；四条腿各摆一次（第 0 周期结束）后消失。FK 复核坐实（非主观）：以「RL 脚在某相内逐帧位移总量 tv（mm）」度量,sc_v0.20 第 0 周期 FR 摆动相 RL_tv=**21.0mm**（z 向 7 次反转）、RR 摆动相 18.9mm,而稳态仅 ~0.16/2.3mm——**第 0 周期是稳态的 ~130 倍**。0.05 基线自身也有(但轻:FR 0.29mm、RR 7.2mm),只是半步小所以肉眼没注意。

**根因**：stock `createWalkingProblem` 的 `firstStep` 缓起步**只把前两个摆动相(rh,rf = 本绑定的 FL,RL)减半、后两个(lh,lf = FR,RR)仍走整步 → 不对称冷启动**。这个不对称让 RL 支撑脚落在一个「代价近乎零」的弱约束位姿上,冷启动的 FDDP 就在那个方向留了小振荡(迭代数也畸高:sc_v0.10 cycle0 曾 135 iter)。**证据**:对照实验(sc_v0.20 cycle0)——firstStep=True(不对称半步)=21.0/18.9mm;四腿整步=0.12/1.93mm 但 CoM 冲到 −0.38(冷启动整步太猛);**四腿对称半步=0.07/0.77mm 且 CoM −0.242(与现状 −0.244 零退化)**。

**修法（v1.4,纯 driver、不改库）**:`gait.firstStep=False`(关掉不对称半步)+ 循环里第 0 周期对**所有四条腿**用 `FIRST_CYCLE_STRIDE_FRAC(=0.5)×stepLength`(对称缓起步),cycle1+ 整步。效果:抖动 21→0.07mm(比稳态还平滑)、**CoM/pitch 零退化**(后腿 dx 均值 −0.233 vs 基线 −0.228、roll |dy| 0.062 vs 0.065,§0.2-B' 稳定指标复核)、**保留从静止 neutral 起步**、迭代数 57→13。

**顺带的间距变化(是改善)**:修复后各档从**名义轨宽 0.406 起步、缓慢变宽 +0.014**,不再是修复前的「外撇 0.45~0.55 → 收窄」。修复前的外撇本身就是同一不对称冷启动的产物;修复后脚回到正确的 0.40 窄间距、且变宽比收窄安全。**注意**:`_verify_tripod_com.py` 的「最差 barycentric 裕度」会显示 0.10/0.15 变差(−0.30/−0.34 vs 修复前 −0.24/−0.26),那是**假象**——重心坐标对三角形大小敏感,脚回到正确窄间距后三角变小,同样的 0.23m 物理 pitch 偏移换算成更负的坐标;真实 pitch(dx 米)没变(见上)。**度量抖动/pitch 请用 dx 均值,别只看单帧 barycentric 最差值。**

> 产物:重新导出 `trajectory_walking_sideways_sc_v0.05/0.10/0.15/0.20.csv`(全部带修复;0.05 也重生成,冻结基线 `trajectory_walking_sideways.csv` 文件不动)。**驱动输出改为一律写 `sc_v*.csv`,永不自动写基线名**(防误覆盖)。**注意**:cycle0 缓起步会经链式(`x0=上一周期末`)传导到**整条轨迹**,不只 cycle0——sc_v0.05 与 stock 基线在稳态也不同(稳态脚间距落在名义 ~0.41m,而基线是外撇 ~0.44m;那个外撇本身就是不对称冷启动的症状)。所以 sc_v0.05 是**真正改进过的 0.05**,不是基线的平移。肉眼复核用 `plot.py <csv> --urdf pcb_v2/pcb_v2/urdf/pcb_v2.urdf --fps 100`（用户自查，**不要再批量生成 PNG/GIF**）。

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

# ---- 生成：stock 三足支撑（侧向，零库改；0.05 基线）----
python quadruped_walking_fwddyn.py                 # -> trajectory_walking_sideways.csv (1320×23)

# ---- 生成：变速交付版（v1.3，步长×步频联合缩放，默认 alpha=0.5）----
python quadruped_walking_fwddyn.py --speed 0.10    # -> trajectory_walking_sideways_sc_v0.10.csv
python quadruped_walking_fwddyn.py --speed 0.15    # -> ..._sc_v0.15.csv
python quadruped_walking_fwddyn.py --speed 0.20    # -> ..._sc_v0.20.csv
# （--cadence-share 0 = 退回旧「纯步长」草稿行为；1 = 纯步频）

# ---- 生成：左移（+Y）= 右移的精确镜像（v1.5，--direction left）----
python quadruped_walking_fwddyn.py --direction left --speed 0.05   # -> ..._sc_left_v0.05.csv
python quadruped_walking_fwddyn.py --direction left --speed 0.20   # -> ..._sc_left_v0.20.csv
# left 会同时翻方向向量 + 镜像左右脚绑定（只翻方向会残留 RR ~1.2mm 抖动）；右移文件不覆盖

# ---- 验收①（核心）：逐摆动相「质心在不在三支撑三角内 + 裕度」----
python _verify_tripod_com.py trajectory_walking_sideways.csv        # 我的 stock 版
python _verify_tripod_com.py trajectory_single_leg_acc_f005.csv     # 学长参考版（对照）

# ---- 验收②（v1.3 新增）：逐周期末左右脚水平间距是否单调收窄 ----
python _verify_foot_spacing.py trajectory_walking_sideways.csv \
  trajectory_walking_sideways_sc_v0.10.csv trajectory_walking_sideways_sc_v0.20.csv   # 多文件 SUMMARY

# ---- 动画（肉眼复核，用户自己用 plot.py；不要再批量生成 PNG/GIF）----
python plot.py trajectory_walking_sideways.csv --urdf pcb_v2/pcb_v2/urdf/pcb_v2.urdf --fps 100          # 务必 --fps 100

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
| **stock 三足支撑驱动（侧向，零库改；本轮基线）** | `examples/quadruped_walking_fwddyn.py --speed 0.05`（默认）→ `trajectory_walking_sideways.csv`（**基线，保持不动**）|
| **变速交付版（v1.3，步长×步频联合缩放）** | `--speed 0.10/0.15/0.20`（默认 `--cadence-share 0.5`）→ `trajectory_walking_sideways_sc_v0.10.csv` / `_sc_v0.15.csv` / `_sc_v0.20.csv`（间距收窄逼近基线，见 §0.5-D 表）|
| 同驱动，纯步长缩放的**旧草稿**（**已知有 §0.5-D 问题，仅留作前后对比基准**）| `--speed 0.10/0.15/0.20 --cadence-share 0`（旧行为）→ `trajectory_walking_sideways_v0.10.csv` / `_v0.15.csv` / `_v0.20.csv` |
| **左移（+Y）= 右移精确镜像（v1.5）** | `--direction left --speed 0.05/0.10/0.15/0.20` → `trajectory_walking_sideways_sc_left_v0.05.csv` … `_v0.20.csv`（同时翻方向向量 + 镜像左右脚绑定；只翻方向会残留 RR ~1.2mm 抖动，见 §六 v1.5）|
| **全状态镜像验收（v1.5）** | `examples/_verify_mirror.py`（无参扫 4 速度对，或 `<right.csv> <left.csv>` 单对：base 位姿 + 四脚 L↔R 交换后的镜像偏差，全速度汇总）|
| **逐周期末左右脚间距验收（v1.3 新增）** | `examples/_verify_foot_spacing.py <csv> [<csv>...] [--cycles 8]`（RL-RR/FL-FR 间距逐周期表 + 单调收窄判定 + 多文件 SUMMARY）|
| 侧向 trot 驱动（上一阶段，勿覆盖）| `examples/quadruped_gaits_fwddyn.py` → `trajectory_trotting_acc_f005.csv` |
| **步态工厂（库；尽量别改）** | `bindings/python/crocoddyl/utils/quadruped.py`：`createWalkingProblem`(单腿摆动、rh→rf→lh→lf)、`createFootstepModels`(单相、driver 可直接调)、`createModel`(接受 `comTask`) |
| **质心-三角形验收（核心）** | `examples/_verify_tripod_com.py <csv>`（逐摆动相 inside/frames + 最差裕度 + 三角质心 vs 实际质心）|
| 动画（肉眼复核，用户自查）| `examples/plot.py <csv> --urdf ... --fps 100`（**不要再批量生成 PNG/GIF**；`render_headless.py` 仅在无显示器时用）|
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

- **v1.5（2026-07-03，六次会话）**：**新增左移（+Y）步态 = 右移的精确镜像（`--direction left`，部分落地 §0.5-B「方向」轴）**。原 `sc_v*.csv` 只走右（−Y）。给 `quadruped_walking_fwddyn.py` 加 `--direction {right,left}`（纯 driver、不改库），左移作为右移的**精确 Y 镜像**导出。**关键：不是只翻 `(0,−1)→(0,+1)`**——只翻方向向量会得到一个「有手性」的左移，冷启动最弱支撑脚落在 **RR ~1.2mm**（右移对应 RL ~0.8mm），肉眼即用户发现的「第一次四腿动时 RR 抖动」。真正的镜像还需**把左右脚角色绑定也镜像**（`rh=FR,rf=RR,lh=FL,lf=RL`，即右移绑定的 L↔R 交换）：这样左移 RR 冷启动降到右移 RL 的 **0.83mm**（各速度对齐到 0.001mm）、摆动顺序镜像成 **FR→RR→FL→RL**、root_y 精确取反。**注意 v1.4 的对称起步修复（`gait.firstStep=False`）与方向无关、两向共享**，所以左移从来没有 21mm 级老 bug，唯一残差就是上面的手性、靠镜像绑定解决。全状态镜像验收（左 vs 右，四速度全 8 周期）：base x 相等/y 取反/z 相等 ≤0.5mm，roll&yaw 取反/pitch 相等 ≤0.9mrad，四脚（L↔R 交换后）≤0.5mm；**非逐位精确**（左右各自独立 FDDP 解 + URDF 非完美左右对称）。CoM 三足结构完全镜像（前腿摆动裕度为正、后腿出界，45/68 与右移一致）。新文件 `trajectory_walking_sideways_sc_left_v{0.05,0.10,0.15,0.20}.csv`（1320/952/776/696×23）；右移文件不动，左移用 `left_` 前缀名。**新增可复用验收工具 `examples/_verify_mirror.py`**（无参 = 扫 4 个速度对；或 `<right.csv> <left.csv>` 单对：逐帧 base 位姿 + 四脚 L↔R 交换后的镜像偏差表）。步态提交 `2ad9242`，文档+工具提交见其后 docs 提交。
- **v1.4（2026-07-02，五次会话）**：**修掉「第 0 周期 RL 支撑脚抖动」（新增 §0.5-E）**。根因 = stock `firstStep` 的**不对称**缓起步（只减半 FL/RL 摆动、FR/RR 仍整步）在冷启动下让 RL 支撑脚留了小振荡（sc_v0.20 cycle0 RL 脚逐帧位移 21mm，稳态仅 0.16mm，~130×；0.1/0.15/0.2 都有，0.05 基线也轻微有）。修法（纯 driver、不改库）：`gait.firstStep=False` + 循环里 cycle0 对**四条腿对称**用 `0.5×stepLength` 缓起步。效果：抖动 21→0.07mm、**CoM/pitch 零退化**（后腿 dx 均值 −0.233 vs 基线 −0.228、roll 0.062 vs 0.065）、保留静止起步、迭代 57→13。顺带间距回名义值 0.406 且转为**变宽 +0.014**（比修复前外撇→收窄更自然安全）。⚠ `_verify_tripod_com` 的单帧 barycentric 最差裕度会因脚间距变窄而显示"变差"（−0.30/−0.34），是三角变小的假象，**真实 pitch(dx 米) 没变**——度量请用 dx 均值。重新导出 `sc_v0.05/0.10/0.15/0.20.csv`（全部带修复，0.05 也重生成）；驱动输出改为**一律 `sc_v*.csv`、永不自动写基线名**（防误覆盖冻结基线，文件本身不动）。cycle0 改动经链式传导到整条轨迹（非仅 cycle0）：sc_v0.05 稳态脚间距落名义 ~0.41m vs 基线外撇 ~0.44m，是真正改进过的 0.05、非平移。肉眼复核用户自己跑 `plot.py --fps 100`。
- **v1.3（2026-07-02，四次会话）**：**完成任务四（§0.5-D）「步长 × 步频联合缩放」**。给 `quadruped_walking_fwddyn.py` 加 `--cadence-share`（alpha，默认 0.5）：`R=speed/0.05`，`cadenceFactor=R**alpha`、步频靠缩小 `stepKnots/supportKnots`（`timeStep` 恒 0.01 不动）实现，`stepLength` 按实际取整后周期时长反算精确命中速度；alpha=0.5 时 0.05 基线参数与产物字节完全不变。新导出 `trajectory_walking_sideways_sc_v0.10/0.15/0.20.csv`（sc=stride×cadence，952/776/696 行，FDDP 全收敛 stop<1e-8）。**间距收窄大幅改善**（0.20：FL-FR −0.058→−0.008、RL-RR −0.028→−0.0175，逼近基线 −0.01 噪声级），步长减半、步频×1.95，CoM/pitch 结构与基线一致（后腿最差裕度 −0.24 vs −0.226，未变糟）。**新增可复用工具 `examples/_verify_foot_spacing.py`**（逐周期末 RL-RR/FL-FR 间距 + 单调收窄判定，用基线/草稿自证能复现 §0.5-D 数字）。纯 driver 层、未改库。纯步长草稿 `_v0.10/0.15/0.20.csv` 保留做前后对比，未覆盖。**已知未解**：RL-RR 收窄 ~−0.017 几乎不随 alpha 变（结构性，疑与后腿 pitch 有关）；峰值摆动脚速 ∝ 平均速度、联合缩放降不了（物理必然）。
- **v1.2（2026-07-02，三次会话）**：给 `quadruped_walking_fwddyn.py` 加 `--speed`（默认 0.05，行为不变）/ `--out`，按姊妹脚本同款公式 `stepLength = speed * T_step_cycle`（`T_step_cycle=(2*supportKnots+4*stepKnots)*timeStep=1.6s`）换算，跑出 0.10/0.15/0.20 三版（`trajectory_walking_sideways_v0.1{0,5}.csv`/`_v0.20.csv`，均 1320×23，FDDP 收敛正常，实测均速比目标低 6~8%、和基线偏差比例一致）。**但用户肉眼在 `plot.py` 里发现 0.20 版步子过大、后腿间距逐渐收窄**，我用 FK 复核坐实（新增 §0.5-D，任务四）：这是「只加步长、步频完全不变」的必然后果——单步位移 0.08m→0.32m，摆动腿尖速度 0.23→0.91 m/s，左右脚水平间距逐周期收窄且速度越高越狠（0.05 基线 8 周期内 RL-RR 仅收窄 1cm 且 FL-FR 还变宽；0.20 版 RL-RR 收窄 2.8cm、FL-FR 收窄 5.8cm）。**用户要求**：0.05 基线保持不动；更高速度不能「只加步长」也不能「只加步频」（后者在固定 100Hz 帧率下会让迈步频率过高），而要**步长和步频联合上调**（类比 步幅×步频=速度）。当前三份变速 CSV 标记为「纯步长缩放草稿，先不要交付」，待 §0.5-D 的联合缩放方案做出来后重新导出替换。**本条只是记录问题，修正方案下一会话再做。**
- **v1.1（2026-07-02，同日细化）**：对 `trajectory_walking_sideways.csv` 把 §0.2-B 的「质心出界」按 body 系轴向拆成 dx(前后/pitch)/dy(左右/roll)（见新增 §0.2-B'）。**结论修正**：roll(dy) 在所有摆动相都很小且稳定（±6~7cm，前腿摆动和后腿摆动数值差不多），**侧向/侧翻方向本来就没问题**；出界完全是 pitch(dx) 的锅——摆后腿时 dx≈−0.22~−0.23m，四个周期几乎恒定（结构性，非漂移）。**所以本轮主线的准确表述是「摆后腿时机身前后倾斜」，不是「质心问题」/「侧翻」**；§0.5-A 的修正杠杆相应收窄为「只加 pitch 轴修正、只在摆后腿相生效、不碰 roll」。同步更新了记忆 `crocoddyl-to-robotcoop-pipeline.md`。
- **v1.0（2026-07-02，建档）**：
  - **复现学长「零库改」三足支撑** = stock `createWalkingProblem` 侧向(−Y)走 8 周期（`quadruped_walking_fwddyn.py`，supportKnots=10/stepKnots=35/stepLength=0.08/stepHeight=0.10；摆动顺序由脚名绑定选，学长=FL,RL,FR,RR）→ `trajectory_walking_sideways.csv`（1320×23），与学长 CSV base ≤0.023 m / 腿关节均差 0.027 rad = 同一步态。
  - **`_verify` 坐实核心问题**：stock 参考摆前腿质心在三角内(+0.12~0.15)、**摆后腿质心出界 14~22 cm**（0 帧在内），200/401 帧在内。因 Crocoddyl 接触力非单侧 + 回放无物理，看着不翻、实则静态不稳。**= 本轮主线。**
  - **我之前的 `createTripodProblem`（235/238 在内）按用户要求回退**（那版向前走 + 改库 + 机身诡异侧偏被否决）；库与 RUN_PIPELINE 回 HEAD；该工作存 scratchpad `my_tripod_work/`。**洞察：诡异侧偏源于"向前走"，本轮把质心修正加到"侧向走"上很可能两全。**
  - 列出三项任务（§0.5）：质心入三角（§0.5-A：driver 组装 / constraint=True 硬约束 / 放软 stateWeights 三条杠杆）、导出多策略（§0.5-B：顺序由脚名绑定、任意顺序需 driver 排相）、模块扩展头脑风暴（§0.5-C）。
