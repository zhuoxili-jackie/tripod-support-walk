# 三足支撑（单腿摆动）参考步态：在「前脚踩台面 + 后脚在地」下把质心做进支撑三角 —— 继续工作文档 v1.0

> 给下一个 Claude Code 会话：本文件是 **「用 Crocoddyl 为轮足四足 pcb_v2 生成『三足支撑（一次只摆一条腿）』参考步态 CSV，并解决『单腿摆动时质心投影不在另外三支撑脚三角形内』的问题」** 的实时交接。**请用简体中文回复。**
>
> 这是 **Crocoddyl 侧（轨迹生成）** 的工作，不是 Isaac Lab / RL 侧。产物是 23 列 100Hz CSV，喂给下游 `Robot_cooperation` 的 `csv_to_npz.py` → BeyondMimic 参考动作（见 `RUN_PIPELINE.md` §4 与记忆 `crocoddyl-to-robotcoop-pipeline`）。
>
> ### ★★★ 本轮范围（2026-07-03 用户最新口径 —— 取代下面 2026-07-02 的「pitch 主线」框架）★★★
>
> 用户 + 刚学长的新判断：三足支撑参考在**摆后腿**时机器人**极可能真的会摔**（不只是"pitch 偏一点"）。根因是**载荷分布**，本会话已用 `scratchpad/probe_loads.py`（读-only FK + 三点力平衡）坐实：
> - CoM 在 **x=+0.060**（很靠后、接近后脚），前脚对在 x=+0.529、后脚对在 x=−0.158 → **前腿只承担 31.8% 体重（每条前脚 ~16% / 31.7 N），后腿承担 68.2%（每条后脚 ~34% / 68 N）**。**= 刚学长"前腿只有约 30% 重力"的判断被数值坐实。**
> - 所以**抬一条后腿 = 移走约占 1/3 总体重的主支撑腿**。实测对当前 `sc_v0.05` 逐摆动相解唯一的三点竖直力平衡：**摆前腿（FL/FR）三支撑力全为正**（静态能站）；**摆后腿（RL/RR）对角前脚出现负力**（RL 摆 → FR 需拉地 −35~−42 N；RR 摆 → FL 需拉地 −25~−28 N）。**负力 = 支撑脚要"把地往下拉"，物理不可能 → 真机 / 物理 sim 会绕对角支撑边朝被抬的后腿方向翻。** Crocoddyl 只因接触力可为负（软摩擦锥、非单侧）才在无物理回放里"看着不翻"。
> - **对旧"纯 pitch"表述的修正**：翻倒其实是**对角**（绕"对角前脚 ↔ 同侧后脚"那条支撑边），不是纯前后 pitch，"拉地"的是**对角前脚**。dx（前后）是主因但**有 dy 分量**，别再当纯 pitch、只修一个轴。
>
> **用户指定「三步走」，严格按此顺序（本会话只调研 + 写文档，下一会话实施）**：
> 1. **先写"会不会摔"判定脚本**（详见 §0.6-1）。用户明确：**质心在不在三角内不是唯一/最好判据**（还有加速度、动力学）。要更物理的判据：**单侧接触力可行性（LP）/ ZMP / 倾覆动力学**。
> 2. **改迈步顺序**（§0.6-2）：右移顺序从现在的 `FL→RL→FR→RR`（用户口径"右前-右后-左前-左后"）改成 **`FR→RL→RR→FL`**（右前-左后-右后-左前）。纯改脚名绑定、零库改。**⚠ 本会话已实测：只改顺序 ≠ 解决**（后腿摆动仍 0 帧在三角内、仍需拉地，203/414 vs 现状 200/410）——它是低成本第一试（可能缓解"后腿内收"观感），但**别指望它修好摔倒**，要准备走到第 3 步。
> 3. **主动调整机身**（§0.6-3）：抬后腿前 / 中主动把（等效）质心送进支撑三角，保证三脚支撑下不摔。**这才是根治。**
>
> ---
>
> ### ★ 本轮范围（2026-07-02 用户口径，同日已细化见下）—— 背景，已被上面 2026-07-03 口径重定向 ★
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

> ⚠ **2026-07-03 更新**：下面 §0.5-A/B/C 是 2026-07-02 的旧优先级（主线=修 pitch）。**已被 §0.6「三步走」取代**（新主线=先写摔倒判据 → 改顺序 → 主动调机身）。§0.5 的**技术细节仍然有用**（尤其 §0.5-A 的三条修质心杠杆 = §0.6-3 的落地方案、§0.5-B 的顺序机制），但**优先级 / 问题表述以 §0.6 为准**（"纯 pitch"已修正为"载荷 / 对角翻"）。

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

## 0.6 ★★★ 本轮「三步走」详细规格（2026-07-03）★★★

> **状态（v1.11，2026-07-03 九次会话，见 §六）**：✅ 第 1/2/3 步（同 v1.10）· ✅ **膝内八已修（用户选「B 彻底」路线）**：`quadruped_tripod_com.py` 改 3 处默认——`--hip-reg` 50→**300**（钉死 hip 除内八）+ 新增 `--hipvel-reg` **25**（只阻尼 hip 速度，**消掉钉 hip 引入的 ~18Hz 全身抖动**——用户肉眼发现的严重坑）+ 抬腿 stepheight 0.12/0.115→**0.155/0.165**（补回被阻尼修掉的抬腿）。**纯 driver、不碰库**。结果：内八 **14.4°→6.3°（<trotting 7.5°）**、**无抖动（比旧版/trotting 还平滑）**、trunk tilt <~5.6°、抬腿前15/后10cm 全保住、迭代 310→13-23；代价 = 后腿摆 worst 拉力 11→**17N**（RL 补偿，用户已接受）。8 版 CSV 已重生成。**待送审 commit**。
> **状态（v1.10，2026-07-03 八次会话，见 §六）**：✅ 第 1 步（`_verify_fall.py`）· ✅ 第 2 步（改顺序 FR→RL→RR→FL）· ✅ 第 3 步（**用户改方向**：`quadruped_tripod_com.py` 限身体倾斜 **5°** + 高抬腿（前 15/后 10cm）+ 接受后腿摆摔由 RL 补偿；**改库** `createModel` 加可选 `stateWeights`，其它步态不变；已 commit `088f8ed`）。~~遗留：膝内八~~（v1.11 已修）。

> 总纲：第 1 步先有个**可信的"会不会摔"判据**（不然改了顺序 / 调了机身也不知道有没有真变好）；第 2 步用最低成本的"改顺序"试一把（本会话已预判它不够，但便宜、先排除）；第 3 步主动把质心送进三角，才是根治。**验收一律用第 1 步的脚本 + `_verify_tripod_com.py` + `plot.py` 肉眼。**

### 0.6-1 ★ 第一步：写"会不会摔"判定脚本（建议 `examples/_verify_fall.py`，读-only、只吃 CSV）

**为什么 CoM-in-triangle 不够（用户的原话"还有加速度之类的"）**：CoM 投影在支撑多边形内只是**静态**稳定判据（假设加速度=0）。真机还有：① CoM 加速度 + 角动量变化率 → 该用 **ZMP/CoP**（动态稳定）；② 接触**单侧**（脚只能推不能拉）+ **摩擦锥**（切向 ≤ μ·法向）→ 该问"给定这条轨迹的加速度，存不存在一组物理合法（fz≥0、在摩擦锥内）的接触力能实现它"；③ 就算某帧质心瞬时出界，**惯性 / 时长短**可能还没转过"不归点"就被下一相拉回 → 要看**倾覆动力学**（转多少角、要多久到不归点）。本速度（0.05）近准静态，ZMP≈CoM；但**判据要为将来提速 / sim2sim 留出动态项**。

**建议分四层输出（从便宜到严格）**，逐摆动相 + 逐帧汇总：
1. **静态裕度（米）**：CoM(x,y) 到当前支撑多边形边界的带符号距离（内为正）。这是 `_verify_tripod_com.py` 的"barycentric 裕度"换算成**米**（更直观、不随三角大小失真，§0.5-E 提过 barycentric 会被三角缩放骗）。
2. **单侧接触力可行性（LP，最关键、最物理）**：变量 = 各支撑脚 3D 力 `f_i`；约束 `f_iz≥0`、金字塔摩擦锥 `|f_ix|,|f_iy| ≤ μ f_iz`（μ=0.7、4 facet，**与库里 `FrictionCone(Rsurf,0.7,4,False)` 一致**）、力平衡 `Σf_i = m(a_com − g)`、力矩平衡 `Σ(p_i−c)×f_i = dL/dt`。**LP 可行 = 这条轨迹物理上真能被"只会推的脚"实现；不可行 = 必摔 / 打滑。** 报告**最大所需拉力（N）**或**松弛量**。→ 准静态特例就是本会话 `probe_loads.py` 的"唯一三点竖直力平衡"（负力=拉地），已证前腿相全正、**后腿相对角前脚要拉 25~42 N**。`scipy.optimize.linprog` 环境已装（1.15.2），LP 很小。
3. **ZMP 裕度（米）**：由 ② 的净力矩解出地面 CoP=ZMP，报其到支撑多边形的带符号距离。准静态下≈静态裕度；提速后是真正的动态判据。
4. **倾覆动力学（可选，回答"惯性能不能拉回来"）**：某相 CoM/ZMP 出界时，把机身近似为绕最近支撑边转动，`角加速度=重力对该边力矩 / 绕该边转动惯量`，在该相时长内积分得**累计倾角** vs **到"不归点"（CoM 竖直投影越过支撑边）所需角**。累计 < 不归 → 可能可救；≥ → 真摔。给"离摔多近 / 多久摔"的量。

**动力学量怎么来（纯 CSV）**：从 CSV 重建 `q(t)`（root 位姿 + 12 关节；4 个 foot_joint 恒 0）；对流形做差分得 `v=pin.difference(model,q[t-1],q[t])/dt`、`a=(v[t+1]−v[t])/dt`（注意浮动基在切空间差分，别直接减四元数）；`m=Σ inertia.mass=20.323 kg`；`c=pin.centerOfMass`；角动量率用 `pin.computeCentroidalMomentum / computeCentroidalMomentumTimeVariation`（或对 `A(q)v` 差分）。接触检测：脚高在其名义接触高度附近（前脚 z≈0.90、后脚 z≈0.09）且竖直速度≈0 → 在支撑。

**`probe_loads.py` 核心（准静态 LP 特例，可直接搬进 `_verify_fall.py` 起步）**：
```python
# 唯一三点竖直力平衡：A[f1,f2,f3]=[mg,0,0]，任一 f_i<0 = 该脚要拉地 = 会摔
A = np.vstack([np.ones(3),
               [xy[i][0]-c[0] for i in range(3)],    # 对 CoM 的 x 力矩
               [xy[i][1]-c[1] for i in range(3)]])    # 对 CoM 的 y 力矩
f = np.linalg.solve(A, np.array([m*g, 0.0, 0.0]))
# f.min()/(m*g) == _verify 的 min barycentric 裕度；<0 → 出界、必翻
```
**输出格式建议**：逐摆动相一行（抬哪条腿 / 支撑三脚 / 静态裕度 m / ZMP 裕度 m / LP 可行? / 最大拉力 N / 各脚竖直载荷 N）+ 全轨迹 PASS/FALL 总判 + 最差相。**阈值建议**：LP 不可行（需拉地 / 超摩擦锥）= 硬摔；裕度 < ~2–3cm 但 LP 可行 = 危险；否则 OK。**顺带把"前 32% / 后 68% 载荷分布"和"每相各脚载荷"打出来**（直接回应刚学长的关切，也便于看抬后腿时载荷怎么转移）。

### 0.6-2 ★ 第二步：改迈步顺序（纯改脚名绑定、零库改）—— 附本会话已测结论

**怎么改（一行）**：`createWalkingProblem` 内部恒按 **rh→rf→lh→lf** 摆，摆动顺序**只由构造函数 `SimpleQuadrupedalGaitProblem(model, lfFoot, rfFoot, lhFoot, rhFoot)` 的脚名绑定决定**。想要任意顺序 `[A,B,C,D]`（真脚），就令 `rhFoot=A, rfFoot=B, lhFoot=C, lfFoot=D`。
- ⚠ **纠正旧文档说的"只有 4 种旋转"**：其实**24 种排列全可达**（createWalkingProblem 对脚名无前 / 后、左 / 右语义假设，comRef=四脚平均对称、各脚都沿同一 `direction` 迈同一步长）。
- **用户要的 `FR→RL→RR→FL`（右前-左后-右后-左前）** → 令 `rhFoot="FR_foot_link", rfFoot="RL_foot_link", lhFoot="RR_foot_link", lfFoot="FL_foot_link"`（右移 `direction=(0,−1)`、其余同 `quadruped_walking_fwddyn.py`）。
- **一个结构限制**：DS（双支撑 / 理论上的重心转移窗口）只在**每 2 次摆动之间**出现一次（loco3dModel = `DS + rhStep + rfStep + DS + lhStep + lfStep + DS`），**不是每次摆动前都有**；且 stock 的 DS 相**没有 comTask**（不主动挪质心）。想"每次抬腿前都先把质心移过去"，stock 给不了 → 得第 3 步在 driver 里自己排相。用户的 `FR→RL→RR→FL` 恰好把两条后腿 RL、RR 分到不同 pair（中间隔一个 DS）、且两个 pair 都是对角对（FR+RL、RR+FL），是合理直觉。

**★ 本会话已实测（`scratchpad/probe_reorder.py`，忠实复刻 driver、4 周期、右移）**：`FR→RL→RR→FL` 跑出 **203/414 帧在内 vs 现状 200/410**——**前腿摆动仍全在内（+0.10~0.14），后腿摆动仍 0 帧在内（最差 −0.08~−0.12、仍需对角前脚拉地）**。**根因**：任何顺序都不改 `comRef=四脚平均`（x≈0.06），而后腿三角形心在 x≈0.30 → 高权重（1e6）comTask 把实际 CoM 钉在 0.06，后腿三角必然够不到。**所以：改顺序是低成本的第一试（也许能改善"后腿内收"观感和过渡瞬态），但它 physically 不可能修好"抬后腿会翻"——请照此设预期，别在它上面反复调。**「后腿内收(往内收)」大概率是优化器为廉价减小软 CoM/摩擦锥代价而**收窄支撑基把三角形心拉向 CoM**（同一根源），改顺序同样不去根。

### 0.6-3 ★ 第三步：主动调整机身，把（等效）质心送进支撑三角（根治）

目标：让 `_verify_fall.py` 在**所有**摆动相都 PASS（LP 可行、无拉地、裕度>0）。核心思路 = **每个摆动相把 comTask 从"四脚平均"换成"当次三支撑脚三角内、朝 CoM 收缩 margin 的可达点"，并在抬腿前插入"预移相"主动把质心挪过去、抬腿后插"沉降相"**。落地路径（承接旧 §0.5-A，按推荐序）：
- **方案 1（首选，driver 组装、零库改）**：不用 `createWalkingProblem`，改在 driver 里用 stock 的 `createFootstepModels`（吃 `comPos0/feetPos0/footContacts/swingFootNames/direction`）+ `createModel`（吃 `comTask`）自己排"单腿相"，每相 comTask 指向该相三角内目标 + 抬腿前加 4 脚 DS 预移相。**现成可搬**：`scratchpad/my_tripod_work/quadruped.py.mymod` 的 `createTripodProblem` + `_closest_point_in_triangle`（三角内最近可达点 + 预移 / 沉降），把方向改 `(0,−1)`。
  - ⚠ **已知障碍（关键）**：`createModel` 里 `stateWeights` **写死**（base 姿态 500、关节 50，很硬），driver **改不动** → 质心可能**够不到**靠前的后腿目标（上次实测硬权重下 x 只到 0.10~0.16，够不到入界的 x≈0.19+）。真够不到就只能：方案 2（硬约束）或方案 3（小改库放软，可回退）。**先 driver-only 试，实测够不够。**
- **方案 2（最物理正确，可能要 SolverIntro）**：`constraint=True` 把摩擦锥从软代价变**硬不等式**（`ConstraintModelResidual`）→ 优化器**不能再拉地**，要抬后腿就**必须**把质心挪进三角，否则报不可行（不可行本身就是"此站姿抬后腿静态不可行"的硬结论）。本 fork 有 `crocoddyl.SolverIntro`（trot 脚本在用）；`WITH_ODYN=False` 故无 SolverOdynSQP。**这把"质心在三角内"和"物理可实现"绑死，强烈建议试。**
- **方案 3（改库，最后手段，留可回退）**：放软 `createModel` 的 stateWeights（base 500→30、关节 50→3）让机身能前倾 / 侧倾送质心进三角（上次 235/238 的关键之一）。用可选参数 `stateWeights=None` 默认保持 stock、只 tripod 传软值，别带坏 trot/walking。**用户偏好不改库，故后备。**
- **方案 4（降难度）**：只把后腿相质心送到"刚好入界(x≈0.20)"而非三角形心(x≈0.30)、或减小后腿抬腿高度 / 步长留裕度、或（需用户同意）降低台高让四脚平均本就落在各三角内。

**验收**：`_verify_fall.py` 全摆动相 PASS（尤其 LP 可行、无拉地）+ `_verify_tripod_com.py` 全相裕度>0 + `plot.py` 肉眼一次一条腿 / 不翻 / 无诡异侧偏 / 后腿不再明显内收。

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
| **摔倒判据脚本（✅ v1.7 已实现，见 §0.6-1 / §六 v1.7）** | `examples/_verify_fall.py <csv...>`（四层：静态裕度 m / 单侧接触力 LP 可行性 / ZMP / 倾覆动力学 + 载荷分布；默认准静态、`--dynamic` 加惯性项）。经 6 维度对抗审查 + 单测；对齐 probe_loads。|
| **改顺序 FR→RL→RR→FL（✅ v1.7 第二步，缓解不根治）** | `examples/quadruped_tripod_reorder.py [--speed --direction]` → `trajectory_tripod_reorder_{,left_}v*.csv`（8 版）|
| **交付驱动（✅ v1.11：tilt≤5° + 抬腿 + 无内八）** | `examples/quadruped_tripod_com.py [--speed --direction --hip-reg --margin ...]` → `trajectory_tripod_com_{,left_}tilt_v*.csv`（driver 组装、零库改；`--hip-reg 300` 默认钉死 hip 除内八→7°；用 v1.9 的可选 `stateWeights` 库参数）|
| **本轮调研探针（scratchpad，会过期）** | `probe_loads.py`（前 32%/后 68% 载荷 + 逐相三点力平衡，负力=拉地=会翻）、`probe_reorder.py`（`FR→RL→RR→FL` 只改顺序不解决的证据，203/414）|
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

- **v1.11（2026-07-03，九次会话，修掉膝内八；用户选「B 彻底」路线；待送审 commit）**：
  - **★ 坐实这是物理权衡，不是调参能同时满足的**：内八（hip 侧向内收）是**单腿摆步态的侧向平衡刚需**——摆一条腿时支撑腿 hip 主动内收把 CoM 拉向支撑侧。侧向 CoM 位移**只能靠 hip 内收或机身 roll 侧倾二选一**（feet planted 时 base 不能凭空侧移）。trotting 靠对角对称免除它（hip≈0），tripod 结构上学不来。**实测前沿（8-cycle，margin0.65，纯 driver 改 `--hip-reg`）**：hip_reg 50→14.4°/worstN10.2、150→9.1°/15.2、200→8.4°/17.3、**300→7.4°/18.9**。**加大 hip 正则确实压内八，但线性地更摔**（钉死 hip = 拿掉侧向平衡手段）。
  - **★ 试过但放弃的"免费午餐"**：① 只开 body-roll（降 roll 权重）想让优化器自发用 roll 代替 hip 内收 → **无效**（hip 14.4→13.5，内收是 comTask+planted 脚运动学逼出来的、非软权衡）；② 钉 hip + 开 roll 补偿 → hip 降到 7° 且 worstN 从 18.9 拉回 16.8，**但因机身 -63° 前倾使 world-roll 投影到 body roll+yaw，引入 ~5° world-yaw 净漂移**（参考动作忌讳），姿态反而更差 → **弃**；③ 钉 hip + 更 aggressive 前送 CoM（margin 0.9）→ worstN 16.8 但**前腿支撑弯 57→74°**（逼近 v1.8 的 83° 深蹲）→ **弃**；④ 只钉支撑腿 hip、放开摆动腿 hip → **反而更糟 22°**（摆动腿 hip 自由去吸收内收）。
  - **★★ 关键坑：只钉 hip（hip-reg 300）会让整个机身抖动（用户肉眼发现，严重）**！stiff hip-位置正则（effective 1e1·300²≈1e6，与 comTrack/footTrack 1e6 同量级）**欠阻尼**，FDDP 解出 ~18Hz 高频振荡：base 二阶差分 RMS 0.18→0.95mm（5×）、RR_hip 0.19°→2.9°（15×）、速度反转 10→37 次/100帧、solve 迭代 ~20→310（迭代畸高就是征兆）。**降 hip-reg 治不了**（hip150 已抖 2.27°）。
  - **★★ 抖动根治 = 只提高 hip 关节的速度正则 `--hipvel-reg`（stock 1→25）**：给欠阻尼的 stiff 弹簧加阻尼。**只阻尼 4 个 hip 速度**（不碰 thigh/calf 速度）→ 抖动消到**比 stock 和 trotting 还平滑**（base_z 二阶差分 0.171mm=旧版0.18、hip 0.025-0.056°<旧版0.05-0.19 也<trotting 0.07-0.23）、迭代 310→13-23（良态收敛，证明是根治非遮掩）；thigh/calf 速度保持自由 → **抬腿不被压没**（均匀提 jvel=15 会把抬腿从 14/9 压到 3/3cm 并假 PASS，是陷阱，弃）。hipvel 阻尼会略修掉抬腿（14/9→11/7），故**抬腿高度参数补偿** front-stepheight 0.12→**0.155**、hind 0.115→**0.165**（恢复实测 前15/后10cm）。
  - **★ 最终配方 = `quadruped_tripod_com.py` 改 3 处默认：`--hip-reg` 50→300、新增 `--hipvel-reg` 25、`--front/hind-stepheight` 0.12/0.115→0.155/0.165**（margin 仍 0.65、base-ori 仍 250；**纯 driver、不碰库**，trot/walking 零风险、不用 cp build_conda）。STATE_W 关节速度块由 `[1]*12` 改成 `[hipvel,1,1]×4`。**0.05 右移 8-cycle 验收**：内八 **14.4°→6.3°**（<trotting 7.5°，达标①）；**无抖动**（见上，比旧版/trotting 都平滑）；trunk tilt roll 2.8/pitch 5.6/yaw 1.2°（pitch 略高因 hipvel 阻尼把平衡转给 trunk，仍~5°、③OK）；抬腿前15/后10cm（③✓）；全收敛 iters 13-23。**代价**：前腿支撑弯 57→67°（pin hip 的姿态预算转移，非深蹲）；后腿摆更吃力 **worst 拉力 11.4→16.7N**（`_verify_fall` 仍 FALL 16、worst_m −0.045，**用户接受 RL 补偿**）。
  - **交付**：重生成 8 版 `trajectory_tripod_com_{,left_}tilt_v{0.05,0.10,0.15,0.20}.csv`（覆盖 v1.9）。高速版 hip 略升（0.20 约 11-12°，大 stride 使摆动侧移更大，同 v1.9 规律，仍远优于旧 23°）。**新增复用工具（scratchpad，会过期）**：`_hip_probe.py`（hip mean/|max|°）、`_pose_probe.py`（hip + world roll/pitch/yaw）、`_splay_split.py`（内收拆 swing/support + 支撑腿弯）、**`_tremble.py`（逐帧二阶差分 RMS + 速度反转 = 抖动量化，关键新工具）**。**验收**：hip 用 `_hip_probe`；**抖动用 `_tremble.py`（务必查，别再交付抖动版）**；稳定性 `_verify_fall`（对照 trotting hip 7.5°）；肉眼 `plot.py`。
  - **本次 commit**：`quadruped_tripod_com.py`（默认值 + STATE_W 速度块 + docstring）+ 8 版 CSV + 本文档。**若嫌 worstN 16.7 更摔可回调 `--hip-reg 150`（内八 9.1°、worstN 15.2）折中，但需一并保留 `--hipvel-reg`（低 hip-reg 也会抖，只是轻些）。** 曾试开 roll 代偿（worstN→16.8）但引入 ~5° world-yaw 净漂移，已弃。
- **v1.10（2026-07-03，八次会话末，记录「膝内八」待修 + 调研线索；用户将新开对话修）**：
  - **遗留问题（用户肉眼确认）**：tripod tilt 版（`trajectory_tripod_com_tilt_v*.csv`，commit `088f8ed`）+ 基线 walking 都有**像人一样的膝内八**（hip 侧向关节内收）。**对角步态 trotting 不内八**（用户指出，作对照 / 线索）。
  - **★ 调研坐实（hip 关节偏离 neutral=0，rad/度；hip 列 = CSV 第 8/11/14/17）**：
    - **trotting（对角，不内八）** `trajectory_trotting_acc_005.csv`：hip mean ≈0（±0.01~0.03 rad）、|max| 仅 **3~7°**（RL/RR 最大 ~0.13rad）。
    - **baseline walking（单腿摆，内八）**：hip mean 明显内收、|max| **6~19°**（RR 0.332rad=19°）。
    - **tripod tilt（单腿摆，内八）**：hip |max| **8~14°**（0.14~0.25rad）。
  - **根因线索**：内八 = **单腿摆步态**（walking / tripod，一次一条腿）的支撑腿 hip 主动内收（把重心拉向支撑侧做侧向平衡）；**对角步态**（trotting，对角两腿同摆）对角自平衡、hip 无需内收 → 不内八。这也解释为何**加大 hip 正则没用**（v1.9 实测 hip-reg 50→200 内八仅 14°→13°）——hip 内收是单腿摆的平衡刚需，纯 stateWeights 正则拉不回。
  - **下一会话修法方向（未做，仅记录）**：① 给 hip 加**关节位置目标 / 硬约束**（`ResidualModelState` 只作用 hip、或 hip 侧向 footTrack），显式把 hip 钉在 ≈0，而非靠正则；② 剖析 trotting 的对角结构为何让 hip 天然不内收，看能否移植到 tripod（成对 / 对角摆动、或对角 comRef）；③ 权衡：钉死 hip 会削弱单腿摆的侧向平衡（可能更易摔），需配合送 CoM 或接受（RL 补偿）。**工具**：driver 已有 `--hip-reg`（纯正则、实测改善甚微）；`examples/quadruped_gaits_fwddyn.py` = trotting 驱动（不内八的对照）；`_verify_fall.py` 判稳、FK 探针量化 hip 偏离。
- **v1.9（2026-07-03，八次会话续，用户据 v1.8 物理结论改方向：限倾斜 5°+接受摔+改库+高抬腿；改库+driver+CSV 待送审 commit）**：
  - **用户新口径**：v1.8 证明「送 CoM 进后腿三角必然大变形（前倾 18°+腿弯 83°+内八 34/59°）」后，用户决定**放弃追求每帧不摔**（后续 RL 训练给奖励补偿），改为「**身体基本平（倾斜 ≤~5°）+ 腿自然（不内八不深蹲）+ 高抬腿**，后腿摆送不满就让它摔一点」。
  - **★ 改库（用户明确授权，可回退）**：给 `createModel`/`createFootstepModels` 加**可选 `stateWeights=None` 参数**（None→stock，**其它步态 trot/walking 完全不受影响**，已验证重生成基线 `max|diff|=0`）。**注意**：用户原说改 1002-1007，但那是 `createPseudoImpulseModel`（跳跃用、walking 不用）；walking 真正用的是 **810-816**（`createModel` 的 stateWeights），已改此处。改后 **cp 到 build_conda**（§0.1 坑：改源不生效；原 `.pyc` 备份 `quadruped.pyc.orig_backup`）。
  - **driver 新版 `quadruped_tripod_com.py`**：传 `stateWeights` = [0]*3 base-pos + **[250]*3 base-ori（倾斜~5°）** + [hip50,thigh50,calf50]×4（stock，不内八不腿弯）+ [10]*6 + [1]*joints-vel；margin0.65 仍送 CoM（倾斜 5° 下送不满）；**前后腿不同抬腿**（`--front-stepheight 0.12`→前腿峰值 ~15cm、`--hind-stepheight 0.115`→后腿 ~10cm）；支持 `--speed`（stride×cadence 联合缩放，同基线驱动）+ `--direction left`（L↔R 镜像 order 的精确左移）。
  - **交付 8 版 `trajectory_tripod_com_{,left_}tilt_v{0.05,0.10,0.15,0.20}.csv`**（右/左 × 4 速度，2272/1632/1344/1184 行）。**0.05 基准验收**：身体倾斜 **5.2°**、内八 **14°**（基线 19°）、腿弯 **13°**（基线 14°）、抬腿 **前 14.9 后 10.0cm**、侧向走 y→−0.47m；`_verify_fall` **FALL 16**（接受，RL 补偿）但**后腿摆失稳比基线减半**（worst −0.032 vs 基线 −0.065、拉力 11.5N vs 30.5N，即倾斜 5° 送了约一半 CoM）。**高速版姿态随 stride 增大而略增**（0.20：倾斜 6.9°/内八 23°/前 18.8 后 13.2cm），左右镜像各指标一致。
  - **标定**：base-ori 60→倾斜 8.3°、120→7.1°、**250→5.0°**（关节保持 stock 50 时）。
  - **本次 commit**：库 `quadruped.py`（源；build_conda 的 cp 是 gitignore 编译产物、不入 git，但已 cp 生效）+ `quadruped_tripod_com.py` + 8 版 `trajectory_tripod_com_*tilt_v*.csv` + 本文档。v1.8 大倾斜对照 `trajectory_tripod_com_root.csv` 已删。
- **v1.8（2026-07-03，八次会话续，第三步根治达成 + 揭示姿态代价物理本质；driver/CSV/本条未 commit，待用户审取舍）**：
  - **★ 第三步根治摔倒达成（`examples/quadruped_tripod_com.py`，全程零库改）**：driver 层用 stock `createFootstepModels`/`createModel` 自排单腿相（预移相 → 摆动 → 沉降相），每摆动相 comTask 指向「三支撑三角内朝 CoM 收缩 marginFrac 的最近可达点」（搬 mymod 的 `_closest_point_in_triangle`）；并用 driver 层 `removeCost`/`addCost` **重建 stateReg**（= 方案3「放软 stateWeights」的等价实现，但**不碰库源码**）。**结果：`_verify_fall` FALL 16→0**，所有后腿摆 LP 可行、**无拉地**、CoM 全相在三角内（8 周期链式收敛稳定 stop<1.5e-8）。方案历程：方案1（stock 硬权重 driver 组装）后腿摆 −0.035 仍 FALL → 方案2（constraint=True 硬锥 + SolverIntro）不收敛（stop~4e2）→ 方案3 driver 层重建 stateReg 放软成功。
  - **★「后腿内八」真相（回应用户 2026-07-03 澄清：内八 = 膝/hip 关节内扣，不是轮距）**：① 起初把整个关节正则 `joint_reg` 50→2 放软送 CoM，**hip 侧向关节一起松了 → 内八从基线 19° 恶化到 59°**（FK 实测各腿 hip 偏离 neutral）。② 分离关节正则（driver 参数 `--hip-reg`/`--joint-reg` 分别控制 hip 与 thigh/calf）：**保持 hip 硬（20）、只放软 thigh/calf → 内八压回 34°**（接近基线）。③ **但姿态代价总量守恒**：抑制内八后，送 CoM 的代价转移到 **thigh/calf 弯曲加剧（72°→83°）**。④ **关键耦合**：hip 内收本身就是「把 CoM 侧向送进后腿三角（侧移 6.7cm 到支撑侧）」的手段之一，完全锁死 hip（硬 50）会侧移不足、后腿摆重新 FALL——侧移必须靠 hip 内收 / roll 侧倾 / base_y 平移三者之一。
  - **★ 物理本质结论（重要，回答刚学长 + 定边界）**：把 CoM 从后脚附近（x≈0.06）前移 13cm + 侧移 6.7cm 进前脚台附近的后腿三角（形心 x≈0.30），**必然需要极大全身变形**（机身前倾 ~18° + 深蹲/thigh-calf 弯 ~83° + 内八 34° 或等量 roll 侧倾）。这是 CoM 位移的固有函数、姿态预算守恒、**不是调参能消除的**——**当前「前脚踩 0.8m 高台、后脚在地」站姿下，三足抬后腿静态不摔是一个勉强的极限动作**（前扑+下蹲+腿调整），而非自然步态。根因是脚的 **x 布局**（前脚 x=0.53 很靠前、后脚 x=−0.16 → 后腿三角形心天然靠前 x≈0.30），降台高（z）帮助有限（不改脚 x 布局）。
  - **交付候选（未 commit）**：`trajectory_tripod_com_root.csv`（margin0.72 hip20 roll200 pitch12 joint2，2272×23，FALL=0、内八 34°、roll/yaw ±0.04、pitch 前倾 18°、thigh/calf 弯 83°）。driver 支持 `--margin/--hip-reg/--joint-reg/--base-pitch-reg/--base-roll-reg/--base-yaw-reg/--constraint/--maxiter/--cycles/--order/--direction`。**注意**：`_verify_tripod_com` 在此新步态（大漂移 + 预移/沉降相）下接触检测失效（分段错乱），**以鲁棒检测的 `_verify_fall` 为准**。
  - **待用户审（第三步暂未 commit）**：交付版取舍（内八 vs 腿弯的姿态分配是价值判断，可导出多版供 `plot.py` 肉眼选）+ 是否接受「大变形是固有代价」结论 + 是否需改站姿/降需求。`quadruped_tripod_com.py` + 候选 CSV + 本条文档更新暂未 commit，等送审。
- **v1.7（2026-07-03，八次会话，实施「三步走」第 1、2 步完成 + 第 3 步开工）**：
  - **★ 第一步完成：`examples/_verify_fall.py`（读-only、吃 CSV、四层判据）**。① 静态裕度(m)：CoM 到支撑多边形带符号距离（内正外负，米制，不被三角大小失真）；② **单侧接触力可行性 LP（主判据、最物理）**：各支撑脚 3D 力 + 力/力矩平衡 + 单侧(f_z≥0) + 金字塔摩擦锥(μ=0.7、4facet，与库 `FrictionCone` 一致)，`scipy.linprog` 求最小锥违反 z（z≤0 可行/裕度 N、z>0 不可行/最大所需拉力 N），**不依赖共面假设**（脚不共面，比 ZMP 严格）；③ ZMP 裕度(m)：Kajita 公式（准静态≈静态裕度）；④ 倾覆动力学：出界时绕最近支撑边翻转角加速度 + 相内累计倾角 vs 不归角。**默认准静态（a_com=dL=0，0.05 近乎精确、无差分噪声）**，`--dynamic` 用 Savitzky-Golay 平滑求 a_com/dL/dt 为提速留接口。**验收对齐**：基线 sc_v0.05 判 **FALL**（8×2=16 后腿摆相 LP 不可行），后腿摆对角前脚拉地 RL摆→FR −36~−43N、RR摆→FL −24~−29N（**精确复现 probe_loads 的 −35~−42/−25~−28**）、前腿摆/DS4 OK；**学长参考 `trajectory_single_leg_acc_f005.csv` 与基线结果几乎完全相同**（FALL 16、worst −0.065）= 坐实同一步态同一问题。
  - **★ 第一步经 6 维度对抗审查（12-agent workflow）+ 修 3 真 bug**：① `vertical_loads` 力矩-y 行 RHS 符号翻（`-tau_y`→`+tau_y`，仅 `--dynamic` 显示列受影响、准静态判据不受影响，已修+单测）；② `signed_margin` 外部点顶点外侧欧氏距离低估（内部 min-over-lines、外部改真实到边界欧氏距离）；③ **`detect_support` 漂移鲁棒性（关键）**：软接触让整条轨迹脚高整体漂移，v0.20 高速下达 **0.12~0.155m**（远超预想）；起初用第0帧固定基线被漂移骗成"全脚抬起"→改**局部窗口 min 基线**（跟随任意大漂移）；曾误加 `STANCE_GUARD` 全局 min 回退，实测在大漂移下反把漂移的支撑脚误判摆动（reorder_v0.20 假报 133 帧 0 支撑）→ **移除 guard、纯局部窗口 min**（实测各速度只剩 3/4 支撑，正确）；另 `<3 支撑长段判 FALL`、短过渡(≤5帧) skip。LP/几何/ZMP-倾覆维度审查判无实质 bug；`SWING_THRESH`4cm 为设计启发式（保留）。单元测试全过（tau_y 力矩重建、signed_margin 内外、前/后腿摆 LP 反对称自洽 ±25.6N）。
  - **★ 第二步完成：改迈步顺序 FR→RL→RR→FL（`examples/quadruped_tripod_reorder.py`，纯改脚名绑定 rh=FR,rf=RL,lh=RR,lf=FL、零库改）**。复刻主 driver 全部逻辑（speed×cadence 联合缩放、cycle0 对称缓起步、左移=L↔R 镜像绑定→FL→RR→RL→FR），导出 **8 版** `trajectory_tripod_reorder_{,left_}v{0.05,0.10,0.15,0.20}.csv`。**验收（`_verify_fall`）：全部仍 FALL**——后腿摆仍 LP 不可行、对角前脚仍拉地，但**最大拉力从基线 30.5N 降到 10.7~15.7N（约减半）**、`_verify_tripod_com` **203/414**（与旧 probe_reorder 完全一致，后腿摆 0 帧在内）。**改顺序缓解失稳严重程度但不根治，完全符合 §0.6-2 预判**（根因 comRef=四脚平均 x≈0.06 与顺序无关）。左右镜像 ≤0.38mm。
  - **★ 姿态问题「后腿内收」仍存在（用户 2026-07-03 指出，记录待第三步解决）**：改顺序缓解了后腿摆的失稳程度，但"后腿内收（往内收）"的姿态/观感问题**未消除**。根因同 §0.6-2：优化器为廉价减小软 CoM/摩擦锥代价而**收窄支撑基、把三角形心拉向靠后的 CoM**（与"抬后腿要拉地"同源）。纯改顺序去不掉，**必须靠第三步主动送质心进三角来根治**（送 CoM 前进、而非收窄支撑基）。
  - **★ 第三步开工（进行中，未纳入本次 commit）**：`examples/quadruped_tripod_com.py`——driver 层用 stock `createFootstepModels`/`createModel` 自排单腿相（零库改），每摆动相 comTask 指向「三支撑三角内朝 CoM 收缩 margin 的最近可达点」（搬 mymod 的 `_closest_point_in_triangle`）+ 抬腿前 4 脚预移相 + 抬腿后沉降相；`--constraint` 走硬摩擦锥(SolverIntro)。待验证硬 stateWeights 下 CoM 能否够到后腿目标（§0.6-3 已知障碍），不够则走方案2(硬约束)/3(放软权重)。
  - **本次 commit**：`_verify_fall.py` + `quadruped_tripod_reorder.py` + 8 版 reorder CSV + 本文档（含 v1.6 的 §0.6 一并提交）。`quadruped_tripod_com.py` 为第三步 WIP、暂不 commit。
- **v1.6（2026-07-03，七次会话，纯调研 + 重定向文档，未改任何代码 / CSV）**：按用户 2026-07-03 最新口径 + 刚学长意见，把主线从旧的「摆后腿 pitch 偏」**重定向为「摆后腿会真摔（载荷 / 单侧接触力问题）」**，并立「三步走：先写摔倒判据 → 改顺序 → 主动调机身」（新增 §0.6，取代旧 §0.5-A/B 的优先级）。**关键实测（scratchpad 探针，读-only）**：① `probe_loads.py` 坐实 **前腿承 31.8% / 后腿 68.2%**（CoM x=+0.060、前脚 x=+0.529、后脚 x=−0.158；M=20.323kg），且当前 `sc_v0.05` **摆后腿相对角前脚需拉地 25~42N**（RL 摆→FR −35~−42N、RR 摆→FL −25~−28N）= 物理会翻、Crocoddyl 靠非单侧接触力"看着不翻"；翻倒是**对角**（绕对角前脚↔同侧后脚支撑边），非纯 pitch。② `probe_reorder.py` 实测用户要的 **`FR→RL→RR→FL` 只改顺序不解决**（203/414 vs 现状 200/410，后腿摆动仍 0 帧在内、仍需拉地）——因 comRef=四脚平均与顺序无关。③ 纠正旧文档"迈步顺序只有 4 种旋转" → 实为 **24 种排列全可达**（纯改脚名绑定、零库改）。④ 设计了 `_verify_fall.py` 四层判据（静态裕度 m / 单侧接触力 LP 可行性 / ZMP / 倾覆动力学），scipy.linprog 已备（1.15.2）。**探针存 scratchpad（会随会话过期）；核心逻辑已内嵌 §0.6-1。下一会话按 §0.6 的 1→2→3 实施。**
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
