# GameController 协议更新说明（v12 → v18）

## 一、新旧数据结构对比

### 1. RoboCupGameControlData（裁判盒 → 机器人）

#### 版本信息

| 项目                            | 旧版（v12） | 新版（v18） |
| ------------------------------- | ----------- | ----------- |
| `GAMECONTROLLER_STRUCT_VERSION` | 12          | 18          |
| `MAX_NUM_PLAYERS`               | 11          | 20          |

#### 结构体字段对比

| 旧版字段                | 类型       | 新版字段           | 类型       | 说明                            |
| ----------------------- | ---------- | ------------------ | ---------- | ------------------------------- |
| `header[4]`             | `char`     | `header[4]`        | `char`     | 不变                            |
| `version`               | `uint16_t` | `version`          | `uint8_t`  | 缩小为8位                       |
| `packetNumber`          | `uint8_t`  | `packetNumber`     | `uint8_t`  | 不变                            |
| `playersPerTeam`        | `uint8_t`  | `playersPerTeam`   | `uint8_t`  | 不变                            |
| `gameType`              | `uint8_t`  | `competitionPhase` | `uint8_t`  | 重命名：比赛阶段                |
| _(无)_                  | —          | `competitionType`  | `uint8_t`  | 新增：比赛类型                  |
| `secondaryState`        | `uint8_t`  | `gamePhase`        | `uint8_t`  | 重命名：比赛阶段（加时/点球等） |
| `secondaryStateInfo[4]` | `char`     | _(删除)_           | —          | 已删除                          |
| `state`                 | `uint8_t`  | `state`            | `uint8_t`  | 不变                            |
| _(无)_                  | —          | `setPlay`          | `uint8_t`  | 新增：定位球类型                |
| `firstHalf`             | `uint8_t`  | `firstHalf`        | `uint8_t`  | 不变                            |
| `kickOffTeam`           | `uint8_t`  | `kickingTeam`      | `uint8_t`  | 重命名：踢球队伍                |
| `dropInTeam`            | `uint8_t`  | _(删除)_           | —          | 已删除                          |
| `dropInTime`            | `uint16_t` | _(删除)_           | —          | 已删除                          |
| `secsRemaining`         | `uint16_t` | `secsRemaining`    | `int16_t`  | 改为有符号整数                  |
| `secondaryTime`         | `uint16_t` | `secondaryTime`    | `int16_t`  | 改为有符号整数                  |
| `teams[2]`              | `TeamInfo` | `teams[2]`         | `TeamInfo` | 结构体内容变化，见下            |

#### 新版常量定义变化

**比赛阶段（gamePhase，原 secondaryState）：**
| 旧版常量 | 新版常量 |
|---------|---------|
| `STATE2_NORMAL` | `GAME_PHASE_NORMAL` |
| `STATE2_PENALTYSHOOT` | `GAME_PHASE_PENALTYSHOOT` |
| `STATE2_OVERTIME` | `GAME_PHASE_OVERTIME` |
| `STATE2_TIMEOUT` | `GAME_PHASE_TIMEOUT` |

**新增定位球类型（setPlay，原 STATE2\_\*KICK 等）：**
| 旧版常量 | 新版常量 |
|---------|---------|
| `STATE2_DIRECT_FREEKICK` | `SET_PLAY_PUSHING_FREE_KICK` |
| `STATE2_INDIRECT_FREEKICK` | _(无对应，已合并)_ |
| `STATE2_PENALTYKICK` | `SET_PLAY_PENALTY_KICK` |
| `STATE2_CORNER_KICK` | `SET_PLAY_CORNER_KICK` |
| `STATE2_GOAL_KICK` | `SET_PLAY_GOAL_KICK` |
| `STATE2_THROW_IN` | `SET_PLAY_KICK_IN` |

---

### 2. TeamInfo

| 旧版字段                               | 类型        | 新版字段            | 类型        | 说明                 |
| -------------------------------------- | ----------- | ------------------- | ----------- | -------------------- |
| `teamNumber`                           | `uint8_t`   | `teamNumber`        | `uint8_t`   | 不变                 |
| `teamColour`                           | `uint8_t`   | `fieldPlayerColour` | `uint8_t`   | 重命名：场上球员颜色 |
| _(无)_                                 | —           | `goalkeeperColour`  | `uint8_t`   | 新增：守门员颜色     |
| _(无)_                                 | —           | `goalkeeper`        | `uint8_t`   | 新增：守门员号码     |
| `score`                                | `uint8_t`   | `score`             | `uint8_t`   | 不变                 |
| `penaltyShot`                          | `uint8_t`   | `penaltyShot`       | `uint8_t`   | 不变                 |
| `singleShots`                          | `uint16_t`  | `singleShots`       | `uint16_t`  | 不变                 |
| `coachSequence`                        | `uint8_t`   | _(删除)_            | —           | 已删除               |
| `coachMessage[SPL_COACH_MESSAGE_SIZE]` | `uint8_t[]` | _(删除)_            | —           | 已删除               |
| `coach`                                | `RobotInfo` | _(删除)_            | —           | 已删除               |
| _(无)_                                 | —           | `messageBudget`     | `uint16_t`  | 新增：消息预算       |
| `players[11]`                          | `RobotInfo` | `players[20]`       | `RobotInfo` | 上限从11增至20       |

---

### 3. RobotInfo

| 旧版字段              | 类型      | 新版字段              | 类型      | 说明                               |
| --------------------- | --------- | --------------------- | --------- | ---------------------------------- |
| `penalty`             | `uint8_t` | `penalty`             | `uint8_t` | 不变                               |
| `secsTillUnpenalised` | `uint8_t` | `secsTillUnpenalised` | `uint8_t` | 不变                               |
| `warningCardCount`    | `uint8_t` | _(删除)_              | —         | 已删除                             |
| `yellowCardCount`     | `uint8_t` | _(删除)_              | —         | 已删除                             |
| `redCardCount`        | `uint8_t` | _(删除)_              | —         | 已删除                             |
| `isGoalie`            | `uint8_t` | _(删除)_              | —         | 已删除（移至 TeamInfo.goalkeeper） |

---

### 4. RoboCupGameControlReturnData（机器人 → 裁判盒）

#### 版本信息

| 项目                                   | 旧版 | 新版 |
| -------------------------------------- | ---- | ---- |
| `GAMECONTROLLER_RETURN_STRUCT_VERSION` | 2    | 4    |

#### 结构体字段对比

| 旧版字段    | 类型      | 新版字段    | 类型      | 说明                                      |
| ----------- | --------- | ----------- | --------- | ----------------------------------------- |
| `header[4]` | `char`    | `header[4]` | `char`    | 不变                                      |
| `version`   | `uint8_t` | `version`   | `uint8_t` | 不变                                      |
| `team`      | `uint8_t` | `teamNum`   | `uint8_t` | 重命名，顺序调换                          |
| `player`    | `uint8_t` | `playerNum` | `uint8_t` | 重命名，顺序调换                          |
| `message`   | `uint8_t` | _(删除)_    | —         | 已删除（不再发送 ALIVE 等消息类型）       |
| _(无)_      | —         | `fallen`    | `uint8_t` | 新增：是否摔倒（0=正常，1=摔倒）          |
| _(无)_      | —         | `pose[3]`   | `float`   | 新增：机器人位姿 (x, y, theta)，毫米/弧度 |
| _(无)_      | —         | `ballAge`   | `float`   | 新增：上次看到球距今秒数，-1为未见球      |
| _(无)_      | —         | `ball[2]`   | `float`   | 新增：球相对机器人的位置 (x, y)，毫米     |

> **注意**：新版 `playerNum` 在 `teamNum` 之前（字段顺序与旧版相反）。

---

## 二、代码修改内容

### 修改文件：`dancer-network/src/gamecontroller.cpp`

#### 1. 构造函数：返回数据初始化（第35-43行）

**修改原因**：`RoboCupGameControlReturnData` 字段名变化，并新增位姿/球信息字段。

```cpp
// 旧版
ret_.team = (uint8_t)teamNumber_;
ret_.player = (uint8_t)playerNumber_;
ret_.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

// 新版
ret_.teamNum = (uint8_t)teamNumber_;
ret_.playerNum = (uint8_t)playerNumber_;
ret_.fallen = 0;       // 0 means robot can play
ret_.ballAge = -1.f;   // -1 means haven't seen the ball
ret_.pose[0] = 0.f;
ret_.pose[1] = 0.f;
ret_.pose[2] = 0.f;
ret_.ball[0] = 0.f;
ret_.ball[1] = 0.f;
```

#### 2. tick()：队伍查找逻辑（第71-78行）

**修改原因**：新版协议不保证 `teams[0]` 一定是青色队，改为按 `teamNumber` 匹配。

```cpp
// 旧版
if (data_.teams[TEAM_CYAN].teamNumber == teamNumber_) {
    ourTeam = &(data_.teams[TEAM_CYAN]);
    enemyTeam = &(data_.teams[TEAM_MAGENTA]);
} else { ... }

// 新版
if (data_.teams[0].teamNumber == teamNumber_) {
    ourTeam = &(data_.teams[0]);
    enemyTeam = &(data_.teams[1]);
} else { ... }
```

#### 3. tick()：定位球/次级状态解析（第86-125行）

**修改原因**：`secondaryState` + `secondaryStateInfo` 拆分为 `gamePhase` + `setPlay`，定位球类型枚举改变，`indirect free kick` 被合并入 `pushing free kick`。

```cpp
// 旧版：从 secondaryState 和 secondaryStateInfo 读取
int state2 = data_.secondaryState;
int state2_team = (int)data_.secondaryStateInfo[0];
// ...判断 STATE2_DIRECT_FREEKICK / STATE2_INDIRECT_FREEKICK 等

// 新版：从 gamePhase 和 setPlay 读取
int gamePhase = data_.gamePhase;
int setPlay = data_.setPlay;
int kickingTeam = (int)data_.kickingTeam;
// ...判断 SET_PLAY_PUSHING_FREE_KICK / SET_PLAY_PENALTY_KICK 等
```

#### 4. tick()：kickoff 判断（第144行）

**修改原因**：`kickOffTeam` 重命名为 `kickingTeam`。

```cpp
// 旧版
bool kickoff = (data_.kickOffTeam == teamNumber_);

// 新版
bool kickoff = (data_.kickingTeam == teamNumber_);
```

#### 5. tick()：secondaryState 发布（第151行）

**修改原因**：`secondaryState` 重命名为 `gamePhase`，将 `gamePhase` 映射到 GCInfo 的 `secondaryState` 字段。

```cpp
// 旧版
info_.secondaryState = data_.secondaryState;

// 新版
info_.secondaryState = data_.gamePhase;
```

#### 6. tick()：state2Ready / state2Freeze（第175-176行）

**修改原因**：旧版通过 `secondaryStateInfo[1]` 区分三个阶段（0=placing, 1=end placing, 2=execute），新版 setPlay 只要非 NONE 即代表定位球激活。

```cpp
// 旧版
info_.state2Ready = state2_ready;
info_.state2Freeze = state2_freeze;

// 新版
info_.state2Ready = setPlayReady;
info_.state2Freeze = setPlayFreeze;
```

#### 7. ParseData()：删除 RawSwapTeams 调用

**修改原因**：新版不再以颜色区分队伍位置，`teams[0]`/`teams[1]` 不保证顺序，改为按 teamNumber 查找，无需交换。

```cpp
// 旧版
if (gameData.teams[TEAM_CYAN].teamColour != TEAM_CYAN)
    RawSwapTeams(gameData);

// 新版（已删除）
```

#### 8. IsThisGame()：删除 TEAM_CYAN/TEAM_MAGENTA 引用（第229-232行）

**修改原因**：新版头文件中已移除 `TEAM_CYAN` / `TEAM_MAGENTA` 常量定义。

```cpp
// 旧版
return !(gameData.teams[TEAM_CYAN].teamNumber != teamNumber_
      && gameData.teams[TEAM_MAGENTA].teamNumber != teamNumber_);

// 新版
return !(gameData.teams[0].teamNumber != teamNumber_
      && gameData.teams[1].teamNumber != teamNumber_);
```

#### 9. IsValidData()：版本号日志格式（第244行）

**修改原因**：`version` 从 `uint16_t` 改为 `uint8_t`，日志格式符号改为 `%u`。

```cpp
// 旧版
ROS_WARN("Version invalid, recv: %d, need: %d", gameData.version, ...);

// 新版
ROS_WARN("Version invalid, recv: %u, need: %d", gameData.version, ...);
```

---

### 修改文件：`dancer-network/include/dnetwork/gamecontroller.hpp`

#### 删除 RawSwapTeams 方法声明

**修改原因**：`RawSwapTeams` 函数已不再需要，同步删除声明。

---

### 修改文件：`dancer-msgs/msg/GCInfo.msg`

#### 补充 STATE_STANDBY 常量

**修改原因**：新版协议新增 `STATE_STANDBY = 5`，旧版 msg 中未定义，下游模块收到 state=5 时无法识别。

```
# 新增
uint8 STATE_STANDBY            =  5
```

---

## 三、与 PDF 对比发现的问题及修复（第二轮）

### 1. secsRemaining / secondaryTime 负值溢出

**问题**：新版字段类型从 `uint16_t` 改为 `int16_t`，可能出现负值（如加时赛倒计时）。旧判断条件 `< 10000` 对负数也成立，会将负值赋给 `GCInfo.msg` 的 `uint16` 字段导致溢出为极大正数。

```cpp
// 旧版（有溢出风险）
info_.secsRemaining = data_.secsRemaining < 10000 ? data_.secsRemaining : 0;

// 修复后
info_.secsRemaining = (data_.secsRemaining >= 0 && data_.secsRemaining < 10000) ? (uint16_t)data_.secsRemaining : 0;
info_.secondaryTime = (data_.secondaryTime >= 0 && data_.secondaryTime < 10000) ? (uint16_t)data_.secondaryTime : 0;
```

### 2. setPlayFreeze 始终为 false

**问题**：旧版通过 `secondaryStateInfo[1]` 区分定位球三阶段，新版该字段已删除。原修复代码将 `setPlayFreeze` 硬编码为 false，导致定位球期间机器人不会静止。

**修复**：利用 `state` 字段判断阶段——`STATE_SET` 时为 freeze（裁判摆球，机器人静止），`STATE_PLAYING` 时为 ready（可以行动）：

```cpp
// 修复后
if (setPlay != SET_PLAY_NONE) {
    setPlayFreeze = (data_.state == STATE_SET);
    setPlayReady  = (data_.state == STATE_PLAYING);
}
```

### 3. info_.gameType 未赋值

**问题**：`GCInfo.msg` 中有 `uint8 gameType` 字段，原代码和初版修改都未对其赋值，下游模块读取到的始终是默认值 0。新版对应字段为 `competitionPhase`。

```cpp
// 修复后（赋值 competitionPhase）
info_.gameType = data_.competitionPhase;
```

### 4. STATE_STANDBY 未定义

**问题**：新版协议新增 `STATE_STANDBY = 5`，`GCInfo.msg` 中只有 STATE 0-4，下游模块无法识别该状态。

**修复**：在 `GCInfo.msg` 的 STATE 常量区追加 `STATE_STANDBY = 5`。

---

## 四、遗留待处理项

- `ret_.fallen`、`ret_.pose`、`ret_.ball` 目前为默认值（0 / -1），需后续从其他模块获取真实数据填充。
- 旧版有 `ourIndirectFreeKick` / `enemyIndirectFreeKick`，新版协议中 indirect free kick 已并入 `SET_PLAY_PUSHING_FREE_KICK`，这两个字段在新版中始终为 false。
- `GCInfo.msg` 中的 `STATE2_*` 常量语义已变为 `GAME_PHASE_*`，如需完整对齐建议后续重命名。
