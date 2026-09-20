# RocketSim WebAssembly C-ABI Bridge (Deobfuscated Source)
## 1. 物理微内核导出符号规范 (C-ABI)

为了保证二次开发与物理引擎调试的确定性，所有混淆别名（如 `__v0`, `__v1`, `__v2` 等）均已剔除，统一采用语义清晰的标准物理接口：

| 函数导出符号 | WASM 内部类型 | C++ 函数签名 / 符号 | 物理功能描述 |
| :--- | :---: | :--- | :--- |
| `memory` | `Memory` | `memory` | WebAssembly 线性共享堆内存 |
| `__wasm_call_ctors` | `() -> ()` | `__wasm_call_ctors()` | 全局静态构造函数链初始化 |
| `physics_init` | `(i32, i32, i32) -> i32` | `int physics_init(uint8_t* meshData, int32_t* meshSizes, int chunkCount)` | 初始化 RocketSim 核心并载入 Soccar 碰撞体网格分块 |
| `physics_createArena` | `() -> i32` | `int physics_createArena()` | 实例化 Soccar 竞技场（注入进球与触球回调，重置事件环形队列） |
| `physics_addCar` | `(i32, i32) -> i32` | `int physics_addCar(int team, int hitboxType)` | 向竞技场添加赛车（0: Octane, 1: Dominus），返回索引 |
| `physics_getCarConfig` | `(i32) -> i32` | `void* physics_getCarConfig(int hitboxIndex)` | 返回车型物理碰撞包预设配置指针 |
| `physics_step` | `(i32) -> ()` | `void physics_step(int ticks)` | 执行物理仿真步进（120Hz 严格步长），执行原生精准防抖判定并推送物理碰撞事件 |
| `physics_resetKickoff` | `(i32) -> ()` | `void physics_resetKickoff(int seed)` | 重置竞技场至开球状态（支持伪随机散列种子） |
| `physics_clearGoalFlag` | `() -> ()` | `void physics_clearGoalFlag()` | 清除当前帧进球得分状态标记位 |
| `physics_controlBall` | `(i32, i32) -> i32` | `int physics_controlBall(int carIndex, int modeIndex)` | 快速球权操控与训练宏指令（0: 停球, 1: 顶球, 2: 传球, 3: 挑球） |
| `physics_setUnlimitedBoost` | `(i32) -> ()` | `void physics_setUnlimitedBoost(int unlimited)` | 开启/关闭无限氮气喷射加速变异器 |
| `physics_getStatePtr` | `() -> i32` | `float* physics_getStatePtr()` | 返回物理共享状态缓冲区内存首地址 (`g_stateBuffer`) |
| `physics_getStateSize` | `() -> i32` | `int physics_getStateSize()` | 返回状态缓冲区长度（512 floats） |
| `physics_getControlsPtr` | `() -> i32` | `float* physics_getControlsPtr()` | 返回赛车输入控制缓冲区首地址 (`g_controlsBuffer`) |
| `physics_getPadInfoPtr` | `() -> i32` | `float* physics_getPadInfoPtr()` | 返回 34 个大/小充气垫空间几何坐标与尺寸配置首地址 |
| `physics_getBallOnGround` | `() -> i32` | `int physics_getBallOnGround()` | 检测足球是否触地（Z <= 半径 91.25 + 2.0 uu） |
| `physics_getBallRadius` | `() -> f32` | `float physics_getBallRadius()` | 返回标准 Soccar 足球物理碰撞半径（91.25 uu） |
| `free` | `(i32) -> ()` | `void free(void* ptr)` | Emscripten 线性堆内存释放 |
| `malloc` | `(i32) -> i32` | `void* malloc(size_t size)` | Emscripten 线性堆内存分配 |
| `physics_setCarState` | `(i32, i32) -> ()` | `void physics_setCarState(int carIndex, float* statePtr)` | 强制重写指定车辆的瞬时物理位姿与速度矢量 |
| `physics_setBallState` | `(i32, i32) -> ()` | `void physics_setBallState(int carIndexOrNull, float* statePtr)` | 强制重写足球的瞬时位置、旋转矩阵与线/角速度 |
| `physics_getEventBufferPtr` | `() -> i32` | `void* physics_getEventBufferPtr()` | 返回物理事件零拷贝环形队列内存首地址 (`g_eventBuffer`) |
| `physics_getEventBufferSize` | `() -> i32` | `int physics_getEventBufferSize()` | 返回物理事件环形缓冲区字节总大小 (3120 bytes) |
| `physics_clearEvents` | `() -> ()` | `void physics_clearEvents()` | 清空物理事件环形队列（重置 writeSeq 与缓冲区） |

---

## 2. 共享内存物理状态布局 (`GameStateBufferPod`)

底层共享内存物理状态通过强类型 Standard Layout POD 结构体 `GameStateBufferPod` 管理，严格按 4 字节自然对齐（`#pragma pack(push, 4)`）。总长度严格为 512 个 32 位浮点数（2048 字节）：

```cpp
struct GameStateBufferPod {
    ArenaHeaderPod header;     // 4 floats (offset 0..3)
    BallStatePod ball;         // 18 floats (offset 4..21)
    CarStatePod cars[8];       // 8 * 51 floats (offset 22..429)
    BoostPadStatePod pads[34]; // 34 * 2 floats (offset 430..497)
    float reserved[14];        // 14 floats (offset 498..511)
};
```

| 字段区间 (Float32) | 结构体类型 | 详细解析 |
| :--- | :--- | :--- |
| `[0..3]` | `ArenaHeaderPod` (4) | `tickCount`, `goalScoredFlag`, `numCars`, `numPads` |
| `[4..21]` | `BallStatePod` (18) | `posX..Z`, `rotFwdX..Z`, `rotRightX..Z`, `rotUpX..Z`, `velX..Z`, `angVelX..Z` |
| `[22..429]` | `CarStatePod[8]` (408) | 每辆车 51 floats（位置、3 轴旋转单位向量、线/角速度、氮气量、触地/音速/翻滚状态、4 轮悬挂与触地数据、触碰法线、跳跃/重置/击球事件序列号） |
| `[430..497]` | `BoostPadStatePod[34]` (68) | 每个垫子 2 floats（`isActive`: 1.0/0.0, `cooldown`: 剩余冷却秒数） |
| `[498..511]` | `float reserved[14]` (14) | 预留对齐空间，保证整包总长为 512 floats (2048 字节) |

---

## 3. 零拷贝物理事件环形队列 (`PhysicsEventBuffer`)

采用 N = 64 容量的单调序列号无锁环形队列，结构严格按 48 字节对齐（单事件 12 单元 * 4 字节）：

```cpp
struct PhysicsEvent {
    uint32_t type;         // [Word 0] 1: CarBallHit, 2: BoostPickup, 3: CarDemo
    uint32_t tick;         // [Word 1] 物理帧序号
    float x, y, z;         // [Word 2, 3, 4] 空间 3D 绝对坐标 (UU)
    union {
        struct {
            uint16_t carIndex;    // [Word 5 low] 车号 (0..7)
            uint16_t team;        // [Word 5 high] 队伍 (0: Blue, 1: Orange)
            float relSpeed;       // [Word 6] 相对撞击速度
            float normalX, normalY, normalZ; // [Word 7, 8, 9] 接触面法线
            float impulse;        // [Word 10] 实际额外冲量大小
        } carBall;
        float fParams[6];
        uint32_t uParams[6];
    };
    uint32_t flags;        // [Word 11] bit0: InitialTouch, bit1: Supersonic, bit2: Flip, bit3: Ground
};
```

---

## 4. 控制输入内存布局

### 赛车控制缓冲区 (`g_controlsBuffer`)
- 大小：64 浮点数（8 辆车 * 8 浮点数/车）
- 每车步长 8：`throttle`, `steer`, `pitch`, `yaw`, `roll`, `jump`, `boost`, `handbrake`

---

## 5. 重新构建命令

在仓库根目录或本目录下直接执行：

```bash
source /tmp/env.sh # 或 source /opt/emsdk/emsdk_env.sh
python3 wasm/compile.py
```

编译输出产物将写入 `wasm/build/` 目录：
- `wasm/build/core.wasm`：优化后的 WebAssembly 物理微内核。
- `wasm/build/core.js`：ES6 模块胶水层代码。
