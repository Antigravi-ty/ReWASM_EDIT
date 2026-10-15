/**
 * RocketSimBridge.cpp
 * Standard C-ABI WebAssembly Bridge for RocketSim Physics Engine.
 * 
 * Reconstructed and extended for high-performance esports simulation.
 * Includes 64-byte CPU cache-line aligned Incident & Signal Event buffer,
 * Normal Relative Velocity (Delta_Vn) ball contact state machine,
 * 5-second Ball Prediction Tracker, 256MB 4-Timeline Replay storage,
 * and dual-touch continuous possession scoring.
 */

#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>
#include <map>
#include <algorithm>
#include <cstdlib>

#include "RocketSim.h"
#include "Sim/Arena/Arena.h"
#include "Sim/Car/Car.h"
#include "Sim/Ball/Ball.h"
#include "Sim/BallPredTracker/BallPredTracker.h"
#include "BulletLink.h"

using namespace RocketSim;

// Buffer layout constants matching RocketSimConstants.js
static constexpr int MAX_ARENA_CARS = 8;
static constexpr int CAR_STRIDE = 40;
static constexpr int CONTROLS_STRIDE = 8;
static constexpr int NUM_ARENA_PADS = 34;
static constexpr int PAD_STATE_STRIDE = 2;

#pragma pack(push, 4)
struct ArenaHeaderPod {
    float tickCount;
    float goalScoredFlag;
    float numCars;
    float numPads;
};

struct BallStatePod {
    float posX, posY, posZ;
    float rotFwdX, rotFwdY, rotFwdZ;
    float rotRightX, rotRightY, rotRightZ;
    float rotUpX, rotUpY, rotUpZ;
    float velX, velY, velZ;
    float angVelX, angVelY, angVelZ;
}; // 18 floats

struct WheelPod {
    float susLength;
    float steerAngle;
    float hasContact;
}; // 3 floats

struct CarStatePod {
    float posX, posY, posZ;
    float rotFwdX, rotFwdY, rotFwdZ;
    float rotRightX, rotRightY, rotRightZ;
    float rotUpX, rotUpY, rotUpZ;
    float velX, velY, velZ;
    float angVelX, angVelY, angVelZ;
    float boost;
    float isOnGround;
    float isSupersonic;
    float isDemoed;
    float hasFlipOrJump;
    float isBoosting;
    float isFlipping;
    WheelPod wheels[4]; // 12 floats (offset 25..36)
    float groundNormalX, groundNormalY, groundNormalZ; // 3 floats (offset 37..39)
}; // 40 floats
static_assert(sizeof(CarStatePod) == 40 * sizeof(float), "CarStatePod must be 40 floats (160 bytes)");

struct BoostPadStatePod {
    float isActive;
    float cooldown;
}; // 2 floats

struct GameStateBufferPod {
    ArenaHeaderPod header;              // 4 floats (0..3)
    BallStatePod ball;                  // 18 floats (4..21)
    CarStatePod cars[MAX_ARENA_CARS];   // 320 floats (22..341)
    BoostPadStatePod pads[NUM_ARENA_PADS]; // 68 floats (342..409)
    float isBallSleeping;               // float (410)
    float isSimulationFrozen;           // float (411)
    float possessionState;              // float (412: 0: Neutral, 1: Blue, 2: Orange)
    float possessionBluePoints;         // float (413)
    float possessionOrangePoints;       // float (414)
    float possessionDuration;           // float (415)
    float reserved[8];                  // 8 floats (416..423)
};
#pragma pack(pop)
static_assert(sizeof(GameStateBufferPod) == 424 * sizeof(float), "GameStateBufferPod must be exactly 424 floats");

// Unified Physics Incident Event Types (0x01..0x1F)
static constexpr uint32_t INCIDENT_NONE = 0x00;
static constexpr uint32_t INCIDENT_CAR_BALL_HIT = 0x01;
static constexpr uint32_t INCIDENT_CAR_CAR_BUMP = 0x02;
static constexpr uint32_t INCIDENT_CAR_CAR_DEMO = 0x03;
static constexpr uint32_t INCIDENT_BALL_WORLD_HIT = 0x04;
static constexpr uint32_t INCIDENT_BALL_GOALPOST_HIT = 0x05;
static constexpr uint32_t INCIDENT_CAR_JUMP = 0x06;
static constexpr uint32_t INCIDENT_CAR_DOUBLE_JUMP = 0x07;
static constexpr uint32_t INCIDENT_CAR_DODGE = 0x08;
static constexpr uint32_t INCIDENT_CAR_SUPERSONIC_ENTER = 0x09;
static constexpr uint32_t INCIDENT_BOOST_PICKUP = 0x0A;
static constexpr uint32_t INCIDENT_BOOST_SPAWN = 0x0B;
static constexpr uint32_t INCIDENT_CAR_BOOST_START = 0x0C;
static constexpr uint32_t INCIDENT_CAR_BOOST_STOP = 0x0D;
static constexpr uint32_t INCIDENT_GOAL_SCORED = 0x0E;
static constexpr uint32_t INCIDENT_FLIP_RESET_GAINED = 0x0F;
static constexpr uint32_t INCIDENT_BALL_STATE_CHANGE = 0x10;
static constexpr uint32_t INCIDENT_POSSESSION_CHANGE = 0x11;

// Server Authoritative Signal Events (0x80..0xBF)
static constexpr uint32_t SIGNAL_MATCH_START = 0x80;
static constexpr uint32_t SIGNAL_KICKOFF_WILL_START = 0x81;
static constexpr uint32_t SIGNAL_KICKOFF_PREPARE = 0x81;
static constexpr uint32_t SIGNAL_KICKOFF_START = 0x82;
static constexpr uint32_t SIGNAL_KICKOFF_GO = 0x82;
static constexpr uint32_t SIGNAL_COUNTDOWN_TICK = 0x83;
static constexpr uint32_t SIGNAL_MATCH_PAUSED = 0x84;
static constexpr uint32_t SIGNAL_MATCH_PAUSE = 0x84;
static constexpr uint32_t SIGNAL_MATCH_WILL_RESUME = 0x85;
static constexpr uint32_t SIGNAL_MATCH_RESUMED = 0x86;
static constexpr uint32_t SIGNAL_MATCH_RESUME = 0x86;
static constexpr uint32_t SIGNAL_GOAL_REPLAY_WILL_START = 0x87;
static constexpr uint32_t SIGNAL_GOAL_REPLAY_START = 0x88;
static constexpr uint32_t SIGNAL_GOAL_REPLAY_WILL_END = 0x89;
static constexpr uint32_t SIGNAL_GOAL_REPLAY_END = 0x8A;
static constexpr uint32_t SIGNAL_OVERTIME_START = 0x8B;
static constexpr uint32_t SIGNAL_TIME_ALERT_60S = 0x8C;
static constexpr uint32_t SIGNAL_TIME_ALERT_30S = 0x8D;
static constexpr uint32_t SIGNAL_PLAYER_JOINED = 0x8E;
static constexpr uint32_t SIGNAL_PLAYER_ENTER_ARENA = 0x8E;
static constexpr uint32_t SIGNAL_PLAYER_LEFT = 0x8F;
static constexpr uint32_t SIGNAL_PLAYER_LEAVE_ARENA = 0x8F;
static constexpr uint32_t SIGNAL_FORFEIT_VOTE = 0x90;
static constexpr uint32_t SIGNAL_SCORE_UPDATED = 0x91;
static constexpr uint32_t SIGNAL_BALL_RESET = 0x92;
static constexpr uint32_t SIGNAL_ROUND_STARTED = 0x93;
static constexpr uint32_t SIGNAL_MATCH_CREATED = 0x94;
static constexpr uint32_t SIGNAL_MATCH_INITIALIZED = 0x95;
static constexpr uint32_t SIGNAL_MATCH_PREPARE = 0x96;
static constexpr uint32_t SIGNAL_MATCH_ENDED = 0x97;
static constexpr uint32_t SIGNAL_MATCH_DESTROYED = 0x98;
static constexpr uint32_t SIGNAL_PODIUM_START = 0x99;
static constexpr uint32_t SIGNAL_PLAYER_LOADOUT_UPDATED = 0x9A;
static constexpr uint32_t SIGNAL_PLAYER_SLOT_UPDATED = 0x9B;
static constexpr uint32_t SIGNAL_DEMOLITION = 0x9C;
static constexpr uint32_t SIGNAL_CLOCK_UPDATED_SECONDS = 0x9D;

// Classified Ball Hit Surface Subtypes
static constexpr uint32_t SUBTYPE_SURFACE_FLOOR = 1;
static constexpr uint32_t SUBTYPE_SURFACE_CEILING = 2;
static constexpr uint32_t SUBTYPE_SURFACE_SIDE_WALL = 3;
static constexpr uint32_t SUBTYPE_SURFACE_BACKBOARD = 4;
static constexpr uint32_t SUBTYPE_SURFACE_CORNER_45 = 5;
static constexpr uint32_t SUBTYPE_RAMP_GROUND = 6;
static constexpr uint32_t SUBTYPE_RAMP_CEILING = 7;
static constexpr uint32_t SUBTYPE_GOALPOST = 8;
static constexpr uint32_t SUBTYPE_CROSSBAR = 9;
static constexpr uint32_t SUBTYPE_OTHER_CURVE = 10;

// Strict 64-byte (CPU Cache-Line Aligned) Physics Event Structure
#pragma pack(push, 4)
struct alignas(64) PhysicsEvent {
    uint32_t type;          // [Word 0] Event type (0x01..0x3F: Incident, 0x80..0xBF: Signal)
    uint32_t tick;          // [Word 1] Physics tick timestamp
    float posX;             // [Word 2] World X in Unreal Units
    float posY;             // [Word 3] World Y in Unreal Units
    float posZ;             // [Word 4] World Z in Unreal Units
    float normX;            // [Word 5] Normalized normal X
    float normY;            // [Word 6] Normalized normal Y
    float normZ;            // [Word 7] Normalized normal Z
    float normalRelVel;     // [Word 8] Delta_Vn = -(v_ball - v_other) . normal
    float speed;            // [Word 9] Scalar speed |v|
    float impulse;          // [Word 10] Applied collision impulse magnitude
    uint16_t primaryId;     // [Word 11, Low 16] Primary Entity ID (car index / pad index)
    uint16_t secondaryId;   // [Word 11, High 16] Secondary Entity ID (victim car index / pad index)
    uint32_t subType;       // [Word 12] SubType (surface classification, action type, etc.)
    uint32_t flags;         // [Word 13] Bitmask flags (InitialTouch, Supersonic, etc.)
    float customFloat;      // [Word 14] Custom parameter (possession time, countdown, etc.)
    uint32_t customInt;     // [Word 15] Custom parameter (possession points, resume frame ID, etc.)
};
#pragma pack(pop)
static_assert(sizeof(PhysicsEvent) == 64, "PhysicsEvent must be strictly 64 bytes");

// Zero-Copy Event Ring Buffer (Capacity 256 entries = 16,384 bytes + 16-byte header = 16,400 bytes)
static constexpr uint32_t EVENT_RING_BUFFER_CAPACITY = 256;

#pragma pack(push, 4)
struct PhysicsEventBuffer {
    uint32_t writeSeq;      // Monotonically increasing sequence
    uint32_t capacity;      // 256
    uint32_t eventSize;     // 64 bytes
    uint32_t droppedCount;  // Overflow drop counter
    PhysicsEvent events[EVENT_RING_BUFFER_CAPACITY];
};
#pragma pack(pop)
static_assert(sizeof(PhysicsEventBuffer) == 16 + 256 * 64, "PhysicsEventBuffer must be exactly 16400 bytes");

static constexpr int STATE_BUFFER_SIZE = sizeof(GameStateBufferPod) / sizeof(float);

// Shared memory buffers mapped to WebAssembly heap
static GameStateBufferPod g_state;
static float* g_stateBuffer = reinterpret_cast<float*>(&g_state);
static float g_controlsBuffer[MAX_ARENA_CARS * CONTROLS_STRIDE]; // 64 floats
static float g_padInfoBuffer[NUM_ARENA_PADS * 4];                 // 136 floats

// Global Ring Buffer for zero-copy physics events
static PhysicsEventBuffer g_eventBuffer = { 0, EVENT_RING_BUFFER_CAPACITY, sizeof(PhysicsEvent), 0, {} };

// Silent resimulation mode flag (suppresses event emission during rollback replay)
static bool g_silentResim = false;

static void PushPhysicsEvent(const PhysicsEvent& ev) {
    if (g_silentResim) return;
    uint32_t slot = g_eventBuffer.writeSeq % EVENT_RING_BUFFER_CAPACITY;
    g_eventBuffer.events[slot] = ev;
    g_eventBuffer.writeSeq++;
}

// Active Arena and simulation state
static void syncStateBuffer();
static Arena* g_arena = nullptr;
static std::vector<Car*> g_cars;
static int g_goalScoredFlag = 0;
static bool g_unlimitedBoost = false;
static bool g_firstTouchFired = false;
static bool g_lastBallSleeping = true;

// Post-goal 30-frame ball flight & freeze latch (cleared idempotently by Goal Replay Start / Kickoff Will Start)
static constexpr uint32_t POST_GOAL_BALL_FLIGHT_TICKS = 30;
static bool g_goalIncidentLatched = false;
static uint32_t g_goalScoredAtTick = 0;
static bool g_ballFrozenPostGoal = false;
static BallState g_frozenBallState = {};

static void resetPostGoalLatchState() {
    g_goalIncidentLatched = false;
    g_goalScoredAtTick = 0;
    g_ballFrozenPostGoal = false;
    g_goalScoredFlag = 0;
    g_state.header.goalScoredFlag = 0.0f;
    if (g_arena && g_arena->ball) {
        g_arena->ball->_rigidBody.setActivationState(ACTIVE_TAG);
    }
}

// Debounce tick memory
static uint64_t g_lastBallWorldHitTick = 0;
static uint64_t g_carLastBallHitTick[MAX_ARENA_CARS] = {0};

// Dynamic Impact Thresholds
static float g_threshBallGround = 140.0f;
static float g_threshBallWall = 160.0f;
static float g_threshCarBall = 50.0f;
static uint32_t g_cooldownTicks = 8; // ~66.7ms at 120Hz
static float g_threshFloorNz = 0.8f;
static float g_threshNetNz = 0.05f;
static float g_threshDirDot = 0.0f; // 0.0 = unconstrained

// Simulation Freeze & Scheduled Unfreeze
static bool g_simulationFrozen = false;
static uint32_t g_unfreezeAtTick = 0;
static bool g_maskInputs = false;
static bool g_wasPadActive[NUM_ARENA_PADS] = {};

// Per-car action tracking
struct CarActionTracker {
    bool wasInAir = false;
    bool hadUnlimitedFlip = false;
    bool wasBoosting = false;
    bool wasSupersonic = false;
    bool wasJumping = false;
    bool hadDoubleJumped = false;
    bool wasFlipping = false;
};
static CarActionTracker g_carTrackers[MAX_ARENA_CARS];

// Per-car linear factor, gravity scale, and collision mask overrides
struct CarPhysicsOverride {
    bool active = false;
    btVector3 linearFactor = btVector3(1.0f, 1.0f, 1.0f);
    float gravityScale = 1.0f;
    btVector3 lockedOriginBT = btVector3(0.0f, 0.0f, 0.0f);
};
static CarPhysicsOverride g_carOverrides[MAX_ARENA_CARS];

struct CarCollisionMask {
    bool interactWithBall = true;
    bool interactWithCars = true;
};
static CarCollisionMask g_carCollisionMasks[MAX_ARENA_CARS];
static bool g_hasCustomCollisionMask = false;

static int getCarIndexFast(const Car* car) {
    if (!car) return -1;
    for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
        if (g_cars[i] == car) return static_cast<int>(i);
    }
    return -1;
}

struct BridgeOverlapFilterCallback : public btOverlapFilterCallback {
    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override {
        bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
        collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
        if (!collides || !g_hasCustomCollisionMask) return collides;

        btCollisionObject* obj0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
        btCollisionObject* obj1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);
        if (!obj0 || !obj1) return true;

        int u0 = obj0->getUserIndex();
        int u1 = obj1->getUserIndex();

        if (u0 == BT_USERINFO_TYPE_CAR && u1 == BT_USERINFO_TYPE_BALL) {
            int idx = getCarIndexFast(static_cast<const Car*>(obj0->getUserPointer()));
            if (idx >= 0 && !g_carCollisionMasks[idx].interactWithBall) return false;
        } else if (u1 == BT_USERINFO_TYPE_CAR && u0 == BT_USERINFO_TYPE_BALL) {
            int idx = getCarIndexFast(static_cast<const Car*>(obj1->getUserPointer()));
            if (idx >= 0 && !g_carCollisionMasks[idx].interactWithBall) return false;
        } else if (u0 == BT_USERINFO_TYPE_CAR && u1 == BT_USERINFO_TYPE_CAR) {
            int idx0 = getCarIndexFast(static_cast<const Car*>(obj0->getUserPointer()));
            int idx1 = getCarIndexFast(static_cast<const Car*>(obj1->getUserPointer()));
            if ((idx0 >= 0 && !g_carCollisionMasks[idx0].interactWithCars) ||
                (idx1 >= 0 && !g_carCollisionMasks[idx1].interactWithCars)) {
                return false;
            }
        }
        return true;
    }
};
static BridgeOverlapFilterCallback g_bridgeOverlapFilter;

// Ball Prediction Pipeline (up to 5.0 seconds = 600 ticks @ 120Hz)
static constexpr size_t MAX_BALL_PRED_TICKS = 600;
#pragma pack(push, 4)
struct BallPredSlicePod {
    float posX, posY, posZ;
    float velX, velY, velZ;
};
#pragma pack(pop)
static BallPredSlicePod g_ballPredBuffer[MAX_BALL_PRED_TICKS];
static uint32_t g_ballPredCount = 0;
static BallPredTracker* g_ballPredTracker = nullptr;

// Dual-Touch Continuous Possession Scoring State
struct PossessionState {
    int lastCarIndex = -1;
    int lastTeam = -1;
    int consecutiveTeamTouches = 0;
    int currentPossessionTeam = 0; // 0: Neutral, 1: Blue, 2: Orange
    float activePossessionTime = 0.0f;
    int bluePoints = 0;
    int orangePoints = 0;
    bool reportingEnabled = true;
};
static PossessionState g_possession;

// 256MB Dedicated Server Replay Storage
static constexpr size_t REPLAY_STORAGE_BYTES = 256 * 1024 * 1024; // 268,435,456 bytes
static uint8_t* g_replayBuffer = nullptr;
static size_t g_replayCursor = 0;

#pragma pack(push, 4)
struct ReplayHeader {
    uint32_t magic;         // 0x5245504C ('REPL')
    uint32_t version;       // 2
    uint32_t totalFrames;   // recorded frame count
    uint32_t stateSize;     // bytes per state snapshot
    uint32_t eventCount;    // recorded incident count
    uint32_t signalCount;   // recorded signal count
    uint32_t inputFrames;   // recorded user input frames
    uint32_t reserved[9];
};
#pragma pack(pop)

// Full Atomic State Snapshot Definitions for Deterministic Rollback
#pragma pack(push, 4)
struct BoostPadSnapshotPod {
    uint32_t isActive;
    float cooldown;
    uint32_t prevLockedCarID;
};

struct CarSnapshotPod {
    uint32_t team;
    uint32_t id;
    CarControls controls;
    CarState state;
    btTransform rbTransform;
    btVector3 rbLinearVelocity;
    btVector3 rbAngularVelocity;
    btWheelInfoRL wheels[4];
};

struct ArenaSnapshotPod {
    uint64_t tickCount;
    uint32_t lastCarID;
    int32_t goalScoredFlag;
    uint32_t numCars;
    uint64_t lastBallWorldHitTick;
    BallState ballState;
    btTransform ballTransform;
    btVector3 ballLinearVelocity;
    btVector3 ballAngularVelocity;
    BoostPadSnapshotPod pads[NUM_ARENA_PADS];
    CarSnapshotPod cars[MAX_ARENA_CARS];
};
#pragma pack(pop)
static_assert(sizeof(ArenaSnapshotPod) % sizeof(float) == 0, "ArenaSnapshotPod must be 4-byte aligned float multiple");

static constexpr int MAX_SNAPSHOT_SLOTS = 256;
static ArenaSnapshotPod g_snapshotSlots[MAX_SNAPSHOT_SLOTS];

static void serializeToSnapshot(ArenaSnapshotPod& snap) {
    if (!g_arena) {
        std::memset((void*)&snap, 0, sizeof(ArenaSnapshotPod));
        return;
    }
    snap.tickCount = g_arena->tickCount;
    snap.lastCarID = g_arena->_lastCarID;
    snap.goalScoredFlag = g_goalScoredFlag;
    snap.numCars = static_cast<uint32_t>(std::min(g_cars.size(), static_cast<size_t>(MAX_ARENA_CARS)));
    snap.lastBallWorldHitTick = g_lastBallWorldHitTick;

    if (g_arena->ball) {
        snap.ballState = g_arena->ball->GetState();
        snap.ballTransform = g_arena->ball->_rigidBody.getWorldTransform();
        snap.ballLinearVelocity = g_arena->ball->_rigidBody.getLinearVelocity();
        snap.ballAngularVelocity = g_arena->ball->_rigidBody.getAngularVelocity();
    }

    const auto& pads = g_arena->GetBoostPads();
    size_t padCount = std::min(pads.size(), static_cast<size_t>(NUM_ARENA_PADS));
    for (size_t p = 0; p < padCount; p++) {
        BoostPad* pad = pads[p];
        snap.pads[p].isActive = pad->_internalState.isActive ? 1 : 0;
        snap.pads[p].cooldown = pad->_internalState.cooldown;
        snap.pads[p].prevLockedCarID = pad->_internalState.prevLockedCarID;
    }

    for (size_t i = 0; i < snap.numCars; i++) {
        Car* car = g_cars[i];
        snap.cars[i].team = static_cast<uint32_t>(car->team);
        snap.cars[i].id = car->id;
        snap.cars[i].controls = car->controls;
        snap.cars[i].state = car->GetState();
        snap.cars[i].rbTransform = car->_rigidBody.getWorldTransform();
        snap.cars[i].rbLinearVelocity = car->_rigidBody.getLinearVelocity();
        snap.cars[i].rbAngularVelocity = car->_rigidBody.getAngularVelocity();
        for (int w = 0; w < 4; w++) {
            snap.cars[i].wheels[w] = car->_bulletVehicle.m_wheelInfo[w];
        }
    }
}

static void deserializeFromSnapshot(const ArenaSnapshotPod& snap) {
    if (!g_arena) return;

    g_arena->tickCount = snap.tickCount;
    g_arena->_lastCarID = snap.lastCarID;
    g_goalScoredFlag = snap.goalScoredFlag;
    g_lastBallWorldHitTick = snap.lastBallWorldHitTick;

    if (g_arena->ball) {
        g_arena->ball->SetState(snap.ballState);
        g_arena->ball->_internalState.tickCountSinceUpdate = snap.ballState.tickCountSinceUpdate;
        g_arena->ball->_rigidBody.setWorldTransform(snap.ballTransform);
        g_arena->ball->_rigidBody.setInterpolationWorldTransform(snap.ballTransform);
        g_arena->ball->_rigidBody.setLinearVelocity(snap.ballLinearVelocity);
        g_arena->ball->_rigidBody.setInterpolationLinearVelocity(snap.ballLinearVelocity);
        g_arena->ball->_rigidBody.setAngularVelocity(snap.ballAngularVelocity);
        g_arena->ball->_rigidBody.setInterpolationAngularVelocity(snap.ballAngularVelocity);
        g_arena->ball->_rigidBody.updateInertiaTensor();
        g_arena->ball->_rigidBody.clearForces();
        if (snap.ballLinearVelocity.length2() == 0 && snap.ballAngularVelocity.length2() == 0) {
            g_arena->ball->_rigidBody.setActivationState(ISLAND_SLEEPING);
        } else {
            g_arena->ball->_rigidBody.setActivationState(ACTIVE_TAG);
        }
        g_arena->_bulletWorld.updateSingleAabb(&g_arena->ball->_rigidBody);
        if (!g_arena->IsBallScored()) {
            g_goalIncidentLatched = false;
            g_ballFrozenPostGoal = false;
        }
    }

    const auto& pads = g_arena->GetBoostPads();
    size_t padCount = std::min(pads.size(), static_cast<size_t>(NUM_ARENA_PADS));
    for (size_t p = 0; p < padCount; p++) {
        BoostPad* pad = pads[p];
        pad->_internalState.isActive = (snap.pads[p].isActive != 0);
        pad->_internalState.cooldown = snap.pads[p].cooldown;
        pad->_internalState.prevLockedCarID = snap.pads[p].prevLockedCarID;
    }

    size_t carCount = std::min(static_cast<size_t>(snap.numCars), g_cars.size());
    for (size_t i = 0; i < carCount; i++) {
        Car* car = g_cars[i];
        car->team = static_cast<Team>(snap.cars[i].team);
        car->id = snap.cars[i].id;
        car->controls = snap.cars[i].controls;

        float* ctrlPtr = &g_controlsBuffer[i * CONTROLS_STRIDE];
        ctrlPtr[0] = car->controls.throttle;
        ctrlPtr[1] = car->controls.steer;
        ctrlPtr[2] = car->controls.pitch;
        ctrlPtr[3] = car->controls.yaw;
        ctrlPtr[4] = car->controls.roll;
        ctrlPtr[5] = car->controls.jump ? 1.0f : 0.0f;
        ctrlPtr[6] = car->controls.boost ? 1.0f : 0.0f;
        ctrlPtr[7] = car->controls.handbrake ? 1.0f : 0.0f;

        car->SetState(snap.cars[i].state);
        car->_internalState.tickCountSinceUpdate = snap.cars[i].state.tickCountSinceUpdate;
        car->_internalState.rotMat = snap.cars[i].rbTransform.getBasis();
        car->_rigidBody.setWorldTransform(snap.cars[i].rbTransform);
        car->_rigidBody.setInterpolationWorldTransform(snap.cars[i].rbTransform);
        car->_rigidBody.setLinearVelocity(snap.cars[i].rbLinearVelocity);
        car->_rigidBody.setInterpolationLinearVelocity(snap.cars[i].rbLinearVelocity);
        car->_rigidBody.setAngularVelocity(snap.cars[i].rbAngularVelocity);
        car->_rigidBody.setInterpolationAngularVelocity(snap.cars[i].rbAngularVelocity);
        car->_rigidBody.updateInertiaTensor();
        car->_rigidBody.clearForces();
        for (int w = 0; w < 4; w++) {
            car->_bulletVehicle.m_wheelInfo[w] = snap.cars[i].wheels[w];
            car->_bulletVehicle.m_wheelInfo[w].m_raycastInfo.m_groundObject = nullptr;
        }
        g_arena->_bulletWorld.updateSingleAabb(&car->_rigidBody);
    }

    // Purge persistent collision manifolds in Bullet collision dispatcher
    btCollisionDispatcher* dispatcher = (btCollisionDispatcher*)g_arena->_bulletWorld.getDispatcher();
    if (dispatcher) {
        int numManifolds = dispatcher->getNumManifolds();
        for (int m = 0; m < numManifolds; m++) {
            btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(m);
            if (manifold) {
                manifold->clearManifold();
            }
        }
    }

    if (g_arena->_bulletWorld.getConstraintSolver()) {
        g_arena->_bulletWorld.getConstraintSolver()->reset();
    }

    syncStateBuffer();
}

static int getCarIndex(Car* car) {
    if (!car) return -1;
    for (size_t i = 0; i < g_cars.size(); i++) {
        if (g_cars[i] == car) return static_cast<int>(i);
    }
    return -1;
}

// Goal score callback
static void onGoalScoredCallback(Arena* arena, Team scoringTeam, void* userInfo) {
    if (g_goalIncidentLatched) {
        return;
    }
    g_goalIncidentLatched = true;
    g_goalScoredAtTick = static_cast<uint32_t>(arena ? arena->tickCount : 0);
    g_ballFrozenPostGoal = false;

    g_goalScoredFlag = (scoringTeam == Team::BLUE) ? 1 : 2;
    g_state.header.goalScoredFlag = static_cast<float>(g_goalScoredFlag);

    if (!g_silentResim) {
        PhysicsEvent ev = {};
        ev.type = INCIDENT_GOAL_SCORED;
        ev.tick = static_cast<uint32_t>(arena && arena->tickCount > 0 ? arena->tickCount - 1 : 0);
        if (arena && arena->ball) {
            Vec p = arena->ball->GetState().pos;
            ev.posX = p.x; ev.posY = p.y; ev.posZ = p.z;
            ev.speed = arena->ball->GetState().vel.Length();
        }
        ev.primaryId = static_cast<uint16_t>(scoringTeam == Team::ORANGE ? 1 : 0);
        PushPhysicsEvent(ev);
    }
}

// Car-Car collision & Demo callback
static void onCarBumpCallback(Arena* arena, Car* bumper, Car* victim, bool isDemo, const Vec& contactPos, float relSpeed, float impulse, void* userInfo) {
    if (g_silentResim) return;
    PhysicsEvent ev = {};
    ev.type = isDemo ? INCIDENT_CAR_CAR_DEMO : INCIDENT_CAR_CAR_BUMP;
    ev.tick = static_cast<uint32_t>(arena->tickCount > 0 ? arena->tickCount - 1 : 0);
    ev.posX = contactPos.x;
    ev.posY = contactPos.y;
    ev.posZ = contactPos.z;

    int bumperIdx = getCarIndex(bumper);
    int victimIdx = getCarIndex(victim);
    ev.primaryId = static_cast<uint16_t>(bumperIdx >= 0 ? bumperIdx : 0);
    ev.secondaryId = static_cast<uint16_t>(victimIdx >= 0 ? victimIdx : 0);
    ev.speed = relSpeed;
    ev.impulse = impulse;
    ev.subType = isDemo ? 1 : 0;
    PushPhysicsEvent(ev);
}

// Surface classifier for 3D bounce geometry
static uint32_t ClassifyBallHitSurface(const Vec& pos, const Vec& normal, bool isGoalpost) {
    if (isGoalpost) {
        bool isCrossbar = (std::abs(pos.z - 642.0f) < 80.0f) && (std::abs(pos.x) <= 950.0f);
        return isCrossbar ? SUBTYPE_CROSSBAR : SUBTYPE_GOALPOST;
    }
    // 3-Category Main Surface Classification:
    // 1. Floor: strictly vertical upward normal
    if (normal.z >= g_threshFloorNz) return SUBTYPE_SURFACE_FLOOR;
    // 2. Field Net (sidewalls, backboards, ceiling net: horizontal or downward normal)
    if (normal.z <= g_threshNetNz) return SUBTYPE_SURFACE_SIDE_WALL;
    // 3. Ground Ramp / Transition Curve
    return SUBTYPE_RAMP_GROUND;
}

// Ball-World & Goalpost collision callback with Normal Relative Velocity State Machine
static void onBallWorldCallback(Arena* arena, const Vec& contactPos, const Vec& normal, float speed, bool isGoalpost, void* userInfo) {
    if (g_silentResim) return;
    uint64_t curTick = arena->tickCount;
    if (curTick <= g_lastBallWorldHitTick + g_cooldownTicks) {
        return;
    }

    Vec ballVel = arena->ball ? arena->ball->GetState().vel : Vec(0, 0, 0);
    // Normal Relative Velocity: Delta_Vn = -(v_ball . normal)
    float deltaVn = -(ballVel.Dot(normal));
    float dirDot = (speed > 1e-4f) ? std::max(0.0f, std::min(1.0f, deltaVn / speed)) : 0.0f;

    // Optional directional unit-vector dot product threshold (suppresses glancing / parallel roll)
    if (g_threshDirDot > 0.0f && dirDot < g_threshDirDot) {
        return;
    }

    uint32_t surface = ClassifyBallHitSurface(contactPos, normal, isGoalpost);
    float threshold = (surface == SUBTYPE_SURFACE_FLOOR) ? g_threshBallGround : g_threshBallWall;

    // Filter resting or sliding roll where normal velocity delta is near zero
    if (deltaVn < threshold) {
        return;
    }
    g_lastBallWorldHitTick = curTick;

    PhysicsEvent ev = {};
    ev.type = isGoalpost ? INCIDENT_BALL_GOALPOST_HIT : INCIDENT_BALL_WORLD_HIT;
    ev.tick = static_cast<uint32_t>(curTick > 0 ? curTick - 1 : 0);
    ev.posX = contactPos.x;
    ev.posY = contactPos.y;
    ev.posZ = contactPos.z;
    ev.normX = normal.x;
    ev.normY = normal.y;
    ev.normZ = normal.z;
    ev.normalRelVel = deltaVn;
    ev.speed = speed;
    ev.impulse = 0.0f;
    ev.primaryId = 0xFFFF;
    ev.secondaryId = 0xFFFF;
    ev.subType = surface;
    ev.flags = 0;
    ev.customFloat = dirDot; // Normalized directional dot product (sin(impactAngle))
    PushPhysicsEvent(ev);
}

// Dual-Touch Continuous Possession Logic
static void HandlePossessionBallHit(int carIndex, Team team) {
    int teamIdx = (team == Team::BLUE ? 0 : 1);
    if (teamIdx == g_possession.lastTeam) {
        g_possession.consecutiveTeamTouches++;
        if (g_possession.consecutiveTeamTouches >= 2) {
            int newTeamState = (team == Team::BLUE ? 1 : 2);
            if (g_possession.currentPossessionTeam != newTeamState) {
                g_possession.currentPossessionTeam = newTeamState;
                g_possession.activePossessionTime = 0.0f;
                if (g_possession.reportingEnabled) {
                    PhysicsEvent ev = {};
                    ev.type = INCIDENT_POSSESSION_CHANGE;
                    ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    ev.primaryId = static_cast<uint16_t>(carIndex);
                    ev.secondaryId = static_cast<uint16_t>(team == Team::ORANGE ? 1 : 0);
                    ev.subType = newTeamState; // 1: Blue, 2: Orange
                    ev.customFloat = 0.0f;
                    ev.customInt = (newTeamState == 1 ? g_possession.bluePoints : g_possession.orangePoints);
                    PushPhysicsEvent(ev);
                }
            }
        }
    } else {
        // Opponent hit resets to neutral
        g_possession.lastTeam = teamIdx;
        g_possession.consecutiveTeamTouches = 1;
        if (g_possession.currentPossessionTeam != 0) {
            g_possession.currentPossessionTeam = 0;
            g_possession.activePossessionTime = 0.0f;
            if (g_possession.reportingEnabled) {
                PhysicsEvent ev = {};
                ev.type = INCIDENT_POSSESSION_CHANGE;
                ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                ev.primaryId = static_cast<uint16_t>(carIndex);
                ev.secondaryId = static_cast<uint16_t>(team == Team::ORANGE ? 1 : 0);
                ev.subType = 0; // Neutral
                ev.customFloat = 0.0f;
                ev.customInt = 0;
                PushPhysicsEvent(ev);
            }
        }
    }
    g_possession.lastCarIndex = carIndex;
}

static void UpdatePossessionTick(float dt) {
    if (g_possession.currentPossessionTeam != 0) {
        g_possession.activePossessionTime += dt;
        if (g_possession.activePossessionTime >= 3.0f) {
            g_possession.activePossessionTime -= 3.0f;
            if (g_possession.currentPossessionTeam == 1) {
                g_possession.bluePoints++;
            } else {
                g_possession.orangePoints++;
            }
            if (g_possession.reportingEnabled) {
                PhysicsEvent ev = {};
                ev.type = INCIDENT_POSSESSION_CHANGE;
                ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                ev.primaryId = static_cast<uint16_t>(g_possession.lastCarIndex >= 0 ? g_possession.lastCarIndex : 0);
                ev.secondaryId = static_cast<uint16_t>(g_possession.currentPossessionTeam == 2 ? 1 : 0);
                ev.subType = g_possession.currentPossessionTeam;
                ev.customFloat = g_possession.activePossessionTime;
                ev.customInt = (g_possession.currentPossessionTeam == 1 ? g_possession.bluePoints : g_possession.orangePoints);
                PushPhysicsEvent(ev);
            }
        }
    }
}

// Synchronize Arena state to g_state
static void syncStateBuffer() {
    if (!g_arena) return;

    // Header
    g_state.header.tickCount = static_cast<float>(g_arena->tickCount);
    g_state.header.goalScoredFlag = static_cast<float>(g_goalScoredFlag);
    g_state.header.numCars = static_cast<float>(g_cars.size());
    g_state.header.numPads = static_cast<float>(NUM_ARENA_PADS);

    // Ball state
    if (g_arena->ball) {
        BallState bs = g_arena->ball->GetState();
        g_state.ball.posX = bs.pos.x;
        g_state.ball.posY = bs.pos.y;
        g_state.ball.posZ = bs.pos.z;

        g_state.ball.rotFwdX = bs.rotMat.forward.x;
        g_state.ball.rotFwdY = bs.rotMat.forward.y;
        g_state.ball.rotFwdZ = bs.rotMat.forward.z;

        g_state.ball.rotRightX = bs.rotMat.right.x;
        g_state.ball.rotRightY = bs.rotMat.right.y;
        g_state.ball.rotRightZ = bs.rotMat.right.z;

        g_state.ball.rotUpX = bs.rotMat.up.x;
        g_state.ball.rotUpY = bs.rotMat.up.y;
        g_state.ball.rotUpZ = bs.rotMat.up.z;

        g_state.ball.velX = bs.vel.x;
        g_state.ball.velY = bs.vel.y;
        g_state.ball.velZ = bs.vel.z;

        g_state.ball.angVelX = bs.angVel.x;
        g_state.ball.angVelY = bs.angVel.y;
        g_state.ball.angVelZ = bs.angVel.z;
    }

    // Cars
    for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
        Car* car = g_cars[i];
        CarState cs = car->GetState();
        CarStatePod& pod = g_state.cars[i];

        pod.posX = cs.pos.x;
        pod.posY = cs.pos.y;
        pod.posZ = cs.pos.z;

        pod.rotFwdX = cs.rotMat.forward.x;
        pod.rotFwdY = cs.rotMat.forward.y;
        pod.rotFwdZ = cs.rotMat.forward.z;

        pod.rotRightX = cs.rotMat.right.x;
        pod.rotRightY = cs.rotMat.right.y;
        pod.rotRightZ = cs.rotMat.right.z;

        pod.rotUpX = cs.rotMat.up.x;
        pod.rotUpY = cs.rotMat.up.y;
        pod.rotUpZ = cs.rotMat.up.z;

        pod.velX = cs.vel.x;
        pod.velY = cs.vel.y;
        pod.velZ = cs.vel.z;

        pod.angVelX = cs.angVel.x;
        pod.angVelY = cs.angVel.y;
        pod.angVelZ = cs.angVel.z;

        pod.boost = cs.boost;
        pod.isOnGround = cs.isOnGround ? 1.0f : 0.0f;
        pod.isSupersonic = cs.isSupersonic ? 1.0f : 0.0f;
        pod.isDemoed = cs.isDemoed ? 1.0f : 0.0f;
        pod.hasFlipOrJump = cs.HasFlipOrJump() ? 1.0f : 0.0f;
        pod.isBoosting = cs.isBoosting ? 1.0f : 0.0f;
        pod.isFlipping = cs.isFlipping ? 1.0f : 0.0f;

        // Wheel suspension & contact
        for (int w = 0; w < 4; w++) {
            const btWheelInfoRL& wheel = car->_bulletVehicle.m_wheelInfo[w];
            pod.wheels[w].susLength = wheel.m_raycastInfo.m_suspensionLength * BT_TO_UU;
            pod.wheels[w].steerAngle = wheel.m_steerAngle;
            pod.wheels[w].hasContact = wheel.m_raycastInfo.m_isInContact ? 1.0f : 0.0f;
        }

        // Contact normal
        btVector3 groundNormal(0, 0, 0);
        int contactCount = 0;
        for (int w = 0; w < 4; w++) {
            if (car->_bulletVehicle.m_wheelInfo[w].m_raycastInfo.m_isInContact) {
                groundNormal += car->_bulletVehicle.m_wheelInfo[w].m_raycastInfo.m_contactNormalWS;
                contactCount++;
            }
        }
        if (contactCount > 0) {
            groundNormal /= static_cast<float>(contactCount);
            groundNormal.normalize();
        } else if (cs.worldContact.hasContact) {
            groundNormal = btVector3(cs.worldContact.contactNormal.x, cs.worldContact.contactNormal.y, cs.worldContact.contactNormal.z);
            if (groundNormal.length2() > 1e-6f) {
                groundNormal.normalize();
            } else {
                groundNormal = btVector3(0, 0, 1);
            }
        } else {
            groundNormal = btVector3(0, 0, 1);
        }
        pod.groundNormalX = groundNormal.x();
        pod.groundNormalY = groundNormal.y();
        pod.groundNormalZ = groundNormal.z();
    }

    // Zero out inactive car slots
    for (size_t i = g_cars.size(); i < MAX_ARENA_CARS; i++) {
        std::memset(&g_state.cars[i], 0, sizeof(CarStatePod));
    }

    // Boost pads
    const auto& pads = g_arena->GetBoostPads();
    for (size_t i = 0; i < pads.size() && i < NUM_ARENA_PADS; i++) {
        g_state.pads[i].isActive = pads[i]->_internalState.isActive ? 1.0f : 0.0f;
        g_state.pads[i].cooldown = pads[i]->_internalState.cooldown;
    }

    // System Telemetry & States in reserved area
    bool isSleeping = (g_arena->ball && g_arena->ball->_rigidBody.getActivationState() == ISLAND_SLEEPING);
    g_state.isBallSleeping = isSleeping ? 1.0f : 0.0f;
    g_state.isSimulationFrozen = g_simulationFrozen ? 1.0f : 0.0f;
    g_state.possessionState = static_cast<float>(g_possession.currentPossessionTeam);
    g_state.possessionBluePoints = static_cast<float>(g_possession.bluePoints);
    g_state.possessionOrangePoints = static_cast<float>(g_possession.orangePoints);
    g_state.possessionDuration = g_possession.activePossessionTime;

    // Populate reserved[0..7] (floats 416..423) with PPO obs_v3 fields for Car 0 and Car 1
    std::memset(g_state.reserved, 0, sizeof(g_state.reserved));
    if (g_cars.size() > 0 && g_cars[0]) {
        const CarState& cs0 = g_cars[0]->_internalState;
        g_state.reserved[0] = cs0.isJumping ? 1.0f : 0.0f;
        g_state.reserved[1] = cs0.airTime;
        g_state.reserved[4] = cs0.jumpTime;
        g_state.reserved[5] = cs0.airTimeSinceJump;
    }
    if (g_cars.size() > 1 && g_cars[1]) {
        const CarState& cs1 = g_cars[1]->_internalState;
        g_state.reserved[2] = cs1.isJumping ? 1.0f : 0.0f;
        g_state.reserved[3] = cs1.airTime;
        g_state.reserved[6] = cs1.jumpTime;
        g_state.reserved[7] = cs1.airTimeSinceJump;
    }
}

extern "C" {

// Export 'physics_init'
void __wasm_call_ctors();

int physics_init(uint8_t* meshData, int32_t* meshSizes, int chunkCount) {
    static bool s_ctorsCalled = false;
    if (!s_ctorsCalled) {
        __wasm_call_ctors();
        s_ctorsCalled = true;
    }

    std::map<GameMode, std::vector<FileData>> meshFilesMap;
    if (meshData && meshSizes && chunkCount > 0) {
        int offset = 0;
        for (int i = 0; i < chunkCount; i++) {
            int chunkSize = meshSizes[i];
            if (chunkSize <= 0) continue;
            FileData chunk(meshData + offset, meshData + offset + chunkSize);
            meshFilesMap[GameMode::SOCCAR].push_back(chunk);
            offset += chunkSize;
        }
    }

    if (RocketSim::GetStage() == RocketSimStage::UNINITIALIZED) {
        RocketSim::InitFromMem(meshFilesMap, true);
    }
    return 1;
}

// Export 'c'
int physics_createArena() {
    if (g_ballPredTracker) {
        delete g_ballPredTracker;
        g_ballPredTracker = nullptr;
    }
    if (g_arena) {
        delete g_arena;
        g_arena = nullptr;
    }
    g_cars.clear();
    resetPostGoalLatchState();
    g_lastBallWorldHitTick = 0;
    g_firstTouchFired = false;
    g_lastBallSleeping = true;
    g_simulationFrozen = false;
    g_unfreezeAtTick = 0;
    g_maskInputs = false;
    std::memset(&g_possession, 0, sizeof(PossessionState));
    for (int p = 0; p < NUM_ARENA_PADS; p++) g_wasPadActive[p] = true;
    g_possession.reportingEnabled = true;

    g_hasCustomCollisionMask = false;
    for (int i = 0; i < MAX_ARENA_CARS; i++) {
        g_carLastBallHitTick[i] = 0;
        g_carTrackers[i] = CarActionTracker();
        g_carOverrides[i] = CarPhysicsOverride();
        g_carCollisionMasks[i] = CarCollisionMask();
    }

    // Initialize arena with full physical rotation enabled
    // Real-time gameplay requires ball angular velocity & spin simulation (noBallRot = false)
    ArenaConfig arenaConfig;
    arenaConfig.noBallRot = false;
    g_arena = Arena::Create(GameMode::SOCCAR, arenaConfig);
    if (!g_arena) return 0;

    if (g_arena->_bulletWorldParams.overlappingPairCache) {
        g_arena->_bulletWorldParams.overlappingPairCache->setOverlapFilterCallback(&g_bridgeOverlapFilter);
    }

    // Register callbacks
    g_arena->SetGoalScoreCallback(onGoalScoredCallback, nullptr);
    g_arena->SetCarBumpCallback(onCarBumpCallback, nullptr);
    g_arena->SetBallWorldCallback(onBallWorldCallback, nullptr);

    // Initialize boost pad positions
    const auto& pads = g_arena->GetBoostPads();
    for (size_t i = 0; i < pads.size() && i < NUM_ARENA_PADS; i++) {
        Vec pos = pads[i]->config.pos;
        g_padInfoBuffer[i * 4 + 0] = pos.x;
        g_padInfoBuffer[i * 4 + 1] = pos.y;
        g_padInfoBuffer[i * 4 + 2] = pos.z;
        g_padInfoBuffer[i * 4 + 3] = pads[i]->config.isBig ? 1.0f : 0.0f;
    }

    syncStateBuffer();
    return 1;
}

static void executePhysicsStep(int ticks, bool silent) {
    if (!g_arena) return;
    g_silentResim = silent;

    for (int step = 0; step < ticks; step++) {
        // Scheduled unfreeze check
        if (g_simulationFrozen && g_unfreezeAtTick > 0 && g_arena->tickCount >= g_unfreezeAtTick) {
            g_simulationFrozen = false;
            g_unfreezeAtTick = 0;
            g_maskInputs = false;
        }

        if (g_simulationFrozen) {
            // Tick advance without physical integration
            g_arena->tickCount++;
            continue;
        }

        // Feed controls for each vehicle
        for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
            Car* car = g_cars[i];
            float* ctrlPtr = &g_controlsBuffer[i * CONTROLS_STRIDE];

            if (g_maskInputs) {
                car->controls = CarControls();
            } else {
                car->controls.throttle  = ctrlPtr[0];
                car->controls.steer     = ctrlPtr[1];
                car->controls.pitch     = ctrlPtr[2];
                car->controls.yaw       = ctrlPtr[3];
                car->controls.roll      = ctrlPtr[4];
                car->controls.jump      = (ctrlPtr[5] > 0.5f);
                car->controls.boost     = (ctrlPtr[6] > 0.5f);
                car->controls.handbrake = (ctrlPtr[7] > 0.5f);
            }

            if (g_unlimitedBoost) {
                car->_internalState.boost = 100.0f;
            }

            if (g_carOverrides[i].active) {
                const CarPhysicsOverride& ov = g_carOverrides[i];
                car->_rigidBody.setLinearFactor(ov.linearFactor);
                car->_rigidBody.setGravity(g_arena->_mutatorConfig.gravity * UU_TO_BT * ov.gravityScale);
            }
        }

        g_arena->Step(1);

        // Enforce locked axes after step (clears residual velocity or position drift on 0-factor axes)
        for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
            if (!g_carOverrides[i].active) continue;
            Car* car = g_cars[i];
            const CarPhysicsOverride& ov = g_carOverrides[i];
            btVector3 vel = car->_rigidBody.getLinearVelocity() * ov.linearFactor;
            car->_rigidBody.setLinearVelocity(vel);
            car->_rigidBody.setInterpolationLinearVelocity(vel);
            btTransform tr = car->_rigidBody.getWorldTransform();
            btVector3 orig = tr.getOrigin();
            if (ov.linearFactor.x() == 0.0f) orig.setX(ov.lockedOriginBT.x());
            if (ov.linearFactor.y() == 0.0f) orig.setY(ov.lockedOriginBT.y());
            if (ov.linearFactor.z() == 0.0f) orig.setZ(ov.lockedOriginBT.z());
            tr.setOrigin(orig);
            car->_rigidBody.setWorldTransform(tr);
            car->_rigidBody.setInterpolationWorldTransform(tr);
            g_arena->_bulletWorld.updateSingleAabb(&car->_rigidBody);
        }

        // Post-goal 30-frame ball flight then freeze inside goal
        if (g_goalIncidentLatched && g_arena->ball) {
            if (g_arena->tickCount > g_goalScoredAtTick + POST_GOAL_BALL_FLIGHT_TICKS) {
                if (!g_ballFrozenPostGoal) {
                    g_frozenBallState = g_arena->ball->GetState();
                    g_frozenBallState.vel = Vec(0.0f, 0.0f, 0.0f);
                    g_frozenBallState.angVel = Vec(0.0f, 0.0f, 0.0f);
                    g_ballFrozenPostGoal = true;
                }
                g_arena->ball->SetState(g_frozenBallState);
                g_arena->ball->_rigidBody.setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
                g_arena->ball->_rigidBody.setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
                g_arena->ball->_rigidBody.clearForces();
                g_arena->ball->_rigidBody.setActivationState(ISLAND_SLEEPING);
                g_arena->_bulletWorld.updateSingleAabb(&g_arena->ball->_rigidBody);
            }
        }

        UpdatePossessionTick(g_arena->tickTime);

        // State detection and incident triggers
        if (!silent) {
            // 1. Car - Ball Contact & Flip Reset Checks
            for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
                Car* car = g_cars[i];
                CarState cs = car->GetState();
                CarActionTracker& tracker = g_carTrackers[i];

                if (cs.ballHitInfo.isValid && (g_arena->tickCount - cs.ballHitInfo.tickCountWhenHit <= 1)) {
                    if (g_arena->tickCount > g_carLastBallHitTick[i] + g_cooldownTicks) {
                        g_carLastBallHitTick[i] = g_arena->tickCount;

                        Vec ballVel = g_arena->ball ? g_arena->ball->GetState().vel : Vec(0, 0, 0);
                        Vec relVel = ballVel - cs.vel;
                        Vec normal = (cs.ballHitInfo.ballPos - cs.pos).Normalized();
                        float deltaVn = -(relVel.Dot(normal));
                        float relSpeed = relVel.Length();

                        PhysicsEvent ev = {};
                        ev.type = INCIDENT_CAR_BALL_HIT;
                        ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                        ev.posX = cs.ballHitInfo.ballPos.x;
                        ev.posY = cs.ballHitInfo.ballPos.y;
                        ev.posZ = cs.ballHitInfo.ballPos.z;
                        ev.normX = normal.x;
                        ev.normY = normal.y;
                        ev.normZ = normal.z;
                        ev.normalRelVel = deltaVn;
                        ev.speed = relSpeed;
                        ev.impulse = cs.ballHitInfo.extraHitVel.Length();
                        ev.primaryId = static_cast<uint16_t>(i);
                        ev.secondaryId = static_cast<uint16_t>(car->team == Team::ORANGE ? 1 : 0);
                        ev.subType = 0;

                        uint32_t flags = 0;
                        if (!g_firstTouchFired) {
                            flags |= (1 << 0);
                            g_firstTouchFired = true;
                        }
                        if (cs.isSupersonic) flags |= (1 << 1);
                        if (cs.isFlipping) flags |= (1 << 2);
                        if (cs.isOnGround) flags |= (1 << 3);
                        ev.flags = flags;
                        PushPhysicsEvent(ev);

                        HandlePossessionBallHit(static_cast<int>(i), car->team);
                    }
                }

                // Flip reset gained (direct native GotFlipReset event with car & ball coordinates)
                bool hasFlipReset = cs.GotFlipReset();
                if (hasFlipReset && !tracker.hadUnlimitedFlip) {
                    Vec ballPos = g_arena->ball ? g_arena->ball->GetState().pos : Vec(0, 0, 0);
                    PhysicsEvent ev = {};
                    ev.type = INCIDENT_FLIP_RESET_GAINED;
                    ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    ev.posX = cs.pos.x;
                    ev.posY = cs.pos.y;
                    ev.posZ = cs.pos.z;
                    ev.normX = ballPos.x;
                    ev.normY = ballPos.y;
                    ev.normZ = ballPos.z;
                    ev.primaryId = static_cast<uint16_t>(i);
                    ev.secondaryId = static_cast<uint16_t>(car->team == Team::ORANGE ? 1 : 0);
                    ev.subType = 1; // BALL_RESET
                    PushPhysicsEvent(ev);
                }
                tracker.hadUnlimitedFlip = hasFlipReset;

                // Supersonic entry
                if (cs.isSupersonic && !tracker.wasSupersonic) {
                    PhysicsEvent ev = {};
                    ev.type = INCIDENT_CAR_SUPERSONIC_ENTER;
                    ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    ev.posX = cs.pos.x;
                    ev.posY = cs.pos.y;
                    ev.posZ = cs.pos.z;
                    ev.primaryId = static_cast<uint16_t>(i);
                    ev.secondaryId = static_cast<uint16_t>(car->team == Team::ORANGE ? 1 : 0);
                    ev.speed = cs.vel.Length();
                    PushPhysicsEvent(ev);
                }
                tracker.wasSupersonic = cs.isSupersonic;

                // Boost Start / Stop
                if (cs.isBoosting && !tracker.wasBoosting) {
                    PhysicsEvent ev = {};
                    ev.type = INCIDENT_CAR_BOOST_START;
                    ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    ev.posX = cs.pos.x;
                    ev.posY = cs.pos.y;
                    ev.posZ = cs.pos.z;
                    ev.primaryId = static_cast<uint16_t>(i);
                    ev.secondaryId = static_cast<uint16_t>(car->team == Team::ORANGE ? 1 : 0);
                    PushPhysicsEvent(ev);
                } else if (!cs.isBoosting && tracker.wasBoosting) {
                    PhysicsEvent ev = {};
                    ev.type = INCIDENT_CAR_BOOST_STOP;
                    ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    ev.posX = cs.pos.x;
                    ev.posY = cs.pos.y;
                    ev.posZ = cs.pos.z;
                    ev.primaryId = static_cast<uint16_t>(i);
                    ev.secondaryId = static_cast<uint16_t>(car->team == Team::ORANGE ? 1 : 0);
                    PushPhysicsEvent(ev);
                }
                tracker.wasBoosting = cs.isBoosting;

                // Jump actions
                if (cs.isJumping && !tracker.wasJumping) {
                    PhysicsEvent ev = {};
                    ev.type = INCIDENT_CAR_JUMP;
                    ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    ev.posX = cs.pos.x; ev.posY = cs.pos.y; ev.posZ = cs.pos.z;
                    ev.primaryId = static_cast<uint16_t>(i);
                    ev.secondaryId = static_cast<uint16_t>(car->team == Team::ORANGE ? 1 : 0);
                    ev.subType = 1; // SINGLE_JUMP
                    PushPhysicsEvent(ev);
                }
                if (cs.hasDoubleJumped && !tracker.hadDoubleJumped) {
                    PhysicsEvent ev = {};
                    ev.type = INCIDENT_CAR_DOUBLE_JUMP;
                    ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    ev.posX = cs.pos.x; ev.posY = cs.pos.y; ev.posZ = cs.pos.z;
                    ev.primaryId = static_cast<uint16_t>(i);
                    ev.secondaryId = static_cast<uint16_t>(car->team == Team::ORANGE ? 1 : 0);
                    ev.subType = 2; // DOUBLE_JUMP
                    PushPhysicsEvent(ev);
                }
                if (cs.isFlipping && !tracker.wasFlipping) {
                    PhysicsEvent ev = {};
                    ev.type = INCIDENT_CAR_DODGE;
                    ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    ev.posX = cs.pos.x; ev.posY = cs.pos.y; ev.posZ = cs.pos.z;
                    ev.primaryId = static_cast<uint16_t>(i);
                    ev.secondaryId = static_cast<uint16_t>(car->team == Team::ORANGE ? 1 : 0);
                    ev.subType = 3; // DODGE
                    PushPhysicsEvent(ev);
                }
                tracker.wasJumping = cs.isJumping;
                tracker.hadDoubleJumped = cs.hasDoubleJumped;
                tracker.wasFlipping = cs.isFlipping;
            }

            // 2. Ball sleeping / waking state change
            bool currentSleeping = (g_arena->ball && g_arena->ball->_rigidBody.getActivationState() == ISLAND_SLEEPING);
            if (g_lastBallSleeping && !currentSleeping) {
                PhysicsEvent ev = {};
                ev.type = INCIDENT_BALL_STATE_CHANGE;
                ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                if (g_arena->ball) {
                    Vec p = g_arena->ball->GetState().pos;
                    ev.posX = p.x; ev.posY = p.y; ev.posZ = p.z;
                }
                ev.subType = 1; // ACTIVE / WOKE_UP
                PushPhysicsEvent(ev);
            } else if (!g_lastBallSleeping && currentSleeping) {
                PhysicsEvent ev = {};
                ev.type = INCIDENT_BALL_STATE_CHANGE;
                ev.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                if (g_arena->ball) {
                    Vec p = g_arena->ball->GetState().pos;
                    ev.posX = p.x; ev.posY = p.y; ev.posZ = p.z;
                }
                ev.subType = 0; // SLEEPING
                PushPhysicsEvent(ev);
            }
            g_lastBallSleeping = currentSleeping;

            // 3. Boost Pad Pickups & Respawns
            const auto& pads = g_arena->GetBoostPads();
            for (size_t p = 0; p < pads.size() && p < NUM_ARENA_PADS; p++) {
                BoostPad* pad = pads[p];
                BoostPadState ps = pad->GetState();
                if (!ps.isActive && g_wasPadActive[p]) {
                    Vec padPos = pad->config.pos;
                    PhysicsEvent pEv = {};
                    pEv.type = INCIDENT_BOOST_PICKUP;
                    pEv.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    pEv.posX = padPos.x; pEv.posY = padPos.y; pEv.posZ = padPos.z;
                    pEv.primaryId = static_cast<uint16_t>(ps.prevLockedCarID < g_cars.size() ? ps.prevLockedCarID : 0xFFFF);
                    pEv.secondaryId = static_cast<uint16_t>(p);
                    pEv.subType = pad->config.isBig ? 1 : 0;
                    PushPhysicsEvent(pEv);
                } else if (ps.isActive && !g_wasPadActive[p]) {
                    Vec padPos = pad->config.pos;
                    PhysicsEvent rEv = {};
                    rEv.type = INCIDENT_BOOST_SPAWN;
                    rEv.tick = static_cast<uint32_t>(g_arena->tickCount > 0 ? g_arena->tickCount - 1 : 0);
                    rEv.posX = padPos.x; rEv.posY = padPos.y; rEv.posZ = padPos.z;
                    rEv.secondaryId = static_cast<uint16_t>(p);
                    rEv.subType = pad->config.isBig ? 1 : 0;
                    PushPhysicsEvent(rEv);
                }
                g_wasPadActive[p] = ps.isActive;
            }
        }
    }

    g_silentResim = false;
    syncStateBuffer();
}

// Export 's'
void physics_step(int ticks) {
    executePhysicsStep(ticks, false);
}

void physics_stepSilent(int ticks) {
    executePhysicsStep(ticks, true);
}

int physics_saveState(float* outBuffer) {
    if (!outBuffer) return 0;
    ArenaSnapshotPod snap;
    serializeToSnapshot(snap);
    std::memcpy(outBuffer, &snap, sizeof(ArenaSnapshotPod));
    return 1;
}

int physics_restoreState(const float* inBuffer) {
    if (!inBuffer) return 0;
    ArenaSnapshotPod snap;
    std::memcpy((void*)&snap, inBuffer, sizeof(ArenaSnapshotPod));
    deserializeFromSnapshot(snap);
    return 1;
}

int physics_saveStateSlot(int slot) {
    if (slot < 0 || slot >= MAX_SNAPSHOT_SLOTS) return 0;
    serializeToSnapshot(g_snapshotSlots[slot]);
    return 1;
}

int physics_restoreStateSlot(int slot) {
    if (slot < 0 || slot >= MAX_SNAPSHOT_SLOTS) return 0;
    deserializeFromSnapshot(g_snapshotSlots[slot]);
    return 1;
}

int physics_getStateSnapshotSize() {
    return static_cast<int>(sizeof(ArenaSnapshotPod) / sizeof(float));
}

// Export 't'
void physics_resetKickoff(int seed) {
    if (!g_arena) return;
    g_arena->ResetToRandomKickoff(seed);
    resetPostGoalLatchState();
    g_lastBallWorldHitTick = 0;
    g_firstTouchFired = false;
    g_lastBallSleeping = true;
    std::memset(&g_possession, 0, sizeof(PossessionState));
    for (int p = 0; p < NUM_ARENA_PADS; p++) g_wasPadActive[p] = true;
    g_possession.reportingEnabled = true;

    for (int i = 0; i < MAX_ARENA_CARS; i++) {
        g_carLastBallHitTick[i] = 0;
        g_carTrackers[i] = CarActionTracker();
    }
    syncStateBuffer();
}

void physics_resetGoalLatch() {
    resetPostGoalLatchState();
}

// Dynamic Impact Threshold Adjustment
void physics_setImpactThresholds(float ballGround, float ballWall, float carBall, uint32_t cooldownTicks) {
    g_threshBallGround = ballGround;
    g_threshBallWall = ballWall;
    g_threshCarBall = carBall;
    g_cooldownTicks = cooldownTicks;
}

void physics_setSurfaceTaxonomyThresholds(float floorNz, float netNz) {
    g_threshFloorNz = floorNz;
    g_threshNetNz = netNz;
}

void physics_setDirectionDotThreshold(float minDirDot) {
    g_threshDirDot = minDirDot;
}

float physics_getDirectionDotThreshold() {
    return g_threshDirDot;
}

// Ball Prediction API (up to 5.0 seconds = 600 ticks @ 120Hz)
void physics_updateBallPrediction(float maxSeconds) {
    if (!g_arena || !g_arena->ball) {
        g_ballPredCount = 0;
        return;
    }
    float seconds = std::clamp(maxSeconds, 0.1f, 5.0f);
    size_t numTicks = std::min(MAX_BALL_PRED_TICKS, static_cast<size_t>(seconds * 120.0f));

    if (!g_ballPredTracker) {
        g_ballPredTracker = new BallPredTracker(g_arena, MAX_BALL_PRED_TICKS);
    }
    g_ballPredTracker->numPredTicks = numTicks;
    g_ballPredTracker->ForceUpdateAllPred(g_arena->ball->GetState());

    for (size_t i = 0; i < numTicks && i < g_ballPredTracker->predData.size(); i++) {
        const BallState& bs = g_ballPredTracker->predData[i];
        g_ballPredBuffer[i].posX = bs.pos.x;
        g_ballPredBuffer[i].posY = bs.pos.y;
        g_ballPredBuffer[i].posZ = bs.pos.z;
        g_ballPredBuffer[i].velX = bs.vel.x;
        g_ballPredBuffer[i].velY = bs.vel.y;
        g_ballPredBuffer[i].velZ = bs.vel.z;
    }
    g_ballPredCount = static_cast<uint32_t>(numTicks);
}

float* physics_getBallPredictionPtr() {
    return reinterpret_cast<float*>(g_ballPredBuffer);
}

uint32_t physics_getBallPredictionCount() {
    return g_ballPredCount;
}

// Ball Sleeping State Inspection
int physics_isBallSleeping() {
    if (!g_arena || !g_arena->ball) return 1;
    return (g_arena->ball->_rigidBody.getActivationState() == ISLAND_SLEEPING) ? 1 : 0;
}

// Simulation Freeze & Monotonic Tick advance
void physics_setSimulationFrozen(int frozen, uint32_t targetResumeTick, int maskInputs) {
    g_simulationFrozen = (frozen != 0);
    g_unfreezeAtTick = targetResumeTick;
    g_maskInputs = (maskInputs != 0);
}

// Possession State Machine API
void physics_setPossessionReportingEnabled(int enabled) {
    g_possession.reportingEnabled = (enabled != 0);
}

void physics_getPossessionStats(int* outTeam, int* outBluePoints, int* outOrangePoints, float* outTime) {
    if (outTeam) *outTeam = g_possession.currentPossessionTeam;
    if (outBluePoints) *outBluePoints = g_possession.bluePoints;
    if (outOrangePoints) *outOrangePoints = g_possession.orangePoints;
    if (outTime) *outTime = g_possession.activePossessionTime;
}

// Signal Event Injection (Idempotent goal latch reset on Goal Replay Start / Kickoff Will Start)
void physics_pushSignalEvent(uint32_t type, uint32_t tick, float posX, float posY, float posZ,
                             float normX, float normY, float normZ, uint16_t primaryId,
                             uint16_t secondaryId, uint32_t subType, uint32_t flags,
                             float customFloat, uint32_t customInt) {
    if (type == SIGNAL_GOAL_REPLAY_START ||
        type == SIGNAL_KICKOFF_WILL_START ||
        type == SIGNAL_KICKOFF_START ||
        type == SIGNAL_BALL_RESET ||
        type == SIGNAL_MATCH_START) {
        resetPostGoalLatchState();
    }
    PhysicsEvent ev = {};
    ev.type = type;
    ev.tick = tick;
    ev.posX = posX;
    ev.posY = posY;
    ev.posZ = posZ;
    ev.normX = normX;
    ev.normY = normY;
    ev.normZ = normZ;
    ev.primaryId = primaryId;
    ev.secondaryId = secondaryId;
    ev.subType = subType;
    ev.flags = flags;
    ev.customFloat = customFloat;
    ev.customInt = customInt;
    PushPhysicsEvent(ev);
}

// 256MB Dedicated Server Replay Storage API
void physics_initReplayBuffer() {
    if (!g_replayBuffer) {
        g_replayBuffer = static_cast<uint8_t*>(std::malloc(REPLAY_STORAGE_BYTES));
        if (g_replayBuffer) {
            std::memset(g_replayBuffer, 0, sizeof(ReplayHeader));
            ReplayHeader* hdr = reinterpret_cast<ReplayHeader*>(g_replayBuffer);
            hdr->magic = 0x5245504C;
            hdr->version = 2;
            hdr->stateSize = sizeof(GameStateBufferPod);
            g_replayCursor = sizeof(ReplayHeader);
        }
    }
}

void physics_clearReplayBuffer() {
    if (g_replayBuffer) {
        std::memset(g_replayBuffer, 0, sizeof(ReplayHeader));
        ReplayHeader* hdr = reinterpret_cast<ReplayHeader*>(g_replayBuffer);
        hdr->magic = 0x5245504C;
        hdr->version = 2;
        hdr->stateSize = sizeof(GameStateBufferPod);
        g_replayCursor = sizeof(ReplayHeader);
    }
}

uint8_t* physics_getReplayBufferPtr() {
    return g_replayBuffer;
}

uint32_t physics_getReplayBufferSize() {
    return static_cast<uint32_t>(g_replayCursor);
}

void physics_recordReplayFrame() {
    if (!g_replayBuffer || !g_arena) return;
    ReplayHeader* hdr = reinterpret_cast<ReplayHeader*>(g_replayBuffer);
    if (g_replayCursor + sizeof(GameStateBufferPod) + sizeof(g_controlsBuffer) > REPLAY_STORAGE_BYTES) {
        return; // Full buffer protection
    }
    std::memcpy(g_replayBuffer + g_replayCursor, &g_state, sizeof(GameStateBufferPod));
    g_replayCursor += sizeof(GameStateBufferPod);
    hdr->totalFrames++;
}

// Zero-copy Event Ring Buffer Pointer & Size
void* physics_getEventBufferPtr() {
    return &g_eventBuffer;
}

int physics_getEventBufferSize() {
    return sizeof(PhysicsEventBuffer);
}

void physics_clearEvents() {
    g_eventBuffer.writeSeq = 0;
    g_eventBuffer.droppedCount = 0;
}

// State Buffer Pointers
float* physics_getStatePtr() {
    return g_stateBuffer;
}

int physics_getStateSize() {
    return STATE_BUFFER_SIZE;
}

float* physics_getControlsPtr() {
    return g_controlsBuffer;
}

float* physics_getPadInfoPtr() {
    return g_padInfoBuffer;
}

// Helper to resolve standard 6 Rocket League hitbox presets
static const CarConfig& getCarConfigForIndex(int hitboxIndex) {
    switch (hitboxIndex) {
        case 1: return CAR_CONFIG_DOMINUS;
        case 2: return CAR_CONFIG_PLANK;    // Batmobile (Plank)
        case 3: return CAR_CONFIG_BREAKOUT;
        case 4: return CAR_CONFIG_HYBRID;
        case 5: return CAR_CONFIG_MERC;
        case 0:
        default: return CAR_CONFIG_OCTANE;
    }
}

// Car Management
int physics_addCar(int team, int hitboxIndex) {
    if (!g_arena || g_cars.size() >= MAX_ARENA_CARS) return -1;

    const CarConfig& config = getCarConfigForIndex(hitboxIndex);

    Car* car = g_arena->AddCar(static_cast<Team>(team), config);
    if (!car) return -1;

    g_cars.push_back(car);
    syncStateBuffer();
    return static_cast<int>(g_cars.size() - 1);
}

int physics_removeCar(int carIndex) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size())) return 0;

    Car* car = g_cars[carIndex];
    if (!car) return 0;

    bool ok = g_arena->RemoveCar(car);
    g_cars.erase(g_cars.begin() + carIndex);
    syncStateBuffer();
    return ok ? 1 : 0;
}

int physics_setCarHitbox(int carIndex, int hitboxIndex) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size())) return 0;

    Car* oldCar = g_cars[carIndex];
    if (!oldCar) return 0;

    CarState state = oldCar->GetState();
    Team team = oldCar->team;

    // Remove old car rigid body from arena
    g_arena->RemoveCar(oldCar);

    // Create new car with desired hitbox configuration
    const CarConfig& config = getCarConfigForIndex(hitboxIndex);
    Car* newCar = g_arena->AddCar(team, config);
    if (!newCar) return 0;

    // Restore exact dynamic physical state (position, rotation, velocities, boost)
    newCar->SetState(state);
    g_arena->_bulletWorld.updateSingleAabb(&newCar->_rigidBody);
    g_cars[carIndex] = newCar;

    syncStateBuffer();
    return 1;
}

int physics_demolishCar(int carIndex, float respawnDelay) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size())) return 0;
    Car* car = g_cars[carIndex];
    if (!car) return 0;
    car->Demolish(respawnDelay > 0.0f ? respawnDelay : RLConst::DEMO_RESPAWN_TIME);
    syncStateBuffer();
    return 1;
}

int physics_respawnCar(int carIndex, int seed, float boostAmount) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size())) return 0;
    Car* car = g_cars[carIndex];
    if (!car) return 0;
    car->Respawn(g_arena->gameMode, seed, boostAmount >= 0.0f ? boostAmount : RLConst::BOOST_SPAWN_AMOUNT);
    g_arena->_bulletWorld.updateSingleAabb(&car->_rigidBody);
    syncStateBuffer();
    return 1;
}

int physics_getCarCount() {
    return static_cast<int>(g_cars.size());
}

void* physics_getCarConfig(int hitboxIndex) {
    return (void*)&getCarConfigForIndex(hitboxIndex);
}

void physics_setCarState(int carIndex, float* statePtr) {
    if (!g_arena || !statePtr || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size())) return;

    Car* car = g_cars[carIndex];
    CarState cs = car->GetState();

    cs.pos = Vec(statePtr[0], statePtr[1], statePtr[2]);
    cs.rotMat.forward = Vec(statePtr[3], statePtr[4], statePtr[5]);
    cs.rotMat.right = Vec(statePtr[6], statePtr[7], statePtr[8]);
    cs.rotMat.up = Vec(statePtr[9], statePtr[10], statePtr[11]);
    cs.vel = Vec(statePtr[12], statePtr[13], statePtr[14]);
    cs.angVel = Vec(statePtr[15], statePtr[16], statePtr[17]);
    cs.boost = statePtr[18];
    cs.isOnGround = statePtr[19] > 0.5f;
    cs.isSupersonic = statePtr[20] > 0.5f;
    cs.isDemoed = statePtr[21] > 0.5f;
    cs.isBoosting = statePtr[23] > 0.5f;
    cs.isFlipping = statePtr[24] > 0.5f;

    car->SetState(cs);
    if (g_carOverrides[carIndex].active) {
        g_carOverrides[carIndex].lockedOriginBT = car->_rigidBody.getWorldTransform().getOrigin();
    }
    g_arena->_bulletWorld.updateSingleAabb(&car->_rigidBody);
    syncStateBuffer();
}

void physics_setBallState(int carIndexOrNull, float* statePtr) {
    float* actualStatePtr = statePtr ? statePtr : reinterpret_cast<float*>(static_cast<uintptr_t>(carIndexOrNull));
    if (!g_arena || !g_arena->ball || !actualStatePtr) return;

    BallState bs = g_arena->ball->GetState();
    bs.pos = Vec(actualStatePtr[0], actualStatePtr[1], actualStatePtr[2]);
    bs.rotMat.forward = Vec(actualStatePtr[3], actualStatePtr[4], actualStatePtr[5]);
    bs.rotMat.right = Vec(actualStatePtr[6], actualStatePtr[7], actualStatePtr[8]);
    bs.rotMat.up = Vec(actualStatePtr[9], actualStatePtr[10], actualStatePtr[11]);
    bs.vel = Vec(actualStatePtr[12], actualStatePtr[13], actualStatePtr[14]);
    bs.angVel = Vec(actualStatePtr[15], actualStatePtr[16], actualStatePtr[17]);

    g_arena->ball->SetState(bs);
    g_arena->_bulletWorld.updateSingleAabb(&g_arena->ball->_rigidBody);
    if (!g_arena->IsBallScored()) {
        resetPostGoalLatchState();
    }
    syncStateBuffer();
}

void physics_setUnlimitedBoost(int unlimited) {
    g_unlimitedBoost = (unlimited != 0);
    if (g_arena) {
        g_arena->_mutatorConfig.boostUsedPerSecond = g_unlimitedBoost ? 0.0f : RLConst::BOOST_USED_PER_SECOND;
    }
}

int physics_getBallOnGround() {
    if (!g_arena || !g_arena->ball) return 0;
    return (g_arena->ball->GetState().pos.z <= RLConst::BALL_COLLISION_RADIUS_SOCCAR + 2.0f) ? 1 : 0;
}

float physics_getBallRadius() {
    return RLConst::BALL_COLLISION_RADIUS_SOCCAR;
}

void physics_clearGoalFlag() {
    g_goalScoredFlag = 0;
    g_state.header.goalScoredFlag = 0.0f;
}

int physics_controlBall(int carIndex, int modeIndex) {
    if (!g_arena || !g_arena->ball || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size())) return 0;

    Car* car = g_cars[carIndex];
    CarState cs = car->GetState();
    BallState bs = g_arena->ball->GetState();

    switch (modeIndex) {
    case 0:
        bs.pos = cs.pos + cs.rotMat.forward * 215.39f;
        bs.pos.z = 93.25f;
        bs.vel = Vec(0, 0, 0);
        bs.angVel = Vec(0, 0, 0);
        break;
    case 1:
        bs.pos = cs.pos + cs.rotMat.forward * 30.0f + Vec(0, 0, 141.33f);
        bs.vel = Vec(0, 0, 0);
        bs.angVel = Vec(0, 0, 0);
        break;
    case 2:
        bs.pos = Vec(0, 0, 93.15f);
        bs.vel = Vec(1735.35f, -2169.19f, 403.76f);
        bs.angVel = Vec(0, 0, 0);
        break;
    case 3:
        bs.pos = Vec(0, 0, 93.15f);
        bs.vel = Vec(0, 0, 750.0f);
        bs.angVel = Vec(0, 0, 0);
        break;
    default:
        return 0;
    }

    g_arena->ball->SetState(bs);
    g_arena->_bulletWorld.updateSingleAabb(&g_arena->ball->_rigidBody);
    if (!g_arena->IsBallScored()) {
        resetPostGoalLatchState();
    }
    syncStateBuffer();
    return 1;
}

void physics_initBallPrediction(float maxSeconds) {
    physics_updateBallPrediction(maxSeconds);
}

void physics_setSimControlFlags(uint32_t flags) {
    if (flags & 1) {
        physics_setSimulationFrozen(1, 0, 0);
    } else {
        physics_setSimulationFrozen(0, 0, 0);
    }
}

uint32_t physics_getSimControlFlags() {
    return g_simulationFrozen ? 1 : 0;
}

int physics_getBallMotionState() {
    return (g_arena && g_arena->ball) ? g_arena->ball->_rigidBody.getActivationState() : 0;
}

void physics_setPossessionEventEnabled(int enabled) {
    physics_setPossessionReportingEnabled(enabled);
}

int physics_getPossessionEventEnabled() {
    return g_possession.reportingEnabled ? 1 : 0;
}

int physics_getCurrentPossessionCar() {
    return g_possession.lastCarIndex;
}

int physics_getCurrentPossessionTeam() {
    return g_possession.currentPossessionTeam;
}

// ============================================================================
// PPO RLRL101 (obs_v3) Native Getters & Deterministic Kickoff Spawn Reset
// ============================================================================

int physics_getCarIsJumping(int carIndex) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size()) || !g_cars[carIndex]) return 0;
    return g_cars[carIndex]->_internalState.isJumping ? 1 : 0;
}

float physics_getCarAirTime(int carIndex) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size()) || !g_cars[carIndex]) return 0.0f;
    return g_cars[carIndex]->_internalState.airTime;
}

float physics_getCarJumpTime(int carIndex) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size()) || !g_cars[carIndex]) return 0.0f;
    return g_cars[carIndex]->_internalState.jumpTime;
}

float physics_getCarAirTimeSinceJump(int carIndex) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size()) || !g_cars[carIndex]) return 0.0f;
    return g_cars[carIndex]->_internalState.airTimeSinceJump;
}

int physics_resetKickoffAtSpawn(int spawnIndex, int settleTicks) {
    if (!g_arena) return 0;
    using namespace RLConst;

    int safeSpawnIdx = ((spawnIndex % CAR_SPAWN_LOCATION_AMOUNT) + CAR_SPAWN_LOCATION_AMOUNT) % CAR_SPAWN_LOCATION_AMOUNT;

    // Record existing cars (team + config) and recreate fresh Bullet vehicles for deterministic suspension state
    struct SavedCarSpec {
        Team team;
        CarConfig config;
    };
    std::vector<SavedCarSpec> specs;
    specs.reserve(g_cars.size());
    for (Car* car : g_cars) {
        if (car) {
            specs.push_back({ car->team, car->config });
            g_arena->RemoveCar(car);
        }
    }
    g_cars.clear();
    g_arena->_lastCarID = 0;

    int blueIdx = 0;
    int orangeIdx = 0;
    for (size_t i = 0; i < specs.size(); i++) {
        Car* newCar = g_arena->AddCar(specs[i].team, specs[i].config);
        if (!newCar) continue;

        bool isBlue = (specs[i].team == Team::BLUE);
        int teamSlot = isBlue ? (blueIdx++) : (orangeIdx++);
        int locIdx = (safeSpawnIdx + teamSlot) % CAR_SPAWN_LOCATION_AMOUNT;
        CarSpawnPos spawnPos = CAR_SPAWN_LOCATIONS_SOCCAR[locIdx];

        CarState spawnState;
        spawnState.boost = g_arena->_mutatorConfig.carSpawnBoostAmount;
        spawnState.pos = Vec(spawnPos.x, spawnPos.y, CAR_SPAWN_REST_Z);
        Angle angle = Angle(spawnPos.yawAng, 0.0f, 0.0f);
        spawnState.isOnGround = true;

        if (!isBlue) {
            spawnState.pos *= Vec(-1.0f, -1.0f, 1.0f);
            angle.yaw += M_PI;
        }
        spawnState.rotMat = angle.ToRotMat();
        newCar->SetState(spawnState);
        newCar->controls = CarControls();
        g_arena->_bulletWorld.updateSingleAabb(&newCar->_rigidBody);
        g_cars.push_back(newCar);
    }

    // Clear controls buffer
    std::memset(g_controlsBuffer, 0, sizeof(g_controlsBuffer));

    // Reset ball and boost pads
    if (g_arena->ball) {
        g_arena->ball->SetState(BallState());
        g_arena->_bulletWorld.updateSingleAabb(&g_arena->ball->_rigidBody);
    }
    for (BoostPad* pad : g_arena->GetBoostPads()) {
        pad->SetState(BoostPadState());
    }

    // Clear persistent collision manifolds and solver cache
    btCollisionDispatcher* dispatcher = (btCollisionDispatcher*)g_arena->_bulletWorld.getDispatcher();
    if (dispatcher) {
        int numManifolds = dispatcher->getNumManifolds();
        for (int m = 0; m < numManifolds; m++) {
            btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(m);
            if (manifold) manifold->clearManifold();
        }
    }
    if (g_arena->_bulletWorld.getConstraintSolver()) {
        g_arena->_bulletWorld.getConstraintSolver()->reset();
    }

    // Step settleTicks silently to allow suspension to settle, then zero velocities
    if (settleTicks > 0) {
        bool prevSilent = g_silentResim;
        g_silentResim = true;
        g_arena->Step(settleTicks);
        g_silentResim = prevSilent;

        for (Car* car : g_cars) {
            if (!car) continue;
            CarState settled = car->GetState();
            settled.vel = Vec(0.0f, 0.0f, 0.0f);
            settled.angVel = Vec(0.0f, 0.0f, 0.0f);
            car->SetState(settled);
            car->_rigidBody.clearForces();
            g_arena->_bulletWorld.updateSingleAabb(&car->_rigidBody);
        }
        if (g_arena->ball) {
            g_arena->ball->SetState(BallState());
            g_arena->ball->_rigidBody.clearForces();
            g_arena->ball->_rigidBody.setActivationState(ISLAND_SLEEPING);
            g_arena->_bulletWorld.updateSingleAabb(&g_arena->ball->_rigidBody);
        }
    }

    g_arena->tickCount = 0;
    resetPostGoalLatchState();
    g_lastBallWorldHitTick = 0;
    g_firstTouchFired = false;
    g_lastBallSleeping = true;
    std::memset(&g_possession, 0, sizeof(PossessionState));
    for (int p = 0; p < NUM_ARENA_PADS; p++) g_wasPadActive[p] = true;
    g_possession.reportingEnabled = true;

    for (int i = 0; i < MAX_ARENA_CARS; i++) {
        g_carLastBallHitTick[i] = 0;
        g_carTrackers[i] = CarActionTracker();
    }

    syncStateBuffer();
    return 1;
}

// ============================================================================
// Stage 2 Multi-Team Extensions: LinearFactor/Gravity, CollisionMask, Mutators, Replay Offset
// ============================================================================

int physics_setCarLinearFactorAndGravity(int carIndex, float fx, float fy, float fz, float gravityScale) {
    if (!g_arena || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size()) || !g_cars[carIndex]) return 0;

    Car* car = g_cars[carIndex];
    CarPhysicsOverride& ov = g_carOverrides[carIndex];
    ov.linearFactor = btVector3(fx, fy, fz);
    ov.gravityScale = gravityScale;
    ov.lockedOriginBT = car->_rigidBody.getWorldTransform().getOrigin();
    ov.active = !(fx == 1.0f && fy == 1.0f && fz == 1.0f && gravityScale == 1.0f);

    car->_rigidBody.setLinearFactor(ov.linearFactor);
    car->_rigidBody.setGravity(g_arena->_mutatorConfig.gravity * UU_TO_BT * gravityScale);
    btVector3 vel = car->_rigidBody.getLinearVelocity() * ov.linearFactor;
    car->_rigidBody.setLinearVelocity(vel);
    car->_rigidBody.setInterpolationLinearVelocity(vel);
    syncStateBuffer();
    return 1;
}

int physics_setCarCollisionMask(int carIndex, int interactWithBall, int interactWithCars) {
    if (!g_arena || carIndex < 0 || carIndex >= MAX_ARENA_CARS) return 0;

    g_carCollisionMasks[carIndex].interactWithBall = (interactWithBall != 0);
    g_carCollisionMasks[carIndex].interactWithCars = (interactWithCars != 0);

    g_hasCustomCollisionMask = false;
    for (int i = 0; i < MAX_ARENA_CARS; i++) {
        if (!g_carCollisionMasks[i].interactWithBall || !g_carCollisionMasks[i].interactWithCars) {
            g_hasCustomCollisionMask = true;
            break;
        }
    }

    if (carIndex < static_cast<int>(g_cars.size()) && g_cars[carIndex]) {
        Car* car = g_cars[carIndex];
        if (car->_rigidBody.getBroadphaseProxy() && g_arena->_bulletWorldParams.overlappingPairCache) {
            g_arena->_bulletWorldParams.overlappingPairCache->cleanProxyFromPairs(
                car->_rigidBody.getBroadphaseProxy(),
                g_arena->_bulletWorld.getDispatcher()
            );
        }
    }
    return 1;
}

int physics_setArenaMutators(float gravityZ, float boostAccelGround, float boostAccelAir, float ballMaxSpeed, float ballBounciness) {
    if (!g_arena) return 0;
    MutatorConfig mc = g_arena->GetMutatorConfig();
    mc.gravity = Vec(0.0f, 0.0f, gravityZ);
    if (boostAccelGround > 0.0f) mc.boostAccelGround = boostAccelGround;
    if (boostAccelAir > 0.0f) mc.boostAccelAir = boostAccelAir;
    if (ballMaxSpeed > 0.0f) mc.ballMaxSpeed = ballMaxSpeed;
    if (ballBounciness >= 0.0f) mc.ballWorldRestitution = ballBounciness;
    g_arena->SetMutatorConfig(mc);

    // Re-apply per-car gravity scale overrides
    for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
        if (g_carOverrides[i].active && g_cars[i]) {
            g_cars[i]->_rigidBody.setGravity(g_arena->_mutatorConfig.gravity * UU_TO_BT * g_carOverrides[i].gravityScale);
        }
    }
    return 1;
}

uint32_t physics_getReplayFrameCount() {
    if (!g_replayBuffer) return 0;
    const ReplayHeader* hdr = reinterpret_cast<const ReplayHeader*>(g_replayBuffer);
    return hdr->totalFrames;
}

float* physics_getReplayFramePtrByOffset(int ticksAgo) {
    if (!g_replayBuffer) return nullptr;
    const ReplayHeader* hdr = reinterpret_cast<const ReplayHeader*>(g_replayBuffer);
    if (hdr->totalFrames == 0) return nullptr;
    int targetIdx = static_cast<int>(hdr->totalFrames) - 1 - std::max(0, ticksAgo);
    targetIdx = std::clamp(targetIdx, 0, static_cast<int>(hdr->totalFrames) - 1);
    size_t byteOffset = sizeof(ReplayHeader) + static_cast<size_t>(targetIdx) * sizeof(GameStateBufferPod);
    if (byteOffset + sizeof(GameStateBufferPod) > REPLAY_STORAGE_BYTES) return nullptr;
    return reinterpret_cast<float*>(g_replayBuffer + byteOffset);
}

float* physics_getReplayFramePtrByIndex(int frameIndex) {
    if (!g_replayBuffer) return nullptr;
    const ReplayHeader* hdr = reinterpret_cast<const ReplayHeader*>(g_replayBuffer);
    if (hdr->totalFrames == 0) return nullptr;
    int targetIdx = std::clamp(frameIndex, 0, static_cast<int>(hdr->totalFrames) - 1);
    size_t byteOffset = sizeof(ReplayHeader) + static_cast<size_t>(targetIdx) * sizeof(GameStateBufferPod);
    if (byteOffset + sizeof(GameStateBufferPod) > REPLAY_STORAGE_BYTES) return nullptr;
    return reinterpret_cast<float*>(g_replayBuffer + byteOffset);
}

} // extern "C" 
