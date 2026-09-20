/**
 * RocketSimBridge.cpp
 * Standard C-ABI WebAssembly Bridge for RocketSim Physics Engine.
 * 
 * Features:
 * - 64-byte CPU cache-line aligned Event Buffer (alignas(64))
 * - Unified Incident (0x01..0x7F) & Signal (0x80..0xFF) Event Schema
 * - Up to 5.0s (600 ticks) Native Ball Prediction Interface
 * - Normal Relative Velocity (Delta_Vn) & Dynamic Contact Thresholds
 * - Simulation Control Flags (Tick Increment without Physics Evaluation)
 * - Native Car Flip Reset Gained (Ball Contact Dedicated State Machine)
 * - Kickoff First Touch & Ball Motion State Tracking
 * - Ball Possession Engine with Configurable Event Emission
 * - Snapshot-based Deterministic Rollback Architecture
 */

#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>
#include <map>
#include <algorithm>

#include "RocketSim.h"
#include "Sim/Arena/Arena.h"
#include "Sim/Car/Car.h"
#include "Sim/Ball/Ball.h"
#include "Sim/BallPredTracker/BallPredTracker.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

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
    ArenaHeaderPod header;
    BallStatePod ball;
    CarStatePod cars[MAX_ARENA_CARS];
    BoostPadStatePod pads[NUM_ARENA_PADS];
    float reserved[14];
};
#pragma pack(pop)

// =============================================================================
// Unified 64-Byte CPU Cache-Line Aligned Event Architecture (Incidents & Signals)
// =============================================================================

// Incident Events (Physics & Gameplay Kernel)
static constexpr uint32_t EVENT_TYPE_NONE                 = 0x00;
static constexpr uint32_t INCIDENT_CAR_BALL_HIT           = 0x01;
static constexpr uint32_t INCIDENT_CAR_CAR_COLLISION      = 0x02;
static constexpr uint32_t INCIDENT_BALL_WORLD_HIT         = 0x03;
static constexpr uint32_t INCIDENT_BALL_GOALPOST_HIT      = 0x04;
static constexpr uint32_t INCIDENT_CAR_ACTION             = 0x05;
static constexpr uint32_t INCIDENT_CAR_SUPERSONIC_ENTER   = 0x06;
static constexpr uint32_t INCIDENT_BOOST_PICKUP           = 0x07;
static constexpr uint32_t INCIDENT_BOOST_RESPAWN          = 0x08;
static constexpr uint32_t INCIDENT_GOAL_SCORED            = 0x09;
static constexpr uint32_t INCIDENT_FLIP_RESET_GAINED      = 0x0A;
static constexpr uint32_t INCIDENT_KICKOFF_FIRST_TOUCH    = 0x0B;
static constexpr uint32_t INCIDENT_CAR_CAR_DEMO           = 0x0C;
static constexpr uint32_t INCIDENT_POSSESSION_CHANGED     = 0x0D;

// Signal Events (External Session & Match Management)
static constexpr uint32_t SIGNAL_MATCH_STARTED            = 0x80;
static constexpr uint32_t SIGNAL_KICKOFF_COUNTDOWN        = 0x81;
static constexpr uint32_t SIGNAL_KICKOFF_GO               = 0x82;
static constexpr uint32_t SIGNAL_SCHEDULE_KICKOFF         = 0x83;
static constexpr uint32_t SIGNAL_SCHEDULE_GOAL_REPLAY     = 0x84;
static constexpr uint32_t SIGNAL_GOAL_REPLAY_START        = 0x85;
static constexpr uint32_t SIGNAL_MATCH_PAUSE              = 0x86;
static constexpr uint32_t SIGNAL_SCHEDULE_RESUME          = 0x87;
static constexpr uint32_t SIGNAL_TIME_ALERT_60S           = 0x88;
static constexpr uint32_t SIGNAL_TIME_ALERT_30S           = 0x89;
static constexpr uint32_t SIGNAL_OVERTIME_START           = 0x8A;
static constexpr uint32_t SIGNAL_PLAYER_SPAWN             = 0x8B;
static constexpr uint32_t SIGNAL_PLAYER_DESPAWN           = 0x8C;
static constexpr uint32_t SIGNAL_FORFEIT_VOTE             = 0x8D;

// Subtypes & Flags
static constexpr uint32_t CAR_ACTION_SINGLE_JUMP = 1;
static constexpr uint32_t CAR_ACTION_DOUBLE_JUMP = 2;
static constexpr uint32_t CAR_ACTION_DODGE       = 3;

static constexpr uint32_t SIM_FLAG_NORMAL         = 0;
static constexpr uint32_t SIM_FLAG_SUPPRESS_INPUT = 1 << 0; // Freeze car controls & engage handbrake
static constexpr uint32_t SIM_FLAG_FREEZE_PHYSICS = 1 << 1; // Increment tickCount without Bullet step
static constexpr uint32_t SIM_FLAG_HOLD_BALL      = 1 << 2; // Clamp ball velocity to 0

// Strictly 64-Byte POD Layout (alignas(64))
#pragma pack(push, 4)
struct alignas(64) CoreEvent {
    uint32_t type;          // [Word 0]  Event type (0x01..0x7F: Incident, 0x80..0xFF: Signal)
    uint32_t tick;          // [Word 1]  Physics tick timestamp / Frame ID
    float posX, posY, posZ; // [Word 2..4] 3D world position (Unreal Units)
    float normX, normY, normZ; // [Word 5..7] 3D contact normal / broadcast direction
    float normalRelVel;     // [Word 8]  Normal relative velocity (Delta_Vn) or countdown parameter
    float speed;            // [Word 9]  Speed magnitude
    float impulse;          // [Word 10] Applied impulse / possession duration
    uint16_t primaryId;     // [Word 11, Low 16]  Car index (0..7) or scoring team
    uint16_t secondaryId;   // [Word 11, High 16] Victim car / Pad index / Dict player ID
    uint32_t subType;       // [Word 12] Subcategory / Surface tag / Pause mode
    uint32_t flags;         // [Word 13] Bitmask attributes (Initial touch, Demo, etc.)
    uint32_t scheduledTick; // [Word 14] Scheduled future target frame ID
    uint32_t customInt;     // [Word 15] Extended payload / voting count
};
#pragma pack(pop)
static_assert(sizeof(CoreEvent) == 64, "CoreEvent must be strictly 64 bytes");

static constexpr uint32_t EVENT_RING_BUFFER_CAPACITY = 256;

#pragma pack(push, 4)
struct CoreEventBuffer {
    uint32_t writeSeq;     // Monotonically increasing write sequence counter
    uint32_t capacity;     // 256
    uint32_t eventSize;    // 64 bytes
    uint32_t reserved;     // 0
    CoreEvent events[EVENT_RING_BUFFER_CAPACITY];
};
#pragma pack(pop)
static_assert(sizeof(CoreEventBuffer) == 16 + EVENT_RING_BUFFER_CAPACITY * 64, "CoreEventBuffer size must be 16400 bytes");

static constexpr int STATE_BUFFER_SIZE = sizeof(GameStateBufferPod) / sizeof(float);

// Shared memory buffers mapped to WebAssembly heap
static GameStateBufferPod g_state;
static float* g_stateBuffer = reinterpret_cast<float*>(&g_state);
static float g_controlsBuffer[MAX_ARENA_CARS * CONTROLS_STRIDE]; // 64 floats
static float g_padInfoBuffer[NUM_ARENA_PADS * 4];                 // 136 floats

// Global Ring Buffer for zero-copy physics events (256 slots = 16KB)
static CoreEventBuffer g_eventBuffer = { 0, EVENT_RING_BUFFER_CAPACITY, sizeof(CoreEvent), 0 };

// Silent resimulation mode flag (suppresses event emission during rollback replay)
static bool g_silentResim = false;

static void PushCoreEvent(const CoreEvent& ev) {
    if (g_silentResim) return;
    uint32_t slot = g_eventBuffer.writeSeq % EVENT_RING_BUFFER_CAPACITY;
    g_eventBuffer.events[slot] = ev;
    g_eventBuffer.writeSeq++;
}

// Active Arena, cars and simulation state
static void syncStateBuffer();
static Arena* g_arena = nullptr;
static std::vector<Car*> g_cars;
static int g_goalScoredFlag = 0;
static bool g_unlimitedBoost = false;

// Dynamic collision thresholds & normal relative velocity (Delta_Vn)
static float g_thresholdBallGround = 120.0f;
static float g_thresholdBallWall = 100.0f;
static float g_thresholdCarBall = 40.0f;
static uint32_t g_cooldownTicks = 4;
static uint64_t g_lastBallWorldHitTick = 0;

// Simulation control flags
static uint32_t g_simControlFlags = SIM_FLAG_NORMAL;

// Ball Prediction Engine (up to 5.0s / 600 ticks @ 120Hz)
static constexpr int MAX_BALL_PRED_TICKS = 600;
static BallPredTracker* g_ballPredTracker = nullptr;
static float g_ballPredBuffer[MAX_BALL_PRED_TICKS * 6]; // 3600 floats (posX, posY, posZ, velX, velY, velZ)
static int g_ballPredCount = 0;

// Kickoff First Touch & Ball Motion State Tracking
static bool g_isAwaitingFirstTouch = false;

// Ball Possession Engine State
static bool g_possessionEventEnabled = true;
static int g_currentPossessionCar = -1;
static int g_currentPossessionTeam = -1;
static uint64_t g_possessionStartTick = 0;

// Car Flip Reset & Aerial State Tracking
struct CarActionTracker {
    bool wasJumping = false;
    bool hadDoubleJumped = false;
    bool wasFlipping = false;
    bool wasSupersonic = false;
    bool holdsUnlimitedFlip = false;
    uint64_t lastBallHitTick = 0;
};
static CarActionTracker g_carActionTrackers[MAX_ARENA_CARS];

// Boost Pad State Tracking
static bool g_wasPadActive[NUM_ARENA_PADS] = {};

// Snapshot Definitions for Deterministic Rollback
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
    CarSnapshotPod cars[MAX_ARENA_CARS];
    btTransform ballRbTransform;
    btVector3 ballRbLinearVelocity;
    btVector3 ballRbAngularVelocity;
    BallState ballState;
    BoostPadSnapshotPod pads[NUM_ARENA_PADS];
};
#pragma pack(pop)

static constexpr int MAX_SNAPSHOT_SLOTS = 16;
static ArenaSnapshotPod g_snapshotSlots[MAX_SNAPSHOT_SLOTS];

static void serializeToSnapshot(ArenaSnapshotPod& snap) {
    if (!g_arena) return;
    snap.tickCount = g_arena->tickCount;
    snap.lastCarID = g_arena->_lastCarID;
    snap.goalScoredFlag = g_goalScoredFlag;
    snap.numCars = static_cast<uint32_t>(std::min(g_cars.size(), static_cast<size_t>(MAX_ARENA_CARS)));

    if (g_arena->ball) {
        snap.ballRbTransform = g_arena->ball->_rigidBody.getWorldTransform();
        snap.ballRbLinearVelocity = g_arena->ball->_rigidBody.getLinearVelocity();
        snap.ballRbAngularVelocity = g_arena->ball->_rigidBody.getAngularVelocity();
        snap.ballState = g_arena->ball->GetState();
    }

    for (uint32_t i = 0; i < snap.numCars; i++) {
        Car* car = g_cars[i];
        snap.cars[i].team = static_cast<uint32_t>(car->team);
        snap.cars[i].id = car->id;
        snap.cars[i].controls = car->controls;
        snap.cars[i].state = car->GetState();
        snap.cars[i].rbTransform = car->_rigidBody.getWorldTransform();
        snap.cars[i].rbLinearVelocity = car->_rigidBody.getLinearVelocity();
        snap.cars[i].rbAngularVelocity = car->_rigidBody.getAngularVelocity();
        int numWheels = std::min(4, car->_bulletVehicle.getNumWheels());
        for (int w = 0; w < numWheels; w++) {
            snap.cars[i].wheels[w] = car->_bulletVehicle.getWheelInfo(w);
        }
    }

    for (int i = 0; i < NUM_ARENA_PADS && i < static_cast<int>(g_arena->_boostPads.size()); i++) {
        BoostPad* pad = g_arena->_boostPads[i];
        BoostPadState ps = pad->GetState();
        snap.pads[i].isActive = ps.isActive ? 1 : 0;
        snap.pads[i].cooldown = ps.cooldown;
        snap.pads[i].prevLockedCarID = ps.prevLockedCarID;
    }
}

static void deserializeFromSnapshot(const ArenaSnapshotPod& snap) {
    if (!g_arena) return;
    g_arena->tickCount = snap.tickCount;
    g_arena->_lastCarID = snap.lastCarID;
    g_goalScoredFlag = snap.goalScoredFlag;
    g_state.header.goalScoredFlag = static_cast<float>(g_goalScoredFlag);

    if (g_arena->ball) {
        g_arena->ball->_rigidBody.setWorldTransform(snap.ballRbTransform);
        g_arena->ball->_rigidBody.setLinearVelocity(snap.ballRbLinearVelocity);
        g_arena->ball->_rigidBody.setAngularVelocity(snap.ballRbAngularVelocity);
        g_arena->ball->SetState(snap.ballState);
    }

    size_t numCars = std::min(g_cars.size(), static_cast<size_t>(snap.numCars));
    for (size_t i = 0; i < numCars; i++) {
        Car* car = g_cars[i];
        car->controls = snap.cars[i].controls;
        car->SetState(snap.cars[i].state);
        car->_rigidBody.setWorldTransform(snap.cars[i].rbTransform);
        car->_rigidBody.setLinearVelocity(snap.cars[i].rbLinearVelocity);
        car->_rigidBody.setAngularVelocity(snap.cars[i].rbAngularVelocity);

        int numWheels = std::min(4, car->_bulletVehicle.getNumWheels());
        for (int w = 0; w < numWheels; w++) {
            car->_bulletVehicle.getWheelInfo(w) = snap.cars[i].wheels[w];
            car->_bulletVehicle.updateWheelTransformsWS(car->_bulletVehicle.getWheelInfo(w));
            car->_bulletVehicle.updateWheelTransform(w);
        }
    }

    for (int i = 0; i < NUM_ARENA_PADS && i < static_cast<int>(g_arena->_boostPads.size()); i++) {
        BoostPad* pad = g_arena->_boostPads[i];
        BoostPadState ps;
        ps.isActive = (snap.pads[i].isActive != 0);
        ps.cooldown = snap.pads[i].cooldown;
        ps.prevLockedCarID = snap.pads[i].prevLockedCarID;
        pad->SetState(ps);
    }

    // Reset Bullet collision manifold cache on rollback
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
    g_goalScoredFlag = (scoringTeam == Team::BLUE) ? 1 : 2;
    g_state.header.goalScoredFlag = static_cast<float>(g_goalScoredFlag);

    if (!g_silentResim && arena->ball) {
        BallState bs = arena->ball->GetState();
        CoreEvent ev = {};
        ev.type = INCIDENT_GOAL_SCORED;
        ev.tick = static_cast<uint32_t>(arena->tickCount > 0 ? arena->tickCount - 1 : 0);
        ev.posX = bs.pos.x;
        ev.posY = bs.pos.y;
        ev.posZ = bs.pos.z;
        ev.speed = bs.vel.Length();
        ev.primaryId = static_cast<uint16_t>(scoringTeam == Team::BLUE ? 0 : 1);
        ev.secondaryId = 0; // Ball
        ev.subType = static_cast<uint32_t>(scoringTeam);
        PushCoreEvent(ev);
    }
}

// Car-Car collision & Demo callback
static void onCarBumpCallback(Arena* arena, Car* bumper, Car* victim, bool isDemo, const Vec& contactPos, float relSpeed, float impulse, void* userInfo) {
    if (g_silentResim) return;
    CoreEvent ev = {};
    ev.type = isDemo ? INCIDENT_CAR_CAR_DEMO : INCIDENT_CAR_CAR_COLLISION;
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
    ev.subType = static_cast<uint32_t>((bumper && bumper->team == Team::ORANGE) ? 1 : 0);
    ev.flags = isDemo ? 1 : 0;
    PushCoreEvent(ev);
}

// Ball-World & Goalpost collision callback with Normal Relative Velocity (Delta_Vn)
static void onBallWorldCallback(Arena* arena, const Vec& contactPos, const Vec& normal, float speed, bool isGoalpost, void* userInfo) {
    if (g_silentResim || !arena->ball) return;
    uint64_t curTick = arena->tickCount;

    // Normal Relative Velocity: Delta_Vn = -v_ball dot n_contact (stationary arena surface)
    BallState bs = arena->ball->GetState();
    float deltaVn = -(bs.vel.x * normal.x + bs.vel.y * normal.y + bs.vel.z * normal.z);

    // Suppress rolling/sliding contact when Delta_Vn is below threshold
    float threshold = isGoalpost ? 80.0f : ((contactPos.z < 25.0f) ? g_thresholdBallGround : g_thresholdBallWall);
    if (deltaVn < threshold || curTick <= g_lastBallWorldHitTick + g_cooldownTicks) {
        return;
    }
    g_lastBallWorldHitTick = curTick;

    CoreEvent ev = {};
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
    ev.primaryId = 0; // Ball ID

    if (isGoalpost) {
        bool isCrossbar = (std::abs(contactPos.z - 642.0f) < 80.0f) && (std::abs(contactPos.x) <= 950.0f);
        ev.subType = isCrossbar ? 2 : 1; // 1: vertical post, 2: crossbar
    } else {
        // Classify surface: 1: ground, 2: side/back wall, 3: ceiling, 4: ramp/corner
        if (contactPos.z < 25.0f) {
            ev.subType = 1; // ground
        } else if (contactPos.z > 2000.0f) {
            ev.subType = 3; // ceiling
        } else {
            ev.subType = 2; // wall
        }
    }

    PushCoreEvent(ev);
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

    // Cars state
    int numCars = static_cast<int>(std::min(static_cast<size_t>(MAX_ARENA_CARS), g_cars.size()));
    for (int i = 0; i < numCars; i++) {
        Car* car = g_cars[i];
        CarState cs = car->GetState();
        CarStatePod& cPod = g_state.cars[i];

        cPod.posX = cs.pos.x;
        cPod.posY = cs.pos.y;
        cPod.posZ = cs.pos.z;

        cPod.rotFwdX = cs.rotMat.forward.x;
        cPod.rotFwdY = cs.rotMat.forward.y;
        cPod.rotFwdZ = cs.rotMat.forward.z;

        cPod.rotRightX = cs.rotMat.right.x;
        cPod.rotRightY = cs.rotMat.right.y;
        cPod.rotRightZ = cs.rotMat.right.z;

        cPod.rotUpX = cs.rotMat.up.x;
        cPod.rotUpY = cs.rotMat.up.y;
        cPod.rotUpZ = cs.rotMat.up.z;

        cPod.velX = cs.vel.x;
        cPod.velY = cs.vel.y;
        cPod.velZ = cs.vel.z;

        cPod.angVelX = cs.angVel.x;
        cPod.angVelY = cs.angVel.y;
        cPod.angVelZ = cs.angVel.z;

        cPod.boost = cs.boost;
        cPod.isOnGround = cs.isOnGround ? 1.0f : 0.0f;
        cPod.isSupersonic = cs.isSupersonic ? 1.0f : 0.0f;
        cPod.isDemoed = cs.isDemoed ? 1.0f : 0.0f;
        cPod.hasFlipOrJump = cs.HasFlipOrJump() ? 1.0f : 0.0f;
        cPod.isBoosting = car->controls.boost ? 1.0f : 0.0f;
        cPod.isFlipping = cs.isFlipping ? 1.0f : 0.0f;

        int numWheels = std::min(4, car->_bulletVehicle.getNumWheels());
        for (int w = 0; w < numWheels; w++) {
            const btWheelInfoRL& wi = car->_bulletVehicle.getWheelInfo(w);
            cPod.wheels[w].susLength = wi.m_raycastInfo.m_suspensionLength;
            cPod.wheels[w].steerAngle = wi.m_steerAngle;
            cPod.wheels[w].hasContact = wi.m_raycastInfo.m_isInContact ? 1.0f : 0.0f;
        }

        cPod.groundNormalX = cs.worldContact.hasContact ? cs.worldContact.contactNormal.x : 0.0f;
        cPod.groundNormalY = cs.worldContact.hasContact ? cs.worldContact.contactNormal.y : 0.0f;
        cPod.groundNormalZ = cs.worldContact.hasContact ? cs.worldContact.contactNormal.z : 1.0f;
    }

    // Pads state
    int numPads = static_cast<int>(std::min(static_cast<size_t>(NUM_ARENA_PADS), g_arena->_boostPads.size()));
    for (int i = 0; i < numPads; i++) {
        BoostPad* pad = g_arena->_boostPads[i];
        BoostPadState ps = pad->GetState();
        g_state.pads[i].isActive = ps.isActive ? 1.0f : 0.0f;
        g_state.pads[i].cooldown = ps.cooldown;
    }
}

static void setupPadInfoBuffer() {
    if (!g_arena) return;
    for (int i = 0; i < NUM_ARENA_PADS && i < static_cast<int>(g_arena->_boostPads.size()); i++) {
        BoostPad* pad = g_arena->_boostPads[i];
        Vec pos = pad->config.pos;
        g_padInfoBuffer[i * 4 + 0] = pos.x;
        g_padInfoBuffer[i * 4 + 1] = pos.y;
        g_padInfoBuffer[i * 4 + 2] = pos.z;
        g_padInfoBuffer[i * 4 + 3] = pad->config.isBig ? 1.0f : 0.0f;
    }
}

// =============================================================================
// Internal Physics Step & Event Detection Pipeline
// =============================================================================

static void detectAndPushPhysicsIncidents(uint32_t currentTick) {
    if (!g_arena || g_silentResim) return;

    BallState bs = g_arena->ball ? g_arena->ball->GetState() : BallState();

    // 1. Car-Ball Hits, Flip Resets & Car Actions
    for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
        Car* car = g_cars[i];
        CarState cs = car->GetState();
        CarActionTracker& tracker = g_carActionTrackers[i];

        // A. Car-Ball Hit Detection
        if (cs.ballHitInfo.isValid && cs.ballHitInfo.tickCountWhenHit == currentTick) {
            tracker.lastBallHitTick = currentTick;

            // Compute relative speed and normal relative velocity
            Vec relVel = cs.vel - bs.vel;
            Vec hitNormal = (bs.pos - cs.pos).Normalized();
            float deltaVn = -(bs.vel.x * hitNormal.x + bs.vel.y * hitNormal.y + bs.vel.z * hitNormal.z);

            CoreEvent hitEv = {};
            hitEv.type = INCIDENT_CAR_BALL_HIT;
            hitEv.tick = currentTick;
            hitEv.posX = cs.ballHitInfo.ballPos.x;
            hitEv.posY = cs.ballHitInfo.ballPos.y;
            hitEv.posZ = cs.ballHitInfo.ballPos.z;
            hitEv.normX = hitNormal.x;
            hitEv.normY = hitNormal.y;
            hitEv.normZ = hitNormal.z;
            hitEv.normalRelVel = deltaVn;
            hitEv.speed = relVel.Length();
            hitEv.impulse = cs.ballHitInfo.extraHitVel.Length();
            hitEv.primaryId = static_cast<uint16_t>(i);
            hitEv.secondaryId = 0; // Ball ID
            hitEv.subType = static_cast<uint32_t>(car->team);

            // Kickoff first touch check
            if (g_isAwaitingFirstTouch) {
                hitEv.flags |= 1; // Bit 0: Initial Touch
                g_isAwaitingFirstTouch = false;

                // Also push dedicated first touch incident
                CoreEvent ftEv = hitEv;
                ftEv.type = INCIDENT_KICKOFF_FIRST_TOUCH;
                PushCoreEvent(ftEv);
            }
            PushCoreEvent(hitEv);

            // Ball Possession Shift on Hit
            if (static_cast<int>(i) != g_currentPossessionCar) {
                uint64_t dur = (currentTick > g_possessionStartTick) ? (currentTick - g_possessionStartTick) : 0;
                if (g_possessionEventEnabled) {
                    CoreEvent possEv = {};
                    possEv.type = INCIDENT_POSSESSION_CHANGED;
                    possEv.tick = currentTick;
                    possEv.posX = bs.pos.x; possEv.posY = bs.pos.y; possEv.posZ = bs.pos.z;
                    possEv.primaryId = static_cast<uint16_t>(i);
                    possEv.secondaryId = static_cast<uint16_t>(g_currentPossessionCar >= 0 ? g_currentPossessionCar : 0xFFFF);
                    possEv.subType = static_cast<uint32_t>(car->team);
                    possEv.impulse = static_cast<float>(dur);
                    possEv.normalRelVel = static_cast<float>(dur) / 120.0f;
                    possEv.flags = 1; // Reason: direct touch
                    PushCoreEvent(possEv);
                }
                g_currentPossessionCar = static_cast<int>(i);
                g_currentPossessionTeam = static_cast<int>(car->team);
                g_possessionStartTick = currentTick;
            }
        }

        // B. Flip Reset Dedicated State Machine (Strictly Ball Contact)
        bool nowUnlimited = cs.HasFlipReset();
        if (!tracker.holdsUnlimitedFlip && nowUnlimited) {
            // Qualify contact source: check if wheels contacted the ball
            bool ballContact = false;
            int ballWheelContacts = 0;
            if (g_arena->ball) {
                const btCollisionObject* ballBody = (const btCollisionObject*)&g_arena->ball->_rigidBody;
                int numWheels = std::min(4, car->_bulletVehicle.getNumWheels());
                for (int w = 0; w < numWheels; w++) {
                    const auto& ray = car->_bulletVehicle.getWheelInfo(w).m_raycastInfo;
                    if (ray.m_isInContact && ray.m_groundObject == ballBody) {
                        ballWheelContacts++;
                    }
                }
            }

            if (ballWheelContacts >= 2 || (cs.ballHitInfo.isValid && (currentTick - cs.ballHitInfo.tickCountWhenHit <= 2))) {
                ballContact = true;
            }

            if (ballContact) {
                CoreEvent frEv = {};
                frEv.type = INCIDENT_FLIP_RESET_GAINED;
                frEv.tick = currentTick;
                frEv.posX = cs.pos.x;
                frEv.posY = cs.pos.y;
                frEv.posZ = cs.pos.z;
                frEv.primaryId = static_cast<uint16_t>(i);
                frEv.secondaryId = 0; // Ball ID
                frEv.subType = static_cast<uint32_t>(car->team);
                PushCoreEvent(frEv);
            }
        }
        tracker.holdsUnlimitedFlip = nowUnlimited;
        if (cs.isOnGround) {
            tracker.holdsUnlimitedFlip = false;
        }

        // C. Jump Actions (Single Jump, Double Jump, Dodge)
        if (cs.isJumping && !tracker.wasJumping) {
            CoreEvent jEv = {};
            jEv.type = INCIDENT_CAR_ACTION;
            jEv.tick = currentTick;
            jEv.posX = cs.pos.x; jEv.posY = cs.pos.y; jEv.posZ = cs.pos.z;
            jEv.primaryId = static_cast<uint16_t>(i);
            jEv.subType = CAR_ACTION_SINGLE_JUMP;
            PushCoreEvent(jEv);
        }
        if (cs.hasDoubleJumped && !tracker.hadDoubleJumped) {
            CoreEvent djEv = {};
            djEv.type = INCIDENT_CAR_ACTION;
            djEv.tick = currentTick;
            djEv.posX = cs.pos.x; djEv.posY = cs.pos.y; djEv.posZ = cs.pos.z;
            djEv.primaryId = static_cast<uint16_t>(i);
            djEv.subType = CAR_ACTION_DOUBLE_JUMP;
            PushCoreEvent(djEv);
        }
        if (cs.isFlipping && !tracker.wasFlipping) {
            CoreEvent flEv = {};
            flEv.type = INCIDENT_CAR_ACTION;
            flEv.tick = currentTick;
            flEv.posX = cs.pos.x; flEv.posY = cs.pos.y; flEv.posZ = cs.pos.z;
            flEv.primaryId = static_cast<uint16_t>(i);
            flEv.subType = CAR_ACTION_DODGE;
            PushCoreEvent(flEv);
        }
        tracker.wasJumping = cs.isJumping;
        tracker.hadDoubleJumped = cs.hasDoubleJumped;
        tracker.wasFlipping = cs.isFlipping;

        // D. Supersonic Enter
        if (cs.isSupersonic && !tracker.wasSupersonic) {
            CoreEvent ssEv = {};
            ssEv.type = INCIDENT_CAR_SUPERSONIC_ENTER;
            ssEv.tick = currentTick;
            ssEv.posX = cs.pos.x; ssEv.posY = cs.pos.y; ssEv.posZ = cs.pos.z;
            ssEv.speed = cs.vel.Length();
            ssEv.primaryId = static_cast<uint16_t>(i);
            ssEv.subType = static_cast<uint32_t>(car->team);
            PushCoreEvent(ssEv);
        }
        tracker.wasSupersonic = cs.isSupersonic;
    }

    // 2. Boost Pad Pickups & Respawns
    for (int p = 0; p < NUM_ARENA_PADS && p < static_cast<int>(g_arena->_boostPads.size()); p++) {
        BoostPad* pad = g_arena->_boostPads[p];
        BoostPadState ps = pad->GetState();
        if (!ps.isActive && g_wasPadActive[p]) {
            // Pad was just picked up
            Vec padPos = pad->config.pos;
            CoreEvent pEv = {};
            pEv.type = INCIDENT_BOOST_PICKUP;
            pEv.tick = currentTick;
            pEv.posX = padPos.x; pEv.posY = padPos.y; pEv.posZ = padPos.z;
            pEv.primaryId = static_cast<uint16_t>(ps.prevLockedCarID < g_cars.size() ? ps.prevLockedCarID : 0xFFFF);
            pEv.secondaryId = static_cast<uint16_t>(p);
            pEv.subType = pad->config.isBig ? 1 : 0;
            PushCoreEvent(pEv);
        } else if (ps.isActive && !g_wasPadActive[p]) {
            // Pad just respawned
            Vec padPos = pad->config.pos;
            CoreEvent rEv = {};
            rEv.type = INCIDENT_BOOST_RESPAWN;
            rEv.tick = currentTick;
            rEv.posX = padPos.x; rEv.posY = padPos.y; rEv.posZ = padPos.z;
            rEv.secondaryId = static_cast<uint16_t>(p);
            rEv.subType = pad->config.isBig ? 1 : 0;
            PushCoreEvent(rEv);
        }
        g_wasPadActive[p] = ps.isActive;
    }
}

static void executePhysicsStep(int ticks, bool silent) {
    if (!g_arena) return;
    g_silentResim = silent;

    for (int step = 0; step < ticks; step++) {
        // Mode 1: Full Physics Freeze (Tick-increment only, zero integration)
        if (g_simControlFlags & SIM_FLAG_FREEZE_PHYSICS) {
            g_arena->tickCount++;
            continue;
        }

        // Mode 2: Input Suppression (Kickoff countdown / hold)
        bool suppressInput = (g_simControlFlags & SIM_FLAG_SUPPRESS_INPUT);

        // Feed controls for each vehicle
        for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
            Car* car = g_cars[i];
            float* ctrlPtr = &g_controlsBuffer[i * CONTROLS_STRIDE];

            if (suppressInput) {
                car->controls = CarControls();
                car->controls.handbrake = true;
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
        }

        // Mode 3: Hold Ball (Clamp velocity to 0)
        if ((g_simControlFlags & SIM_FLAG_HOLD_BALL) && g_arena->ball) {
            BallState bs = g_arena->ball->GetState();
            bs.vel = Vec(0, 0, 0);
            bs.angVel = Vec(0, 0, 0);
            g_arena->ball->SetState(bs);
        }

        // Advance authoritative Bullet physics
        g_arena->Step(1);

        // Scan and emit physics incident events
        detectAndPushPhysicsIncidents(static_cast<uint32_t>(g_arena->tickCount));
    }

    g_silentResim = false;
    syncStateBuffer();
}

// =============================================================================
// C-ABI Exports (WebAssembly Bindings)
// =============================================================================

extern "C" {

void __wasm_call_ctors();

void physics_clearEvents() {
    g_eventBuffer.writeSeq = 0;
    g_eventBuffer.capacity = EVENT_RING_BUFFER_CAPACITY;
    g_eventBuffer.eventSize = sizeof(CoreEvent);
    g_eventBuffer.reserved = 0;
    std::memset(g_eventBuffer.events, 0, sizeof(g_eventBuffer.events));
}

void* physics_getEventBufferPtr() {
    return &g_eventBuffer;
}

int physics_getEventBufferSize() {
    return sizeof(CoreEventBuffer);
}

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
    setupPadInfoBuffer();
    return 1;
}

int physics_createArena() {
    if (RocketSim::GetStage() != RocketSimStage::INITIALIZED) {
        physics_init(nullptr, nullptr, 0);
    }

    if (g_ballPredTracker) {
        delete g_ballPredTracker;
        g_ballPredTracker = nullptr;
    }

    if (g_arena) {
        delete g_arena;
        g_arena = nullptr;
    }
    g_cars.clear();
    g_goalScoredFlag = 0;
    g_isAwaitingFirstTouch = false;
    g_currentPossessionCar = -1;
    g_currentPossessionTeam = -1;
    g_possessionStartTick = 0;

    physics_clearEvents();

    ArenaConfig arenaConfig;
    arenaConfig.noBallRot = false;
    g_arena = Arena::Create(GameMode::SOCCAR, arenaConfig);
    if (!g_arena) return 0;

    g_arena->SetGoalScoreCallback(onGoalScoredCallback, nullptr);
    g_arena->SetCarBumpCallback(onCarBumpCallback, nullptr);
    g_arena->SetBallWorldCallback(onBallWorldCallback, nullptr);

    g_lastBallWorldHitTick = 0;

    if (g_unlimitedBoost) {
        g_arena->_mutatorConfig.boostUsedPerSecond = 0.0f;
    }

    // Initialize Ball Prediction Tracker (600 ticks = 5.0s @ 120Hz)
    g_ballPredTracker = new BallPredTracker(g_arena, MAX_BALL_PRED_TICKS);
    g_ballPredCount = 0;

    syncStateBuffer();
    return 1;
}

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
    std::memcpy(&snap, inBuffer, sizeof(ArenaSnapshotPod));
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

void physics_resetKickoff(int seed) {
    if (!g_arena) return;
    g_arena->ResetToRandomKickoff(seed);
    g_goalScoredFlag = 0;
    g_isAwaitingFirstTouch = true;
    g_currentPossessionCar = -1;
    g_currentPossessionTeam = -1;
    g_possessionStartTick = g_arena->tickCount;
    for (int i = 0; i < MAX_ARENA_CARS; i++) {
        g_carActionTrackers[i] = CarActionTracker();
    }
    syncStateBuffer();
}

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

int physics_addCar(int team, int hitboxType) {
    if (!g_arena || g_cars.size() >= MAX_ARENA_CARS) return -1;

    Team t = (team == 0) ? Team::BLUE : Team::ORANGE;
    const CarConfig& cfg = (hitboxType == 1) ? CAR_CONFIG_DOMINUS : CAR_CONFIG_OCTANE;
    Car* car = g_arena->AddCar(t, cfg);
    g_cars.push_back(car);
    int newIndex = static_cast<int>(g_cars.size()) - 1;

    syncStateBuffer();
    return newIndex;
}

void* physics_getCarConfig(int hitboxIndex) {
    if (hitboxIndex == 1) return (void*)&CAR_CONFIG_DOMINUS;
    return (void*)&CAR_CONFIG_OCTANE;
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
    syncStateBuffer();
    return 1;
}

// =============================================================================
// New C-ABI Exports: Prediction, Control Flags, Thresholds, Possession & Signals
// =============================================================================

int physics_initBallPrediction(int maxTicks) {
    if (!g_arena) return 0;
    if (g_ballPredTracker) delete g_ballPredTracker;
    int ticks = std::clamp(maxTicks, 1, MAX_BALL_PRED_TICKS);
    g_ballPredTracker = new BallPredTracker(g_arena, ticks);
    g_ballPredCount = 0;
    return ticks;
}

int physics_updateBallPrediction(int requestedTicks) {
    if (!g_arena || !g_arena->ball || !g_ballPredTracker) return 0;

    int ticks = std::clamp(requestedTicks, 1, MAX_BALL_PRED_TICKS);
    g_ballPredTracker->numPredTicks = ticks;
    g_ballPredTracker->ForceUpdateAllPred(g_arena->ball->GetState());

    int count = std::min(ticks, static_cast<int>(g_ballPredTracker->predData.size()));
    for (int i = 0; i < count; i++) {
        const BallState& bs = g_ballPredTracker->predData[i];
        g_ballPredBuffer[i * 6 + 0] = bs.pos.x;
        g_ballPredBuffer[i * 6 + 1] = bs.pos.y;
        g_ballPredBuffer[i * 6 + 2] = bs.pos.z;
        g_ballPredBuffer[i * 6 + 3] = bs.vel.x;
        g_ballPredBuffer[i * 6 + 4] = bs.vel.y;
        g_ballPredBuffer[i * 6 + 5] = bs.vel.z;
    }
    g_ballPredCount = count;
    return count;
}

float* physics_getBallPredictionPtr() {
    return g_ballPredBuffer;
}

int physics_getBallPredictionCount() {
    return g_ballPredCount;
}

void physics_setSimControlFlags(uint32_t flags) {
    g_simControlFlags = flags;
}

uint32_t physics_getSimControlFlags() {
    return g_simControlFlags;
}

void physics_setImpactThresholds(float ballGround, float ballWall, float carBall, uint32_t cooldownTicks) {
    g_thresholdBallGround = ballGround;
    g_thresholdBallWall = ballWall;
    g_thresholdCarBall = carBall;
    g_cooldownTicks = cooldownTicks;
}

int physics_getBallMotionState() {
    if (!g_arena || !g_arena->ball) return 0;
    BallState bs = g_arena->ball->GetState();
    float speed = bs.vel.Length();
    if (speed < 5.0f && bs.pos.z <= 95.0f) {
        return 0; // AT_REST
    }
    if (g_isAwaitingFirstTouch && bs.vel.z < -10.0f) {
        return 1; // NATURAL_FALLING
    }
    return 2; // IN_FLIGHT
}

void physics_setPossessionEventEnabled(int enabled) {
    g_possessionEventEnabled = (enabled != 0);
}

int physics_getPossessionEventEnabled() {
    return g_possessionEventEnabled ? 1 : 0;
}

int physics_getCurrentPossessionCar() {
    return g_currentPossessionCar;
}

int physics_getCurrentPossessionTeam() {
    return g_currentPossessionTeam;
}

void physics_pushSignalEvent(
    uint32_t type,
    uint32_t tick,
    float posX, float posY, float posZ,
    float normX, float normY, float normZ,
    float paramVal,
    uint16_t primaryId,
    uint16_t secondaryId,
    uint32_t subType,
    uint32_t flags,
    uint32_t scheduledTick,
    uint32_t customInt
) {
    CoreEvent ev = {};
    ev.type = type;
    ev.tick = tick;
    ev.posX = posX; ev.posY = posY; ev.posZ = posZ;
    ev.normX = normX; ev.normY = normY; ev.normZ = normZ;
    ev.normalRelVel = paramVal;
    ev.primaryId = primaryId;
    ev.secondaryId = secondaryId;
    ev.subType = subType;
    ev.flags = flags;
    ev.scheduledTick = scheduledTick;
    ev.customInt = customInt;
    PushCoreEvent(ev);
}

} // extern "C"
