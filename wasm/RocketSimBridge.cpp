/**
 * RocketSimBridge.cpp
 * Standard C-ABI WebAssembly Bridge for RocketSim Physics Engine.
 * 
 * Reconstructed from rocketsim.wasm (Emscripten compiled).
 * Provides clean, semantic exports mapping 1:1 to obfuscated single-character exports.
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

// Unified Physics Event Types
static constexpr uint32_t EVENT_TYPE_NONE = 0;
static constexpr uint32_t EVENT_TYPE_CAR_BALL_HIT = 1;
static constexpr uint32_t EVENT_TYPE_CAR_CAR_COLLISION = 2;
static constexpr uint32_t EVENT_TYPE_BALL_WORLD_HIT = 3;
static constexpr uint32_t EVENT_TYPE_BALL_GOALPOST_HIT = 4;
static constexpr uint32_t EVENT_TYPE_CAR_ACTION = 5;
static constexpr uint32_t EVENT_TYPE_CAR_SUPERSONIC_ENTER = 6;
static constexpr uint32_t EVENT_TYPE_BOOST_PICKUP = 7;

static constexpr uint32_t CAR_ACTION_SINGLE_JUMP = 1;
static constexpr uint32_t CAR_ACTION_DOUBLE_JUMP = 2;
static constexpr uint32_t CAR_ACTION_DODGE = 3;

// Zero-Copy Ring Buffer & Physics Event Structure (Strictly 48 bytes per event, aligned to 4 bytes)
struct PhysicsEvent {
    uint32_t type;         // [Word 0] Event type (1..7)
    uint32_t tick;         // [Word 1] Physics tick timestamp
    float x, y, z;         // [Word 2, 3, 4] 3D world position in Unreal Units

    union {
        // type == 1 (CarBallHit)
        struct {
            uint16_t carIndex;    // [Word 5, Low 16 bits] Car index (0~7)
            uint16_t team;        // [Word 5, High 16 bits] Team (0: Blue, 1: Orange)
            float relSpeed;       // [Word 6] Relative collision velocity (|v_car - v_ball|)
            float normalX;        // [Word 7] Contact normal X
            float normalY;        // [Word 8] Contact normal Y
            float normalZ;        // [Word 9] Contact normal Z
            float impulse;        // [Word 10] Applied extra impulse magnitude
        } carBall;

        // type == 2 (CarCarCollision & Demo)
        struct {
            uint16_t car1Index;   // [Word 5, Low 16 bits] Attacker / Bumper car index
            uint16_t car2Index;   // [Word 5, High 16 bits] Victim car index
            uint16_t car1Team;    // [Word 6, Low 16 bits] Team (0: Blue, 1: Orange)
            uint16_t car2Team;    // [Word 6, High 16 bits] Team (0: Blue, 1: Orange)
            float relSpeed;       // [Word 7] Relative collision velocity
            float impulse;        // [Word 8] Bump impulse magnitude
            uint32_t isDemo;      // [Word 9] 1 if demo, 0 if bump
            uint32_t reserved;    // [Word 10]
        } carCar;

        // type == 3 (BallWorldHit - Pitch impact)
        struct {
            float speed;          // [Word 5] Ball speed at impact
            float normalX;        // [Word 6] Contact normal X
            float normalY;        // [Word 7] Contact normal Y
            float normalZ;        // [Word 8] Contact normal Z
            uint32_t surfaceTag;  // [Word 9] 1: ground/pitch, 2: wall/ceiling
            uint32_t reserved;    // [Word 10]
        } ballWorld;

        // type == 4 (BallGoalpostHit - Goalpost & crossbar)
        struct {
            float speed;          // [Word 5] Ball speed at impact
            float normalX;        // [Word 6] Contact normal X
            float normalY;        // [Word 7] Contact normal Y
            float normalZ;        // [Word 8] Contact normal Z
            uint32_t isCrossbar;  // [Word 9] 1: crossbar, 0: vertical post
            uint32_t reserved;    // [Word 10]
        } ballGoalpost;

        // type == 5 (CarAction - Single Jump, Double Jump, Dodge)
        struct {
            uint16_t carIndex;    // [Word 5, Low 16 bits]
            uint16_t team;        // [Word 5, High 16 bits]
            uint32_t subType;     // [Word 6] 1: Single Jump, 2: Double Jump, 3: Dodge
            float reserved[4];    // [Word 7..10]
        } carAction;

        // type == 6 (CarSupersonicEnter)
        struct {
            uint16_t carIndex;    // [Word 5, Low 16 bits]
            uint16_t team;        // [Word 5, High 16 bits]
            float speed;          // [Word 6] Entry speed
            float reserved[4];    // [Word 7..10]
        } supersonicEnter;

        // type == 7 (BoostPickup)
        struct {
            uint16_t carIndex;    // [Word 5, Low 16 bits] Collector car index
            uint16_t team;        // [Word 5, High 16 bits] Collector team
            uint32_t padIndex;    // [Word 6] Boost pad index (0..33)
            uint32_t isBig;       // [Word 7] 1: Big pad (100), 0: Small pad (12)
            float reserved[3];    // [Word 8..10]
        } boostPickup;

        float fParams[6];
        uint32_t uParams[6];
    };

    uint32_t flags;        // [Word 11] Bit 0: InitialTouch, Bit 1: Supersonic, Bit 2: Flip, Bit 3: Ground
};

static constexpr uint32_t EVENT_RING_BUFFER_CAPACITY = 64;

struct PhysicsEventBuffer {
    uint32_t writeSeq;     // Monotonically increasing write sequence counter
    uint32_t capacity;     // 64
    uint32_t eventSize;    // 48 bytes
    uint32_t reserved;     // 0
    PhysicsEvent events[EVENT_RING_BUFFER_CAPACITY];
};
#pragma pack(pop)

static_assert(sizeof(GameStateBufferPod) == 424 * sizeof(float), "Memory layout size must be exactly 424 floats");
static_assert(sizeof(PhysicsEvent) == 48, "PhysicsEvent size must be exactly 48 bytes");
static_assert(sizeof(PhysicsEventBuffer) == 16 + 64 * 48, "PhysicsEventBuffer size must be exactly 3120 bytes");

static constexpr int STATE_BUFFER_SIZE = sizeof(GameStateBufferPod) / sizeof(float);

// Shared memory buffers mapped to WebAssembly heap
static GameStateBufferPod g_state;
static float* g_stateBuffer = reinterpret_cast<float*>(&g_state);
static float g_controlsBuffer[MAX_ARENA_CARS * CONTROLS_STRIDE]; // 64 floats
static float g_padInfoBuffer[NUM_ARENA_PADS * 4];                 // 136 floats

// Global Ring Buffer for zero-copy physics events
static PhysicsEventBuffer g_eventBuffer = { 0, EVENT_RING_BUFFER_CAPACITY, sizeof(PhysicsEvent), 0 };

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

static uint64_t g_lastBallWorldHitTick = 0;

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
    // Exact Bullet rigid body state
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
        std::memset(&snap, 0, sizeof(ArenaSnapshotPod));
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
        int numWheels = std::min(4, car->_bulletVehicle.getNumWheels());
        for (int w = 0; w < numWheels; w++) {
            snap.cars[i].wheels[w] = car->_bulletVehicle.getWheelInfo(w);
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
        g_arena->ball->_rigidBody.setWorldTransform(snap.ballTransform);
        g_arena->ball->_rigidBody.setLinearVelocity(snap.ballLinearVelocity);
        g_arena->ball->_rigidBody.setAngularVelocity(snap.ballAngularVelocity);
        g_arena->ball->_rigidBody.setInterpolationWorldTransform(snap.ballTransform);
        g_arena->ball->_rigidBody.setInterpolationLinearVelocity(snap.ballLinearVelocity);
        g_arena->ball->_rigidBody.setInterpolationAngularVelocity(snap.ballAngularVelocity);
        g_arena->ball->_rigidBody.updateInertiaTensor();
        g_arena->_bulletWorld.updateSingleAabb(&g_arena->ball->_rigidBody);
    }

    const auto& pads = g_arena->GetBoostPads();
    size_t padCount = std::min(pads.size(), static_cast<size_t>(NUM_ARENA_PADS));
    for (size_t p = 0; p < padCount; p++) {
        pads[p]->_internalState.isActive = (snap.pads[p].isActive != 0);
        pads[p]->_internalState.cooldown = snap.pads[p].cooldown;
        pads[p]->_internalState.prevLockedCarID = snap.pads[p].prevLockedCarID;
        pads[p]->_internalState.curLockedCar = nullptr;
    }

    size_t numCars = std::min(g_cars.size(), static_cast<size_t>(snap.numCars));
    for (size_t i = 0; i < numCars; i++) {
        Car* car = g_cars[i];
        car->team = static_cast<Team>(snap.cars[i].team);
        car->id = snap.cars[i].id;
        car->controls = snap.cars[i].controls;
        car->SetState(snap.cars[i].state);

        car->_rigidBody.setWorldTransform(snap.cars[i].rbTransform);
        car->_rigidBody.setLinearVelocity(snap.cars[i].rbLinearVelocity);
        car->_rigidBody.setAngularVelocity(snap.cars[i].rbAngularVelocity);
        car->_rigidBody.setInterpolationWorldTransform(snap.cars[i].rbTransform);
        car->_rigidBody.setInterpolationLinearVelocity(snap.cars[i].rbLinearVelocity);
        car->_rigidBody.setInterpolationAngularVelocity(snap.cars[i].rbAngularVelocity);
        car->_rigidBody.updateInertiaTensor();
        g_arena->_bulletWorld.updateSingleAabb(&car->_rigidBody);

        int numWheels = std::min(4, car->_bulletVehicle.getNumWheels());
        for (int w = 0; w < numWheels; w++) {
            car->_bulletVehicle.getWheelInfo(w) = snap.cars[i].wheels[w];
            car->_bulletVehicle.updateWheelTransformsWS(car->_bulletVehicle.getWheelInfo(w));
            car->_bulletVehicle.updateWheelTransform(w);
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
}

// Car-Car collision & Demo callback
static void onCarBumpCallback(Arena* arena, Car* bumper, Car* victim, bool isDemo, const Vec& contactPos, float relSpeed, float impulse, void* userInfo) {
    if (g_silentResim) return;
    PhysicsEvent ev = {};
    ev.type = EVENT_TYPE_CAR_CAR_COLLISION;
    ev.tick = static_cast<uint32_t>(arena->tickCount > 0 ? arena->tickCount - 1 : 0);
    ev.x = contactPos.x;
    ev.y = contactPos.y;
    ev.z = contactPos.z;

    int bumperIdx = getCarIndex(bumper);
    int victimIdx = getCarIndex(victim);
    ev.carCar.car1Index = static_cast<uint16_t>(bumperIdx >= 0 ? bumperIdx : 0);
    ev.carCar.car2Index = static_cast<uint16_t>(victimIdx >= 0 ? victimIdx : 0);
    ev.carCar.car1Team = static_cast<uint16_t>((bumper && bumper->team == Team::ORANGE) ? 1 : 0);
    ev.carCar.car2Team = static_cast<uint16_t>((victim && victim->team == Team::ORANGE) ? 1 : 0);
    ev.carCar.relSpeed = relSpeed;
    ev.carCar.impulse = impulse;
    ev.carCar.isDemo = isDemo ? 1 : 0;
    PushPhysicsEvent(ev);
}

// Ball-World & Goalpost collision callback
static void onBallWorldCallback(Arena* arena, const Vec& contactPos, const Vec& normal, float speed, bool isGoalpost, void* userInfo) {
    if (g_silentResim) return;
    uint64_t curTick = arena->tickCount;
    // Debounce successive contact frames (min 4 ticks ≈ 33ms) and ignore rolling/low speeds (<120 UU/s)
    if (curTick <= g_lastBallWorldHitTick + 4 || speed < 120.0f) {
        return;
    }
    g_lastBallWorldHitTick = curTick;

    PhysicsEvent ev = {};
    ev.type = isGoalpost ? EVENT_TYPE_BALL_GOALPOST_HIT : EVENT_TYPE_BALL_WORLD_HIT;
    ev.tick = static_cast<uint32_t>(curTick > 0 ? curTick - 1 : 0);
    ev.x = contactPos.x;
    ev.y = contactPos.y;
    ev.z = contactPos.z;

    if (isGoalpost) {
        ev.ballGoalpost.speed = speed;
        ev.ballGoalpost.normalX = normal.x;
        ev.ballGoalpost.normalY = normal.y;
        ev.ballGoalpost.normalZ = normal.z;
        bool isCrossbar = (std::abs(contactPos.z - 642.0f) < 80.0f) && (std::abs(contactPos.x) <= 950.0f);
        ev.ballGoalpost.isCrossbar = isCrossbar ? 1 : 0;
    } else {
        ev.ballWorld.speed = speed;
        ev.ballWorld.normalX = normal.x;
        ev.ballWorld.normalY = normal.y;
        ev.ballWorld.normalZ = normal.z;
        ev.ballWorld.surfaceTag = (contactPos.z < 20.0f) ? 1 : 2; // 1: ground/pitch, 2: wall/ceiling
    }
    PushPhysicsEvent(ev);
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

        cPod.boost = g_unlimitedBoost ? 100.0f : cs.boost;
        cPod.isOnGround = cs.isOnGround ? 1.0f : 0.0f;
        cPod.isSupersonic = cs.isSupersonic ? 1.0f : 0.0f;
        cPod.isDemoed = cs.isDemoed ? 1.0f : 0.0f;
        cPod.hasFlipOrJump = cs.HasFlipOrJump() ? 1.0f : 0.0f;
        cPod.isBoosting = cs.isBoosting ? 1.0f : 0.0f;
        cPod.isFlipping = cs.isFlipping ? 1.0f : 0.0f;

        // 4 wheels * 3 floats [susLength, steerAngle, hasContact]
        btVector3 accumulatedWheelNormal(0.0f, 0.0f, 0.0f);
        int wheelsOnGroundCount = 0;

        for (int w = 0; w < 4; w++) {
            if (w < car->_bulletVehicle.getNumWheels()) {
                const auto& wheelInfo = car->_bulletVehicle.getWheelInfo(w);
                cPod.wheels[w].susLength = wheelInfo.m_raycastInfo.m_suspensionLength;
                cPod.wheels[w].steerAngle = wheelInfo.m_steerAngle;
                cPod.wheels[w].hasContact = cs.wheelsWithContact[w] ? 1.0f : 0.0f;

                if (wheelInfo.m_raycastInfo.m_isInContact) {
                    accumulatedWheelNormal += wheelInfo.m_raycastInfo.m_contactNormalWS;
                    wheelsOnGroundCount++;
                }
            } else {
                cPod.wheels[w].susLength = 0.0f;
                cPod.wheels[w].steerAngle = 0.0f;
                cPod.wheels[w].hasContact = 0.0f;
            }
        }

        // Ground normal and ground contact calculation
        if (wheelsOnGroundCount > 0) {
            btVector3 trueGroundNormal = accumulatedWheelNormal.safeNormalized();
            cPod.groundNormalX = trueGroundNormal.x();
            cPod.groundNormalY = trueGroundNormal.y();
            cPod.groundNormalZ = trueGroundNormal.z();
            cPod.isOnGround = 1.0f;
        } else if (cs.worldContact.hasContact) {
            cPod.groundNormalX = cs.worldContact.contactNormal.x;
            cPod.groundNormalY = cs.worldContact.contactNormal.y;
            cPod.groundNormalZ = cs.worldContact.contactNormal.z;
            cPod.isOnGround = 1.0f;
        } else {
            cPod.groundNormalX = 0.0f;
            cPod.groundNormalY = 0.0f;
            cPod.groundNormalZ = 0.0f;
            cPod.isOnGround = 0.0f;
        }

    }

    // Boost pad states
    const auto& pads = g_arena->GetBoostPads();
    int padCount = static_cast<int>(std::min(static_cast<size_t>(NUM_ARENA_PADS), pads.size()));
    for (int p = 0; p < padCount; p++) {
        BoostPad* pad = pads[p];
        BoostPadState pState = pad->GetState();
        g_state.pads[p].isActive = pState.isActive ? 1.0f : 0.0f;
        g_state.pads[p].cooldown = pState.cooldown;
    }
}

// Populate static boost pad coordinates
static void setupPadInfoBuffer() {
    using namespace RLConst::BoostPads;
    int idx = 0;
    for (int i = 0; i < LOCS_AMOUNT_BIG; i++) {
        g_padInfoBuffer[idx * 4 + 0] = LOCS_BIG_SOCCAR[i].x;
        g_padInfoBuffer[idx * 4 + 1] = LOCS_BIG_SOCCAR[i].y;
        g_padInfoBuffer[idx * 4 + 2] = LOCS_BIG_SOCCAR[i].z;
        g_padInfoBuffer[idx * 4 + 3] = 1.0f; // isBig
        idx++;
    }
    for (int i = 0; i < LOCS_AMOUNT_SMALL_SOCCAR; i++) {
        g_padInfoBuffer[idx * 4 + 0] = LOCS_SMALL_SOCCAR[i].x;
        g_padInfoBuffer[idx * 4 + 1] = LOCS_SMALL_SOCCAR[i].y;
        g_padInfoBuffer[idx * 4 + 2] = LOCS_SMALL_SOCCAR[i].z;
        g_padInfoBuffer[idx * 4 + 3] = 0.0f; // small
        idx++;
    }
}

// EXPORTED C-ABI FUNCTIONS
extern "C" {

void __wasm_call_ctors();

void physics_clearEvents() {
    g_eventBuffer.writeSeq = 0;
    g_eventBuffer.capacity = EVENT_RING_BUFFER_CAPACITY;
    g_eventBuffer.eventSize = sizeof(PhysicsEvent);
    g_eventBuffer.reserved = 0;
    std::memset(g_eventBuffer.events, 0, sizeof(g_eventBuffer.events));
}

void* physics_getEventBufferPtr() {
    return &g_eventBuffer;
}

int physics_getEventBufferSize() {
    return sizeof(PhysicsEventBuffer);
}

// Export 'o'
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

// Export 'p'
int physics_createArena() {
    if (RocketSim::GetStage() != RocketSimStage::INITIALIZED) {
        physics_init(nullptr, nullptr, 0);
    }

    if (g_arena) {
        delete g_arena;
        g_arena = nullptr;
    }
    g_cars.clear();
    g_goalScoredFlag = 0;



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

    syncStateBuffer();
    return 1;
}

static void executePhysicsStep(int ticks, bool silent) {
    if (!g_arena) return;
    g_silentResim = silent;

    for (int step = 0; step < ticks; step++) {
        // Feed controls for each vehicle
        for (size_t i = 0; i < g_cars.size() && i < MAX_ARENA_CARS; i++) {
            Car* car = g_cars[i];
            float* ctrlPtr = &g_controlsBuffer[i * CONTROLS_STRIDE];

            car->controls.throttle  = ctrlPtr[0];
            car->controls.steer     = ctrlPtr[1];
            car->controls.pitch     = ctrlPtr[2];
            car->controls.yaw       = ctrlPtr[3];
            car->controls.roll      = ctrlPtr[4];
            car->controls.jump      = (ctrlPtr[5] > 0.5f);
            car->controls.boost     = (ctrlPtr[6] > 0.5f);
            car->controls.handbrake = (ctrlPtr[7] > 0.5f);

            if (g_unlimitedBoost) {
                car->_internalState.boost = 100.0f;
            }
        }

        g_arena->Step(1);
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

// Export 't'
void physics_resetKickoff(int seed) {
    if (!g_arena) return;
    g_arena->ResetToRandomKickoff(seed);
    g_goalScoredFlag = 0;
    syncStateBuffer();
}

// Export 'x'
float* physics_getStatePtr() {
    return g_stateBuffer;
}

// Export 'y'
int physics_getStateSize() {
    return STATE_BUFFER_SIZE;
}

// Export 'z'
float* physics_getControlsPtr() {
    return g_controlsBuffer;
}

// Export 'A'
float* physics_getPadInfoPtr() {
    return g_padInfoBuffer;
}

// Export 'q'
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

// Export 'r'
void* physics_getCarConfig(int hitboxIndex) {
    if (hitboxIndex == 1) return (void*)&CAR_CONFIG_DOMINUS;
    return (void*)&CAR_CONFIG_OCTANE;
}

// Export '_physics_setCarState'
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

// Export '_physics_setBallState'
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

// Export 'w'
void physics_setUnlimitedBoost(int unlimited) {
    g_unlimitedBoost = (unlimited != 0);
    if (g_arena) {
        g_arena->_mutatorConfig.boostUsedPerSecond = g_unlimitedBoost ? 0.0f : RLConst::BOOST_USED_PER_SECOND;
    }
}

// Export 'B'
int physics_getBallOnGround() {
    if (!g_arena || !g_arena->ball) return 0;
    return (g_arena->ball->GetState().pos.z <= RLConst::BALL_COLLISION_RADIUS_SOCCAR + 2.0f) ? 1 : 0;
}

// Export 'C'
float physics_getBallRadius() {
    return RLConst::BALL_COLLISION_RADIUS_SOCCAR;
}

// Export 'u'
void physics_clearGoalFlag() {
    g_goalScoredFlag = 0;
    g_state.header.goalScoredFlag = 0.0f;
}

// Export 'v'
int physics_controlBall(int carIndex, int modeIndex) {
    if (!g_arena || !g_arena->ball || carIndex < 0 || carIndex >= static_cast<int>(g_cars.size())) return 0;

    Car* car = g_cars[carIndex];
    CarState cs = car->GetState();
    BallState bs = g_arena->ball->GetState();

    switch (modeIndex) {
    case 0: // takePossession: place ball directly in front of vehicle at rest height
        bs.pos = cs.pos + cs.rotMat.forward * 215.39f;
        bs.pos.z = 93.25f;
        bs.vel = Vec(0, 0, 0);
        bs.angVel = Vec(0, 0, 0);
        break;
    case 1: // startDribble: place ball above hood/roof of vehicle
        bs.pos = cs.pos + cs.rotMat.forward * 30.0f + Vec(0, 0, 141.33f);
        bs.vel = Vec(0, 0, 0);
        bs.angVel = Vec(0, 0, 0);
        break;
    case 2: // passBall: center pass arc trajectory
        bs.pos = Vec(0, 0, 93.15f);
        bs.vel = Vec(1735.35f, -2169.19f, 403.76f);
        bs.angVel = Vec(0, 0, 0);
        break;
    case 3: // launchBall: vertical launch from ground center
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

} // extern "C"
