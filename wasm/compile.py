import os
import sys
import subprocess
from concurrent.futures import ProcessPoolExecutor

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
OUT_DIR = os.path.join(SCRIPT_DIR, "build")
OBJ_DIR = os.path.join(OUT_DIR, "obj")
os.makedirs(OBJ_DIR, exist_ok=True)

sources = []
for base in [os.path.join(ROOT_DIR, "libsrc", "bullet3-3.24"), os.path.join(ROOT_DIR, "src")]:
    for root, dirs, files in os.walk(base):
        for f in files:
            if f.endswith(".cpp"):
                sources.append(os.path.join(root, f))
sources.append(os.path.join(SCRIPT_DIR, "RocketSimBridge.cpp"))

common_flags = [
    "-std=c++20",
    "-O3",
    "-fno-fast-math",
    "-fno-associative-math",
    "-DNDEBUG",
    "-g2",
    f"-I{os.path.join(ROOT_DIR, 'src')}",
    f"-I{os.path.join(ROOT_DIR, 'libsrc', 'bullet3-3.24')}"
]

def compile_file(src):
    rel = os.path.relpath(src, ROOT_DIR).replace("/", "_").replace("\\", "_") + ".o"
    obj = os.path.join(OBJ_DIR, rel)
    cmd = ["em++"] + common_flags + ["-c", src, "-o", obj]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(f"Error compiling {src}:\n{res.stderr}")
        return False
    return True

print(f"Compiling {len(sources)} files with {os.cpu_count()} workers...")
with ProcessPoolExecutor(max_workers=os.cpu_count()) as executor:
    results = list(executor.map(compile_file, sources))

if not all(results):
    print("Compilation failed!")
    sys.exit(1)

print("All object files compiled successfully! Linking...")

objs = [os.path.join(OBJ_DIR, f) for f in os.listdir(OBJ_DIR) if f.endswith(".o")]

exported_functions = (
    "['_malloc','_free','_physics_init','_physics_createArena','_physics_step',"
    "'_physics_resetKickoff','_physics_clearGoalFlag','_physics_getStatePtr',"
    "'_physics_getStateSize','_physics_getControlsPtr','_physics_getPadInfoPtr',"
    "'_physics_addCar','_physics_getCarConfig','_physics_setCarState',"
    "'_physics_setBallState','_physics_setUnlimitedBoost','_physics_getBallOnGround',"
    "'_physics_getBallRadius','_physics_controlBall',"
    "'_physics_getEventBufferPtr','_physics_getEventBufferSize','_physics_clearEvents']"
)

exported_runtime_methods = (
    "['ccall','cwrap','HEAP8','HEAPU8','HEAP16','HEAPU16','HEAP32','HEAPU32','HEAPF32','HEAPF64']"
)

link_cmd = [
    "em++",
    "-std=c++20",
    "-O3",
    "-fno-fast-math",
    "-fno-associative-math",
    "-DNDEBUG",
    "-g2",
    "-sFILESYSTEM=0",
    "-sUSE_CLOSURE_COMPILER=0",
    "-sENVIRONMENT=web,node,worker",
    "-sMODULARIZE=1",
    "-sEXPORT_ES6=1",
    '-sEXPORT_NAME="loadDirectRocketSimWasm"',
    "-sALLOW_MEMORY_GROWTH=1",
    "-sINITIAL_MEMORY=67108864",
    f"-sEXPORTED_FUNCTIONS={exported_functions}",
    f"-sEXPORTED_RUNTIME_METHODS={exported_runtime_methods}",
    "-o", os.path.join(OUT_DIR, "core.js")
] + objs

res = subprocess.run(link_cmd, capture_output=True, text=True)
if res.returncode != 0:
    print(f"Linking error:\n{res.stderr}")
    sys.exit(1)

print(f"Successfully generated {os.path.join(OUT_DIR, 'core.wasm')} and {os.path.join(OUT_DIR, 'core.js')}!")
