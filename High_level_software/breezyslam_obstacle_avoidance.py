import serial
import glob
import time
import math
import sys
import threading
from rplidar import RPLidar
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1

# ============================================================
# CONFIGURATION PARAMETERS
# ============================================================
LIDAR_PORT = "/dev/ttyUSB0"

# Strict safety margins (in millimeters)
SAFETY_BUBBLE_THRESHOLD = 500   # Trigger escape routine if any obstacle falls within this radius
CLEAR_CORRIDOR_TARGET   = 1000  # Ensure straight path is open up to this distance before resuming FWD
MAX_REVERSE_TIME        = 2.2   # seconds - Fail-safe time limit to prevent getting stuck reversing

# BreezySLAM Map Configuration
MAP_SIZE_PIXELS = 800  # Resolution size of the occupancy grid canvas
MAP_SIZE_METERS = 16.0 # Real-world size representing a 16m x 16m operating zone

# ============================================================
# HARDWARE INITIALIZATION
# ============================================================
ports = glob.glob('/dev/ttyACM*')
if not ports:
    print("CRITICAL: RP2040 microcontroller not found on /dev/ttyACM*")
    sys.exit(1)

motor = serial.Serial(ports[0], 115200, timeout=0.01)
time.sleep(2.0)

try:
    lidar = RPLidar(LIDAR_PORT, baudrate=115200, timeout=3)
    lidar.start_motor()
    time.sleep(2.0)
except Exception as e:
    print(f"CRITICAL: Failed to initialize LiDAR assembly: {e}")
    motor.close()
    sys.exit(1)

# ============================================================
# GLOBAL SLAM ENGINE STORAGE
# ============================================================
slam_lock = threading.Lock()
current_x = 0.0
current_y = 0.0
current_heading = 0.0
latest_distances = [4000.0] * 360

slam_engine = RMHC_SLAM(RPLidarA1(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

def slam_worker_loop():
    global current_x, current_y, current_heading, latest_distances
    print("--> Background BreezySLAM Engine Started Successfully.")
    
    while True:
        try:
            for scan in lidar.iter_scans(max_buf_meas=500):
                scan_buffer = [0.0] * 360
                
                for (_, angle, distance) in scan:
                    idx = int(math.floor(angle)) % 360
                    if 120.0 < distance < 6000.0:
                        scan_buffer[idx] = distance
                
                for i in range(360):
                    if scan_buffer[i] == 0.0:
                        scan_buffer[i] = 4000.0
                
                try:
                    with slam_lock:
                        slam_engine.update(scan_buffer)
                        current_x, current_y, current_heading = slam_engine.getpos()
                        latest_distances = list(scan_buffer)
                except Exception as inner_exc:
                    continue
                    
        except Exception as outer_exc:
            try:
                lidar._serial_port.reset_input_buffer()
            except:
                pass
            time.sleep(0.05)

slam_thread = threading.Thread(target=slam_worker_loop, daemon=True)
slam_thread.start()
time.sleep(1.0)

# ============================================================
# CORE TRAJECTORY NAVIGATION METRICS
# ============================================================
def get_spatial_metrics():
    with slam_lock:
        local_distances = list(latest_distances)
    
    front_cone = local_distances[315:] + local_distances[:45]
    right_cone = local_distances[45:135]
    left_cone  = local_distances[225:315]
    
    bubble_min = min(local_distances[:135] + local_distances[225:])
    front_min  = min(front_cone)
    avg_left   = sum(left_cone) / len(left_cone)
    avg_right  = sum(right_cone) / len(right_cone)
    
    return bubble_min, front_min, avg_left, avg_right

def execute_counteractive_escape(target_direction_cmd):
    """
    Time-bounded, adaptive anti-drift turn routine. Breaks out of deadlocks
    automatically if tracks experience high surface friction or slippage.
    """
    global current_heading
    
    with slam_lock:
        start_angle = current_heading
        
    if target_direction_cmd == b'L':
        target_angle = (start_angle + 90.0) % 360.0
        print(f"Executing Smooth SLAM-Monitored 90° Turn LEFT (Start: {start_angle:.1f}° -> Target: {target_angle:.1f}°)")
    else:
        target_angle = (start_angle - 90.0) % 360.0
        print(f"Executing Smooth SLAM-Monitored 90° Turn RIGHT (Start: {start_angle:.1f}° -> Target: {target_angle:.1f}°)")

    angle_error = 90.0
    turn_start_time = time.time()
    MAX_TURN_DURATION = 2.5  # seconds - Maximum time allowed to complete the pivot
    
    while angle_error > 8.0:
        # Check if the wheels are binding or slipping past the safe time threshold
        if time.time() - turn_start_time > MAX_TURN_DURATION:
            print(f"🔄 Turn timeout triggered ({MAX_TURN_DURATION}s). Surface resistance detected. Handing off to final alignment.")
            break
            
        with slam_lock:
            now_angle = current_heading
            
        diff = (now_angle - target_angle + 180) % 360 - 180
        angle_error = abs(diff)
        
        # 1. Rotational pulse (Restored to 0.16s to ensure it overcomes floor friction)
        motor.write(target_direction_cmd)
        motor.flush()
        time.sleep(0.16)
        
        # 2. Counteract forward drift with an explicit backing step
        motor.write(b'B')
        motor.flush()
        time.sleep(0.11)
        
        # 3. Quick settle stop to process crisp data
        motor.write(b'S')
        motor.flush()
        time.sleep(0.06)

# ============================================================
# MAIN STATE MACHINE EXECUTION
# ============================================================
try:
    print("\n============================================================")
    print("  BreezySLAM Smooth Integrated Autonomous Navigation Active")
    print("============================================================\n")
    
    motor.write(b'S')
    motor.flush()

    while True:
        bubble_min, front_min, avg_left, avg_right = get_spatial_metrics()
        
        with slam_lock:
            print(f"SLAM POS: X={current_x:5.0f} Y={current_y:5.0f} θ={current_heading:5.1f}° | "
                  f"Bubble Min: {bubble_min:4.0f}mm | Front Min: {front_min:4.0f}mm")

        # ── CRUISE NAVIGATION STATE ────────────────────────────────
        if bubble_min > SAFETY_BUBBLE_THRESHOLD:
            motor.write(b'F')
            motor.flush()
            time.sleep(0.1)
            continue

        # ── SAFETY ENVELOPE BREACHED -> SMOOTH DECELERATION GEARSHIFT ──
        print(f"\n⚠ Safety Bubble Breached! Obstacle at {bubble_min:.0f} mm.")
        
        # ACTIVE BRAKING PHASE: Kill forward kinetic energy before throwing into reverse
        print("⚡ Active Braking: Killing linear momentum...")
        motor.write(b'S')
        motor.flush()
        time.sleep(0.25)  # Let physical inertia settle completely. Prevents LiDAR shake.

        # STEP A: CLOSED-LOOP SMOOTH REVERSING PHASE
        print(f"⏮  Engaging Smooth Closed-Loop Reverse. Target: > {CLEAR_CORRIDOR_TARGET}mm...")
        reverse_start_time = time.time()
        
        while bubble_min <= CLEAR_CORRIDOR_TARGET:
            if time.time() - reverse_start_time > MAX_REVERSE_TIME:
                print(f"⏮  Reverse timeout fail-safe reached ({MAX_REVERSE_TIME}s).")
                break
                
            motor.write(b'B')
            motor.flush()
            bubble_min, _, _, _ = get_spatial_metrics()
            time.sleep(0.05)

        # BRAKING AT THE END OF REVERSE
        print(f"✓ Backing complete. Halting wheels smoothly.")
        motor.write(b'S')
        motor.flush()
        time.sleep(0.20)  # Let backing inertia dissipate before pivoting

        # STEP B: EVALUATE SURROUNDING SPACE
        bubble_min, front_min, avg_left, avg_right = get_spatial_metrics()
        if avg_left > avg_right:
            escape_cmd = b'L'
            dir_label  = "LEFT"
        else:
            escape_cmd = b'R'
            dir_label  = "RIGHT"

        print(f"🔄 Selecting optimal exit route: {dir_label} (Left: {avg_left:.0f}mm vs Right: {avg_right:.0f}mm)")

        # STEP C: EXECUTE MONITORED ANTI-DRIFT RADIAL PIVOT
        execute_counteractive_escape(escape_cmd)

        # STEP D: AXIAL SPIN RE-VERIFICATION (SMOOTH DRIFT-COMPENSATED)
        while front_min <= CLEAR_CORRIDOR_TARGET:
            print(f"  --> Smooth alignment correction... Nose proximity: {front_min:.0f} mm")
            
            motor.write(escape_cmd)
            motor.flush()
            time.sleep(0.09)  # Tiny micro-steps
            
            motor.write(b'B')
            motor.flush()
            time.sleep(0.07)
            
            motor.write(b'S')
            motor.flush()
            time.sleep(0.06)
            
            _, front_min, _, _ = get_spatial_metrics()

        print("✓ Path unlocked! Transitioning smoothly back to forward cruise.\n")
        motor.write(b'S')
        motor.flush()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nManual Override Register Triggered: Initiating Safe Shutdown Sequence...")

finally:
    try:
        motor.write(b'S')
        motor.flush()
    except:
        pass
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except:
        pass
    motor.close()
    print("System Shutdown Executed Successfully.")
