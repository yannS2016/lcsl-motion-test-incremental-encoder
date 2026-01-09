from epics import PV
import time
import sys
import random
import argparse

def get_args():
    parser = argparse.ArgumentParser(
        description="EPICS Axis State Enum/Bulk Move/Halt/Reset Test Script"
    )
    parser.add_argument(
        "--motor", type=str, required=True,
        help="EPICS PV prefix for the axis, e.g. 'TST:MOTION:M1:'"
    )
    parser.add_argument(
        "--num-moves", type=int, default=5,
        help="Number of random state moves to test (default: 5)"
    )
    parser.add_argument(
        "--num-states", type=int, default=15,
        help="Number of possible valid states (default: 15, valid range is 1..15)"
    )
    parser.add_argument(
        "-v", "--verbose", action='store_true',
        help="Print detailed debug info"
    )
    return parser.parse_args()

args = get_args()
MOTOR = args.motor
num_moves = args.num_moves
num_states = args.num_states
VERBOSE = args.verbose

def vprint(*a, **kw):
    if VERBOSE:
        print(*a, **kw)

PVNAMES = {
    "STATES:SET": MOTOR + ":STATES:SET",
    "STATES:GET": MOTOR + ":STATES:GET_RBV",
    "bReset": MOTOR + ":PLC:bReset",
    "bHalt": MOTOR + "-Stop",
    "bDone": MOTOR + ".DMOV",
    "bBusy": MOTOR + ":STATE:BUSY_RBV",
    "bError": MOTOR + ":PLC:bError_RBV",
    "nErrorId": MOTOR + ":PLC:nErrorId_RBV",
    "sErrorMessage": MOTOR + ":PLC:sErrorMessage_RBV",
}
PVs = {key: PV(pvname) for key, pvname in PVNAMES.items()}

def assert_true(cond, msg):
    if not cond:
        print('[FAIL]', msg)
        raise AssertionError(msg)
    print('[PASS]', msg)

def assert_false(cond, msg):
    assert_true(not cond, msg)

def check_connected():
    for key, pv in PVs.items():
        if not pv.wait_for_connection(timeout=2.0):
            raise Exception(f"PV {key} ({pv.pvname}) not connected!")

def wait_pv_flag(key, value=1, timeout=14.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = PVs[key].get(timeout=1)
        if v == value:
            return True
        time.sleep(0.2)
    raise AssertionError(f"Timeout waiting for {key} == {value}")

def wait_move_done(timeout=20.0):
    done_flag = [False]
    def cb_done(pvname=None, value=None, **_):
        if value: done_flag[0] = True
    PVs["bDone"].add_callback(cb_done)
    PVs["bDone"].use_monitor = True
    t0 = time.time()
    while not done_flag[0]:
        if time.time() - t0 > timeout:
            raise AssertionError("Timeout waiting for bDone")
        time.sleep(0.2)
    vprint("[INFO] Move done detected.")

def get_current_state():
    return PVs["STATES:GET"].get(timeout=2)

def move_to_state(state_enum):
    print(f"[STEP] Move to state {state_enum}")
    PVs["STATES:SET"].put(int(state_enum), timeout=1.0)
    wait_move_done()

def test_halt_or_reset_state(target_state, do_halt=True):
    '''
    If current state is 0 (unknown): abort.
    If axis is not bDone: abort.
    Pick a different state to move to. Issue move to it, wait for bBusy==1, then sleep briefly,
    then issue halt or reset, and await completion.
    '''
    done = PVs["bDone"].get(timeout=2)
    current_state = get_current_state()

    if not done:
        print(f"[ABORT] Axis not in DONE state (bDone={done}); not ready to start test.")
        return
    if int(current_state) == 0:
        print(f"[ABORT] Axis in UNKNOWN state ({current_state}), can't test halt/reset.")
        return

    all_states = list(range(1, num_states+1))
    alt_states = [s for s in all_states if s != current_state]
    if not alt_states:
        print("[WARN] Only one state; cannot test halt/reset.")
        return
    move_state = random.choice(alt_states)

    print(f"[INFO] Initiating move from state {current_state} -> {move_state} to test {('HALT' if do_halt else 'RESET')}.")
    PVs["STATES:SET"].put(int(move_state), timeout=1.0)

    # Wait for axis to go busy (i.e., started real move)
    t0 = time.time()
    while not PVs["bBusy"].get(timeout=0.7):
        if time.time() - t0 > 7.0:
            raise AssertionError("Axis never went busy after move start.")
        time.sleep(0.1)
    vprint("[INFO] Axis is busy, about to trigger halt/reset")

    time.sleep(1.5)  # let the motion start for real

    action = "HALT" if do_halt else "RESET"
    print(f"[STEP] While moving toward {move_state}, issue {action}")
    if do_halt:
        PVs["bHalt"].put(1, timeout=1.0)
        time.sleep(0.3)
        PVs["bHalt"].put(0, timeout=1.0)
    else:
        PVs["bReset"].put(1, timeout=1.0)
        time.sleep(0.3)
        PVs["bReset"].put(0, timeout=1.0)

    wait_pv_flag("bDone", 1, timeout=10.0)
    assert_true(PVs["bDone"].get(timeout=1.0) == 1,
                f"bDone should be set after {action}")
    print(f"[PASS] Move with {action} test complete.")

# --- Main test sequence ---
check_connected()
print("[INFO] All key PVs connected.")

print("[STEP] Performing initial reset...")
PVs["bReset"].put(1, timeout=1.0)
time.sleep(0.4)
PVs["bReset"].put(0, timeout=1.0)
time.sleep(0.4)
err = PVs["bError"].get(timeout=1.0)
errmsg = PVs["sErrorMessage"].get(as_string=True, timeout=1.0)
assert_false(err, f"Error after initial reset. Message: {errmsg}")
print('[PASS] Initial reset: no error, proceeding with state moves.')

STATE_ENUMS = list(range(1, num_states + 1))
current_state = get_current_state()
vprint(f"[INFO] Current state at start: {current_state}")

states_pending = [s for s in STATE_ENUMS if s != current_state]

# 1. Move through all defined states (excluding current)
for state in states_pending:
    move_to_state(state)

# 2. Test halt/reset at random states
# for _ in range(2):
    # ts = random.choice(STATE_ENUMS)
    # test_halt_or_reset_state(target_state=ts, do_halt=True)
    # test_halt_or_reset_state(target_state=ts, do_halt=False)

# 3. Batch random moves with errors/halts/resets
print(f"[STEP] Batch testing {num_moves} random state moves/includes halt/resets")
for i in range(num_moves):
    state = random.choice(STATE_ENUMS)
    movetype = random.choice(['normal', 'halt', 'reset'])
    print(f" [BATCH MOVE] #{i+1}: {movetype}, state={state}")
    if movetype == 'normal':
        move_to_state(state)
    elif movetype == 'halt':
        test_halt_or_reset_state(target_state=state, do_halt=True)
    elif movetype == 'reset':
        test_halt_or_reset_state(target_state=state, do_halt=False)

print("[PASS] All state move, halt, and reset tests complete.")