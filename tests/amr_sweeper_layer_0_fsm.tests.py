#!/usr/bin/env python3
"""amr_sweeper_layer_0_fsm.tests.py

FSM test harness with detailed node lifecycle reporting.

Why your run showed "<no status>"
--------------------------------
Your output indicates the script did not receive any messages on the configured
FSM status topic within the polling windows. To make the test robust, this
version:
- Always prints lifecycle states for all FSM nodes (unconfigured/inactive/active)
- Validates success primarily via lifecycle states (target node becomes ACTIVE)
- Optionally also validates via FSMStatus if it is available

Default behavior (no args)
--------------------------
- namespace: /amr_sweeper
- service:   /amr_sweeper/request_state
- status:    /amr_sweeper/fsm_status  (best-effort; not required)
- plan:      IDLING -> RUNNING -> CHARGING -> FAULT
- timeouts:  call=5s, wait=5s

Exit code:
- 0 if all steps pass
- 1 if any step fails
"""

import argparse
import subprocess
import time
import re
import sys
from typing import Dict, Optional, Tuple, List

STATE_NAME_TO_ID = {"INITIALIZING":0,"IDLING":1,"RUNNING":2,"CHARGING":3,"FAULT":4}
ID_TO_STATE_NAME = {v:k for k,v in STATE_NAME_TO_ID.items()}
TRANSITION_STATE = {0:"STABLE",1:"TRANSITIONING",2:"FAILED"}

STATE_NODES = {
    0: "initializing_state",
    1: "idling_state",
    2: "running_state",
    3: "charging_state",
    4: "fault_state",
}

STATE_DEFAULT_PROFILE_ID = {
    "INITIALIZING": 0,
    "IDLING": 100,
    "RUNNING": 200,
    "CHARGING": 300,
    "FAULT": 400,
}

def sh(cmd: str, timeout: Optional[float]=None) -> Tuple[int,str,str]:
    p = subprocess.run(["bash","-lc",cmd], capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout, p.stderr

def ns_name(namespace: str, name: str) -> str:
    ns = namespace.strip("/")
    n = name.strip("/")
    return f"/{ns}/{n}" if ns else f"/{n}"

def parse_kv_echo(text: str) -> Dict[str,str]:
    out: Dict[str,str] = {}
    for line in text.splitlines():
        m = re.match(r"^([a-zA-Z_][a-zA-Z0-9_]*)\s*:\s*(.*)$", line.strip())
        if not m:
            continue
        k,v = m.group(1), m.group(2)
        out[k] = v.strip().strip("'").strip('"')
    return out

def get_one_status(status_topic: str, echo_timeout_sec: float=2.0) -> Optional[Dict[str,str]]:
    # Try explicit QoS flags (helpful in some setups)
    cmd = (
        f"timeout {echo_timeout_sec} ros2 topic echo -n 1 "
        f"--qos-reliability reliable --qos-durability volatile {status_topic}"
    )
    rc,out,_ = sh(cmd)
    if rc != 0 or not out.strip():
        return None
    return parse_kv_echo(out)

def fmt_status(st: Optional[Dict[str,str]]) -> str:
    if not st:
        return "<no status>"

    cur = st.get("current_state", "?")
    lc  = st.get("current_lifecycle_state", "?")
    prof = st.get("current_profile", "?")
    tr_state = st.get("transitioning_to_profile", "")
    tr_status = st.get("transition_status", "")
    gate = st.get("effective_priority_gate", "")
    age = st.get("priority_age_sec", "")
    msg = st.get("last_message", "")

    return (
        f"state={cur} lifecycle={lc} profile={prof} "
        f"transitioning_to_profile={tr_state} transition_status='{tr_status}' "
        f"gate={gate} age_s={age} msg='{msg}'"
    )

def lifecycle_get(node: str) -> Tuple[bool,str]:
    rc,out,err = sh(f"ros2 lifecycle get {node}", timeout=5.0)
    txt = (out + "\n" + err).strip()
    return rc==0, txt

def lifecycle_state_from_get(output: str) -> str:
    # ros2 lifecycle get /ns/node prints like: "active [3]"
    m = re.search(r"\b(unconfigured|inactive|active|finalized)\b", output.lower())
    return m.group(1) if m else "unknown"

def lifecycle_snapshot(namespace: str) -> Dict[str,str]:
    snap: Dict[str,str] = {}
    for _, node_rel in STATE_NODES.items():
        fq = ns_name(namespace, node_rel)
        ok, txt = lifecycle_get(fq)
        st = lifecycle_state_from_get(txt) if ok else "error"
        snap[fq] = st
    return snap

def print_snapshot(snap: Dict[str,str]) -> None:
    for fq, st in snap.items():
        print(f"  {fq}: {st}")

def service_call_request_state(service: str, target_state: str, target_profile_id: int, target_lifecycle: str, requester: str, priority: int, force: bool, reason: str, timeout_sec: float) -> Tuple[bool,str]:
    force_txt = "true" if force else "false"
    # target_lifecycle is optional; empty => default Active
    payload = (
        f"{{target_state: '{target_state}', "
        f"target_lifecycle: '{target_lifecycle}', "
        f"target_profile_id: {target_profile_id}, "
        f"requester: '{requester}', priority: {priority}, force: {force_txt}, reason: '{reason}'}}"
    )
    cmd = f"timeout {timeout_sec} ros2 service call {service} amr_sweeper_layer_0_fsm/srv/RequestState \"{payload}\""
    rc,out,err = sh(cmd)
    txt = (out + "\n" + err).strip()
    accepted = bool(re.search(r"accepted=\s*True", txt) or re.search(r"accepted:\s*true", txt, re.I))
    return accepted, txt

def wait_for_target_lifecycle_active(namespace: str, target_state: int, timeout_sec: float) -> Tuple[bool, Dict[str,str]]:
    target_node = STATE_NODES[target_state]
    fq_target = ns_name(namespace, target_node)
    deadline = time.time() + timeout_sec
    last_snap: Dict[str,str] = {}
    while time.time() < deadline:
        snap = lifecycle_snapshot(namespace)
        last_snap = snap
        if snap.get(fq_target) == "active":
            return True, snap
        time.sleep(0.2)
    return False, last_snap

def print_step_header(i: int, title: str) -> None:
    print("\n" + "="*72)
    print(f"STEP {i}: {title}")
    print("="*72)

def main(argv: List[str]) -> int:
    # IMPORTANT:
    #   argparse allows abbreviated long options by default. That means "--wait"
    #   is interpreted as an abbreviation of "--wait-timeout" and then argparse
    #   expects a value, which matches the error you observed.
    #   We disable abbreviation and also add a real --wait flag for compatibility
    #   with the command examples.
    ap = argparse.ArgumentParser(allow_abbrev=False)
    ap.add_argument("--namespace", default="amr_sweeper")

    # One of: --plan (comma list), --target (single state), --sequence (state[:dwell] list)
    ap.add_argument("--plan", default=None,
                    help="Comma-separated list of target states, e.g. IDLING,RUNNING")
    ap.add_argument("--target", default=None,
                    help="Single target state, e.g. RUNNING")
    ap.add_argument("--sequence", default=None,
                    help="Sequence with optional dwell times, e.g. IDLING:2,RUNNING:5")
    ap.add_argument("--call-timeout", type=float, default=5.0)
    ap.add_argument("--wait-timeout", type=float, default=5.0)
    ap.add_argument("--wait", action="store_true",
                    help="Compatibility flag: wait for target lifecycle to become ACTIVE.")
    ap.add_argument("--priority", type=int, default=10)
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--reason", default="",
                    help="Reason string forwarded to the RequestState service.")
    ap.add_argument("--status-topic", default="fsm_status",
                    help="FSMStatus topic name (best-effort).")
    args = ap.parse_args(argv)

    fq_service = ns_name(args.namespace, "request_state")
    fq_status  = args.status_topic if args.status_topic.startswith("/") else ns_name(args.namespace, args.status_topic)

    print("AMR Sweeper FSM Test")
    print(f"Service:      {fq_service}")
    print(f"Status topic: {fq_status} (best-effort)")
    print(f"Plan:         {args.plan or args.target or args.sequence or 'IDLING,RUNNING,CHARGING,FAULT'}")
    print(f"Timeouts:     call={args.call_timeout}s wait={args.wait_timeout}s")
    print(f"Options:      priority={args.priority} force={args.force} wait={args.wait or True}")

    rc, out, err = sh("ros2 service list")
    if rc != 0:
        print("ERROR: ros2 CLI failed. Ensure ROS + workspace are sourced.")
        print(err.strip())
        return 2
    if fq_service not in out.splitlines():
        print(f"ERROR: Service not found: {fq_service}")
        return 2

    # Build execution plan.
    # - If --sequence is used, we also record dwell seconds after each step.
    # - If --target is used, the plan is one step.
    # - Else if --plan is used, it is a comma-separated list.
    # - Else fall back to the historical default.
    plan_spec = args.sequence or args.target or args.plan or "IDLING,RUNNING,CHARGING,FAULT"
    steps: List[Tuple[int, float]] = []  # (state_id, dwell_sec)

    if args.sequence:
        parts = [p.strip() for p in args.sequence.split(",") if p.strip()]
        for part in parts:
            if ":" in part:
                st_name, dwell_txt = part.split(":", 1)
                st_name = st_name.strip().upper()
                try:
                    dwell = float(dwell_txt.strip())
                except Exception:
                    print(f"ERROR: Bad dwell time in --sequence element '{part}'.")
                    return 2
            else:
                st_name = part.strip().upper()
                dwell = 0.0

            if st_name not in STATE_NAME_TO_ID:
                print(f"ERROR: Unknown state '{st_name}' in --sequence. Allowed: {list(STATE_NAME_TO_ID)}")
                return 2
            steps.append((STATE_NAME_TO_ID[st_name], dwell))

    elif args.target:
        st_name = args.target.strip().upper()
        if st_name not in STATE_NAME_TO_ID:
            print(f"ERROR: Unknown state '{st_name}' in --target. Allowed: {list(STATE_NAME_TO_ID)}")
            return 2
        steps.append((STATE_NAME_TO_ID[st_name], 0.0))

    else:
        plan_names = [p.strip().upper() for p in plan_spec.split(",") if p.strip()]
        bad = [p for p in plan_names if p not in STATE_NAME_TO_ID]
        if bad:
            print(f"ERROR: Unknown states in --plan: {bad}. Allowed: {list(STATE_NAME_TO_ID)}")
            return 2
        steps = [(STATE_NAME_TO_ID[p], 0.0) for p in plan_names]

    # Initial status + lifecycle
    st0 = get_one_status(fq_status, echo_timeout_sec=2.0)
    print(f"Initial FSMStatus: {fmt_status(st0)}")
    print("Initial lifecycle snapshot:")
    print_snapshot(lifecycle_snapshot(args.namespace))

    passed = 0
    failed = 0

    for idx, (target, dwell_sec) in enumerate(steps, start=1):
        name = ID_TO_STATE_NAME[target]
        print_step_header(idx, f"Request {name} ({target})")

        target_profile_id = STATE_DEFAULT_PROFILE_ID[name]
        accepted, resp = service_call_request_state(
            fq_service, name, target_profile_id, '', requester="fsm_test",
            priority=args.priority, force=args.force,
            reason=(args.reason if args.reason else f"test {name}"),
            timeout_sec=args.call_timeout
        )
        print("Service response:")
        print(resp)

        print("Lifecycle snapshot immediately after request:")
        snap1 = lifecycle_snapshot(args.namespace)
        print_snapshot(snap1)

        if not accepted:
            print("RESULT: FAIL (request not accepted)")
            failed += 1
            continue

        # Wait for lifecycle to reach ACTIVE for the target node
        ok_lc, snap2 = wait_for_target_lifecycle_active(args.namespace, target, args.wait_timeout)
        stN = get_one_status(fq_status, echo_timeout_sec=2.0)
        print(f"FSMStatus (latest): {fmt_status(stN)}")
        print("Lifecycle snapshot after wait:")
        print_snapshot(snap2)

        if not ok_lc:
            print("RESULT: FAIL (target node did not become ACTIVE within timeout)")
            failed += 1
            continue

        print("RESULT: PASS")
        passed += 1

        if dwell_sec > 0.0:
            print(f"Dwell: sleeping {dwell_sec}s")
            time.sleep(dwell_sec)

    print("\n" + "="*72)
    print("FSM TEST SUMMARY")
    print("="*72)
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Total:  {passed+failed}")
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
