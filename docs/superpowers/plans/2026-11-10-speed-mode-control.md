# Speed Mode Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the gimbal sliders send direct MAVLink speed-mode commands and reliably send a zero-speed command when released.

**Architecture:** The worker retains the two current slider values as normalized percentages and sends them directly as pitch/yaw command parameters. It configures both axes for MAVLink angular-rate input and tracks whether a zero-speed command must be emitted after motion stops. The UI owns display text and slider ranges only.

**Tech Stack:** Python 3, PyQt5, pyserial, generated MAVLink dialect.

**Spec:** `docs/superpowers/specs/2026-11-10-speed-mode-control-design.md`

## Global Constraints

- Pitch and yaw MAVLink input modes use value `1` (angular rate).
- Slider range and displayed unit use `-100%` to `+100%` only.
- `MAV_CMD_DO_MOUNT_CONTROL` sends direct current values and does not integrate an angle target.
- A transition to both axes at zero sends one explicit zero-speed command.

## Review Focus

- A zero command is sent after a nonzero yaw command even if no `MOUNT_ORIENTATION` message arrives.
- A zero command is sent after a nonzero pitch command even if no `MOUNT_ORIENTATION` message arrives.
- If one axis is released while the other remains nonzero, the next command preserves the moving axis value.
- Repeated idle control cycles do not flood the serial link with zero commands.
- The control panel has no Center button; returning both sliders to zero is the stop action.

---

### Task 1: Direct speed-mode command worker

**Files:**
- Modify: `serial_mavlink_ui.py:63-245`
- Create: `tests/test_serial_mavlink_ui.py`

**Interfaces:**
- Produces: `SerialMavlinkWorker._send_attitude_control(dt)` sends current pitch/yaw values as direct command parameters and sends exactly one zero command after an active command stops.
- Produces: `SerialMavlinkWorker._send_mount_configure()` configures pitch/yaw input mode as angular rate.

- [ ] **Step 1: Write failing worker regression tests**

Create tests with a fake MAVLink sender that assert the mount configuration has parameter 6 and parameter 7 equal to `1`, a nonzero yaw command uses the same yaw value as the worker state, and the first idle cycle after motion sends a command with pitch and yaw equal to `0`.

- [ ] **Step 2: Run the regression tests to verify they fail**

Run: `python3 -m unittest tests.test_serial_mavlink_ui -v`

Expected: FAIL because the implementation still configures angle mode, integrates angle targets, and suppresses the stop command.

- [ ] **Step 3: Implement direct speed commands**

Modify `SerialMavlinkWorker` to remove target-angle synchronization/integration from its control path, configure parameters 6 and 7 as `1`, and maintain only enough state to emit a one-time zero command on transition from active to idle.

- [ ] **Step 4: Run worker regression tests to verify they pass**

Run: `python3 -m unittest tests.test_serial_mavlink_ui -v`

Expected: PASS.

### Task 2: Percentage-only UI

**Files:**
- Modify: `serial_mavlink_ui.py:344-550`
- Modify: `README.md:47-57`
- Modify: `tests/test_serial_mavlink_ui.py`

**Interfaces:**
- Consumes: `SerialMavlinkWorker.set_control_percentages(yaw_value, pitch_value)` as direct `-100` to `100` speed proportions.
- Produces: yaw and pitch sliders with a `-100` to `100` range and percent-only labels.

- [ ] **Step 1: Extend the failing tests for UI control values**

Assert the control limits are `-100` and `100`, and that published worker values retain the selected percentage without degree conversion.

- [ ] **Step 2: Run the regression tests to verify they fail**

Run: `python3 -m unittest tests.test_serial_mavlink_ui -v`

Expected: FAIL because sliders still use degree-per-second limits and labels.

- [ ] **Step 3: Implement percentage UI and documentation**

Set both slider ranges to `-100` through `100`, change value labels to `%`, rename internal UI variables to remove degree terminology where practical, and update README control documentation.

- [ ] **Step 4: Run full verification**

Run: `python3 -m unittest discover -v && python3 -m py_compile serial_mavlink_ui.py && git diff --check`

Expected: all tests pass, the module compiles, and the diff has no whitespace errors.

- [ ] **Step 5: Commit**

```bash
git add serial_mavlink_ui.py README.md tests/test_serial_mavlink_ui.py docs/superpowers/specs/2026-11-10-speed-mode-control-design.md docs/superpowers/plans/2026-11-10-speed-mode-control.md
git commit -m "Use direct gimbal speed control"
```
