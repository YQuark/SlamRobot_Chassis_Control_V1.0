# Chassis Safety and Closed-loop Prerequisites

## Purpose

This note records the safety and measurement rules that must be satisfied before enabling speed-loop and current-loop control on the two-wheel differential chassis.

## Current Development Stage

- The wheel radius is fixed at `CHASSIS_WHEEL_RADIUS_M = 0.035f`.
- The wheel base now uses the measured initial value `CHASSIS_WHEEL_BASE_M = 0.178f`.
- If wheel base is later set to an invalid value, `angular_z != 0` in `SET_VELOCITY` rejects that source frame and clears that source command. If no higher-priority valid source remains, the chassis stays stopped.
- The current stage supports differential `linear_x` + `angular_z` control, but effective wheel base still needs rotation testing and calibration.

## Command Safety Rules

- Emergency stop entry and exit both clear every stored source command.
- Latched fault entry and clear both clear every stored source command.
- During emergency stop or latched fault stop, incoming `SET_VELOCITY` commands are not stored as recoverable commands.
- After emergency stop is cleared or a latched fault is cleared, the chassis must wait for a new accepted `SET_VELOCITY` before moving.
- PWM disable is enforced in the chassis/control layer, not in communication, ADC, or monitor modules.

## Speed Feedback Rules

`EncoderDriver_Update()` uses actual tick delta instead of a fixed task period.

Speed feedback is invalid when:

- `dt <= CHASSIS_MIN_ENCODER_DT_MS`
- `dt > CHASSIS_MAX_ENCODER_DT_MS`
- wheel radius is invalid
- counts-per-revolution is invalid

When speed feedback is invalid:

- `speed_valid = 0`
- left and right speed are set to `0`
- PID must not use the feedback
- chassis control clears PWM output if PID mode is enabled

## Current Sampling Rules

Current sensing applies only to the present low-side sampling circuit:

```text
TB67H450FNG EP/GND/RS node -> 0.1 ohm shunt -> system GND
```

Assumptions:

- no amplifier
- no bias
- ADC reference is `3.3 V`
- current direction is not represented

Formula:

```text
Vadc = raw / 4095.0 * 3.3
I = abs((Vadc - MOTOR_CURRENT_ZERO_V) / MOTOR_CURRENT_VOLTS_PER_AMP)
```

Default parameters:

```text
MOTOR_CURRENT_ZERO_V = 0.0f
MOTOR_CURRENT_VOLTS_PER_AMP = 0.1f
```

If real hardware shows ADC zero offset, calibrate it through `MOTOR_CURRENT_ZERO_V`.

## Shunt Power Check

For the 0.1 ohm shunt:

```text
P = I^2 * R
```

Expected dissipation:

```text
0.65 A rated current -> 0.65^2 * 0.1 = 0.042 W
2.4 A stall current  -> 2.4^2 * 0.1  = 0.576 W
```

The physical shunt resistor power rating must exceed the stall transient dissipation with margin before enabling aggressive current protection or current-loop control.

## Closed-loop Roadmap

1. Current monitoring, filtering, debounce, fault latch, and stop entry.
2. Linear speed loop with equal left/right wheel targets.
3. Differential angular control calibration after effective wheel base is measured.
4. Speed outer loop plus current inner PI loop.

The current PI inner loop remains deferred.
