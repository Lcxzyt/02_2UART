# Hardcoded bit FSM line follow

This document records the current hardcoded line-following controller.
It replaces the previous continuous normalized-error PID steering path for line following.

## Control period

The control period is still 20 ms.

- Motor speed PID runs on the existing 20 ms timing path.
- LineFollow_Update() is called from the main loop when g_SampleReady is set.
- Bluetooth IR stream is printed every 100 ms, so the log rate is lower than the control rate.

Do not treat Bluetooth log spacing as the control period.

## Sensor order and bit format

The four sensors are interpreted in this physical order:

```text
L2 L1 R1 R2
```

The bit weights are:

```text
L2 = 0x8
L1 = 0x4
R1 = 0x2
R2 = 0x1
```

So the readable pattern is:

```text
0110 = L1 and R1 see black, centered
0100 = L1 sees black, line slightly left
0010 = R1 sees black, line slightly right
1000 = L2 sees black, line far left
0001 = R2 sees black, line far right
0000 = all white / lost
1111 = all black / crossing / finish area
```

The Bluetooth parameter line prints these as hex:

```text
LFBits=0x6 LFPat=0x6
```

`LFBits` is the immediate threshold result.
`LFPat` is the debounced accepted pattern.

## Thresholding

Each channel has ON/OFF hysteresis. The current hardcoded defaults are:

```c
#define LF_IR_L2_ON  700U
#define LF_IR_L2_OFF 500U
#define LF_IR_L1_ON  700U
#define LF_IR_L1_OFF 500U
#define LF_IR_R1_ON  700U
#define LF_IR_R1_OFF 500U
#define LF_IR_R2_ON  700U
#define LF_IR_R2_OFF 500U
```

Rule:

```text
filt >= ON  -> bit becomes 1
filt <= OFF -> bit becomes 0
between OFF and ON -> keep previous bit
```

This avoids single-sample jitter around the threshold.

## State debounce

The immediate bit pattern is not accepted at once. It must remain stable for:

```c
#define LF_STATE_STABLE_TICKS 2U
```

At 20 ms per tick, this is about 40 ms.

Lost-line and all-black states also have confirmation delays:

```c
#define LF_LOST_CONFIRM_TICKS 3U
#define LF_BLACK_CONFIRM_TICKS 8U
```

That means:

- Lost must persist for about 60 ms before recovery starts.
- All-black must persist for about 160 ms before black-area behavior is accepted.

## Action table

The current action table is deliberately conservative:

| Pattern | Meaning | Action |
| --- | --- | --- |
| 0110 | centered | straight |
| 0100 | left inner on line | small left |
| 1100 | left pair on line | medium left |
| 1110 | left and center wide | medium left |
| 1000 | far left only | large left |
| 0010 | right inner on line | small right |
| 0011 | right pair on line | medium right |
| 0111 | right and center wide | medium right |
| 0001 | far right only | large right |
| 0000 | all white | recover after confirmation |
| 1111 | all black | black-area handling after confirmation |
| other sparse/conflicting states | uncertain | small correction by last valid direction |

The implementation is in `LineFollow_ActionFromPattern()`.

## Action speed levels

The controller no longer computes line-follow turn output from PID. It uses fixed action levels:

```c
#define LF_BASE_SPEED_DEFAULT 25
#define LF_TURN_LIMIT_DEFAULT 8
#define LF_TURN_SMALL        4
#define LF_TURN_MEDIUM       8
#define LF_TURN_LARGE        12
#define LF_TURN_RECOVER      8
```

The runtime command `u` still sets base speed.
The runtime command `w` still sets the maximum allowed turn.

Because `w` clamps the hardcoded turn level, with `w8`:

```text
small  = 4
medium = 8
large  = 8, because LF_TURN_LARGE is clamped by w
recover = 8
```

This is intentional for first testing, to avoid aggressive swinging.

## Left/right wheel output

A positive turn means right correction:

```text
left  = base + turn
right = base - turn
```

A negative turn means left correction:

```text
left  = base - turn_abs
right = base + turn_abs
```

Both wheel targets are clamped to non-negative values in line-follow mode. The current recovery logic does not reverse either wheel, which avoids repeated in-place spinning after line loss.

## Bluetooth log format

The IR stream line is now:

```text
R0,R1,R2,R3,F0,F1,F2,F3,N0,N1,N2,N3,S,E,B,P
```

Fields:

- `R0-R3`: raw ADC values.
- `F0-F3`: filtered ADC values used by the bit threshold FSM.
- `N0-N3`: old normalized values, kept for observation.
- `S`: old normalized strength, kept for observation.
- `E`: old normalized weighted error, kept for observation.
- `B`: immediate bit pattern as decimal.
- `P`: debounced accepted pattern as decimal.

For example:

```text
...,6,6
```

means immediate bits and accepted pattern are both `0x6`, i.e. `0110` centered.

If reading Bluetooth CSV by eye, convert common decimal values like this:

| Decimal | Hex | Pattern |
| --- | --- | --- |
| 0 | 0x0 | 0000 |
| 1 | 0x1 | 0001 |
| 2 | 0x2 | 0010 |
| 3 | 0x3 | 0011 |
| 4 | 0x4 | 0100 |
| 6 | 0x6 | 0110 |
| 8 | 0x8 | 1000 |
| 12 | 0xC | 1100 |
| 14 | 0xE | 1110 |
| 15 | 0xF | 1111 |

## Files to inspect

Main file:

```text
LineFollow.c
```

Important sections:

- Top macros: thresholds, speed levels, debounce ticks, bit masks, pattern names.
- `LineFollow_UpdateBits()`: filtered ADC values to 4bit black/white pattern.
- `LineFollow_DebouncePattern()`: pattern debounce.
- `LineFollow_ActionFromPattern()`: hardcoded state table.
- `LineFollow_ApplyAction()`: converts action to left/right target speed.
- `LineFollow_Update()`: 20 ms control entry.

Related files:

```text
Tracking.c
Tracking.h
CmdDispatch.c
empty.c
```

- `Tracking.c`: ADC reading and filtering. The FSM currently uses `track->filt[]`.
- `Tracking.h`: `Tracking_Data` structure.
- `CmdDispatch.c`: `x`, `X`, `f`, `u`, `w` command handling and Bluetooth log fields.
- `empty.c`: main loop timing and startup help text.

## Suggested first test

Start conservative:

```text
u25
w8
x
f
```

Watch:

```text
B,P,LFSta,TL,TR,AL,AR
```

Expected basic behavior:

- Centered on line: `P=6`, car drives straight.
- Line moves left: `P=4`, `12`, or `8`, car turns left.
- Line moves right: `P=2`, `3`, or `1`, car turns right.
- Momentary `P=0` should not immediately spin; recovery starts only after confirmation.