# Teeter Rotor PX4 v1.13 Gazebo Classic Model

## 1. Project Purpose

This package is a custom Gazebo Classic model for a non-standard two-blade rotorcraft intended for PX4 v1.13 SITL development.

The aircraft concept is not a conventional quadrotor or a conventional swashplate helicopter. The current design concept includes:

- two large rotor blades,
- independent pitch control for each blade,
- no conventional helicopter swashplate,
- GP-76 engine/prop units located about 3 ft from the rotor center,
- engine thrust generating torque to spin the rotor,
- target empty weight around 42 lb,
- adjustable payload,
- design gross weight around 440 lb,
- long-term target payload-to-empty ratio around 10:1.

This is an engineering/debugging simulation model. It is not yet a high-fidelity aerodynamic model.

---

## 2. What We Have Built So Far

### 2.1 Initial SDF/Gazebo Model

We started with a simple Gazebo model containing:

- `base_link`,
- `rotor_link`,
- `main_rotor_joint`,
- a custom Gazebo plugin.

At first, the rotor RPM was forced directly and lift was applied as a simple upward force.

### 2.2 Payload and RPM Variables

We changed the model so that:

- empty vehicle weight is fixed at 42 lb,
- payload is adjustable with `payloadWeightLb`,
- rotor RPM can be tested independently.

This allowed us to test the payload capability concept.

### 2.3 Fixed `kThrust`

We decided to fix `kThrust` instead of recalculating it every time RPM changes.

The final value is:

```text
kThrust = 3.69 N/(rad/s)^2
```

This is based on the fact-sheet design point:

```text
440 lb lift at 220 rpm and 7.27 deg pitch
```

### 2.4 Independent Two-Blade Pitch

We added independent blade pitch variables:

```xml
<blade1PitchDeg>7.27</blade1PitchDeg>
<blade2PitchDeg>7.27</blade2PitchDeg>
```

The plugin computes blade lift separately and applies each blade force at its blade position.

### 2.5 Visual Blade Pitch Indicators

We added green/red visual-only pitch indicators so blade pitch can be seen in Gazebo.

Important:

```text
The indicators do not affect physics.
They are only for visualization and debugging.
```

### 2.6 GP-76 Engine Visuals

We added visible GP-76 engine placeholders at the rotor hub/beam region.

These are simplified visuals, not CAD-accurate models.

### 2.7 Engine Torque Dynamics

We replaced forced RPM with a simple engine/rotor dynamic model:

```text
I * omega_dot = drive_torque - drag_torque
```

where:

```text
drive_torque = numEngines * engineThrottle * engineThrust * engineRadius
drag_torque  = kTorque * omega^2
```

Given:

```text
numEngines = 2
engineThrust = 54 lbf each
engineRadius = 3 ft
```

Full throttle drive torque is:

```text
2 * 54 lbf * 3 ft = 324 ft-lb ≈ 439 N*m
```

This closely matches the design torque required:

```text
Torque Required ≈ 325 ft-lb
```

### 2.8 Tripod Mast

We removed the visible base box and added a tripod-style mast structure between the lower body region and rotor hub area.

This is a visual structural approximation to make the model look closer to the intended design.

### 2.9 Pre-cone

We applied approximately 11 degrees of blade pre-cone.

This is represented in:

- blade visual/collision pose,
- tip weight position,
- blade lift force application point.

---

## 3. Main Design Data Reflected in the Model

| Parameter | Value |
|---|---:|
| Rotor radius | 12 ft |
| Hub radius | 3 ft |
| Number of blades | 2 |
| Chord | 10.5 in |
| Airfoil | NACA 23015 |
| Max RPM | 220 rpm |
| Tip speed | 276 ft/s |
| Tip weight | 2.53 lb each |
| Empty weight | 42 lb |
| Design gross weight | 440 lb |
| Design pitch / collective | 7.27 deg |
| Pitch / collective range | 2 to 12 deg |
| Forward speed metadata | 20 mph |
| Torque required | 325 ft-lb |
| Equivalent shaft HP | 16 hp |
| Engine type | 2x GP-76 |
| Engine thrust | 54 lbf each |
| Engine radius | 3 ft |
| Pre-cone | about 11 deg |
| Teeter range metadata | +/- 15 deg |
| Rotor twist metadata | 6 deg linear |

---

## 4. Current Physics Model

### 4.1 Lift

Current lift model:

```text
lift = kThrust * omega^2 * pitch_factor
```

This is an equivalent rotor model.

It is not yet:

- blade-element aerodynamics,
- airfoil table based,
- induced velocity modeled,
- dynamic inflow modeled.

### 4.2 Pitch Factor

Pitch factor is currently a simple linear scaling between:

```text
zeroLiftPitchDeg = 1.0 deg
hoverPitchDeg    = 7.27 deg
```

This is a stable tuning model, not final airfoil physics.

### 4.3 Engine Dynamics

Current rotor RPM model:

```text
I * omega_dot = drive_torque - kTorque * omega^2
```

Main tuning variables:

```xml
<engineThrottle>1.0</engineThrottle>
<rotorInertiaKgm2>35.0</rotorInertiaKgm2>
<kTorque>0.831</kTorque>
```

### 4.4 Payload

Payload is currently a virtual downward force:

```xml
<payloadWeightLb>360.0</payloadWeightLb>
```

It does not yet affect:

- inertia,
- center of gravity,
- pitch/roll moments due to payload position.

This is acceptable for vertical lift testing but must be improved later.

---

## 5. Important SDF Variables

### Forced RPM vs Engine Dynamics

```xml
<useEngineDynamics>true</useEngineDynamics>
<targetRpm>0.0</targetRpm>
```

If `useEngineDynamics=true`, `targetRpm` is only the initial RPM.

For forced RPM testing:

```xml
<useEngineDynamics>false</useEngineDynamics>
<targetRpm>220.0</targetRpm>
```

### Payload

```xml
<payloadWeightLb>360.0</payloadWeightLb>
```

Suggested values:

```text
0 lb   -> empty vehicle only
200 lb -> medium payload
360 lb -> should lift clearly
398 lb -> near 440 lb design gross
420 lb -> 10:1 payload target, likely overloaded in current model
```

### Blade Pitch

```xml
<blade1PitchDeg>7.27</blade1PitchDeg>
<blade2PitchDeg>7.27</blade2PitchDeg>
```

Suggested tests:

```text
7.27 / 7.27 -> symmetric design pitch
8.5 / 8.5   -> more lift
12.0 / 2.0  -> visible pitch difference and rotating imbalance
```

### Engine

```xml
<engineThrottle>1.0</engineThrottle>
<engineRadiusFt>3.0</engineRadiusFt>
<engineThrustLbEach>54.0</engineThrustLbEach>
<rotorInertiaKgm2>35.0</rotorInertiaKgm2>
<kTorque>0.831</kTorque>
```

### Pre-cone

```xml
<preConeAngleDeg>11.0</preConeAngleDeg>
```

Current implementation includes visual blade pre-cone and pre-cone force application point.

---

## 6. Recommended Validation Tests

### Test 1: Forced RPM Baseline

```xml
<useEngineDynamics>false</useEngineDynamics>
<targetRpm>220.0</targetRpm>
<payloadWeightLb>360.0</payloadWeightLb>
```

Expected:

- rotor spins at 220 rpm,
- vehicle lifts,
- pre-cone is visible,
- engine visuals are visible.

### Test 2: Engine Spin-up

```xml
<useEngineDynamics>true</useEngineDynamics>
<targetRpm>0.0</targetRpm>
<engineThrottle>1.0</engineThrottle>
<payloadWeightLb>360.0</payloadWeightLb>
```

Expected:

- RPM starts near zero,
- RPM increases toward 220 rpm,
- drive torque and drag torque approach balance,
- vehicle lifts after enough RPM.

### Test 3: Half Throttle

```xml
<engineThrottle>0.5</engineThrottle>
```

Expected:

```text
equilibrium RPM ≈ sqrt(0.5) * 220 ≈ 156 rpm
```

Heavy payload may not lift.

### Test 4: Payload Sweep

Try:

```xml
<payloadWeightLb>0.0</payloadWeightLb>
<payloadWeightLb>200.0</payloadWeightLb>
<payloadWeightLb>360.0</payloadWeightLb>
<payloadWeightLb>398.0</payloadWeightLb>
<payloadWeightLb>420.0</payloadWeightLb>
```

Expected:

- lower payload lifts easily,
- 398 lb is near design gross,
- 420 lb is likely beyond current modeled lift at 220 rpm.

---

## 7. Remaining Development Roadmap

### Step 1: Stabilize the Current Manual Gazebo Model

Confirm:

- engine spin-up behaves reasonably,
- RPM settles near expected values,
- payload sweep makes sense,
- pre-cone geometry looks correct,
- GP-76 engine visuals are positioned correctly,
- tripod mast geometry matches the intended design.

### Step 2: Tune Engine / Rotor Dynamics

Use SolidWorks rotor inertia if available:

```xml
<rotorInertiaKgm2>...</rotorInertiaKgm2>
```

Tune:

- spool-up time,
- drag torque coefficient,
- throttle-to-RPM behavior,
- engine response lag.

Future improvements:

- engine RPM curve,
- propeller thrust curve,
- throttle lag,
- torque saturation,
- idle RPM.

### Step 3: Azimuth-Based Cyclic Pitch

This is the next major functional step.

Current blade pitch is fixed:

```text
blade1PitchDeg = constant
blade2PitchDeg = constant
```

For real attitude control, pitch must vary with rotor azimuth:

```text
blade_pitch = collective
            + rollCyclic * cos(azimuth + phase)
            + pitchCyclic * sin(azimuth + phase)
```

This will allow controlled roll and pitch moments.

### Step 4: PX4 Actuator Output Integration

Map PX4 actuator outputs to plugin inputs:

```text
PX4 output 1 -> engineThrottle
PX4 output 2 -> collective
PX4 output 3 -> roll cyclic
PX4 output 4 -> pitch cyclic
```

This requires:

- plugin subscription to actuator controls,
- PX4 airframe setup,
- actuator/mixer configuration.

### Step 5: Custom PX4 Airframe

Create a custom airframe for PX4 v1.13 under:

```text
ROMFS/px4fmu_common/init.d-posix/airframes
```

Configure:

- vehicle startup,
- actuator outputs,
- control allocation / mixer,
- SITL model selection.

### Step 6: Physical Payload Link

Replace virtual payload force with a real payload link:

```text
payload_link
payload_joint
payload mass
payload inertia
payload center-of-gravity offset
```

This is required for realistic attitude dynamics.

### Step 7: Teeter Hinge / Flapping

Current rotor is still a rigid spinning link.

Future model should include:

- teeter hinge,
- +/- 15 degree teeter range,
- flapping response,
- pre-cone with hinge geometry.

This should be added after the simpler model is stable.

### Step 8: Blade-Element Aerodynamics

Replace equivalent lift model with blade-element integration using:

- NACA 23015,
- chord = 10.5 in,
- radius = 12 ft,
- hub radius = 3 ft,
- twist = 6 deg,
- local velocity = omega * r,
- section lift/drag integration.

This is the higher-fidelity final aerodynamic model.

---

## 8. Build Instructions

```bash
cd PX4-Autopilot
make px4_sitl gazebo
```

Manual Gazebo run:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/sitl_gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(dirname $(find build -name "libteeter_rotor_plugin.so" | head -n 1))

gazebo --verbose Tools/sitl_gazebo/worlds/empty.world
```

Insert model:

```text
teeter_rotor
```

---

## 9. Current Limitations

The model currently does not include:

- blade-element aerodynamics,
- airfoil lookup tables,
- real payload inertia,
- true teeter/flapping dynamics,
- PX4 closed-loop actuator control,
- fuel consumption,
- engine RPM curve,
- servo dynamics,
- structural flexibility,
- final ground contact tuning.

---

## 10. Best Next Step

The best next technical step is:

```text
Azimuth-based cyclic pitch
```

Add SDF variables like:

```xml
<collectiveDeg>7.27</collectiveDeg>
<rollCyclicDeg>0.0</rollCyclicDeg>
<pitchCyclicDeg>0.0</pitchCyclicDeg>
<cyclicPhaseOffsetDeg>0.0</cyclicPhaseOffsetDeg>
```

Then compute blade pitch from rotor azimuth every update.

Once manual cyclic works in Gazebo, connect it to PX4 actuator outputs.
