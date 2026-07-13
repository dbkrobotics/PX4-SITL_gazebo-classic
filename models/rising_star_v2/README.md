# Rising Star V2 Gazebo Classic Model

## Geometry

| Parameter | Value |
| --- | ---: |
| Teeter beam | 48 in root-to-root |
| Beam half-span / blade start radius | 2 ft |
| Blade length | 12 ft each |
| Nominal rotor radius | 14 ft |
| Nominal rotor diameter | 28 ft |
| Blade pre-cone | 7 deg |
| Teeter hinge limit | +/-15 deg |

The SDF follows the plugin convention:

```text
blade length = rotorRadiusFt - hubRadiusFt
rotor diameter = 2 * rotorRadiusFt
```

Therefore v2 uses `rotorRadiusFt=14` and `hubRadiusFt=2`. The 48 in beam is
represented as a 24 in half-span on each side. Its visible V geometry follows
the 7 deg pre-cone.

The engine thrust-line radius remains 3 ft, matching the existing Rising Star
engine layout. Rotor inertia remains the current tuned 45 kg*m^2 placeholder
until the v2 assembly mass and CG are measured.

## Run

```sh
make px4_sitl_default gazebo-classic_rising_star_v2
```

Payload can be changed with the shared helper:

```sh
Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/set_rising_star_payload.py 300 \
  --sdf Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/rising_star_v2/rising_star_v2.sdf
```
