# Two-Level Pedestrian Model (SocialForceModelIPP)

Based on: *"When legs and bodies synchronize: Two-level collective dynamics in dense crowds"*
(Chatagnon, Chraibi, Pettré, Seyfried, Tordeux, 2026)

Reference implementation: [NetLogo](https://antoinetordeux.github.io/Two-Level-Pedestrian-Model)

## Overview

Each agent has two coupled components:
- **Upper body** — position `x_n`, velocity `v_n`
- **Ground support (legs)** — position `xℓ_n`, velocity `vℓ_n`

The coupling direction vector is:

```
e_n = (x_n − xℓ_n) / |x_n − xℓ_n|     (from legs toward upper body)
```

## Equations of Motion

### Upper body (Eq. 1 + driving force)

```
v̇_n = (v0·ê0 − v_n)/τ  +  λ_u·(−v_s·e_n − v_n)  −  λ·v_n  +  Σ_{m≠n} F_ub(x_n, x_m)  +  Σ_w F_ub(x_n, x_w)
       ├── driving force ──┤  ├── unbalancing ──────┤  ├─ damping ┤  ├── agent repulsion ─────┤  ├── wall repulsion ──┤
```

**Note:** The unbalancing term uses `−v_s·e_n` (pointing **away** from the legs, i.e. toward the
direction the body is "falling"). This is confirmed by the NetLogo code:
`- Balance_speed * sin(towards own-leg)` = velocity directed away from the leg.

### Ground support / legs (Eq. 2)

```
v̇ℓ_n = λ_b·(v_s·e_n − vℓ_n)  +  Σ_{m≠n} Fℓ(xℓ_n, xℓ_m)  +  Σ_w Fℓ(xℓ_n, x_w)
        ├── balance recovery ──┤  ├── agent leg repulsion ───┤  ├── wall leg repulsion ─┤
```

**Note:** The balance recovery uses `+v_s·e_n` (pointing **toward** the upper body). The legs
"chase" the upper body to restore balance.

### Position updates (Euler integration, dt = 0.01)

```
v_n(t+dt)  = v_n(t)  + v̇_n·dt
x_n(t+dt)  = x_n(t)  + v_n(t+dt)·dt

vℓ_n(t+dt) = vℓ_n(t) + v̇ℓ_n·dt
xℓ_n(t+dt) = xℓ_n(t) + vℓ_n(t+dt)·dt
```

## Interaction Forces

Both levels use exponential repulsion. The force on agent `n` from agent `m`:

```
F_ub(x_n, x_m) = A · exp(−|x_n − x_m| / B)  · n̂_nm       (upper body)
Fℓ(xℓ_n, xℓ_m) = A · exp(−|xℓ_n − xℓ_m| / Bℓ) · n̂_nm    (legs)
```

Where `n̂_nm = (x_n − x_m)/|x_n − x_m|` points away from the neighbor (repulsive).

**No contact forces.** All interactions are purely exponential.

## Self-Propulsion Mechanism

The two-level coupling creates spontaneous locomotion **without an explicit driving force**:

1. Upper body "falls" away from legs (unbalancing: `−v_s·e_n`)
2. Legs "chase" the upper body to restore balance (recovery: `+v_s·e_n`)
3. This creates a sustained walking cycle

In JuPedSim we **add** a driving force `(v0·ê0 − v_n)/τ` to steer agents toward their
destination. The self-propulsion mechanism still operates on top of this.

## Parameters

### NetLogo Reference Values

From the NetLogo implementation (slider defaults):

| NetLogo name | Symbol | Default | Description |
|-------------|--------|---------|-------------|
| `Unbalancing_rate` | `λ_u` | 0.5 | Upper body coupling rate |
| `Balancing_rate` | `λ_b` | 1.0 | Leg coupling rate |
| `Balance_speed` | `v_s` | 1.0 | Coupling speed (same for both levels) |
| (hardcoded) | `λ` | 1.0 | Damping (`−speedx` term) |
| (hardcoded) | `A` | 5.0 | Repulsion amplitude (both levels) |
| (hardcoded) | `B` | 0.5 | Upper body interaction range |
| (hardcoded) | `Bℓ` | 0.3 | Leg interaction range (shorter!) |
| (hardcoded) | `dt` | 0.1 | Time step |

**Key observation:** `Balance_speed` is used for **both** unbalancing and balancing with the
same magnitude — only the direction differs (away from legs vs. toward upper body). This is a
single parameter `v_s`, not two separate `v_u`/`v_b`.

### JuPedSim Parameters

#### Model-level (no per-agent override)

| Symbol | Code name | Default | Unit | Description |
|--------|-----------|---------|------|-------------|
| — | `cutoff_radius` | 2.5 | m | Neighborhood search radius |

#### Agent-level

| Symbol | Code name | Default | Unit | Description |
|--------|-----------|---------|------|-------------|
| `v0` | `desired_speed` | 1.34 | m/s | Driving force target speed |
| `τ` | `reaction_time` | 0.5 | s | Driving force relaxation time |
| `λ_u` | `lambda_u` | 0.5 | 1/s | Unbalancing rate |
| `λ_b` | `lambda_b` | 1.0 | 1/s | Balancing rate |
| `v_s` | `balance_speed` | 1.0 | m/s | Coupling speed (unbalancing & recovery) |
| `λ` | `damping` | 1.0 | 1/s | Upper body velocity dissipation |
| `A` | `agent_scale` | 5.0 | N | Repulsion amplitude (agents + walls) |
| `B` | `force_distance` | 0.5 | m | Upper body interaction range |
| `Bℓ` | `leg_force_distance` | 0.3 | m | Leg interaction range |
| `r` | `radius` | 0.15 | m | Upper body display radius |
| `h` | `height` | 1.75 | m | Agent height (scales leg radius) |

**Removed** (not in paper model): `mass`, `bodyForce`, `friction`, `obstacleScale`

**Derived:** Leg display radius = `radius × GS_SCALING_FACTOR × height`

## Comparison: NetLogo ↔ Current C++ Implementation

| Aspect | NetLogo (reference) | Current C++ code | Action needed |
|--------|-------------------|------------------|---------------|
| Driving force | None | `(v0·ê0 − v)/τ` | **Keep** (JuPedSim needs goal navigation) |
| Unbalancing | `λ_u·(−v_s·e_n − v_n)` | `LAMBDA_2·(e_n − v_n)` | **Fix:** negate `e_n`, add `v_s` scaling |
| Damping | `−1.0·v_n` | `−LAMBDA_3·v_n` | ✓ Correct (value matches) |
| Balance recovery | `λ_b·(v_s·e_n − vℓ_n)` | `LAMBDA_RECOVERY_2·(e_n − vℓ_n)` | **Fix:** add `v_s` scaling |
| UB interaction | `5·exp(−d/0.5)` | `A·exp(−d/B)` social + contact forces | **Fix:** remove contacts, use A=5 B=0.5 |
| Leg interaction | `5·exp(−d/0.3)` | Contact forces only | **Fix:** add exponential with Bℓ=0.3 |
| Separate Bℓ | Yes (0.3 vs 0.5) | No | **Fix:** add `leg_force_distance` |
| Mass division | No | Social forces ÷ mass | **Fix:** remove mass division |
| Contact forces | None | bodyForce + friction | **Fix:** remove entirely |
| Single v_s | Yes | No speed scaling | **Fix:** one `balance_speed` param |

## Implementation Checklist

### C++ core (`libsimulator/src/`)

- [ ] `SocialForceModelIPPData.hpp`: replace params
  - [ ] Remove: `mass`, `agentScale`, `obstacleScale`, `forceDistance`
  - [ ] Add: `lambdaU`, `lambdaB`, `balanceSpeed`, `damping`, `agentScale`, `forceDistance`, `legForceDistance`
- [ ] `SocialForceModelIPP.hpp`:
  - [ ] Remove: `bodyForce`, `friction` constructor params
  - [ ] Remove: `DEFAULT_BODY_FORCE`, `DEFAULT_FRICTION` constants
  - [ ] Remove: `LAMBDA_LOCOMOTION_2`, `LAMBDA_LOCOMOTION_3`, `LAMBDA_RECOVERY_2` (now per-agent)
  - [ ] Remove: contact force function declarations
- [ ] `SocialForceModelIPP.cpp` — rewrite `ComputeNewPosition()`:
  - [ ] Unbalancing: `λ_u · (−v_s·e_n − v_n)` (note minus sign!)
  - [ ] Damping: `−λ·v_n`
  - [ ] Balance recovery: `λ_b · (v_s·e_n − vℓ_n)`
  - [ ] Upper body repulsion: exponential only (no contacts)
  - [ ] Leg repulsion: exponential with `legForceDistance`
  - [ ] Wall repulsion: exponential for both levels
  - [ ] No mass division
  - [ ] Remove all contact force functions
- [ ] `SocialForceModelIPP.cpp` — update `CheckModelConstraint()`
- [ ] `SocialForceModelIPPBuilder.cpp/.hpp`: update parameter list

### pybind11 (`python_bindings_jupedsim/`)

- [ ] `social_force_model_IPP.cpp`:
  - [ ] Model: remove `body_force`/`friction` from constructor
  - [ ] Agent params: add `lambda_u`, `lambda_b`, `balance_speed`, `damping`, `leg_force_distance`
  - [ ] Agent params: remove `mass`
  - [ ] State: expose new params as read/write properties

### Python wrapper (`python_modules/jupedsim/`)

- [ ] `jupedsim/models/social_force_model_ipp.py`: update class + params
- [ ] `jupedsim/serialization.py`: update writer columns if needed

### Example + tests

- [ ] `examples/2levels_model/sim_bottleneck.py`: update to new API
- [ ] Update/add tests for new parameter validation
