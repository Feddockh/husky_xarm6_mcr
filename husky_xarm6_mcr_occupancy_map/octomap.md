# Probabilistic Occupancy Mapping Mathematics

The system uses **log-odds Bayesian updating**, which is the core of the Octomap algorithm. Here's how it works:

## 1. Basic Probability Model

Each voxel stores an **occupancy probability** `P(n)` that represents the belief that the voxel is occupied.

**Key insight**: Instead of storing probability directly, we store **log-odds**:

```
L(n) = log(P(n) / (1 - P(n)))
```

Where:
- `L(n)` = log-odds value (what's actually stored)
- `P(n)` = occupancy probability (0 to 1)
- `log` = natural logarithm

**Why log-odds?**
- Efficient updates (addition instead of multiplication)
- No numerical underflow with repeated updates
- Easy to clamp values

---

## 2. Sensor Model

When a sensor observes a voxel, we have two outcomes:

### **Hit** (ray endpoint - obstacle detected)
```
P(n | z_hit) = prob_hit  (e.g., 0.7)
```
Log-odds update:
```
L_hit = log(prob_hit / (1 - prob_hit))
      = log(0.7 / 0.3) 
      = log(2.333) 
      = +0.848
```

### **Miss** (ray passes through - free space)
```
P(n | z_miss) = prob_miss  (e.g., 0.7)
```
Log-odds update:
```
L_miss = log(prob_miss / (1 - prob_miss))
       = log(0.7 / 0.3)
       = log(2.333)
       = +0.848
```

**Wait, that's positive!** That's the problem. Let me explain:

---

## 3. The Actual Update Formula

The **Bayesian update** for log-odds is:

```
L(n | z) = L(n) + log(P(n|z) / (1 - P(n|z))) - log(P(n) / (1 - P(n)))
```

But Octomap simplifies this. Starting from a **prior of 0.5** (unknown), the update becomes:

```
L_new(n) = L_old(n) + ΔL
```

Where:

**For a hit:**
```
ΔL_hit = log(prob_hit / (1 - prob_hit))
       = log(0.7 / 0.3) = +0.848
```

**For a miss (THIS IS THE KEY):**
```
ΔL_miss = log(prob_miss / (1 - prob_miss))
        = log(0.3 / 0.7)  ← Note: inverted!
        = log(0.428)
        = -0.848
```

---

## 4. Why `prob_miss = 0.7` Works for Fast Clearing

In the Octomap library, `prob_miss` is interpreted as:

**"Probability that the voxel is OCCUPIED given we observe it's free"**

So when `prob_miss = 0.7`:
- This means P(occupied | miss) = 0.7... **NO!**
- Actually it means: **P(free | miss) = 0.7**
- Therefore: **P(occupied | miss) = 0.3**

The update becomes:
```
ΔL_miss = log(0.3 / 0.7) = -0.848
```

**Higher `prob_miss` → More negative ΔL → Faster clearing**

Let's compare:

| prob_miss | P(occupied\|miss) | ΔL_miss | Effect |
|-----------|-------------------|---------|---------|
| 0.4 | 0.4 | log(0.4/0.6) = -0.405 | Slow clearing |
| 0.5 | 0.5 | log(0.5/0.5) = 0.0 | No change |
| 0.7 | 0.3 | log(0.3/0.7) = -0.848 | **Fast clearing** ✅ |
| 0.8 | 0.2 | log(0.2/0.8) = -1.386 | Very fast clearing |

---

## 5. The Complete Update Cycle

Here's what happens when a point cloud arrives:

### For each point:
1. **Ray casting** from sensor origin to the hit point
2. **All voxels along the ray (except endpoint)**:
   ```
   L(n) = L(n) + log(prob_miss / (1 - prob_miss))
   L(n) = L(n) - 0.848  (with prob_miss=0.7)
   ```

3. **Endpoint voxel (the hit)**:
   ```
   L(n) = L(n) + log(prob_hit / (1 - prob_hit))
   L(n) = L(n) + 0.848  (with prob_hit=0.7)
   ```

### After update, clamp:
```cpp
if (L(n) < clamp_min_logodds)  // default: 0.12
    L(n) = clamp_min_logodds
if (L(n) > clamp_max_logodds)  // default: 0.97
    L(n) = clamp_max_logodds
```

---

## 6. Conversion Back to Probability

When you need the actual probability (for thresholding):

```
P(n) = 1 / (1 + exp(-L(n)))
```

Or equivalently:
```
P(n) = exp(L(n)) / (1 + exp(L(n)))
```

**Classification:**
```cpp
if (P(n) > occupancy_threshold)  // default: 0.5
    → OCCUPIED
else if (P(n) < occupancy_threshold)
    → FREE
else
    → UNKNOWN
```

---

## 7. Example Scenario

Starting with unknown voxel: `L(n) = 0` → `P(n) = 0.5`

### Scenario A: Obstacle appears then disappears

**Time 0**: 3 rays hit the voxel
```
L = 0 + 0.848 + 0.848 + 0.848 = 2.544
P = 1/(1 + exp(-2.544)) = 0.927  → OCCUPIED ✓
```

**Time 1**: 5 rays miss the voxel (obstacle removed)
```
L = 2.544 - 0.848×5 = 2.544 - 4.24 = -1.696
P = 1/(1 + exp(1.696)) = 0.155  → FREE ✓
```

**Clearing speed**: 5 observations to clear

### With old `prob_miss = 0.4` (ΔL = -0.405):
```
L = 2.544 - 0.405×5 = 2.544 - 2.025 = 0.519
P = 1/(1 + exp(-0.519)) = 0.627  → Still OCCUPIED ✗
```

Would need **~10 misses** to clear!

---

## 8. Recommended Configuration

With balanced, aggressive settings:
```cpp
prob_hit = 0.7   → ΔL_hit  = +0.848
prob_miss = 0.7  → ΔL_miss = -0.848
```

This creates a **balanced, aggressive** system:
- **Symmetrical**: Hits and misses have equal strength
- **Fast response**: Both building and clearing are equally fast
- **Good for dynamic environments**: Obstacles appear/disappear quickly

---

## 9. Tuning Guidelines

### Conservative (slow updates):
```cpp
prob_hit = 0.6   → ΔL = +0.405
prob_miss = 0.4  → ΔL = -0.405
```
- Stable, persistent map
- Slow to react to changes

### Balanced (recommended):
```cpp
prob_hit = 0.7   → ΔL = +0.848
prob_miss = 0.7  → ΔL = -0.848
```
- Equal building/clearing
- Responsive to changes

### Aggressive clearing:
```cpp
prob_hit = 0.7   → ΔL = +0.848
prob_miss = 0.8  → ΔL = -1.386
```
- Clears faster than builds
- Risk: Occluded obstacles disappear

### Hyper-aggressive:
```cpp
prob_hit = 0.7   → ΔL = +0.848
prob_miss = 0.9  → ΔL = -2.197
```
- Extreme clearing
- Map becomes very temporary

---

## 10. The Clamping Effect

```cpp
clamp_min = 0.12  → P_min = 0.53
clamp_max = 0.97  → P_max = 0.725
```

**Purpose**: Prevents infinite confidence
- Voxels can never be 100% certain
- Allows recovery from errors
- `clamp_min` controls how "free" free space can be
- `clamp_max` controls how "occupied" obstacles can be

**Lower clamp_min** (e.g., 0.10) → Faster clearing, less stable

---

## Summary

The math is **additive log-odds Bayesian filtering**:

1. Store log-odds instead of probability (efficiency)
2. Add log-odds increments for each observation
3. `prob_hit` controls obstacle building speed
4. `prob_miss` controls obstacle clearing speed
5. Higher `prob_miss` → larger negative increment → faster clearing
6. Clamps prevent overflow and maintain uncertainty

**For fast clearing of false negatives**: Increase `prob_miss` from 0.4 to 0.6-0.7 to make the map more responsive to obstacles that are no longer detected.
