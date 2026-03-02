"""
evaluate_scaling.py
-------------------
Simulates the GlamCam avatar scaling pipeline on all three datasets and
computes error % between the scaled avatar's dimensions and each person's actual dimensions.

Scaling pipeline (mirrors AvatarController.cs):
  1. Uniform scale       → all avatar dims scale by userHeight / AVATAR_HEIGHT
  2. Arm length scale    → arm length matches person's arm-length exactly
  3. Leg length scale    → leg length matches person's leg-length exactly
  4. Waist thickness     → waist circ matches person's waist exactly
  5. Limb circumferences → predicted from waist via population ratios (where error comes from)
"""

import pandas as pd
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# AVATAR BASE DIMENSIONS  (measured from the 3D model, in cm)
# ─────────────────────────────────────────────────────────────────────────────
AVATAR_HEIGHT     = 168.33  # cm  head to foot
AVATAR_UPPER_ARM  =  28.97  # cm  shoulder → elbow
AVATAR_LOWER_ARM  =  25.13  # cm  elbow    → wrist
AVATAR_UPPER_LEG  =  38.51  # cm  hip      → knee
AVATAR_LOWER_LEG  =  47.97  # cm  knee     → foot
AVATAR_WAIST_CIRC =  73.41  # cm  waist circumference

AVATAR_ARM_LENGTH = AVATAR_UPPER_ARM + AVATAR_LOWER_ARM   # 54.10 cm
AVATAR_LEG_LENGTH = AVATAR_UPPER_LEG + AVATAR_LOWER_LEG   # 86.48 cm

# Population waist-to-limb circumference ratios (from AvatarController.cs, N=2505)
POPULATION_WAIST_ARM_RATIO     = 2.9527   # waist / bicep   (upper arm)
POPULATION_WAIST_FOREARM_RATIO = 3.3806   # waist / forearm (lower arm)
POPULATION_WAIST_THIGH_RATIO   = 1.6570   # waist / thigh   (upper leg)
POPULATION_WAIST_CALF_RATIO    = 2.3948   # waist / calf    (lower leg)

# Avatar limb circumferences derived from waist + population ratios
AVATAR_BICEP_CIRC   = AVATAR_WAIST_CIRC / POPULATION_WAIST_ARM_RATIO
AVATAR_FOREARM_CIRC = AVATAR_WAIST_CIRC / POPULATION_WAIST_FOREARM_RATIO
AVATAR_THIGH_CIRC   = AVATAR_WAIST_CIRC / POPULATION_WAIST_THIGH_RATIO
AVATAR_CALF_CIRC    = AVATAR_WAIST_CIRC / POPULATION_WAIST_CALF_RATIO


def compute_errors(df):
    df = df.copy()
    required = ["height", "arm_length", "leg_length", "waist", "bicep", "forearm", "thigh", "calf"]
    df = df.dropna(subset=required)
    df = df[(df[required] > 0).all(axis=1)]

    # Scaling factors
    df["uniform_scale"]         = df["height"] / AVATAR_HEIGHT
    df["arm_length_scale"]      = df["arm_length"] / (AVATAR_ARM_LENGTH * df["uniform_scale"])
    df["leg_length_scale"]      = df["leg_length"] / (AVATAR_LEG_LENGTH * df["uniform_scale"])
    df["waist_thickness_scale"] = df["waist"] / (AVATAR_WAIST_CIRC * df["uniform_scale"])
    df["bicep_scale"]   = (df["waist"] / POPULATION_WAIST_ARM_RATIO)     / (AVATAR_BICEP_CIRC   * df["uniform_scale"])
    df["forearm_scale"] = (df["waist"] / POPULATION_WAIST_FOREARM_RATIO) / (AVATAR_FOREARM_CIRC * df["uniform_scale"])
    df["thigh_scale"]   = (df["waist"] / POPULATION_WAIST_THIGH_RATIO)   / (AVATAR_THIGH_CIRC   * df["uniform_scale"])
    df["calf_scale"]    = (df["waist"] / POPULATION_WAIST_CALF_RATIO)    / (AVATAR_CALF_CIRC    * df["uniform_scale"])

    # Predicted circumferences
    df["pred_bicep"]   = AVATAR_BICEP_CIRC   * df["uniform_scale"] * df["bicep_scale"]
    df["pred_forearm"] = AVATAR_FOREARM_CIRC * df["uniform_scale"] * df["forearm_scale"]
    df["pred_thigh"]   = AVATAR_THIGH_CIRC   * df["uniform_scale"] * df["thigh_scale"]
    df["pred_calf"]    = AVATAR_CALF_CIRC    * df["uniform_scale"] * df["calf_scale"]

    # Errors
    for name, actual in [("bicep","bicep"),("forearm","forearm"),("thigh","thigh"),("calf","calf")]:
        df[f"err_{name}"] = (df[f"pred_{name}"] - df[actual]).abs() / df[actual] * 100

    df["mean_err"] = df[["err_bicep","err_forearm","err_thigh","err_calf"]].mean(axis=1)
    return df


# ─────────────────────────────────────────────────────────────────────────────
# LOAD DATA
# ─────────────────────────────────────────────────────────────────────────────
def load(path):
    d = pd.read_csv(path)
    d = d.rename(columns={"arm-length": "arm_length", "leg-length": "leg_length"})
    d["__dataset__"] = path.split("/")[-1].replace("_measurements.csv", "")
    return d

testA = load("bodym_measurements/testA_measurements.csv")
testB = load("bodym_measurements/testB_measurements.csv")
train = load("bodym_measurements/train_measurements.csv")

# Process each dataset separately
dfs = {}
for name, raw in [("testA", testA), ("testB", testB), ("train", train)]:
    dfs[name] = compute_errors(raw)

limbs = ["bicep", "forearm", "thigh", "calf"]

# ─────────────────────────────────────────────────────────────────────────────
# REPORT  (median error only, all datasets combined)
# ─────────────────────────────────────────────────────────────────────────────
all_data = pd.concat(dfs.values(), ignore_index=True)

print(f"── Circumference median error % (all datasets, N={len(all_data)}) ──")
for name in limbs:
    print(f"  {name:<10}  {all_data[f'err_{name}'].median():.2f}%")
print(f"  {'mean_err':<10}  {all_data['mean_err'].median():.2f}%")

# ─────────────────────────────────────────────────────────────────────────────
# EXPORT CSV  (testB + train combined, one file)
# ─────────────────────────────────────────────────────────────────────────────
combined = pd.concat([dfs["testB"], dfs["train"]], ignore_index=True)

out_cols = ["subject_id", "__dataset__",
            "bicep",   "pred_bicep",   "err_bicep",
            "forearm", "pred_forearm", "err_forearm",
            "thigh",   "pred_thigh",   "err_thigh",
            "calf",    "pred_calf",    "err_calf",
            "mean_err"]

out = combined[out_cols].copy().round(4)

# Summary row
summary = {c: "" for c in out_cols}
summary["subject_id"] = "MEDIAN"
for name in limbs:
    summary[f"err_{name}"] = round(combined[f"err_{name}"].median(), 4)
summary["mean_err"] = round(combined["mean_err"].median(), 4)
out = pd.concat([out, pd.DataFrame([summary])], ignore_index=True)

out_path = "bodym_measurements/scaling_errors.csv"
out.to_csv(out_path, index=False)
print(f"\nSaved → {out_path}  (N={len(combined)}  testB + train combined)")
