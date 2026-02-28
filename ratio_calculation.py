import pandas as pd
import glob
import re

def norm(s: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(s).strip().lower()).strip("_")

def pick_col(cols, must_have):
    """
    Pick the best column that contains all tokens in must_have (after normalization).
    Example must_have=["waist","girth"].
    """
    cols_norm = [(c, norm(c)) for c in cols]
    for c, cn in cols_norm:
        if all(tok in cn for tok in must_have):
            return c
    return None

# Load the three measurements files
files = sorted(glob.glob("bodym_measurements/*_measurements.csv"))
if not files:
    raise SystemExit("No files found. Expected bodym_measurements/train_measurements.csv etc.")

dfs = []
for f in files:
    df = pd.read_csv(f)
    df.columns = [norm(c) for c in df.columns]
    df["__source_file__"] = f
    dfs.append(df)

df = pd.concat(dfs, ignore_index=True)

# Detect columns (BodyM should contain these in some form)
waist_col = pick_col(df.columns, ["waist"])        # e.g., "waist_girth"
thigh_col = pick_col(df.columns, ["thigh"])        # e.g., "thigh_girth"
bicep_col   = pick_col(df.columns, ["bicep"])        # upper arm girth
forearm_col = pick_col(df.columns, ["forearm"])      # lower arm girth
calf_col    = pick_col(df.columns, ["calf"])         # lower leg girth

# If they include "girth" token explicitly, prefer those
waist_col   = pick_col(df.columns, ["waist","girth"])   or waist_col
thigh_col   = pick_col(df.columns, ["thigh","girth"])   or thigh_col
bicep_col   = pick_col(df.columns, ["bicep","girth"])   or bicep_col
forearm_col = pick_col(df.columns, ["forearm","girth"]) or forearm_col
calf_col    = pick_col(df.columns, ["calf","girth"])    or calf_col

print("Detected columns:")
print("  waist:  ", waist_col)
print("  thigh:  ", thigh_col)
print("  bicep:  ", bicep_col)
print("  forearm:", forearm_col)
print("  calf:   ", calf_col)

if not (waist_col and thigh_col and bicep_col and forearm_col and calf_col):
    print("\nAll columns:")
    print(list(df.columns))
    raise SystemExit("Couldn't detect required columns. Paste the column list above and I'll map it.")

# Convert to numeric (handles strings like "85" or "85.0")
all_cols = [waist_col, thigh_col, bicep_col, forearm_col, calf_col]
for c in all_cols:
    df[c] = pd.to_numeric(df[c], errors="coerce")

# Clean
d = df.dropna(subset=all_cols)
d = d[(d[waist_col] > 0) & (d[thigh_col] > 0) & (d[bicep_col] > 0)
      & (d[forearm_col] > 0) & (d[calf_col] > 0)]

# Ratios (girth ratios == width ratios under circular assumption)
d["waist_to_thigh"]   = d[waist_col] / d[thigh_col]
d["waist_to_arm"]     = d[waist_col] / d[bicep_col]
d["waist_to_forearm"] = d[waist_col] / d[forearm_col]
d["waist_to_calf"]    = d[waist_col] / d[calf_col]

print("\nN used:", len(d))
print("Mean waist/thigh ratio:   ", d["waist_to_thigh"].mean())
print("Mean waist/arm (bicep) ratio:", d["waist_to_arm"].mean())
print("Mean waist/forearm ratio: ", d["waist_to_forearm"].mean())
print("Mean waist/calf ratio:    ", d["waist_to_calf"].mean())