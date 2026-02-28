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
bicep_col = pick_col(df.columns, ["bicep"])        # e.g., "bicep_girth"

# If they include "girth" token explicitly, prefer those
waist_col = pick_col(df.columns, ["waist","girth"]) or waist_col
thigh_col = pick_col(df.columns, ["thigh","girth"]) or thigh_col
bicep_col = pick_col(df.columns, ["bicep","girth"]) or bicep_col

print("Detected columns:")
print("  waist:", waist_col)
print("  thigh:", thigh_col)
print("  bicep:", bicep_col)

if not (waist_col and thigh_col and bicep_col):
    print("\nAll columns:")
    print(list(df.columns))
    raise SystemExit("Couldn't detect required columns. Paste the column list above and I'll map it.")

# Convert to numeric (handles strings like "85" or "85.0")
for c in [waist_col, thigh_col, bicep_col]:
    df[c] = pd.to_numeric(df[c], errors="coerce")

# Clean
d = df.dropna(subset=[waist_col, thigh_col, bicep_col])
d = d[(d[waist_col] > 0) & (d[thigh_col] > 0) & (d[bicep_col] > 0)]

# Ratios (girth ratios == width ratios under circular assumption)
d["waist_to_thigh"] = d[waist_col] / d[thigh_col]
d["waist_to_arm"]   = d[waist_col] / d[bicep_col]

print("\nN used:", len(d))
print("Mean waist/thigh ratio:", d["waist_to_thigh"].mean())
print("Mean waist/arm (bicep) ratio:", d["waist_to_arm"].mean())