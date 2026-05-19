"""
DC motor robust-control lab -- RPM-deviation analysis
=====================================================

Reads one or more measurement tables (rows = repeated trials, columns = PWM
values) from .xlsx OR .csv files and, for each file:

  1. computes the MEAN RPM for every PWM column,
  2. computes the RPM deviation about a reference PWM operating point
        d_RPM = mean_RPM(PWM) - mean_RPM(PWM_ref),
  3. computes the per-unit versions
        d_V  (pu) = d_V  / max|d_V|     -> input  normalised to [-1, 1]
        d_RPM(pu) = d_RPM / RPM_ref     -> output as a fraction of nominal RPM
                    (--rpm-pu-mode max forces the output into [-1, 1] instead),
  4. fits a least-squares straight line  y = a*x + b  to each curve,
  5. plots  RPM deviation vs PWM deviation  (absolute and per-unit),
  6. writes a markdown report with every table.

When more than one file is given it also produces a comparison plot that
overlays the per-unit curves of all resistor values -- the fitted slope is the
plant gain, so this chart shows directly how much the gain varies with Rv.

Examples
--------
    # easiest: just run it -- with no arguments it analyses every .xlsx
    # file sitting in the same folder as this script
    python rpm_deviation_analysis.py

    # one specific file
    python rpm_deviation_analysis.py RNom_measurements.xlsx

    # all three, with custom Rv labels (order must match the files)
    python rpm_deviation_analysis.py RNom_measurements.xlsx \\
           RMin_measurements.xlsx RMax_measurements.xlsx \\
           --labels "RvNOM 5,22 kOhm;RvMIN;RvMAX"

Accepts .xlsx / .xlsm / .xls and .csv / .tsv (CSV may use European decimal
commas). A 'MEAN RPM' summary row, if present, is detected and ignored.
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")          # works headless; comment out for an interactive window
import matplotlib.pyplot as plt

PALETTE = ["#3b7dd8", "#cc3333", "#e0a000", "#2a9d4a", "#7d4fc0"]


# --------------------------------------------------------------------------
#  Data loading
# --------------------------------------------------------------------------
def load_table(path):
    """Load a measurement table from .xlsx or .csv.

    Rows are repeated trials, columns are PWM values. Excel files are read
    directly; CSV files are read tolerantly (';', tab or ',' separator and
    decimal comma). A non-measurement row such as 'MEAN RPM' is dropped.
    """
    path = Path(path)
    ext = path.suffix.lower()

    if ext in (".xlsx", ".xlsm"):
        df = pd.read_excel(path, index_col=0)
    elif ext == ".xls":
        df = pd.read_excel(path, index_col=0, engine="xlrd")
    else:                                               # CSV / TSV
        df = None
        for sep in (";", "\t", ","):
            try:
                tmp = pd.read_csv(path, sep=sep, decimal=",", index_col=0)
            except Exception:
                continue
            if tmp.shape[1] >= 2:                       # parsed into real columns
                df = tmp
                break
        if df is None:
            raise ValueError(f"Could not parse '{path}'. Check the file format.")

    # drop summary rows (e.g. 'MEAN RPM') that some exports place at the bottom
    keep = ~df.index.astype(str).str.lower().str.contains("mean", na=False)
    df = df[keep]

    # PWM headers -> integers, so the reference PWM can be looked up
    df.columns = [int(round(float(str(c).replace(",", ".")))) for c in df.columns]

    # force numeric and drop completely empty rows
    df = df.apply(pd.to_numeric, errors="coerce").dropna(how="all")
    df.index.name = "Trial"
    return df


# --------------------------------------------------------------------------
#  Analysis
# --------------------------------------------------------------------------
def analyze(df, ref_pwm=200, rpm_pu_mode="ref"):
    """Return a tidy results table and the reference RPM (RPM_ref)."""
    pwm = np.array(sorted(df.columns))
    mean_rpm = df[pwm].mean(axis=0).to_numpy()      # mean over the repeated trials

    if ref_pwm not in pwm:
        raise ValueError(f"Reference PWM {ref_pwm} is not among {list(pwm)}.")
    rpm_ref = float(mean_rpm[list(pwm).index(ref_pwm)])

    v_dev   = pwm.astype(float) - ref_pwm           # PWM (~voltage) deviation  d_V
    rpm_dev = mean_rpm - rpm_ref                    # RPM deviation             d_RPM

    v_dev_pu = v_dev / np.max(np.abs(v_dev))        # input  -> [-1, 1]

    if rpm_pu_mode == "ref":                        # output as fraction of nominal
        rpm_dev_pu = rpm_dev / rpm_ref
    elif rpm_pu_mode == "max":                      # output forced into [-1, 1]
        rpm_dev_pu = rpm_dev / np.max(np.abs(rpm_dev))
    else:
        raise ValueError("rpm_pu_mode must be 'ref' or 'max'.")

    results = pd.DataFrame({
        "PWM":              pwm,
        "MEAN_RPM":         mean_rpm,
        "V_DEVIATION":      v_dev,
        "RPM_DEVIATION":    rpm_dev,
        "V_DEVIATION_pu":   v_dev_pu,
        "RPM_DEVIATION_pu": rpm_dev_pu,
    })
    return results, rpm_ref


def linear_fit(x, y):
    """Least-squares straight line; returns (slope, intercept)."""
    slope, intercept = np.polyfit(np.asarray(x, float), np.asarray(y, float), 1)
    return float(slope), float(intercept)


# --------------------------------------------------------------------------
#  Plotting  (all titles / labels in English)
# --------------------------------------------------------------------------
def _panel(ax, x, y, slope, intercept, title, xlabel, ylabel, color="#3b7dd8"):
    """Draw one deviation panel: measured points, dotted line and linear fit."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)

    ax.plot(x, y, linestyle=":", marker="o", color=color, label="measured")

    xs = np.linspace(x.min(), x.max(), 100)
    ax.plot(xs, slope * xs + intercept, color="#d8261c", linewidth=2.2,
            label="linear fit")

    sign = "+" if intercept >= 0 else "-"
    ax.text(0.05, 0.90, f"y = {slope:.4f}x {sign} {abs(intercept):.4f}",
            transform=ax.transAxes, fontsize=11, color="#d8261c")

    ax.axhline(0, color="0.7", linewidth=0.8)
    ax.axvline(0, color="0.7", linewidth=0.8)
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=9)


def plot_file(results, fit_abs, fit_pu, label, out_path):
    """One figure per file: absolute and per-unit panels side by side."""
    fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.8))
    _panel(axes[0], results["V_DEVIATION"], results["RPM_DEVIATION"], *fit_abs,
           f"RPM Deviation vs PWM Deviation  (Rv = {label})",
           "PWM Deviation", "RPM Deviation")
    _panel(axes[1], results["V_DEVIATION_pu"], results["RPM_DEVIATION_pu"], *fit_pu,
           f"RPM Deviation vs PWM Deviation, per-unit  (Rv = {label})",
           "PWM Deviation (pu)", "RPM Deviation (pu)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_comparison(entries, ref_pwm, out_path):
    """Overlay the per-unit curves of all resistor values on one chart."""
    fig, ax = plt.subplots(figsize=(7.6, 5.4))
    for i, e in enumerate(entries):
        color = PALETTE[i % len(PALETTE)]
        x = e["results"]["V_DEVIATION_pu"].to_numpy()
        y = e["results"]["RPM_DEVIATION_pu"].to_numpy()
        slope, intercept = e["fit_pu"]
        ax.plot(x, y, linestyle=":", marker="o", color=color, alpha=0.85)
        xs = np.linspace(x.min(), x.max(), 100)
        ax.plot(xs, slope * xs + intercept, color=color, linewidth=2.4,
                label=f"{e['label']}  |  gain = {slope:.4f}")
    ax.axhline(0, color="0.7", linewidth=0.8)
    ax.axvline(0, color="0.7", linewidth=0.8)
    ax.set_title(f"RPM Deviation vs PWM Deviation, per-unit  -  "
                 f"Rv comparison  (reference PWM = {ref_pwm})",
                 fontsize=12, fontweight="bold")
    ax.set_xlabel("PWM Deviation (pu)")
    ax.set_ylabel("RPM Deviation (pu)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# --------------------------------------------------------------------------
#  Markdown report
# --------------------------------------------------------------------------
def _num(v, dec):
    """Format a number for a markdown cell; blank for missing values."""
    if v is None:
        return ""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    if np.isnan(f):
        return ""
    return f"{f:.{dec}f}"


def _md_table(headers, rows):
    """Render a GitHub-flavoured markdown table from string cells."""
    line = lambda cells: "| " + " | ".join(cells) + " |"
    out = [line(headers), line(["---"] * len(headers))]
    out += [line(r) for r in rows]
    return "\n".join(out)


def markdown_section(entry):
    """Build the markdown for one resistor value."""
    label = entry["label"]
    df    = entry["df"]            # raw measurements (index = trial, cols = PWM)
    res   = entry["results"]
    fa, fp = entry["fit_abs"], entry["fit_pu"]
    parts = [f"# {label}\n"]

    # --- Measurements Table -------------------------------------------------
    parts.append("## Measurements Table\n")
    headers = ["Trial"] + [str(c) for c in df.columns]
    rows = [[str(idx)] + [_num(v, 1) for v in row]
            for idx, row in df.iterrows()]
    parts.append(_md_table(headers, rows) + "\n")

    # --- Mean RPM -----------------------------------------------------------
    parts.append("## Mean RPM\n")
    rows = [[str(int(p)), _num(m, 3)]
            for p, m in zip(res["PWM"], res["MEAN_RPM"])]
    parts.append(_md_table(["PWM", "Mean RPM"], rows) + "\n")

    # --- MeanRPM-V_Deviation-RPM_Deviation ---------------------------------
    parts.append("## MeanRPM-V_Deviation-RPM_Deviation\n")
    rows = [[_num(m, 3), _num(v, 0), _num(d, 3)]
            for m, v, d in zip(res["MEAN_RPM"], res["V_DEVIATION"],
                               res["RPM_DEVIATION"])]
    parts.append(_md_table(["Mean RPM", "V_Deviation", "RPM_Deviation"], rows)
                 + "\n")

    # --- V_Deviation-RPM_Deviation PU --------------------------------------
    parts.append("## V_Deviation-RPM_Deviation PU\n")
    rows = [[_num(v, 2), _num(d, 4)]
            for v, d in zip(res["V_DEVIATION_pu"], res["RPM_DEVIATION_pu"])]
    parts.append(_md_table(["V_Deviation (pu)", "RPM_Deviation (pu)"], rows)
                 + "\n")

    # --- Plots --------------------------------------------------------------
    parts.append("## Plots\n")
    parts.append(f"Linear fit (absolute): `y = {fa[0]:.4f} x + {fa[1]:.4f}`  ")
    parts.append(f"Linear fit (per-unit): `y = {fp[0]:.4f} x + {fp[1]:.4f}`\n")
    parts.append(f"![{label} deviation plots]({entry['plot_name']})\n")

    return "\n".join(parts)


def write_markdown(entries, summary_df, comparison_png, ref_pwm, out_path):
    """Write the full markdown report."""
    doc = [f"*RPM-deviation analysis for the DC-motor robust-control lab "
           f"(reference PWM = {ref_pwm}).*\n"]

    for e in entries:
        doc.append(markdown_section(e))

    if comparison_png is not None and len(entries) > 1:
        doc.append("# Rv Comparison\n")
        headers = ["File", "Rv", "RPM_ref", "slope (abs)", "per-unit gain"]
        rows = []
        for _, r in summary_df.iterrows():
            rows.append([str(r["file"]), str(r["Rv"]),
                         _num(r["Omega_ref"], 3), _num(r["slope_abs"], 4),
                         _num(r["gain_pu"], 4)])
        doc.append(_md_table(headers, rows) + "\n")
        g = summary_df["gain_pu"]
        doc.append(f"Per-unit gain spread: **{g.min():.4f} ... {g.max():.4f}** "
                   f"(max/min = {g.max() / g.min():.3f}).\n")
        doc.append(f"![Rv comparison]({comparison_png})\n")

    Path(out_path).write_text("\n".join(doc), encoding="utf-8")
    return out_path


# --------------------------------------------------------------------------
#  Main
# --------------------------------------------------------------------------
def clean_label(path):
    """Derive a readable Rv label from a file name."""
    stem = Path(path).stem.lower()
    if "nom" in stem:
        return "RvNOM"
    if "max" in stem:
        return "RvMAX"
    if "min" in stem:
        return "RvMIN"
    s = Path(path).stem
    for junk in ("_measurements", "_measurement", "_data"):
        s = s.replace(junk, "")
    return s.replace("_", " ").strip()


def order_key(label):
    """Sort key so sections come out NOM, MAX, MIN (the preferred order)."""
    l = label.lower()
    if "nom" in l:
        return (0, label)
    if "max" in l:
        return (1, label)
    if "min" in l:
        return (2, label)
    return (3, label)


def discover_files(folder):
    """Find measurement workbooks sitting next to the script.

    Used when the script is launched with no file arguments (e.g. the
    'Run' button in an editor). Picks up .xlsx/.xlsm; if there are none it
    falls back to .csv, skipping the *_analysis.csv / *_summary.csv files
    that this script itself writes.
    """
    folder = Path(folder)
    found = sorted(folder.glob("*.xlsx")) + sorted(folder.glob("*.xlsm"))
    if not found:
        found = sorted(p for p in folder.glob("*.csv")
                       if not p.stem.endswith(("_analysis", "_summary")))
    return [str(p) for p in found]


def main():
    ap = argparse.ArgumentParser(
        description="RPM-deviation analysis for the DC-motor robust-control lab.")
    ap.add_argument("files", nargs="*",
                    help="one or more .xlsx or .csv measurement files. "
                         "If omitted, every .xlsx file sitting next to this "
                         "script is used automatically.")
    ap.add_argument("--labels", default=None,
                    help="semicolon-separated labels, one per file (order must "
                         "match); default = derived from each file name. "
                         "Semicolon is used so labels may contain decimal commas, "
                         'e.g.  --labels "RvNOM 5,22 kOhm;RvMIN;RvMAX"')
    ap.add_argument("--ref-pwm", type=int, default=200,
                    help="PWM operating point used as the deviation origin")
    ap.add_argument("--rpm-pu-mode", choices=["ref", "max"], default="ref",
                    help="per-unit RPM base: 'ref' = divide by RPM_ref "
                         "(matches your spreadsheet), 'max' = divide by max|d_RPM| "
                         "(forces the curve into [-1, 1])")
    ap.add_argument("--outdir", default=None,
                    help="folder for the output files "
                         "(default: the folder this script lives in)")
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent

    # no files given (e.g. 'Run' button) -> use the workbooks next to the script
    files = list(args.files)
    if not files:
        files = discover_files(script_dir)
        if not files:
            ap.error("no measurement files given, and no .xlsx/.csv files were "
                     f"found next to the script:\n  {script_dir}\n"
                     "Put the workbooks in that folder, or pass them explicitly.")
        print("No files were given - using the workbooks found next to the script:")
        for f in files:
            print(f"  {Path(f).name}")

    outdir = Path(args.outdir) if args.outdir else script_dir
    outdir.mkdir(parents=True, exist_ok=True)

    if args.labels:
        labels = [s.strip() for s in args.labels.split(";")]
        if len(labels) != len(files):
            ap.error(f"{len(labels)} labels given for {len(files)} files.")
    else:
        labels = [clean_label(f) for f in files]

    # process NOM, MAX, MIN in that order so every output is consistent
    pairs = sorted(zip(files, labels), key=lambda fl: order_key(fl[1]))

    pd.set_option("display.float_format", lambda v: f"{v:9.4f}")
    entries, summary = [], []

    for path, label in pairs:
        df = load_table(path)
        results, rpm_ref = analyze(df, ref_pwm=args.ref_pwm,
                                   rpm_pu_mode=args.rpm_pu_mode)
        fit_abs = linear_fit(results["V_DEVIATION"],    results["RPM_DEVIATION"])
        fit_pu  = linear_fit(results["V_DEVIATION_pu"], results["RPM_DEVIATION_pu"])

        print(f"\n{'='*70}\nFile: {path}   (Rv = {label})")
        print(f"trials: {len(df)}   reference PWM: {args.ref_pwm}   "
              f"RPM_ref: {rpm_ref:.3f}\n")
        print(results.to_string(index=False))
        print(f"\n  RPM dev = f(PWM dev)        ->  y = "
              f"{fit_abs[0]:.4f} x + {fit_abs[1]:.4f}")
        print(f"  RPM dev = f(PWM dev) (pu)   ->  y = "
              f"{fit_pu[0]:.4f} x + {fit_pu[1]:.4f}")

        stem = Path(path).stem
        plot_name = f"{stem}_plots.png"
        plot_file(results, fit_abs, fit_pu, label, outdir / plot_name)
        results.to_csv(outdir / f"{stem}_analysis.csv",
                       sep=";", decimal=",", index=False)

        entries.append({"label": label, "df": df, "results": results,
                        "rpm_ref": rpm_ref, "fit_abs": fit_abs,
                        "fit_pu": fit_pu, "plot_name": plot_name})
        summary.append({"file": Path(path).name, "Rv": label,
                        "Omega_ref": rpm_ref,
                        "slope_abs": fit_abs[0], "intercept_abs": fit_abs[1],
                        "gain_pu": fit_pu[0],   "intercept_pu": fit_pu[1]})

    summary_df = pd.DataFrame(summary)
    comparison_png = None

    # comparison across resistor values
    if len(entries) > 1:
        comparison_png = "Rv_comparison_pu.png"
        plot_comparison(entries, args.ref_pwm, outdir / comparison_png)
        summary_df.to_csv(outdir / "Rv_comparison_summary.csv",
                          sep=";", decimal=",", index=False)
        print(f"\n{'='*70}\nGAIN COMPARISON (per-unit slope)\n")
        print(summary_df.to_string(index=False))
        g = summary_df["gain_pu"]
        print(f"\n  gain spread: {g.min():.4f} ... {g.max():.4f}   "
              f"(max/min = {g.max()/g.min():.3f})")

    # markdown report with every table
    report = write_markdown(entries, summary_df, comparison_png,
                            args.ref_pwm, outdir / "rpm_deviation_report.md")

    print(f"\nMarkdown report: {report}")
    print(f"All outputs written to: {outdir.resolve()}")


if __name__ == "__main__":
    main()