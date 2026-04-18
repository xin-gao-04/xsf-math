#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
from html import escape
from pathlib import Path


def load_json(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def load_events(path: Path) -> list[dict]:
    if not path.exists():
        return []
    events = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            events.append(json.loads(line))
    return events


def maybe_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def status_badge(passed: bool) -> str:
    cls = "pass" if passed else "fail"
    label = "PASS" if passed else "FAIL"
    return f'<span class="badge {cls}">{label}</span>'


def chart_specs(case_id: str) -> list[dict]:
    if case_id == "radar_detection":
        return [
            {
                "title": "SNR vs Range",
                "x": "range_km",
                "series": [("snr_db", "SNR dB", "#0f766e")],
            },
            {
                "title": "Pd vs Range",
                "x": "range_km",
                "series": [
                    ("pd_sw0", "Pd Sw0", "#1d4ed8"),
                    ("pd_sw1", "Pd Sw1", "#dc2626"),
                    ("pd_sw3", "Pd Sw3", "#9333ea"),
                ],
            },
        ]
    if case_id == "missile_engagement":
        return [
            {
                "title": "Range vs Time",
                "x": "time_s",
                "series": [("range_m", "Range m", "#1d4ed8")],
            },
            {
                "title": "Closing Velocity vs Time",
                "x": "time_s",
                "series": [("closing_velocity_mps", "Closing Velocity m/s", "#0f766e")],
            },
            {
                "title": "Acceleration vs Time",
                "x": "time_s",
                "series": [("accel_g", "Accel g", "#dc2626")],
            },
        ]
    if case_id == "orbital_example":
        return [
            {
                "title": "Altitude vs Time",
                "x": "time_min",
                "series": [("altitude_km", "Altitude km", "#1d4ed8")],
            },
            {
                "title": "True Anomaly vs Time",
                "x": "time_min",
                "series": [("true_anomaly_deg", "True Anomaly deg", "#0f766e")],
            },
        ]
    if case_id == "behavior_chain":
        return [
            {
                "title": "Range vs Time",
                "x": "time_s",
                "series": [("range_km", "Range km", "#1d4ed8")],
            },
            {
                "title": "SNR and Pd vs Time",
                "x": "time_s",
                "series": [
                    ("snr_db", "SNR dB", "#0f766e"),
                    ("pd", "Pd", "#dc2626"),
                ],
            },
            {
                "title": "Track Count vs Time",
                "x": "time_s",
                "series": [("track_count", "Track Count", "#9333ea")],
            },
        ]
    return []


def line_chart(rows: list[dict], x_key: str, series: list[tuple[str, str, str]], title: str) -> str:
    points = []
    for row in rows:
        x_val = maybe_float(row.get(x_key))
        if x_val is None:
            continue
        series_values = []
        for key, label, color in series:
            y_val = maybe_float(row.get(key))
            if y_val is not None:
                series_values.append((key, label, color, y_val))
        if series_values:
            points.append((x_val, series_values))

    if len(points) < 2:
        return f"<div class='chart'><h4>{escape(title)}</h4><p>Not enough numeric points.</p></div>"

    x_values = [x for x, _ in points]
    y_values = [y for _, values in points for _, _, _, y in values]
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    if math.isclose(x_min, x_max):
        x_max = x_min + 1.0
    if math.isclose(y_min, y_max):
        y_max = y_min + 1.0

    width, height = 720, 240
    left, right, top, bottom = 52, 16, 20, 34
    plot_w = width - left - right
    plot_h = height - top - bottom

    def scale_x(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * plot_w

    def scale_y(value: float) -> float:
        return top + plot_h - (value - y_min) / (y_max - y_min) * plot_h

    polylines = []
    for key, label, color in series:
        coords = []
        for x_val, values in points:
            for current_key, _, _, y_val in values:
                if current_key == key:
                    coords.append(f"{scale_x(x_val):.2f},{scale_y(y_val):.2f}")
        if len(coords) >= 2:
            polylines.append(
                f"<polyline fill='none' stroke='{color}' stroke-width='2.5' points='{' '.join(coords)}' />"
            )

    legend = "".join(
        f"<span class='legend-item'><span class='legend-line' style='background:{color}'></span>{escape(label)}</span>"
        for _, label, color in series
    )

    grid = []
    for frac in (0.0, 0.5, 1.0):
        y_val = y_min + frac * (y_max - y_min)
        y_pos = scale_y(y_val)
        grid.append(
            f"<line x1='{left}' y1='{y_pos:.2f}' x2='{width-right}' y2='{y_pos:.2f}' stroke='#d6dde8' stroke-dasharray='4 4' />"
        )
        grid.append(
            f"<text x='4' y='{y_pos + 4:.2f}' fill='#475569' font-size='11'>{y_val:.2f}</text>"
        )

    svg = f"""
    <div class="chart">
      <h4>{escape(title)}</h4>
      <div class="legend">{legend}</div>
      <svg viewBox="0 0 {width} {height}" role="img" aria-label="{escape(title)}">
        <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#f8fafc" stroke="#cbd5e1" />
        {''.join(grid)}
        {''.join(polylines)}
        <text x="{left}" y="{height-8}" fill="#475569" font-size="11">{x_min:.2f}</text>
        <text x="{width-right-32}" y="{height-8}" fill="#475569" font-size="11">{x_max:.2f}</text>
      </svg>
    </div>
    """
    return svg


def render_table(values: dict) -> str:
    if not values:
        return "<p>No structured values.</p>"
    rows = "".join(
        f"<tr><th>{escape(str(key))}</th><td>{escape(str(value))}</td></tr>"
        for key, value in values.items()
    )
    return f"<table>{rows}</table>"


def render_events(events: list[dict]) -> str:
    if not events:
        return "<p>No recorded events.</p>"
    rendered = []
    for event in events[:12]:
        attrs = ", ".join(f"{key}={value}" for key, value in event.get("attributes", {}).items())
        rendered.append(
            "<tr>"
            f"<td>{escape(str(event.get('time_s', '')))}</td>"
            f"<td>{escape(event.get('event_type', ''))}</td>"
            f"<td>{escape(event.get('message', ''))}</td>"
            f"<td>{escape(attrs)}</td>"
            "</tr>"
        )
    return "<table><tr><th>Time</th><th>Type</th><th>Message</th><th>Attributes</th></tr>" + "".join(rendered) + "</table>"


def render_case_section(case_record: dict, input_dir: Path) -> str:
    artifact_dir = input_dir / case_record["artifact_dir"]
    summary = load_json(artifact_dir / "run_summary.json")
    metrics = load_json(artifact_dir / "metrics.json")
    timeseries = load_csv(artifact_dir / "timeseries.csv")
    events = load_events(artifact_dir / "events.jsonl")

    charts = "".join(
        line_chart(timeseries, spec["x"], spec["series"], spec["title"])
        for spec in chart_specs(case_record["id"])
    )

    raw_links = (
        f"<a href='../{escape(case_record['artifact_dir'])}/timeseries.csv'>timeseries.csv</a> | "
        f"<a href='../{escape(case_record['artifact_dir'])}/events.jsonl'>events.jsonl</a> | "
        f"<a href='../{escape(case_record['artifact_dir'])}/console.log'>console.log</a>"
    )

    return f"""
    <section class="case">
      <div class="case-head">
        <h2>{escape(case_record['title'])}</h2>
        {status_badge(case_record['passed'])}
      </div>
      <p class="muted">{escape(case_record.get('summary', ''))}</p>
      <div class="summary-grid">
        <div>
          <h3>Purpose</h3>
          <p>{escape(summary.get('purpose', ''))}</p>
        </div>
        <div>
          <h3>Expected Behavior</h3>
          <p>{escape(summary.get('expected_behavior', ''))}</p>
        </div>
      </div>
      <div>
        <h3>Conclusion</h3>
        <p>{escape(summary.get('conclusion', ''))}</p>
        <p class="muted">{escape(summary.get('failure_reason', ''))}</p>
      </div>
      <div class="summary-grid">
        <div>
          <h3>Inputs</h3>
          {render_table(summary.get('inputs', {}))}
        </div>
        <div>
          <h3>Metrics</h3>
          {render_table(metrics or summary.get('metrics', {}))}
        </div>
      </div>
      <div class="charts">{charts or '<p>No charts available.</p>'}</div>
      <div>
        <h3>Events</h3>
        {render_events(events)}
      </div>
      <p class="muted">Artifacts: {raw_links}</p>
    </section>
    """


def build_markdown(index: dict) -> str:
    case_lines = []
    for case in index.get("cases", []):
        case_lines.append(
            f"- {'PASS' if case['passed'] else 'FAIL'} `{case['id']}`: {case['summary']} ({case['artifact_dir']})"
        )
    ctest = index.get("ctest", {})
    ctest_lines = []
    for test in ctest.get("tests", []):
        ctest_lines.append(f"- {'PASS' if test['passed'] else 'FAIL'} `{test['name']}`")
    return "\n".join(
        [
            "# Validation Report",
            "",
            f"- Generated at: `{index.get('generated_at', '')}`",
            f"- Build dir: `{index.get('build_dir', '')}`",
            "",
            "## Case Summary",
            *case_lines,
            "",
            "## CTest Summary",
            f"- Overall: `{'PASS' if ctest.get('passed', False) else 'FAIL'}`",
            *ctest_lines,
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a static validation report from collected artifacts.")
    parser.add_argument("--input-dir", required=True, help="Directory produced by run_cases.py")
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Report output directory. Defaults to <input-dir>/report.",
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    output_dir = (Path(args.output_dir).resolve() if args.output_dir else (input_dir / "report"))
    output_dir.mkdir(parents=True, exist_ok=True)

    index = load_json(input_dir / "run_index.json")
    ctest = index.get("ctest", {})
    passed_cases = sum(1 for case in index.get("cases", []) if case.get("passed"))
    total_cases = len(index.get("cases", []))

    case_sections = "".join(render_case_section(case, input_dir) for case in index.get("cases", []))

    ctest_rows = "".join(
        f"<tr><td>{escape(test['name'])}</td><td>{status_badge(test['passed'])}</td><td>{escape(test['line'])}</td></tr>"
        for test in ctest.get("tests", [])
    )

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>xsf validation report</title>
  <style>
    :root {{
      --bg: #f3f6fb;
      --card: #ffffff;
      --border: #d9e2ec;
      --text: #0f172a;
      --muted: #475569;
      --pass: #166534;
      --pass-bg: #dcfce7;
      --fail: #991b1b;
      --fail-bg: #fee2e2;
      --accent: #0f766e;
    }}
    body {{
      margin: 0;
      font-family: "SFMono-Regular", "Consolas", monospace;
      background: linear-gradient(180deg, #eff6ff 0%, var(--bg) 100%);
      color: var(--text);
    }}
    main {{
      max-width: 1160px;
      margin: 0 auto;
      padding: 32px 20px 48px;
    }}
    h1, h2, h3, h4 {{ margin: 0 0 12px; }}
    p {{ line-height: 1.5; }}
    .hero, .case, .ctest {{
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 20px;
      box-shadow: 0 16px 32px rgba(15, 23, 42, 0.06);
      margin-bottom: 20px;
    }}
    .badge {{
      display: inline-block;
      padding: 4px 10px;
      border-radius: 999px;
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.08em;
    }}
    .badge.pass {{ color: var(--pass); background: var(--pass-bg); }}
    .badge.fail {{ color: var(--fail); background: var(--fail-bg); }}
    .summary-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 16px;
      margin: 16px 0;
    }}
    .metric-card {{
      background: #f8fafc;
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 16px;
    }}
    .metric-card .value {{
      font-size: 28px;
      font-weight: 700;
      margin-top: 8px;
    }}
    .muted {{ color: var(--muted); }}
    .case-head {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
    }}
    .charts {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
      gap: 16px;
      margin: 16px 0;
    }}
    .chart {{
      background: #f8fafc;
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 12px;
    }}
    .legend {{
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-bottom: 8px;
      color: var(--muted);
      font-size: 12px;
    }}
    .legend-item {{
      display: inline-flex;
      align-items: center;
      gap: 6px;
    }}
    .legend-line {{
      display: inline-block;
      width: 18px;
      height: 3px;
      border-radius: 2px;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }}
    th, td {{
      text-align: left;
      border-bottom: 1px solid var(--border);
      padding: 8px 10px;
      vertical-align: top;
    }}
    a {{ color: var(--accent); text-decoration: none; }}
  </style>
</head>
<body>
  <main>
    <section class="hero">
      <h1>xsf Validation Report</h1>
      <p class="muted">Generated from structured example artifacts plus the current CTest regression suite.</p>
      <div class="summary-grid">
        <div class="metric-card">
          <div>Case Results</div>
          <div class="value">{passed_cases} / {total_cases}</div>
        </div>
        <div class="metric-card">
          <div>CTest</div>
          <div class="value">{'PASS' if ctest.get('passed', False) else 'FAIL'}</div>
        </div>
        <div class="metric-card">
          <div>Generated At</div>
          <div class="value" style="font-size:18px">{escape(index.get('generated_at', ''))}</div>
        </div>
      </div>
    </section>
    <section class="ctest">
      <div class="case-head">
        <h2>Regression Summary</h2>
        {status_badge(ctest.get('passed', False))}
      </div>
      <p class="muted">Raw log: <a href="../ctest.log">ctest.log</a></p>
      <table>
        <tr><th>Test</th><th>Status</th><th>Details</th></tr>
        {ctest_rows}
      </table>
    </section>
    {case_sections}
  </main>
</body>
</html>
"""

    (output_dir / "report.html").write_text(html, encoding="utf-8")
    (output_dir / "report.md").write_text(build_markdown(index), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
