// SPDX-License-Identifier: Apache-2.0
import type { MapJson, PlanRequest } from "./planners";
import { plannerFromString } from "./planners";

const BASE = import.meta.env.BASE_URL;
const GLOBAL_PLANNERS = ["astar", "dijkstra", "lazy_theta", "rrt_star"] as const;

interface PlanResultPayload {
  id: number;
  result?: {
    success: boolean;
    reason: string;
    path: { x: number; y: number }[];
    metrics: {
      length_m: number;
      objective_cost: number;
      compute_ms: number;
      coverage_ratio_reachable?: number;
      clearance_m: number;
    };
  };
  compare?: Array<{
    planner: string;
    success: boolean;
    reason: string;
    length_m: number;
    compute_ms: number;
  }>;
  error?: string;
}

const LEGEND = [
  { label: "Free", color: "#d8dee9" },
  { label: "Occupied", color: "#c44e4e" },
  { label: "Unknown", color: "#c9a227" },
  { label: "Inflation / cost", color: "#3d5a80" },
  { label: "Path", color: "#4ade80" },
  { label: "Coverage trace", color: "#38bdf8" },
];

export function mountApp(root: HTMLElement): void {
  root.innerHTML = `
    <h1>nav2-coverage-planner-compare</h1>
    <p class="subtitle">Portfolio demo (simulation only) — Nav2-style costmaps, planner comparison</p>
    <div class="toolbar">
      <label>Map <select id="mapSelect"></select></label>
      <label>Planner <select id="planner"></select></label>
      <button type="button" id="planBtn">Plan</button>
      <button type="button" id="coverageBtn">Coverage</button>
      <label><input type="checkbox" id="compareMode" /> Compare all planners</label>
    </div>
    <div class="layout">
      <div class="canvas-wrap"><canvas id="canvas"></canvas></div>
      <div class="side">
        <motion class="card"><h2>Metrics</h2><div id="metrics" data-testid="metrics"></div></motion>
        <motion class="card"><h2>Legend</h2><div class="legend" id="legend"></div></motion>
        <motion class="card" id="compareCard" style="display:none"><h2>Compare</h2><div id="compareTable"></div></motion>
      </div>
    </div>
  `.replace(/<motion/g, "<div").replace(/<\/motion>/g, "</div>");

  const mapSelect = root.querySelector<HTMLSelectElement>("#mapSelect")!;
  const plannerSelect = root.querySelector<HTMLSelectElement>("#planner")!;
  const metricsEl = root.querySelector<HTMLDivElement>("#metrics")!;
  const compareCard = root.querySelector<HTMLDivElement>("#compareCard")!;
  const compareTable = root.querySelector<HTMLDivElement>("#compareTable")!;
  const canvas = root.querySelector<HTMLCanvasElement>("#canvas")!;
  const ctx = canvas.getContext("2d")!;

  for (const p of GLOBAL_PLANNERS) {
    const opt = document.createElement("option");
    opt.value = p;
    opt.textContent = p;
    plannerSelect.appendChild(opt);
  }

  const legendRoot = root.querySelector("#legend")!;
  for (const item of LEGEND) {
    const sw = document.createElement("span");
    sw.className = "swatch";
    sw.style.background = item.color;
    legendRoot.appendChild(sw);
    const lb = document.createElement("span");
    lb.textContent = item.label;
    legendRoot.appendChild(lb);
  }

  const maps = ["office", "open", "hole", "narrow_pass", "unreachable", "unknown_row", "tiny"];
  for (const m of maps) {
    const opt = document.createElement("option");
    opt.value = m;
    opt.textContent = m;
    mapSelect.appendChild(opt);
  }

  const planPresets: Record<string, { start: { x: number; y: number }; goal: { x: number; y: number } }> =
    {
      office: { start: { x: 0.5, y: 1.05 }, goal: { x: 2.5, y: 1.05 } },
      open: { start: { x: 0.5, y: 0.5 }, goal: { x: 1.5, y: 1.0 } },
      hole: { start: { x: 0.5, y: 0.5 }, goal: { x: 1.0, y: 1.0 } },
      narrow_pass: { start: { x: 0.5, y: 1.05 }, goal: { x: 2.5, y: 1.05 } },
      unreachable: { start: { x: 0.5, y: 1.0 }, goal: { x: 1.5, y: 1.0 } },
      unknown_row: { start: { x: 0.5, y: 0.05 }, goal: { x: 1.5, y: 0.05 } },
      tiny: { start: { x: 0.05, y: 0.05 }, goal: { x: 0.05, y: 0.05 } },
    };

  let currentMap: MapJson | null = null;
  let currentMapId = mapSelect.value;
  let lastPath: { x: number; y: number }[] = [];
  let coverageMode = false;
  let reqId = 0;
  const worker = new Worker(new URL("./worker/planWorker.ts", import.meta.url), { type: "module" });

  function resizeCanvas(): void {
    const wrap = canvas.parentElement!;
    canvas.width = Math.floor(wrap.clientWidth);
    canvas.height = Math.max(400, Math.floor(wrap.clientWidth * 0.62));
    drawMap(currentMap, lastPath, coverageMode);
  }

  function formatMetrics(r: NonNullable<PlanResultPayload["result"]>, mode: string): string {
    const m = r.metrics;
    const clr = Number.isNaN(m.clearance_m) ? "n/a" : m.clearance_m.toFixed(4);
    const cov =
      m.coverage_ratio_reachable != null && m.coverage_ratio_reachable > 0
        ? ` coverage=${(m.coverage_ratio_reachable * 100).toFixed(1)}%`
        : "";
    const status = r.success
      ? `success length=${m.length_m.toFixed(4)} m`
      : `failed: ${r.reason}`;
    return `${mode} ${status} | cost=${m.objective_cost.toFixed(4)} | compute_ms=${m.compute_ms.toFixed(2)} | clearance=${clr}m${cov}`;
  }

  worker.onmessage = (ev: MessageEvent<PlanResultPayload>) => {
    const msg = ev.data;
    if (msg.error) {
      metricsEl.innerHTML = `<p class="metrics-line">error: ${msg.error}</p>`;
      return;
    }
    if (msg.compare) {
      compareCard.style.display = "block";
      const rows = msg.compare
        .map(
          (c) =>
            `<tr><td>${c.planner}</td><td>${c.success}</td><td>${c.length_m.toFixed(4)}</td><td>${c.compute_ms.toFixed(2)}</td><td>${c.reason || "—"}</td></tr>`,
        )
        .join("");
      compareTable.innerHTML = `<table class="compare"><thead><tr><th>Planner</th><th>OK</th><th>Length</th><th>ms</th><th>Reason</th></tr></thead><tbody>${rows}</tbody></table>`;
      metricsEl.innerHTML = `<p class="metrics-line">Compare mode on ${currentMapId}</p>`;
      return;
    }
    if (!msg.result) return;
    const r = msg.result;
    lastPath = r.path;
    metricsEl.innerHTML = `<p class="metrics-line" data-testid="metrics-text">${formatMetrics(r, coverageMode ? "coverage" : plannerSelect.value)}</p>`;
    drawMap(currentMap, lastPath, coverageMode);
  };

  async function loadMap(id: string): Promise<void> {
    currentMapId = id;
    const res = await fetch(`${BASE}fixtures/${id}/map.json`);
    currentMap = (await res.json()) as MapJson;
    lastPath = [];
    coverageMode = false;
    resizeCanvas();
    metricsEl.innerHTML = `<p class="metrics-line">loaded ${id}</p>`;
  }

  function drawMap(map: MapJson | null, path: { x: number; y: number }[], isCoverage: boolean): void {
    ctx.fillStyle = "#0a0e12";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (!map) return;
    const pad = 8;
    const scale = Math.min((canvas.width - pad * 2) / map.width, (canvas.height - pad * 2) / map.height);
    const ox = (canvas.width - map.width * scale) / 2;
    const oy = (canvas.height - map.height * scale) / 2;

    for (let y = 0; y < map.height; y++) {
      for (let x = 0; x < map.width; x++) {
        const i = y * map.width + x;
        const occ = map.occupancy[i];
        const c = map.costs[i] ?? 0;
        if (occ === 1) ctx.fillStyle = "#c44e4e";
        else if (occ === 2) ctx.fillStyle = "#c9a227";
        else if (c > 0 && c < 253) ctx.fillStyle = "#3d5a80";
        else ctx.fillStyle = "#d8dee9";
        ctx.fillRect(ox + x * scale, oy + (map.height - 1 - y) * scale, scale + 0.5, scale + 0.5);
      }
    }

    if (path.length > 0) {
      ctx.strokeStyle = isCoverage ? "#38bdf8" : "#4ade80";
      ctx.lineWidth = 2;
      ctx.beginPath();
      const [mx, my, yaw] = map.origin;
      const res = map.resolution;
      for (let i = 0; i < path.length; i++) {
        const px = ox + ((path[i]!.x - mx) / res) * scale;
        const py = oy + (map.height - (path[i]!.y - my) / res) * scale;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
    }
  }

  function runPlan(coverage: boolean): void {
    if (!currentMap) return;
    coverageMode = coverage;
    const compare = (root.querySelector("#compareMode") as HTMLInputElement).checked;
    const preset = planPresets[currentMapId] ?? planPresets.open!;
    const id = ++reqId;
    if (compare && !coverage) {
      worker.postMessage({
        kind: "compare",
        id,
        map: currentMap,
        start: preset.start,
        goal: preset.goal,
      });
      plannerSelect.value = "astar";
      return;
    }
    const planner = plannerFromString(plannerSelect.value);
    worker.postMessage({
      kind: "plan",
      id,
      map: currentMap,
      request: {
        planner,
        start: preset.start,
        goal: coverage ? preset.start : preset.goal,
        rrt_seed: 42,
        rrt_max_iterations: 3000,
        rrt_max_time_ms: 0,
      } satisfies PlanRequest,
      coverage,
      coverageOverlap: 0.02,
    });
  }

  window.addEventListener("resize", resizeCanvas);
  void loadMap(mapSelect.value);
  mapSelect.addEventListener("change", () => void loadMap(mapSelect.value));
  plannerSelect.addEventListener("change", () => {
    metricsEl.innerHTML = `<p class="metrics-line">planner: ${plannerSelect.value}</p>`;
  });
  root.querySelector("#planBtn")!.addEventListener("click", () => runPlan(false));
  root.querySelector("#coverageBtn")!.addEventListener("click", () => runPlan(true));
}
