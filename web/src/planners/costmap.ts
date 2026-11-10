// SPDX-License-Identifier: Apache-2.0
import { kCostInscribed, kCostLethal, kCostUnknown } from "./constants";
import type { CostmapParams, MapJson, UnknownPolicy, WorldPoint } from "./types";
import { defaultCostmapParams } from "./types";

type Occupancy = 0 | 1 | 2;

export class Costmap {
  readonly width: number;
  readonly height: number;
  readonly resolution: number;
  readonly origin: [number, number, number];
  readonly params: CostmapParams;
  private readonly occupancy: Occupancy[];
  private readonly costs: Uint8Array;
  private readonly hardMask: Uint8Array;

  private constructor(
    width: number,
    height: number,
    resolution: number,
    origin: [number, number, number],
    params: CostmapParams,
    occupancy: Occupancy[],
    costs: Uint8Array,
    hardMask: Uint8Array,
  ) {
    this.width = width;
    this.height = height;
    this.resolution = resolution;
    this.origin = origin;
    this.params = params;
    this.occupancy = occupancy;
    this.costs = costs;
    this.hardMask = hardMask;
  }

  static fromMapJson(map: MapJson, override?: Partial<CostmapParams>): Costmap {
    const params: CostmapParams = {
      ...defaultCostmapParams(),
      robot_radius_m: map.params.robot_radius_m,
      inflation_radius_m: map.params.inflation_radius_m,
      w_cost: map.params.w_cost,
      cost_decay: map.params.cost_decay,
      unknown_policy: map.params.unknown_policy,
      ...override,
    };
  const occ = map.occupancy.map((v) => v as Occupancy);
    const cm = new Costmap(
      map.width,
      map.height,
      map.resolution,
      map.origin,
      params,
      occ,
      new Uint8Array(map.costs),
      new Uint8Array(map.width * map.height),
    );
    cm.buildInflationAndCosts();
    return cm;
  }

  private static idx(x: number, y: number, w: number): number {
    return y * w + x;
  }

  private at(x: number, y: number): Occupancy {
    if (x < 0 || y < 0 || x >= this.width || y >= this.height) return 2;
    return this.occupancy[Costmap.idx(x, y, this.width)] ?? 2;
  }

  cost(x: number, y: number): number {
    if (x < 0 || y < 0 || x >= this.width || y >= this.height) return kCostUnknown;
    return this.costs[Costmap.idx(x, y, this.width)] ?? kCostUnknown;
  }

  /** Raw PGM occupancy: 0 free, 1 occupied, 2 unknown (matches C++ Costmap::occupancy). */
  occupancyAt(x: number, y: number): Occupancy {
    return this.at(x, y);
  }

  isHardBlocked(x: number, y: number): boolean {
    if (x < 0 || y < 0 || x >= this.width || y >= this.height) return true;
    const i = Costmap.idx(x, y, this.width);
    if (this.hardMask[i]) return true;
    if (this.params.collide_inscribed && this.costs[i]! >= kCostInscribed) return true;
    return false;
  }

  cellCenter(x: number, y: number): WorldPoint {
    const [ox, oy, yaw] = this.origin;
    const res = this.resolution;
    const cx = ox + (x + 0.5) * res;
    const cy = oy + (y + 0.5) * res;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return { x: c * cx - s * cy, y: s * cx + c * cy };
  }

  worldToCell(w: WorldPoint): { x: number; y: number } | null {
    const [ox, oy, yaw] = this.origin;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const lx = c * w.x + s * w.y;
    const ly = -s * w.x + c * w.y;
    const res = this.resolution;
    const fx = (lx - ox) / res - 0.5;
    const fy = (ly - oy) / res - 0.5;
    const x = Math.floor(fx + 0.5);
    const y = Math.floor(fy + 0.5);
    if (x < 0 || y < 0 || x >= this.width || y >= this.height) return null;
    return { x, y };
  }

  mapMeta(): { origin_x: number; origin_y: number } {
    return { origin_x: this.origin[0], origin_y: this.origin[1] };
  }

  private cellBlocksCenter(x: number, y: number): boolean {
    const o = this.at(x, y);
    if (o === 1) return true;
    if (o === 2) return this.params.unknown_policy === "lethal";
    return false;
  }

  private buildInflationAndCosts(): void {
    const n = this.width * this.height;
    this.costs.fill(0);
    this.hardMask.fill(0);
    const res = this.resolution;
    const inflateCells = Math.ceil(this.params.inflation_radius_m / res);
    const distSq = new Int32Array(n).fill(inflateCells * inflateCells + 1);
    const q: number[] = [];
    for (let y = 0; y < this.height; y++) {
      for (let x = 0; x < this.width; x++) {
        if (this.cellBlocksCenter(x, y)) {
          const idx = Costmap.idx(x, y, this.width);
          distSq[idx] = 0;
          this.hardMask[idx] = 1;
          q.push(idx);
        }
      }
    }
    const dx = [-1, 0, 1, 0];
    const dy = [0, 1, 0, -1];
    let head = 0;
    while (head < q.length) {
      const idx = q[head++]!;
      const x0 = idx % this.width;
      const y0 = Math.floor(idx / this.width);
      for (let k = 0; k < 4; k++) {
        const x = x0 + dx[k]!;
        const y = y0 + dy[k]!;
        if (x < 0 || y < 0 || x >= this.width || y >= this.height) continue;
        const nidx = Costmap.idx(x, y, this.width);
        const nd2 = distSq[idx]! + 1;
        if (nd2 < distSq[nidx]!) {
          distSq[nidx] = nd2;
          if (nd2 <= inflateCells * inflateCells) this.hardMask[nidx] = 1;
          q.push(nidx);
        }
      }
    }
    if (this.params.cost_decay > 0) {
      const radius = this.params.inflation_radius_m;
      for (let y = 0; y < this.height; y++) {
        for (let x = 0; x < this.width; x++) {
          const i = Costmap.idx(x, y, this.width);
          if (this.hardMask[i]) {
            this.costs[i] = kCostLethal;
            continue;
          }
          const dM = Math.sqrt(distSq[i]!) * res;
          if (dM <= this.params.robot_radius_m) {
            this.costs[i] = kCostInscribed;
          } else if (dM < radius) {
            const t = (radius - dM) / Math.max(radius, 1e-9);
            const c = Math.min(252, (t * 252 * this.params.cost_decay) / 5);
            this.costs[i] = Math.max(this.costs[i]!, c);
          }
          if (this.at(x, y) === 2 && this.params.unknown_policy === "lethal") {
            this.costs[i] = kCostUnknown;
          }
        }
      }
    } else {
      for (let i = 0; i < n; i++) {
        this.costs[i] = this.hardMask[i] ? kCostLethal : 0;
      }
    }
  }
}
