// sokoban-solver-01-run-20261121 — push-optimal Sokoban solver.
//
// Strategy: every level is a 1-wide hallway with 1-4 rooms off it, and a box
// pushed out of its room can never come back, so rooms are independent.
// Each room is solved push-optimally on its own with A* over pushes
// (min-cost box->goal matching heuristic on true shortest-path distances,
// plus corner / wall-segment / 2x2 deadlock pruning), then the room
// solutions are stitched together with hallway walks.
//
// Deterministic: fixed work budgets, fixed neighbour order, no clock, no RNG.
import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { join } from "node:path";

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? "/input";
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? "/output";
const IDS = ["i001", "i002", "i003", "i004", "i005"];

// Directions: 0 = up, 1 = down, 2 = left, 3 = right.
const DR = [-1, 1, 0, 0];
const DC = [0, 0, -1, 1];
const LETTER = ["u", "d", "l", "r"];
const MAX_EXPANSIONS = 2000000;

type Grid = {
  w: number;
  h: number;
  wall: boolean[];
  goal: boolean[];
  boxes: number[];
  player: number;
};

type Room = {
  cells: Set<number>;
  door: number;
  entry: number;
  boxes: number[];
  goals: number[];
};

function idx(r: number, c: number, w: number): number {
  return r * w + c;
}

function parseLevel(text: string): Grid {
  const lines = text.split("\n");
  while (lines.length > 0 && lines[lines.length - 1] === "") lines.pop();
  const h = lines.length;
  let w = 0;
  for (const l of lines) if (l.length > w) w = l.length;
  const wall: boolean[] = new Array(w * h).fill(false);
  const goal: boolean[] = new Array(w * h).fill(false);
  const boxes: number[] = [];
  let player = -1;
  for (let r = 0; r < h; r++) {
    for (let c = 0; c < w; c++) {
      const ch = c < lines[r].length ? lines[r][c] : " ";
      const i = idx(r, c, w);
      if (ch === "#") wall[i] = true;
      else if (ch === ".") goal[i] = true;
      else if (ch === "$") boxes.push(i);
      else if (ch === "*") {
        boxes.push(i);
        goal[i] = true;
      } else if (ch === "@") player = i;
      else if (ch === "+") {
        player = i;
        goal[i] = true;
      }
    }
  }
  if (player < 0) throw new Error("no player in level");
  return { w, h, wall, goal, boxes, player };
}

// Split the level into the hallway row and the rooms hanging off it.
function decompose(g: Grid): { hallway: Set<number>; rooms: Room[] } {
  const hallwayRow = Math.floor(g.player / g.w);
  const hallway = new Set<number>();
  for (let c = 0; c < g.w; c++) {
    const i = idx(hallwayRow, c, g.w);
    if (!g.wall[i]) hallway.add(i);
  }
  const seen = new Set<number>();
  const rooms: Room[] = [];
  for (let i = 0; i < g.w * g.h; i++) {
    const r = Math.floor(i / g.w);
    if (g.wall[i] || r === hallwayRow || seen.has(i)) continue;
    // Flood-fill one room component.
    const cells = new Set<number>();
    const stack = [i];
    seen.add(i);
    while (stack.length > 0) {
      const cur = stack.pop() as number;
      cells.add(cur);
      const cr = Math.floor(cur / g.w);
      const cc = cur % g.w;
      for (let d = 0; d < 4; d++) {
        const nr = cr + DR[d];
        const nc = cc + DC[d];
        if (nr < 0 || nr >= g.h || nc < 0 || nc >= g.w) continue;
        const ni = idx(nr, nc, g.w);
        if (g.wall[ni] || nr === hallwayRow || seen.has(ni)) continue;
        seen.add(ni);
        stack.push(ni);
      }
    }
    // The door is the room cell touching the hallway; entry is that hallway cell.
    let door = -1;
    let entry = -1;
    for (const cell of cells) {
      const cr = Math.floor(cell / g.w);
      const cc = cell % g.w;
      for (let d = 0; d < 4; d++) {
        const nr = cr + DR[d];
        const nc = cc + DC[d];
        if (nr < 0 || nr >= g.h || nc < 0 || nc >= g.w) continue;
        const ni = idx(nr, nc, g.w);
        if (hallway.has(ni)) {
          door = cell;
          entry = ni;
          break;
        }
      }
      if (door >= 0) break;
    }
    if (door < 0) throw new Error("room with no door onto the hallway");
    const boxes = g.boxes.filter((b) => cells.has(b)).sort((a, b) => a - b);
    const goals: number[] = [];
    for (const cell of cells) if (g.goal[cell]) goals.push(cell);
    goals.sort((a, b) => a - b);
    if (boxes.length !== goals.length) {
      throw new Error("room box/goal count mismatch");
    }
    rooms.push({ cells, door, entry, boxes, goals });
  }
  rooms.sort((a, b) => a.door - b.door);
  return { hallway, rooms };
}

// Breadth-first distances from one start cell over a floor set (no boxes).
// Returns a map cell -> distance (Infinity when unreachable).
function bfsDist(
  start: number,
  floor: Set<number>,
  w: number,
  h: number,
): Map<number, number> {
  const dist = new Map<number, number>();
  dist.set(start, 0);
  const queue = [start];
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const d = dist.get(cur) as number;
    const cr = Math.floor(cur / w);
    const cc = cur % w;
    for (let dir = 0; dir < 4; dir++) {
      const nr = cr + DR[dir];
      const nc = cc + DC[dir];
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
      const ni = idx(nr, nc, w);
      if (!floor.has(ni) || dist.has(ni)) continue;
      dist.set(ni, d + 1);
      queue.push(ni);
    }
  }
  return dist;
}

// Reachable cells for the player from `start`, treating boxes as obstacles.
function reachableSet(
  start: number,
  floor: Set<number>,
  boxSet: Set<number>,
  w: number,
  h: number,
): Set<number> {
  const seen = new Set<number>([start]);
  const queue = [start];
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const cr = Math.floor(cur / w);
    const cc = cur % w;
    for (let dir = 0; dir < 4; dir++) {
      const nr = cr + DR[dir];
      const nc = cc + DC[dir];
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
      const ni = idx(nr, nc, w);
      if (!floor.has(ni) || boxSet.has(ni) || seen.has(ni)) continue;
      seen.add(ni);
      queue.push(ni);
    }
  }
  return seen;
}

// Shortest walk (list of directions) from `from` to `to` over floor cells,
// avoiding `obstacles`. Fixed neighbour order, so deterministic.
function bfsPath(
  from: number,
  to: number,
  floor: Set<number>,
  obstacles: Set<number>,
  w: number,
  h: number,
): number[] | null {
  if (from === to) return [];
  const prev = new Map<number, { at: number; dir: number }>();
  const queue = [from];
  prev.set(from, { at: -1, dir: -1 });
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const cr = Math.floor(cur / w);
    const cc = cur % w;
    for (let dir = 0; dir < 4; dir++) {
      const nr = cr + DR[dir];
      const nc = cc + DC[dir];
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
      const ni = idx(nr, nc, w);
      if (!floor.has(ni) || obstacles.has(ni) || prev.has(ni)) continue;
      prev.set(ni, { at: cur, dir });
      if (ni === to) {
        const path: number[] = [];
        let node = ni;
        while (node !== from) {
          const step = prev.get(node) as { at: number; dir: number };
          path.push(step.dir);
          node = step.at;
        }
        path.reverse();
        return path;
      }
      queue.push(ni);
    }
  }
  return null;
}

class Heap<T> {
  private a: T[] = [];
  private less: (x: T, y: T) => boolean;
  constructor(less: (x: T, y: T) => boolean) {
    this.less = less;
  }
  get size(): number {
    return this.a.length;
  }
  push(v: T): void {
    this.a.push(v);
    let i = this.a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(this.a[i], this.a[p])) break;
      [this.a[i], this.a[p]] = [this.a[p], this.a[i]];
      i = p;
    }
  }
  pop(): T {
    const top = this.a[0];
    const last = this.a.pop() as T;
    if (this.a.length > 0) {
      this.a[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < this.a.length && this.less(this.a[l], this.a[m])) m = l;
        if (r < this.a.length && this.less(this.a[r], this.a[m])) m = r;
        if (m === i) break;
        [this.a[i], this.a[m]] = [this.a[m], this.a[i]];
        i = m;
      }
    }
    return top;
  }
}

type Push = { box: number; dir: number };

// Solve one room push-optimally. Returns the push list (box cell, direction)
// from the initial state, player starting on the room's entry (hallway) cell.
function solveRoom(room: Room, g: Grid): Push[] {
  const floor = new Set<number>(room.cells);
  floor.add(room.entry);
  const w = g.w;
  const h = g.h;

  // goalDist[goalIdx][cell] = shortest pushes-free distance goal -> cell.
  const goalDist: Map<number, number>[] = room.goals.map((gl) =>
    bfsDist(gl, floor, w, h),
  );

  // Minimum-cost matching of boxes to goals (brute force, <= 4 boxes).
  const k = room.boxes.length;
  function heuristic(boxes: number[]): number {
    if (k === 0) return 0;
    let best = Infinity;
    const used = new Array<boolean>(k).fill(false);
    const perm: number[] = new Array<number>(k).fill(0);
    function rec(i: number, sum: number): void {
      if (sum >= best) return;
      if (i === k) {
        best = sum;
        return;
      }
      for (let j = 0; j < k; j++) {
        if (used[j]) continue;
        const d = goalDist[j].get(boxes[i]);
        if (d === undefined) continue;
        used[j] = true;
        perm[i] = j;
        rec(i + 1, sum + d);
        used[j] = false;
      }
    }
    rec(0, 0);
    return best;
  }

  function isWallCell(cell: number): boolean {
    return !floor.has(cell);
  }

  // Deadlock test for a box that just moved to `cell`, given all boxes.
  function isDead(cell: number, boxSet: Set<number>): boolean {
    if (g.goal[cell]) return false;
    const r = Math.floor(cell / w);
    const c = cell % w;
    const at = (dr: number, dc: number): number => idx(r + dr, c + dc, w);
    const inBounds = (dr: number, dc: number): boolean =>
      r + dr >= 0 && r + dr < h && c + dc >= 0 && c + dc < w;
    const wallAt = (dr: number, dc: number): boolean =>
      !inBounds(dr, dc) || isWallCell(at(dr, dc));
    // Corner: walls on two perpendicular sides.
    if (
      (wallAt(-1, 0) && wallAt(0, -1)) ||
      (wallAt(-1, 0) && wallAt(0, 1)) ||
      (wallAt(1, 0) && wallAt(0, -1)) ||
      (wallAt(1, 0) && wallAt(0, 1))
    ) {
      return true;
    }
    // Against a wall: the box can only slide along it, and can only leave
    // the wall line where the wall itself ends. It is dead only when the
    // whole wall segment it can slide over holds no goal AND both ends of
    // the segment are closed (floor ends there, so the box cannot escape).
    const wallDirs: [number, number][] = [
      [-1, 0],
      [1, 0],
      [0, -1],
      [0, 1],
    ];
    for (const [wr, wc] of wallDirs) {
      if (!wallAt(wr, wc)) continue;
      // Slide direction is perpendicular to the wall.
      const sr = wc;
      const sc = -wr;
      let hasGoal = false;
      let closedEnds = 0;
      for (const sgn of [1, -1]) {
        let endClosed = false;
        for (let step = 1; ; step++) {
          const cr2 = r + sr * sgn * step;
          const cc2 = c + sc * sgn * step;
          if (cr2 < 0 || cr2 >= h || cc2 < 0 || cc2 >= w) {
            endClosed = true;
            break;
          }
          const ci = idx(cr2, cc2, w);
          if (!floor.has(ci)) {
            endClosed = true;
            break;
          }
          if (!wallAt(cr2 - r + wr, cc2 - c + wc)) break; // wall ends: open end
          if (g.goal[ci]) hasGoal = true;
        }
        if (endClosed) closedEnds++;
      }
      if (!hasGoal && closedEnds === 2) return true;
    }
    // 2x2 block: every 2x2 square holding this box, all wall-or-box,
    // no goal inside.
    for (let dr = -1; dr <= 0; dr++) {
      for (let dc = -1; dc <= 0; dc++) {
        const cells = [
          at(dr, dc),
          at(dr, dc + 1),
          at(dr + 1, dc),
          at(dr + 1, dc + 1),
        ];
        if (!cells.includes(cell)) continue;
        let allFilled = true;
        let anyGoal = false;
        for (const ci of cells) {
          const rr = Math.floor(ci / w);
          const cc2 = ci % w;
          if (rr < 0 || rr >= h || cc2 < 0 || cc2 >= w) continue;
          if (g.goal[ci]) anyGoal = true;
          if (!isWallCell(ci) && !boxSet.has(ci)) {
            allFilled = false;
            break;
          }
        }
        if (allFilled && !anyGoal) return true;
      }
    }
    return false;
  }

  function solved(boxes: number[]): boolean {
    for (const b of boxes) if (!g.goal[b]) return false;
    return true;
  }

  function keyOf(boxes: number[], player: number): string {
    return boxes.join(",") + "|" + player;
  }

  // A* over pushes. `greedy` switches the priority to h-only (fallback).
  function search(greedy: boolean): Push[] | null {
    type Node = {
      f: number;
      h: number;
      seq: number;
      boxes: number[];
      player: number;
      g: number;
    };
    let seq = 0;
    const open = new Heap<Node>(
      (a, b) => a.f < b.f || (a.f === b.f && (a.h < b.h || (a.h === b.h && a.seq < b.seq))),
    );
    const startBoxes = [...room.boxes];
    const startH = heuristic(startBoxes);
    if (startH === Infinity) return null;
    open.push({
      f: greedy ? startH : startH,
      h: startH,
      seq: seq++,
      boxes: startBoxes,
      player: room.entry,
      g: 0,
    });
    const bestG = new Map<string, number>();
    bestG.set(keyOf(startBoxes, room.entry), 0);
    const parent = new Map<string, { prev: string; push: Push }>();
    let expansions = 0;
    while (open.size > 0) {
      const cur = open.pop();
      const curKey = keyOf(cur.boxes, cur.player);
      if ((bestG.get(curKey) as number) < cur.g) continue;
      expansions++;
      if (expansions > MAX_EXPANSIONS) return null;
      // Goal: every box on a goal and the player can walk back to the door.
      if (solved(cur.boxes)) {
        const boxSet = new Set(cur.boxes);
        const reach = reachableSet(cur.player, floor, boxSet, w, h);
        if (reach.has(room.entry)) {
          const pushes: Push[] = [];
          let k2 = curKey;
          while (parent.has(k2)) {
            const p = parent.get(k2) as { prev: string; push: Push };
            pushes.push(p.push);
            k2 = p.prev;
          }
          pushes.reverse();
          return pushes;
        }
      }
      const boxSet = new Set<number>(cur.boxes);
      const reach = reachableSet(cur.player, floor, boxSet, w, h);
      for (let bi = 0; bi < cur.boxes.length; bi++) {
        const b = cur.boxes[bi];
        const br = Math.floor(b / w);
        const bc = b % w;
        for (let d = 0; d < 4; d++) {
          const dr = br + DR[d];
          const dc = bc + DC[d];
          const fr = br - DR[d];
          const fc = bc - DC[d];
          if (dr < 0 || dr >= h || dc < 0 || dc >= w) continue;
          if (fr < 0 || fr >= h || fc < 0 || fc >= w) continue;
          const dest = idx(dr, dc, w);
          const from = idx(fr, fc, w);
          // Boxes never leave their room: the entry (hallway) cell is for
          // the player only, never a box destination.
          if (!room.cells.has(dest) || boxSet.has(dest)) continue;
          if (!reach.has(from)) continue;
          const next = [...cur.boxes];
          next[bi] = dest;
          next.sort((a, b2) => a - b2);
          const nextSet = new Set<number>(next);
          if (isDead(dest, nextSet)) continue;
          const hh = heuristic(next);
          if (hh === Infinity) continue;
          const ng = cur.g + 1;
          const nk = keyOf(next, b);
          const prevG = bestG.get(nk);
          if (prevG !== undefined && prevG <= ng) continue;
          bestG.set(nk, ng);
          parent.set(nk, { prev: curKey, push: { box: b, dir: d } });
          open.push({
            f: greedy ? hh : ng + hh,
            h: hh,
            seq: seq++,
            boxes: next,
            player: b,
            g: ng,
          });
        }
      }
    }
    return null;
  }

  let pushes = search(false);
  if (pushes === null) pushes = search(true);
  if (pushes === null) throw new Error("room unsolved");
  return pushes;
}

// Turn a push list into the full step string for one room, starting and
// ending with the player on the room's entry cell.
function roomMoves(room: Room, pushes: Push[], g: Grid): string {
  const floor = new Set<number>(room.cells);
  floor.add(room.entry);
  const w = g.w;
  const h = g.h;
  let boxes = [...room.boxes];
  let player = room.entry;
  let out = "";
  for (const p of pushes) {
    const br = Math.floor(p.box / w);
    const bc = p.box % w;
    const from = idx(br - DR[p.dir], bc - DC[p.dir], w);
    const walk = bfsPath(player, from, floor, new Set<number>(boxes), w, h);
    if (walk === null) throw new Error("no walk to push position");
    for (const d of walk) out += LETTER[d];
    out += LETTER[p.dir];
    boxes = boxes.map((b) => (b === p.box ? idx(br + DR[p.dir], bc + DC[p.dir], w) : b));
    player = p.box;
  }
  const exit = bfsPath(player, room.entry, floor, new Set<number>(boxes), w, h);
  if (exit === null) throw new Error("player cannot leave the room");
  for (const d of exit) out += LETTER[d];
  return out;
}

function solveLevel(g: Grid): string {
  const { hallway, rooms } = decompose(g);
  let moves = "";
  let player = g.player;
  for (const room of rooms) {
    const walk = bfsPath(player, room.entry, hallway, new Set<number>(), g.w, g.h);
    if (walk === null) throw new Error("hallway walk to room door failed");
    for (const d of walk) moves += LETTER[d];
    player = room.entry;
    const pushes = solveRoom(room, g);
    moves += roomMoves(room, pushes, g);
    player = room.entry;
  }
  return moves;
}

function main(): void {
  for (const id of IDS) {
    const text = readFileSync(join(INPUT_DIR, id, "level.txt"), "utf8");
    const grid = parseLevel(text);
    const moves = solveLevel(grid);
    const outDir = join(OUTPUT_DIR, id);
    mkdirSync(outDir, { recursive: true });
    writeFileSync(join(outDir, "solution.txt"), moves, "utf8");
  }
}

main();
