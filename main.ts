// Sokoban solver — IDA* over pushes, per room (steady-badger-95).
// Design: split level at hallway row into independent rooms; solve each
// room push-optimally with iterative-deepening A* (bound = pushes + h,
// bound raised to smallest f exceeding it); h = min-cost box->goal
// matching over precomputed all-pairs BFS distances (admissible);
// prune dead squares (reverse-pull fixpoint) + frozen 2x2 blocks;
// transposition table per iteration (boxes + player region mask).
// Stitch rooms left-to-right, walking in the hallway between doors.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

export const solveStats = { expansions: 0, perRoom: [] as number[] };

const DIRS: [number, number, string][] = [[0, -1, 'u'], [0, 1, 'd'], [-1, 0, 'l'], [1, 0, 'r']];
const CAP_IDA_PER_ROOM = 150_000;
const CAP_FALLBACK_PER_ROOM = 100_000;

type Grid = { w: number; h: number; wall: Uint8Array; goals: Set<number>; boxes: number[]; player: number };

function parse(text: string): Grid {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goals = new Set<number>();
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.add(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// BFS walk path (letters) from `from` to `to` over allowed cells, boxes as obstacles. '' if unreachable / same.
function walkPath(w: number, wall: Uint8Array, allowed: Set<number> | null, boxes: Set<number>, from: number, to: number): string | null {
  if (from === to) return '';
  const prev = new Map<number, [number, string]>();
  const q = [from];
  prev.set(from, [-1, '']);
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    for (const [dx, dy, ch] of DIRS) {
      const n = c + dy * w + dx;
      if (wall[n] || boxes.has(n) || (allowed && !allowed.has(n)) || prev.has(n)) continue;
      prev.set(n, [c, ch]);
      if (n === to) {
        let s = '';
        for (let t = n; t !== from;) { const [p, m] = prev.get(t)!; s = m + s; t = p; }
        return s;
      }
      q.push(n);
    }
  }
  return null;
}

function floodMask(cells: number[], idx: Map<number, number>, wall: Uint8Array, w: number, boxes: Set<number>, start: number): bigint {
  let mask = 0n;
  const q = [start];
  const seen = new Set([start]);
  while (q.length) {
    const c = q.pop()!;
    const i = idx.get(c);
    if (i !== undefined) mask |= 1n << BigInt(i);
    for (const [dx, dy] of DIRS) {
      const n = c + dy * w + dx;
      if (wall[n] || boxes.has(n) || seen.has(n) || !idx.has(n)) continue;
      seen.add(n);
      q.push(n);
    }
  }
  return mask;
}

type Room = { cells: number[]; door: number; entry: number; boxes: number[]; goals: number[] };

function findRooms(g: Grid): Room[] {
  const hallY = Math.floor(g.player / g.w);
  const hall = new Set<number>();
  for (let x = 0; x < g.w; x++) { const i = hallY * g.w + x; if (!g.wall[i]) hall.add(i); }
  const seen = new Set<number>();
  const rooms: Room[] = [];
  for (let i = 0; i < g.w * g.h; i++) {
    if (g.wall[i] || hall.has(i) || seen.has(i)) continue;
    const cells: number[] = [];
    const q = [i];
    seen.add(i);
    while (q.length) {
      const c = q.pop()!;
      cells.push(c);
      for (const [dx, dy] of DIRS) {
        const n = c + dy * g.w + dx;
        if (g.wall[n] || hall.has(n) || seen.has(n)) continue;
        seen.add(n);
        q.push(n);
      }
    }
    let door = -1;
    let entry = -1;
    for (const c of cells) {
      for (const [dx, dy] of DIRS) {
        const n = c + dy * g.w + dx;
        if (hall.has(n)) { door = c; entry = n; break; }
      }
      if (door >= 0) break;
    }
    if (door < 0) continue;
    rooms.push({
      cells,
      door,
      entry,
      boxes: g.boxes.filter((b) => cells.includes(b)).sort((a, b) => a - b),
      goals: [...g.goals].filter((x) => cells.includes(x)).sort((a, b) => a - b),
    });
  }
  rooms.sort((a, b) => (a.door % g.w) - (b.door % g.w) || Math.floor(a.door / g.w) - Math.floor(b.door / g.w));
  return rooms;
}

// Push-optimal IDA* for one room. Player starts at `entry` (hallway cell).
// Returns push list [{box, dirIdx}] or null. cellsAll = room cells + entry.
function solveRoom(g: Grid, room: Room, needExit: boolean): { box: number; dir: number }[] | null {
  const cellsAll = [...room.cells, room.entry];
  const idx = new Map(cellsAll.map((c, i) => [c, i] as [number, number]));
  const n = cellsAll.length;
  const inRoom = new Set(room.cells);
  const goalSet = new Set(room.goals);
  // All-pairs BFS distances over room cells (+entry) ignoring boxes.
  const dist: number[][] = [];
  for (let s = 0; s < n; s++) {
    const d = new Array<number>(n).fill(Infinity);
    d[s] = 0;
    const q = [s];
    for (let qi = 0; qi < q.length; qi++) {
      const c = cellsAll[q[qi]!]!;
      for (const [dx, dy] of DIRS) {
        const m = idx.get(c + dy * g.w + dx);
        if (m === undefined || d[m] !== Infinity) continue;
        d[m] = d[q[qi]!]! + 1;
        q.push(m);
      }
    }
    dist.push(d);
  }
  // Dead squares: cells from which no box can be pushed onto a goal
  // (reverse-pull fixpoint, ignoring other boxes / player reachability).
  const alive = new Array<boolean>(n).fill(false);
  for (const gl of room.goals) alive[idx.get(gl)!] = true;
  let changed = true;
  while (changed) {
    changed = false;
    for (let c = 0; c < n; c++) {
      if (alive[c] || !inRoom.has(cellsAll[c]!)) continue;
      for (const [dx, dy] of DIRS) {
        const fwd = idx.get(cellsAll[c]! + dy * g.w + dx);
        const back = idx.get(cellsAll[c]! - dy * g.w - dx);
        if (fwd !== undefined && back !== undefined && alive[fwd]) { alive[c] = true; changed = true; break; }
      }
    }
  }
  const dead = alive.map((a, i) => inRoom.has(cellsAll[i]!) && !a);
  // Min-cost matching heuristic (permutations, <=4 boxes).
  const goalIdx = room.goals.map((x) => idx.get(x)!);
  function heur(boxes: number[]): number {
    const bi = boxes.map((b) => idx.get(b)!);
    let best = Infinity;
    const perm = goalIdx.slice();
    const rec = (k: number, sum: number): void => {
      if (sum >= best) return;
      if (k === bi.length) { best = sum; return; }
      for (let j = k; j < perm.length; j++) {
        const d = dist[bi[k]!]![perm[j]!]!;
        if (d === Infinity) continue;
        [perm[k], perm[j]] = [perm[j]!, perm[k]!];
        rec(k + 1, sum + d);
        [perm[k], perm[j]] = [perm[j]!, perm[k]!];
      }
    };
    rec(0, 0);
    return best;
  }
  const doorBit = 1n << BigInt(idx.get(room.door)!);
  const boxSet = (bs: number[]): Set<number> => new Set(bs);
  function frozen(bs: number[], moved = -1): boolean {
    const s = boxSet(bs);
    for (const b of bs) {
      if (goalSet.has(b)) continue;
      if (dead[idx.get(b)!]) return true;
    }
    // General freeze deadlock (sound fixpoint): a box is frozen when it
    // is blocked in both axes by walls or boxes already proven frozen
    // (base case: blocked by walls only). A frozen box never moves, so
    // a frozen box off-goal makes the state dead. Pure-box 2x2 cycles
    // have no wall seed, so the explicit 2x2 check below still applies.
    const frozenSet = new Set<number>();
    let fChanged = true;
    while (fChanged) {
      fChanged = false;
      for (const b of bs) {
        if (frozenSet.has(b)) continue;
        const x = b % g.w;
        const y = Math.floor(b / g.w);
        const blocked = (nx: number, ny: number): boolean => {
          if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) return true;
          const c = ny * g.w + nx;
          return g.wall[c] === 1 || frozenSet.has(c);
        };
        if ((blocked(x - 1, y) || blocked(x + 1, y)) && (blocked(x, y - 1) || blocked(x, y + 1))) {
          frozenSet.add(b);
          fChanged = true;
        }
      }
    }
    for (const b of frozenSet) if (!goalSet.has(b)) return true;
    // Frozen 2x2: all four cells wall-or-box, >=1 box off-goal inside.
    // Only windows touching the just-moved box can newly freeze (full
    // scan only for the start state, moved === -1).
    const wins: [number, number][] = [];
    if (moved >= 0) {
      const mx = moved % g.w;
      const my = Math.floor(moved / g.w);
      for (let y = my - 1; y <= my; y++) for (let x = mx - 1; x <= mx; x++) wins.push([x, y]);
    } else {
      for (let y = 0; y < g.h - 1; y++) for (let x = 0; x < g.w - 1; x++) wins.push([x, y]);
    }
    for (const [x, y] of wins) {
      if (x < 0 || y < 0 || x >= g.w - 1 || y >= g.h - 1) continue;
      const cs = [y * g.w + x, y * g.w + x + 1, (y + 1) * g.w + x, (y + 1) * g.w + x + 1];
      if (!cs.every((c) => g.wall[c] || s.has(c))) continue;
      if (cs.some((c) => s.has(c) && !goalSet.has(c))) return true;
    }
    return false;
  }
  function isGoal(bs: number[], region: bigint): boolean {
    // Last room needs no exit: the player may end inside it, so a box
    // may finish on the door cell itself. Other rooms must stay
    // escapable, otherwise stitching to the next room is impossible.
    // (Checker scores pushes <= P* as 1000, so a shorter last-room
    // solution loses nothing.)
    if (needExit && (region & doorBit) === 0n) return false;
    return bs.every((b, i) => b === room.goals[i]);
  }
  const startBoxes = room.boxes.slice();
  const startRegion = floodMask(cellsAll, idx, g.wall, g.w, boxSet(startBoxes), room.entry);
  const h0 = heur(startBoxes);
  if (h0 === Infinity || frozen(startBoxes)) return null;
  let expansions = 0;
  let aborted = false;
  const path: { box: number; dir: number }[] = [];
  let bound = h0;
  // Best-first (A*) fallback, run only if IDA* aborts on its cap. Same
  // heuristic, deadlock pruning and goal test as IDA*; a visited set
  // (boxes + player region) keeps it from re-expanding. Returns a VALID
  // push sequence that is usually near-optimal, so a hard room still
  // earns partial points (500*P*/pushes) instead of 0.
  function fallbackSearch(): { box: number; dir: number }[] | null {
    type FNode = { boxes: number[]; region: bigint; g: number; h: number; parent: FNode | null; push: { box: number; dir: number } | null; seq: number };
    let seq = 0;
    const heap: FNode[] = [];
    const cmp = (a: FNode, b: FNode): number => (a.g + a.h) - (b.g + b.h) || a.h - b.h || a.seq - b.seq;
    const heapPush = (nd: FNode): void => {
      heap.push(nd);
      for (let i = heap.length - 1; i > 0;) {
        const p = (i - 1) >> 1;
        if (cmp(heap[p]!, heap[i]!) <= 0) break;
        [heap[p], heap[i]] = [heap[i]!, heap[p]!];
        i = p;
      }
    };
    const heapPop = (): FNode | undefined => {
      const top = heap[0];
      const last = heap.pop()!;
      if (heap.length) {
        heap[0] = last;
        for (let i = 0; ;) {
          const l = i * 2 + 1;
          const r = l + 1;
          let m = i;
          if (l < heap.length && cmp(heap[l]!, heap[m]!) < 0) m = l;
          if (r < heap.length && cmp(heap[r]!, heap[m]!) < 0) m = r;
          if (m === i) break;
          [heap[m], heap[i]] = [heap[i]!, heap[m]!];
          i = m;
        }
      }
      return top;
    };
    const closed = new Map<string, number>();
    const keyOf = (boxes: number[], region: bigint): string => boxes.join(',') + '|' + region.toString(36);
    let fbExp = 0;
    const start: FNode = { boxes: startBoxes.slice(), region: startRegion, g: 0, h: h0, parent: null, push: null, seq: seq++ };
    heapPush(start);
    closed.set(keyOf(start.boxes, start.region), 0);
    while (heap.length) {
      const nd = heapPop()!;
      if (isGoal(nd.boxes, nd.region)) {
        const out: { box: number; dir: number }[] = [];
        for (let cur: FNode | null = nd; cur && cur.push; cur = cur.parent) out.push(cur.push);
        out.reverse();
        return out;
      }
      if (++fbExp > CAP_FALLBACK_PER_ROOM) return null;
      expansions++;
      const bs = boxSet(nd.boxes);
      const succ: { nb: number[]; nr: bigint; box: number; dir: number; h: number }[] = [];
      for (let bi = 0; bi < nd.boxes.length; bi++) {
        const b = nd.boxes[bi]!;
        for (let di = 0; di < 4; di++) {
          const [dx, dy] = DIRS[di]!;
          const dest = b + dy * g.w + dx;
          const from = b - dy * g.w - dx;
          if (!inRoom.has(dest) || bs.has(dest)) continue;
          const fromIdx = idx.get(from);
          if (fromIdx === undefined || (nd.region & (1n << BigInt(fromIdx))) === 0n) continue;
          const nb = nd.boxes.slice();
          nb[bi] = dest;
          nb.sort((a, z) => a - z);
          if (frozen(nb, dest)) continue;
          const nh = heur(nb);
          if (nh === Infinity) continue;
          const nr = floodMask(cellsAll, idx, g.wall, g.w, boxSet(nb), b);
          succ.push({ nb, nr, box: b, dir: di, h: nh });
        }
      }
      succ.sort((a, b) => a.h - b.h || a.dir - b.dir || a.box - b.box);
      for (const s of succ) {
        const ng = nd.g + 1;
        const k = keyOf(s.nb, s.nr);
        const prev = closed.get(k);
        if (prev !== undefined && prev <= ng) continue;
        closed.set(k, ng);
        heapPush({ boxes: s.nb, region: s.nr, g: ng, h: s.h, parent: nd, push: { box: s.box, dir: s.dir }, seq: seq++ });
      }
    }
    return null;
  }
  for (;;) {
    let minNext = Infinity;
    const tt = new Map<string, number>();
    let found = false;
    const dfs = (boxes: number[], region: bigint, playerCell: number, depth: number, h: number): boolean => {
      if (aborted) return false;
      const f = depth + h;
      if (f > bound) { if (f < minNext) minNext = f; return false; }
      if (isGoal(boxes, region)) { found = true; return true; }
      const key = boxes.join(',') + '|' + region.toString(36);
      const prev = tt.get(key);
      if (prev !== undefined && prev <= depth) return false;
      tt.set(key, depth);
      if (++expansions > CAP_IDA_PER_ROOM) { aborted = true; return false; }
      const bs = boxSet(boxes);
      // Generate successors, cheapest heuristic first (deterministic tie-break).
      const succ: { nb: number[]; nr: bigint; box: number; dir: number; h: number }[] = [];
      for (let bi = 0; bi < boxes.length; bi++) {
        const b = boxes[bi]!;
        for (let di = 0; di < 4; di++) {
          const [dx, dy] = DIRS[di]!;
          const dest = b + dy * g.w + dx;
          const from = b - dy * g.w - dx;
          if (!inRoom.has(dest) || bs.has(dest)) continue;
          const fromIdx = idx.get(from);
          if (fromIdx === undefined || (region & (1n << BigInt(fromIdx))) === 0n) continue;
          const nb = boxes.slice();
          nb[bi] = dest;
          nb.sort((a, z) => a - z);
          if (frozen(nb, dest)) continue;
          const nh = heur(nb);
          if (nh === Infinity) continue;
          const nr = floodMask(cellsAll, idx, g.wall, g.w, boxSet(nb), b);
          succ.push({ nb, nr, box: b, dir: di, h: nh });
        }
      }
      succ.sort((a, b) => a.h - b.h || a.dir - b.dir || a.box - b.box);
      for (const s of succ) {
        path.push({ box: s.box, dir: s.dir });
        if (dfs(s.nb, s.nr, s.box, depth + 1, s.h)) return true;
        path.pop();
        if (aborted) return false;
      }
      return false;
    };
    if (dfs(startBoxes, startRegion, room.entry, 0, h0)) {
      solveStats.perRoom.push(expansions);
      solveStats.expansions += expansions;
      return path.slice();
    }
    if (aborted) break; // IDA* hit its cap: fall through to best-first fallback
    if (minNext === Infinity) {
      solveStats.perRoom.push(expansions);
      solveStats.expansions += expansions;
      return null;
    }
    bound = minNext;
  }
  // Fallback (only reached when IDA* aborted on its expansion cap):
  // a valid, usually near-optimal push sequence instead of 0 points
  // for the room. The old 4M cap could burn far past the 60 s wall
  // budget; 150k IDA* + 100k fallback bounds one bad room to ~25-35 s.
  const fb = fallbackSearch();
  solveStats.perRoom.push(expansions);
  solveStats.expansions += expansions;
  return fb;
}

export function solveLevel(levelText: string): string {
  const g = parse(levelText);
  const rooms = findRooms(g);
  let moves = '';
  let player = g.player;
  const boxes = new Set(g.boxes);
  const roomCellsAll = (room: Room): Set<number> => new Set([...room.cells, room.entry]);
  // Only the last non-empty room may end with the player inside it.
  let lastWithBoxes = -1;
  for (let i = 0; i < rooms.length; i++) if (rooms[i]!.boxes.length > 0) lastWithBoxes = i;
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri]!;
    if (room.boxes.length === 0) continue;
    const w1 = walkPath(g.w, g.wall, null, boxes, player, room.entry);
    if (w1 === null) return moves;
    moves += w1;
    player = room.entry;
    const pushes = solveRoom(g, room, ri < lastWithBoxes);
    if (!pushes) continue; // room unsolved: keep other rooms' points, don't abort the level
    const allowed = roomCellsAll(room);
    for (const p of pushes) {
      const [dx, dy, ch] = DIRS[p.dir]!;
      const from = p.box - dy * g.w - dx;
      const w2 = walkPath(g.w, g.wall, allowed, boxes, player, from);
      if (w2 === null) return moves;
      moves += w2 + ch;
      boxes.delete(p.box);
      boxes.add(p.box + dy * g.w + dx);
      player = p.box;
    }
  }
  return moves;
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
  const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
  for (const id of readdirSync(I).sort()) {
    const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
    mkdirSync(join(O, id), { recursive: true });
    writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
  }
}
