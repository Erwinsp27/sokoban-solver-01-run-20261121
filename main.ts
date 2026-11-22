// Sokoban solver: push-based best-first search per room, with an
// admissible matching heuristic, deadlock pruning, counted work
// budgets, and deterministic tie-breaking throughout. No randomness,
// no clock use, no I/O beyond the fixed input/output paths.
const fs = require("node:fs");
const path = require("node:path");

const IDS = ["i001", "i002", "i003", "i004", "i005"];
const DIRS = [
  { dr: -1, dc: 0, ch: "u" },
  { dr: 1, dc: 0, ch: "d" },
  { dr: 0, dc: -1, ch: "l" },
  { dr: 0, dc: 1, ch: "r" },
];

// Counted-work budgets (never wall-clock). Tuned from local runs so
// the whole set finishes comfortably inside the 60s wall budget.
const GLOBAL_CAP = 900000;
const ROOM_CAP = 250000;
const GREEDY_CAP = 60000;
const globalBudget = { expansions: 0 };

function parseLevel(text) {
  const raw = text.split(/\r\n|\n|\r/);
  while (raw.length > 0 && raw[raw.length - 1] === "") raw.pop();
  const H = raw.length;
  let W = 0;
  for (const row of raw) if (row.length > W) W = row.length;
  const N = H * W;
  const wall = new Uint8Array(N);
  const goal = new Uint8Array(N);
  const box = new Uint8Array(N);
  let player = -1;
  const boxes = [];
  const goals = [];
  for (let r = 0; r < H; r++) {
    const row = raw[r];
    for (let c = 0; c < W; c++) {
      const ch = c < row.length ? row[c] : "#";
      const i = r * W + c;
      if (ch === "#") wall[i] = 1;
      else if (ch === ".") { goals.push(i); goal[i] = 1; }
      else if (ch === "$") { boxes.push(i); box[i] = 1; }
      else if (ch === "*") { boxes.push(i); box[i] = 1; goals.push(i); goal[i] = 1; }
      else if (ch === "@") player = i;
      else if (ch === " ") { /* floor */ }
      else wall[i] = 1;
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { H, W, N, wall, goal, box, player, boxes, goals };
}

function flood(start, boxSet, allowed, W, N) {
  const seen = new Uint8Array(N);
  if (start < 0 || !allowed[start] || boxSet[start]) return seen;
  const stack = [start];
  seen[start] = 1;
  while (stack.length > 0) {
    const cur = stack.pop();
    const r = Math.floor(cur / W), c = cur % W;
    for (let d = 0; d < 4; d++) {
      const nr = r + DIRS[d].dr, nc = c + DIRS[d].dc;
      if (nr < 0 || nc < 0) continue;
      const ni = nr * W + nc;
      if (ni >= N || seen[ni] || !allowed[ni] || boxSet[ni]) continue;
      seen[ni] = 1;
      stack.push(ni);
    }
  }
  return seen;
}

function floodMin(start, boxSet, allowed, W, N) {
  const seen = flood(start, boxSet, allowed, W, N);
  for (let i = 0; i < N; i++) if (seen[i]) return i;
  return -1;
}

function permutations(n) {
  const out = [];
  const cur = [];
  const used = new Uint8Array(n);
  (function rec() {
    if (cur.length === n) { out.push(cur.slice()); return; }
    for (let i = 0; i < n; i++) {
      if (used[i]) continue;
      used[i] = 1; cur.push(i); rec(); cur.pop(); used[i] = 0;
    }
  })();
  return out;
}

// Shortest walking path (neighbour order u,d,l,r). blocked(i) -> bool.
function findPath(from, to, blocked, W, N) {
  if (from === to) return [];
  const prev = new Int32Array(N).fill(-1);
  const prevDir = new Int8Array(N).fill(-1);
  const queue = [from];
  prev[from] = from;
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const r = Math.floor(cur / W), c = cur % W;
    for (let d = 0; d < 4; d++) {
      const nr = r + DIRS[d].dr, nc = c + DIRS[d].dc;
      if (nr < 0 || nc < 0) continue;
      const ni = nr * W + nc;
      if (ni >= N || prev[ni] !== -1 || blocked(ni)) continue;
      prev[ni] = cur; prevDir[ni] = d;
      if (ni === to) {
        const dirs = [];
        let x = ni;
        while (x !== from) { dirs.push(prevDir[x]); x = prev[x]; }
        dirs.reverse();
        return dirs;
      }
      queue.push(ni);
    }
  }
  return null;
}

class Heap {
  constructor(cmp) { this.a = []; this.cmp = cmp; }
  get size() { return this.a.length; }
  push(v) {
    const a = this.a;
    a.push(v);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.cmp(a[i], a[p]) < 0) { const t = a[i]; a[i] = a[p]; a[p] = t; i = p; }
      else break;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1, r = l + 1;
        let m = i;
        if (l < a.length && this.cmp(a[l], a[m]) < 0) m = l;
        if (r < a.length && this.cmp(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        const t = a[i]; a[i] = a[m]; a[m] = t; i = m;
      }
    }
    return top;
  }
}

function makeSolver(allowed, boxAllowed, entry, boxes0, goals, exitIdx, W, N) {
  const n = boxes0.length;
  const goalSet = new Uint8Array(N);
  for (const g of goals) goalSet[g] = 1;
  // Distances ignoring boxes, from each goal over allowed squares.
  const distFromGoal = goals.map((g) => {
    const dist = new Int32Array(N).fill(-1);
    const q = [g];
    dist[g] = 0;
    let head = 0;
    while (head < q.length) {
      const cur = q[head++];
      const r = Math.floor(cur / W), c = cur % W;
      for (let d = 0; d < 4; d++) {
        const nr = r + DIRS[d].dr, nc = c + DIRS[d].dc;
        if (nr < 0 || nc < 0) continue;
        const ni = nr * W + nc;
        if (ni >= N || dist[ni] !== -1 || !allowed[ni]) continue;
        dist[ni] = dist[cur] + 1;
        q.push(ni);
      }
    }
    return dist;
  });
  const perms = permutations(n);
  const BIG = 100000;
  function heuristic(boxes) {
    let best = BIG;
    for (const perm of perms) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = distFromGoal[perm[i]][boxes[i]];
        if (d < 0) { s = BIG; break; }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  }
  function isGoal(boxes) {
    for (let i = 0; i < n; i++) if (!goalSet[boxes[i]]) return false;
    return true;
  }
  function blockedAt(i, r, c) {
    if (r < 0 || c < 0 || c >= W || r >= Math.floor(N / W)) return true;
    if (i < 0 || i >= N) return true;
    return !allowed[i];
  }
  function isDeadlocked(boxes, boxSet) {
    for (let b = 0; b < n; b++) {
      const i = boxes[b];
      if (goalSet[i]) continue;
      const r = Math.floor(i / W), c = i % W;
      const up = blockedAt(i - W, r - 1, c);
      const down = blockedAt(i + W, r + 1, c);
      const left = blockedAt(i - 1, r, c - 1);
      const right = blockedAt(i + 1, r, c + 1);
      if ((up && left) || (up && right) || (down && left) || (down && right)) return true;
      // Wall segments: run along a wall, bounded at both ends, no goal.
      for (const side of [-1, 1]) {
        if (blockedAt(i + side * W, r + side, c)) {
          let l = c, rr = c;
          while (l - 1 >= 0 && allowed[r * W + l - 1] && blockedAt(r * W + l - 1 + side * W, r + side, l - 1)) l--;
          while (rr + 1 < W && allowed[r * W + rr + 1] && blockedAt(r * W + rr + 1 + side * W, r + side, rr + 1)) rr++;
          const endL = l - 1 < 0 || !allowed[r * W + l - 1];
          const endR = rr + 1 >= W || !allowed[r * W + rr + 1];
          if (endL && endR) {
            let hasGoal = false;
            for (let cc = l; cc <= rr; cc++) if (goalSet[r * W + cc]) { hasGoal = true; break; }
            if (!hasGoal) return true;
          }
        }
        if (blockedAt(i + side, r, c + side)) {
          let t = r, bb = r;
          while (t - 1 >= 0 && allowed[(t - 1) * W + c] && blockedAt((t - 1) * W + c + side, t - 1, c + side)) t--;
          while (bb + 1 < Math.floor(N / W) && allowed[(bb + 1) * W + c] && blockedAt((bb + 1) * W + c + side, bb + 1, c + side)) bb++;
          const H = Math.floor(N / W);
          const endT = t - 1 < 0 || !allowed[(t - 1) * W + c];
          const endB = bb + 1 >= H || !allowed[(bb + 1) * W + c];
          if (endT && endB) {
            let hasGoal = false;
            for (let rrr = t; rrr <= bb; rrr++) if (goalSet[rrr * W + c]) { hasGoal = true; break; }
            if (!hasGoal) return true;
          }
        }
      }
      // Frozen 2x2 block of walls/boxes with a non-goal box.
      for (let dr = -1; dr <= 0; dr++) {
        for (let dc = -1; dc <= 0; dc++) {
          let all = true, hasNonGoal = false;
          for (let a = 0; a < 2; a++) {
            for (let bb2 = 0; bb2 < 2; bb2++) {
              const rr2 = r + dr + a, cc2 = c + dc + bb2;
              if (rr2 < 0 || cc2 < 0 || rr2 * W + cc2 >= N) continue;
              const j = rr2 * W + cc2;
              if (boxSet[j]) { if (!goalSet[j]) hasNonGoal = true; }
              else if (!allowed[j]) { /* wall */ }
              else { all = false; }
            }
          }
          if (all && hasNonGoal) return true;
        }
      }
    }
    return false;
  }
  function exitOk(boxes, canon) {
    if (exitIdx < 0) return true;
    const boxSet = new Uint8Array(N);
    for (const b of boxes) boxSet[b] = 1;
    const seen = flood(canon, boxSet, allowed, W, N);
    return seen[exitIdx] === 1;
  }
  function cmpBoxes(a, b) {
    for (let i = 0; i < a.length; i++) {
      if (a[i] !== b[i]) return a[i] - b[i];
    }
    return 0;
  }
  function search(cap, greedy) {
    const nodes = [];
    const bestG = new Map();
    const keyOf = (boxes, canon) => boxes.join(",") + "|" + canon;
    const initSet = new Uint8Array(N);
    for (const b of boxes0) initSet[b] = 1;
    const initCanon = floodMin(entry, initSet, allowed, W, N);
    if (initCanon < 0) return null;
    const h0 = heuristic(boxes0);
    const root = { boxes: boxes0.slice(), canon: initCanon, g: 0, h: h0, f: h0, parent: -1, push: null };
    nodes.push(root);
    bestG.set(keyOf(boxes0, initCanon), 0);
    const cmp = greedy
      ? (a, b) => {
          const na = nodes[a], nb = nodes[b];
          if (na.h !== nb.h) return na.h - nb.h;
          if (na.g !== nb.g) return nb.g - na.g;
          if (na.canon !== nb.canon) return na.canon - nb.canon;
          return cmpBoxes(na.boxes, nb.boxes);
        }
      : (a, b) => {
          const na = nodes[a], nb = nodes[b];
          if (na.f !== nb.f) return na.f - nb.f;
          if (na.h !== nb.h) return na.h - nb.h;
          if (na.g !== nb.g) return na.g - nb.g;
          if (na.canon !== nb.canon) return na.canon - nb.canon;
          return cmpBoxes(na.boxes, nb.boxes);
        };
    const heap = new Heap(cmp);
    heap.push(0);
    let expansions = 0;
    let bestGoalId = -1;
    let budgetHit = false;
    while (heap.size > 0) {
      const id = heap.pop();
      const node = nodes[id];
      const key = keyOf(node.boxes, node.canon);
      if (node.g > bestG.get(key)) continue;
      if (expansions >= cap || globalBudget.expansions >= GLOBAL_CAP) { budgetHit = true; break; }
      expansions++;
      globalBudget.expansions++;
      if (isGoal(node.boxes)) {
        if (exitOk(node.boxes, node.canon)) {
          return reconstruct(id);
        }
        continue;
      }
      const boxSet = new Uint8Array(N);
      for (const b of node.boxes) boxSet[b] = 1;
      const reach = flood(node.canon, boxSet, allowed, W, N);
      for (let b = 0; b < n; b++) {
        const bi = node.boxes[b];
        const br = Math.floor(bi / W), bc = bi % W;
        for (let d = 0; d < 4; d++) {
          const dr = DIRS[d].dr, dc = DIRS[d].dc;
          const destR = br + dr, destC = bc + dc;
          if (destR < 0 || destC < 0) continue;
          const dest = destR * W + destC;
          if (dest >= N || !boxAllowed[dest] || boxSet[dest]) continue;
          const behindR = br - dr, behindC = bc - dc;
          if (behindR < 0 || behindC < 0) continue;
          const behind = behindR * W + behindC;
          if (behind >= N || !reach[behind]) continue;
          const nb = node.boxes.slice();
          nb[b] = dest;
          nb.sort((x, y) => x - y);
          const nSet = new Uint8Array(N);
          for (const x of nb) nSet[x] = 1;
          if (isDeadlocked(nb, nSet)) continue;
          const nCanon = floodMin(bi, nSet, allowed, W, N);
          if (nCanon < 0) continue;
          const ng = node.g + 1;
          const nKey = keyOf(nb, nCanon);
          const prevG = bestG.get(nKey);
          if (prevG !== undefined && prevG <= ng) continue;
          const nh = heuristic(nb);
          const nid = nodes.length;
          nodes.push({ boxes: nb, canon: nCanon, g: ng, h: nh, f: ng + nh, parent: id, push: { from: bi, to: dest, dir: d } });
          bestG.set(nKey, ng);
          if (isGoal(nb) && exitOk(nb, nCanon)) {
            if (bestGoalId < 0 || ng < nodes[bestGoalId].g) bestGoalId = nid;
          }
          heap.push(nid);
        }
      }
    }
    if (budgetHit && bestGoalId >= 0 && !greedy) return reconstruct(bestGoalId);
    return null;
    function reconstruct(id2) {
      const pushes = [];
      let cur = id2;
      while (nodes[cur].push) { pushes.push(nodes[cur].push); cur = nodes[cur].parent; }
      pushes.reverse();
      return { pushes, finalBoxes: nodes[id2].boxes.slice() };
    }
  }
  return {
    solve() {
      const a = search(ROOM_CAP, false);
      if (a) return a;
      return search(GREEDY_CAP, true);
    },
  };
}

function decompose(level) {
  const { H, W, N, wall, boxes, goals, player } = level;
  const hallRow = Math.floor(player / W);
  const allowed = new Uint8Array(N); // open squares, all rows
  for (let i = 0; i < N; i++) if (!wall[i]) allowed[i] = 1;
  const seen = new Uint8Array(N);
  const rooms = [];
  for (let i = 0; i < N; i++) {
    const r = Math.floor(i / W);
    if (wall[i] || r === hallRow || seen[i]) continue;
    // BFS component excluding the hallway row.
    const comp = [];
    const stack = [i];
    seen[i] = 1;
    while (stack.length > 0) {
      const cur = stack.pop();
      comp.push(cur);
      const cr = Math.floor(cur / W), cc = cur % W;
      for (let d = 0; d < 4; d++) {
        const nr = cr + DIRS[d].dr, nc = cc + DIRS[d].dc;
        if (nr < 0 || nc < 0 || nr >= H || nc >= W) continue;
        const ni = nr * W + nc;
        if (nr === hallRow || wall[ni] || seen[ni]) continue;
        seen[ni] = 1;
        stack.push(ni);
      }
    }
    const compSet = new Uint8Array(N);
    for (const x of comp) compSet[x] = 1;
    const doorSet = new Set();
    for (const x of comp) {
      const cr = Math.floor(x / W), cc = x % W;
      for (let d = 0; d < 4; d++) {
        const nr = cr + DIRS[d].dr, nc = cc + DIRS[d].dc;
        if (nr === hallRow && nc >= 0 && nc < W && !wall[nr * W + nc]) doorSet.add(nr * W + nc);
      }
    }
    const doors = Array.from(doorSet).sort((a, b) => a - b);
    const roomBoxes = boxes.filter((b) => compSet[b]);
    const roomGoals = goals.filter((g) => compSet[g]);
    rooms.push({ squares: compSet, boxes: roomBoxes, goals: roomGoals, doors, door: doors[0] });
  }
  return { rooms, allowed, hallRow };
}

function solveInstance(text) {
  const level = parseLevel(text);
  const { H, W, N, wall, boxes, goals, player } = level;
  if (player < 0 || boxes.length !== goals.length || boxes.length === 0) return "";
  const { rooms, allowed } = decompose(level);
  const validDecomp =
    rooms.length > 0 &&
    rooms.every((rm) => rm.doors.length === 1 && rm.boxes.length === rm.goals.length) &&
    rooms.reduce((s, rm) => s + rm.boxes.length, 0) === boxes.length;
  // Solver for a single region (room or whole board).
  function solveRegion(regionAllowed, boxAllowed, entry, rBoxes, rGoals, exitIdx) {
    return makeSolver(regionAllowed, boxAllowed, entry, rBoxes, rGoals, exitIdx, W, N).solve();
  }
  if (!validDecomp) {
    const sol = solveRegion(allowed, allowed, player, boxes, goals, -1);
    if (!sol) return "";
    return expandAndVerify(level, null, sol, player);
  }
  // Solve rooms easiest-first for budgeting; stitch in door-column order.
  const order = rooms.map((rm, idx) => idx).sort((a, b) => {
    const ra = rooms[a], rb = rooms[b];
    if (ra.boxes.length !== rb.boxes.length) return ra.boxes.length - rb.boxes.length;
    const sa = countSet(ra.squares), sb = countSet(rb.squares);
    if (sa !== sb) return sa - sb;
    return ra.door - rb.door;
  });
  const solutions = new Array(rooms.length).fill(null);
  for (const idx of order) {
    const rm = rooms[idx];
    const regionAllowed = new Uint8Array(N);
    for (let i = 0; i < N; i++) if (rm.squares[i] || i === rm.door) regionAllowed[i] = 1;
    const sol = solveRegion(regionAllowed, rm.squares, rm.door, rm.boxes, rm.goals, rm.door);
    if (!sol) return "";
    solutions[idx] = sol;
  }
  const stitchOrder = rooms.map((rm, idx) => idx).sort((a, b) => rooms[a].door - rooms[b].door);
  let moves = [];
  let curPlayer = player;
  const fullBoxSet = new Uint8Array(N);
  for (const b of boxes) fullBoxSet[b] = 1;
  for (const idx of stitchOrder) {
    const rm = rooms[idx];
    const sol = solutions[idx];
    // Walk to the door on the full board.
    const toDoor = findPath(curPlayer, rm.door, (i) => wall[i] || fullBoxSet[i], W, N);
    if (toDoor === null) return "";
    moves = moves.concat(toDoor.map((d) => DIRS[d].ch));
    curPlayer = rm.door;
    // Expand room pushes into walks inside the room.
    const regionAllowed = new Uint8Array(N);
    for (let i = 0; i < N; i++) if (rm.squares[i] || i === rm.door) regionAllowed[i] = 1;
    const roomSet = new Uint8Array(N);
    for (const b of rm.boxes) roomSet[b] = 1;
    let roomPlayer = rm.door;
    for (const push of sol.pushes) {
      const br = Math.floor(push.from / W), bc = push.from % W;
      const behind = (br - DIRS[push.dir].dr) * W + (bc - DIRS[push.dir].dc);
      const walk = findPath(roomPlayer, behind, (i) => !regionAllowed[i] || roomSet[i], W, N);
      if (walk === null) return "";
      moves = moves.concat(walk.map((d) => DIRS[d].ch));
      moves.push(DIRS[push.dir].ch);
      roomSet[push.from] = 0;
      roomSet[push.to] = 1;
      roomPlayer = push.from;
    }
    // Update the full-board box set to this room's final arrangement.
    for (const b of rm.boxes) fullBoxSet[b] = 0;
    for (const b of sol.finalBoxes) fullBoxSet[b] = 1;
    curPlayer = roomPlayer;
  }
  const moveStr = moves.join("");
  if (!replayOk(level, moveStr)) return "";
  return moveStr;
}

function countSet(a) {
  let s = 0;
  for (let i = 0; i < a.length; i++) if (a[i]) s++;
  return s;
}

function expandAndVerify(level, _unused, sol, player) {
  // Whole-board fallback: expand pushes with full-board walks.
  const { W, N, wall } = level;
  const boxSet = new Uint8Array(N);
  for (const b of level.boxes) boxSet[b] = 1;
  let curPlayer = player;
  const moves = [];
  for (const push of sol.pushes) {
    const br = Math.floor(push.from / W), bc = push.from % W;
    const behind = (br - DIRS[push.dir].dr) * W + (bc - DIRS[push.dir].dc);
    const walk = findPath(curPlayer, behind, (i) => wall[i] || boxSet[i], W, N);
    if (walk === null) return "";
    for (const d of walk) moves.push(DIRS[d].ch);
    moves.push(DIRS[push.dir].ch);
    boxSet[push.from] = 0;
    boxSet[push.to] = 1;
    curPlayer = push.from;
  }
  const moveStr = moves.join("");
  if (!replayOk(level, moveStr)) return "";
  return moveStr;
}

function replayOk(level, moveStr) {
  const { W, N, wall, goal, player } = level;
  const boxSet = new Uint8Array(N);
  for (const b of level.boxes) boxSet[b] = 1;
  let cur = player;
  const dirOf = { u: 0, d: 1, l: 2, r: 3 };
  for (const ch of moveStr) {
    const d = dirOf[ch];
    if (d === undefined) return false;
    const r = Math.floor(cur / W), c = cur % W;
    const nr = r + DIRS[d].dr, nc = c + DIRS[d].dc;
    if (nr < 0 || nc < 0 || nr * W + nc >= N) return false;
    const ni = nr * W + nc;
    if (wall[ni]) return false;
    if (boxSet[ni]) {
      const br = nr + DIRS[d].dr, bc = nc + DIRS[d].dc;
      if (br < 0 || bc < 0 || br * W + bc >= N) return false;
      const bi = br * W + bc;
      if (wall[bi] || boxSet[bi]) return false;
      boxSet[ni] = 0;
      boxSet[bi] = 1;
    }
    cur = ni;
  }
  for (let i = 0; i < N; i++) if (boxSet[i] && !goal[i]) return false;
  return true;
}

function resolveRoots() {
  // The grading driver names the input and output directories in
  // MAKEFUN_INPUT_DIR / MAKEFUN_OUTPUT_DIR. In the docker sandbox
  // those point at the fixed /input and /output roots. Fall back to
  // a working-directory-relative "input" directory, then to the
  // absolute "/input" path. The output root is the matching
  // MAKEFUN_OUTPUT_DIR, or the sibling "output" directory of the
  // input root found. The choice depends only on the directories
  // and variables the driver supplies, never on the clock.
  const envInput = process.env.MAKEFUN_INPUT_DIR || "";
  const envOutput = process.env.MAKEFUN_OUTPUT_DIR || "";
  let inputRoot = envInput;
  if (!inputRoot) {
    const candidates = [path.join(process.cwd(), "input"), "/input"];
    inputRoot = candidates[1];
    for (const cand of candidates) {
      try {
        if (fs.statSync(path.join(cand, "i001", "level.txt")).isFile()) { inputRoot = cand; break; }
      } catch (e) { /* try next */ }
    }
  }
  const outputRoot = envOutput || path.join(path.dirname(inputRoot), "output");
  const outputRoots = [outputRoot];
  if (outputRoot !== "/output") outputRoots.push("/output");
  return { inputRoot, outputRoots };
}

function main() {
  const { inputRoot, outputRoots } = resolveRoots();
  for (const id of IDS) {
    const inPath = path.join(inputRoot, id, "level.txt");
    let answer = "";
    try {
      const text = fs.readFileSync(inPath, "utf8");
      answer = solveInstance(text);
    } catch (e) {
      answer = "";
    }
    for (const outRoot of outputRoots) {
      try {
        const outDir = path.join(outRoot, id);
        fs.mkdirSync(outDir, { recursive: true });
        fs.writeFileSync(path.join(outDir, "solution.txt"), answer, "utf8");
      } catch (e) {
        // Last resort: nothing more can be done for this instance.
      }
    }
  }
}

main();
