export type Point = { x: number; y: number };
export type StaticRect = { x: number; y: number; w: number; h: number };
export type PlannerKey = "astar" | "dijkstra" | "rrt" | "rrtstar";
export type DynamicMotion = "regular" | "random";
export type DynamicObstacleSeed = Point & { phase: number };

export type SearchStep = {
  pick: Point;
  generated: Point[];
  open: number;
  closed: number;
  edge?: [Point, Point];
};

export type PlannerResult = {
  key: PlannerKey;
  steps: SearchStep[];
  path: Point[];
  expanded: number;
  generated: number;
  pathLength: number;
  durationMs: number;
  found: boolean;
};

export type CostField = {
  raw: Set<string>;
  blocked: Set<string>;
  costs: number[][];
};

// The obstacle template uses the original compact coordinate system, while the
// workspace opens on a more spacious 120 × 100 map.
const BASE_COLS = 35;
const BASE_ROWS = 24;
const DEFAULT_COLS = 120;
const DEFAULT_ROWS = 100;
const MAX_RENDER_WIDTH = 1200;
const MAX_RENDER_HEIGHT = 800;
export const GRID_LIMITS = { minCols: 32, maxCols: 240, minRows: 20, maxRows: 200 };
export let CELL = Math.max(3, Math.min(24, Math.floor(Math.min(MAX_RENDER_WIDTH / DEFAULT_COLS, MAX_RENDER_HEIGHT / DEFAULT_ROWS))));
export let COLS = DEFAULT_COLS;
export let ROWS = DEFAULT_ROWS;
export let CANVAS_WIDTH = COLS * CELL;
export let CANVAS_HEIGHT = ROWS * CELL;

const BASE_STATIC_RECTS: StaticRect[] = [
  { x: 6, y: 3, w: 4, h: 7 },
  { x: 15, y: 0, w: 3, h: 8 },
  { x: 22, y: 10, w: 9, h: 3 },
  { x: 12, y: 17, w: 3, h: 6 },
  { x: 25, y: 17, w: 3, h: 4 },
  { x: 4, y: 14, w: 5, h: 2 },
];

function staticRectsForGrid(cols: number, rows: number): StaticRect[] {
  return BASE_STATIC_RECTS.map((rect) => {
    const x = Math.max(1, Math.min(cols - 2, Math.round((rect.x / BASE_COLS) * cols)));
    const y = Math.max(0, Math.min(rows - 2, Math.round((rect.y / BASE_ROWS) * rows)));
    const w = Math.max(1, Math.min(cols - 1 - x, Math.round((rect.w / BASE_COLS) * cols)));
    const h = Math.max(1, Math.min(rows - 1 - y, Math.round((rect.h / BASE_ROWS) * rows)));
    return { x, y, w, h };
  });
}

export let STATIC_RECTS = staticRectsForGrid(COLS, ROWS);

export function configureGrid(cols: number, rows: number) {
  COLS = Math.max(GRID_LIMITS.minCols, Math.min(GRID_LIMITS.maxCols, Math.round(cols)));
  ROWS = Math.max(GRID_LIMITS.minRows, Math.min(GRID_LIMITS.maxRows, Math.round(rows)));
  CELL = Math.max(3, Math.min(24, Math.floor(Math.min(MAX_RENDER_WIDTH / COLS, MAX_RENDER_HEIGHT / ROWS))));
  CANVAS_WIDTH = COLS * CELL;
  CANVAS_HEIGHT = ROWS * CELL;
  STATIC_RECTS = staticRectsForGrid(COLS, ROWS);
  return { cols: COLS, rows: ROWS };
}

export const keyOf = (point: Point) => `${Math.round(point.x)},${Math.round(point.y)}`;

export function defaultDynamicObstacles(): DynamicObstacleSeed[] {
  const scaleX = COLS / BASE_COLS;
  const scaleY = ROWS / BASE_ROWS;
  return [
    { x: 19 * scaleX, y: 14.7 * scaleY, phase: 0 },
    { x: 29.4 * scaleX, y: 5.7 * scaleY, phase: 1.4 },
    { x: 8.5 * scaleX, y: 19.2 * scaleY, phase: 2.2 },
  ];
}

export function dynamicObstaclesAt(
  timeMs: number,
  seeds: DynamicObstacleSeed[] = defaultDynamicObstacles(),
  motion: DynamicMotion = "regular",
): Point[] {
  const t = timeMs / 1000;
  return seeds.map((seed, index) => {
    let offsetX = 0;
    let offsetY = 0;
    if (motion === "random") {
      offsetX = Math.sin(t * (0.41 + index * 0.023) + seed.phase) * 2.1
        + Math.sin(t * (0.77 + index * 0.017) + seed.phase * 1.9) * 0.9;
      offsetY = Math.cos(t * (0.37 + index * 0.019) + seed.phase * 1.3) * 1.8
        + Math.sin(t * (0.69 + index * 0.013) + seed.phase * 2.4) * 0.8;
    } else if (index % 3 === 1) {
      offsetY = Math.sin(t * 0.58 + seed.phase) * 3.1;
    } else {
      const speed = index % 3 === 0 ? 0.72 : 0.49;
      const amplitude = index % 3 === 0 ? 3.4 : 2.3;
      offsetX = Math.sin(t * speed + seed.phase) * amplitude;
    }
    return {
      x: Math.max(2, Math.min(COLS - 3, seed.x + offsetX)),
      y: Math.max(2, Math.min(ROWS - 3, seed.y + offsetY)),
    };
  });
}

function addRectCells(target: Set<string>, x: number, y: number, w: number, h: number) {
  for (let row = y; row < y + h; row += 1) {
    for (let col = x; col < x + w; col += 1) target.add(`${col},${row}`);
  }
}

export function makeCostField(
  dynamicObstacles: Point[],
  inflationRadius: number,
  staticRects: StaticRect[] = STATIC_RECTS,
): CostField {
  const raw = new Set<string>();
  staticRects.forEach((rect) => addRectCells(raw, rect.x, rect.y, rect.w, rect.h));
  for (let x = 0; x < COLS; x += 1) {
    raw.add(`${x},0`);
    raw.add(`${x},${ROWS - 1}`);
  }
  for (let y = 0; y < ROWS; y += 1) {
    raw.add(`0,${y}`);
    raw.add(`${COLS - 1},${y}`);
  }
  dynamicObstacles.forEach((point) => {
    const cx = Math.round(point.x);
    const cy = Math.round(point.y);
    for (let y = cy - 1; y <= cy + 1; y += 1) {
      for (let x = cx - 1; x <= cx + 1; x += 1) {
        if ((x - cx) ** 2 + (y - cy) ** 2 <= 1.5) raw.add(`${x},${y}`);
      }
    }
  });

  const obstaclePoints = [...raw]
    .map((key) => {
      const [x, y] = key.split(",").map(Number);
      return { x, y };
    })
    .filter((point) => point.x >= 0 && point.x < COLS && point.y >= 0 && point.y < ROWS);
  const blocked = new Set<string>(raw);
  const costs = Array.from({ length: ROWS }, () => Array.from({ length: COLS }, () => 1));
  obstaclePoints.forEach((point) => { costs[point.y][point.x] = 99; });

  // Only a local safety band has non-neutral cost. Updating that band per obstacle
  // keeps the cost map responsive even at the 240 × 200 grid limit.
  const safetyReach = inflationRadius + 3.2;
  const extent = Math.ceil(safetyReach);
  obstaclePoints.forEach((obstacle) => {
    for (let y = Math.max(0, obstacle.y - extent); y <= Math.min(ROWS - 1, obstacle.y + extent); y += 1) {
      for (let x = Math.max(0, obstacle.x - extent); x <= Math.min(COLS - 1, obstacle.x + extent); x += 1) {
        const distance = Math.hypot(x - obstacle.x, y - obstacle.y);
        if (distance > safetyReach) continue;
        const key = `${x},${y}`;
        if (distance <= inflationRadius) blocked.add(key);
        if (!raw.has(key)) {
          const safetyBand = safetyReach - distance;
          costs[y][x] = Math.max(costs[y][x], 1 + safetyBand * safetyBand * 0.65);
        }
      }
    }
  });
  return { raw, blocked, costs };
}

const octile = (a: Point, b: Point) => {
  const dx = Math.abs(a.x - b.x);
  const dy = Math.abs(a.y - b.y);
  return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy);
};

function reconstruct(parent: Map<string, string>, goal: Point): Point[] {
  const path: Point[] = [goal];
  let current = keyOf(goal);
  let guard = 0;
  while (parent.has(current) && guard < COLS * ROWS + 20) {
    current = parent.get(current)!;
    const [x, y] = current.split(",").map(Number);
    path.push({ x, y });
    guard += 1;
  }
  return path.reverse();
}

function pathLength(path: Point[]) {
  let total = 0;
  for (let i = 1; i < path.length; i += 1) {
    total += Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return total;
}

function gridSearch(
  key: "astar" | "dijkstra",
  start: Point,
  goal: Point,
  field: CostField,
): PlannerResult {
  const begin = performance.now();
  const open: Array<{ point: Point; g: number; priority: number }> = [
    { point: start, g: 0, priority: key === "astar" ? octile(start, goal) : 0 },
  ];
  const gScore = new Map<string, number>([[keyOf(start), 0]]);
  const parent = new Map<string, string>();
  const closed = new Set<string>();
  const steps: SearchStep[] = [];
  let generatedCount = 1;
  let finalPath: Point[] = [];
  const directions = [
    [-1, 0], [1, 0], [0, -1], [0, 1],
    [-1, -1], [1, -1], [-1, 1], [1, 1],
  ];

  while (open.length && steps.length < COLS * ROWS * 3) {
    open.sort((a, b) => a.priority - b.priority || a.g - b.g);
    const current = open.shift()!;
    const currentKey = keyOf(current.point);
    if (closed.has(currentKey)) continue;
    closed.add(currentKey);

    if (currentKey === keyOf(goal)) {
      steps.push({ pick: current.point, generated: [], open: open.length, closed: closed.size });
      finalPath = reconstruct(parent, goal);
      break;
    }

    const generated: Point[] = [];
    for (const [dx, dy] of directions) {
      const neighbor = { x: current.point.x + dx, y: current.point.y + dy };
      const neighborKey = keyOf(neighbor);
      if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= COLS || neighbor.y >= ROWS) continue;
      if (field.blocked.has(neighborKey) || closed.has(neighborKey)) continue;
      if (dx !== 0 && dy !== 0) {
        if (
          field.blocked.has(`${current.point.x + dx},${current.point.y}`) ||
          field.blocked.has(`${current.point.x},${current.point.y + dy}`)
        ) continue;
      }
      const moveCost = (dx !== 0 && dy !== 0 ? Math.SQRT2 : 1) * field.costs[neighbor.y][neighbor.x];
      const tentative = current.g + moveCost;
      if (tentative >= (gScore.get(neighborKey) ?? Number.POSITIVE_INFINITY)) continue;
      gScore.set(neighborKey, tentative);
      parent.set(neighborKey, currentKey);
      const priority = tentative + (key === "astar" ? octile(neighbor, goal) : 0);
      open.push({ point: neighbor, g: tentative, priority });
      generated.push(neighbor);
      generatedCount += 1;
    }
    steps.push({ pick: current.point, generated, open: open.length, closed: closed.size });
  }

  return {
    key,
    steps,
    path: finalPath,
    expanded: closed.size,
    generated: generatedCount,
    pathLength: pathLength(finalPath),
    durationMs: performance.now() - begin,
    found: finalPath.length > 0,
  };
}

function mulberry32(seed: number) {
  return () => {
    seed |= 0;
    seed = (seed + 0x6d2b79f5) | 0;
    let value = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    value = (value + Math.imul(value ^ (value >>> 7), 61 | value)) ^ value;
    return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
  };
}

function segmentIsFree(a: Point, b: Point, blocked: Set<string>) {
  const distance = Math.hypot(b.x - a.x, b.y - a.y);
  const samples = Math.max(2, Math.ceil(distance / 0.08));
  for (let i = 0; i <= samples; i += 1) {
    const t = i / samples;
    const point = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
    if (blocked.has(keyOf(point))) return false;
  }
  return true;
}

function rrtSearch(start: Point, goal: Point, field: CostField): PlannerResult {
  const begin = performance.now();
  const random = mulberry32(Math.round(start.x * 997 + start.y * 131 + goal.x * 67 + goal.y * 17));
  const nodes: Array<Point & { parent: number }> = [{ ...start, parent: -1 }];
  const steps: SearchStep[] = [];
  let goalIndex = -1;
  const stepSize = 1.65;

  for (let iteration = 0; iteration < 1300; iteration += 1) {
    const sample = random() < 0.13
      ? goal
      : { x: 1 + random() * (COLS - 3), y: 1 + random() * (ROWS - 3) };
    let nearestIndex = 0;
    let nearestDistance = Number.POSITIVE_INFINITY;
    nodes.forEach((node, index) => {
      const distance = Math.hypot(node.x - sample.x, node.y - sample.y);
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestIndex = index;
      }
    });
    const nearest = nodes[nearestIndex];
    const scale = Math.min(stepSize, nearestDistance) / Math.max(nearestDistance, 0.0001);
    const next = {
      x: nearest.x + (sample.x - nearest.x) * scale,
      y: nearest.y + (sample.y - nearest.y) * scale,
    };
    if (next.x <= 1 || next.y <= 1 || next.x >= COLS - 2 || next.y >= ROWS - 2) continue;
    if (!segmentIsFree(nearest, next, field.blocked)) continue;
    nodes.push({ ...next, parent: nearestIndex });
    const nextIndex = nodes.length - 1;
    steps.push({
      pick: { x: nearest.x, y: nearest.y },
      generated: [next],
      open: nodes.length,
      closed: iteration + 1,
      edge: [{ x: nearest.x, y: nearest.y }, next],
    });

    if (Math.hypot(next.x - goal.x, next.y - goal.y) < 2.25 && segmentIsFree(next, goal, field.blocked)) {
      nodes.push({ ...goal, parent: nextIndex });
      goalIndex = nodes.length - 1;
      steps.push({
        pick: next,
        generated: [goal],
        open: nodes.length,
        closed: iteration + 2,
        edge: [next, goal],
      });
      break;
    }
  }

  const path: Point[] = [];
  if (goalIndex >= 0) {
    let index = goalIndex;
    while (index >= 0) {
      path.push({ x: nodes[index].x, y: nodes[index].y });
      index = nodes[index].parent;
    }
    path.reverse();
  }

  return {
    key: "rrt",
    steps,
    path,
    expanded: steps.length,
    generated: nodes.length,
    pathLength: pathLength(path),
    durationMs: performance.now() - begin,
    found: path.length > 0,
  };
}

function rrtStarSearch(start: Point, goal: Point, field: CostField): PlannerResult {
  const begin = performance.now();
  const random = mulberry32(
    Math.round(start.x * 997 + start.y * 131 + goal.x * 67 + goal.y * 17 + 4049),
  );
  const nodes: Array<Point & { parent: number; cost: number }> = [
    { ...start, parent: -1, cost: 0 },
  ];
  const steps: SearchStep[] = [];
  const stepSize = 1.55;
  const neighborRadius = 3.6;
  let bestGoalParent = -1;
  let bestGoalCost = Number.POSITIVE_INFINITY;

  const isAncestor = (candidate: number, index: number) => {
    let current = index;
    let guard = 0;
    while (current >= 0 && guard <= nodes.length) {
      if (current === candidate) return true;
      current = nodes[current].parent;
      guard += 1;
    }
    return false;
  };

  const refreshDescendantCosts = (parentIndex: number) => {
    for (let index = 0; index < nodes.length; index += 1) {
      if (nodes[index].parent !== parentIndex) continue;
      nodes[index].cost = nodes[parentIndex].cost + Math.hypot(
        nodes[index].x - nodes[parentIndex].x,
        nodes[index].y - nodes[parentIndex].y,
      );
      refreshDescendantCosts(index);
    }
  };

  for (let iteration = 0; iteration < 1600; iteration += 1) {
    const sample = random() < 0.1
      ? goal
      : { x: 1 + random() * (COLS - 3), y: 1 + random() * (ROWS - 3) };
    let nearestIndex = 0;
    let nearestDistance = Number.POSITIVE_INFINITY;
    nodes.forEach((node, index) => {
      const distance = Math.hypot(node.x - sample.x, node.y - sample.y);
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestIndex = index;
      }
    });

    const nearest = nodes[nearestIndex];
    const scale = Math.min(stepSize, nearestDistance) / Math.max(nearestDistance, 0.0001);
    const next = {
      x: nearest.x + (sample.x - nearest.x) * scale,
      y: nearest.y + (sample.y - nearest.y) * scale,
    };
    if (next.x <= 1 || next.y <= 1 || next.x >= COLS - 2 || next.y >= ROWS - 2) continue;
    if (!segmentIsFree(nearest, next, field.blocked)) continue;

    const nearIndices = nodes
      .map((node, index) => ({
        index,
        distance: Math.hypot(node.x - next.x, node.y - next.y),
      }))
      .filter(({ distance }) => distance <= neighborRadius);

    let parentIndex = nearestIndex;
    let parentCost = nearest.cost + Math.hypot(nearest.x - next.x, nearest.y - next.y);
    for (const candidate of nearIndices) {
      const node = nodes[candidate.index];
      const candidateCost = node.cost + candidate.distance;
      if (candidateCost >= parentCost || !segmentIsFree(node, next, field.blocked)) continue;
      parentIndex = candidate.index;
      parentCost = candidateCost;
    }

    nodes.push({ ...next, parent: parentIndex, cost: parentCost });
    const nextIndex = nodes.length - 1;

    for (const candidate of nearIndices) {
      if (candidate.index === 0 || candidate.index === parentIndex) continue;
      if (isAncestor(candidate.index, nextIndex)) continue;
      const node = nodes[candidate.index];
      const rewiredCost = parentCost + candidate.distance;
      if (rewiredCost + 1e-6 >= node.cost || !segmentIsFree(next, node, field.blocked)) continue;
      node.parent = nextIndex;
      node.cost = rewiredCost;
      refreshDescendantCosts(candidate.index);
    }

    const parent = nodes[parentIndex];
    steps.push({
      pick: { x: parent.x, y: parent.y },
      generated: [next],
      open: nodes.length,
      closed: iteration + 1,
      edge: [{ x: parent.x, y: parent.y }, next],
    });

    const distanceToGoal = Math.hypot(next.x - goal.x, next.y - goal.y);
    const goalCost = parentCost + distanceToGoal;
    if (
      distanceToGoal < 2.5 &&
      goalCost < bestGoalCost &&
      segmentIsFree(next, goal, field.blocked)
    ) {
      bestGoalParent = nextIndex;
      bestGoalCost = goalCost;
    }
  }

  const path: Point[] = [];
  if (bestGoalParent >= 0) {
    path.push(goal);
    let index = bestGoalParent;
    while (index >= 0) {
      path.push({ x: nodes[index].x, y: nodes[index].y });
      index = nodes[index].parent;
    }
    path.reverse();
  }

  return {
    key: "rrtstar",
    steps,
    path,
    expanded: steps.length,
    generated: nodes.length,
    pathLength: pathLength(path),
    durationMs: performance.now() - begin,
    found: path.length > 0,
  };
}

export function runPlanner(key: PlannerKey, start: Point, goal: Point, field: CostField) {
  if (key === "rrt") return rrtSearch(start, goal, field);
  if (key === "rrtstar") return rrtStarSearch(start, goal, field);
  return gridSearch(key, start, goal, field);
}

export function pointToCanvas(point: Point): Point {
  return { x: point.x * CELL + CELL / 2, y: point.y * CELL + CELL / 2 };
}
