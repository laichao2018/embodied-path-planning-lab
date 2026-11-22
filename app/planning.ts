export type Point = { x: number; y: number };
export type PlannerKey = "astar" | "dijkstra" | "rrt";

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

export const COLS = 35;
export const ROWS = 24;
export const CELL = 24;
export const CANVAS_WIDTH = COLS * CELL;
export const CANVAS_HEIGHT = ROWS * CELL;

export const STATIC_RECTS = [
  { x: 6, y: 3, w: 4, h: 7 },
  { x: 15, y: 0, w: 3, h: 8 },
  { x: 22, y: 10, w: 9, h: 3 },
  { x: 12, y: 17, w: 3, h: 6 },
  { x: 25, y: 17, w: 3, h: 4 },
  { x: 4, y: 14, w: 5, h: 2 },
];

export const keyOf = (point: Point) => `${Math.round(point.x)},${Math.round(point.y)}`;

export function dynamicObstaclesAt(timeMs: number): Point[] {
  const t = timeMs / 1000;
  return [
    { x: 19 + Math.sin(t * 0.72) * 3.4, y: 14.7 },
    { x: 29.4, y: 5.7 + Math.sin(t * 0.58 + 1.4) * 3.1 },
    { x: 8.5 + Math.sin(t * 0.49 + 2.2) * 2.3, y: 19.2 },
  ];
}

function addRectCells(target: Set<string>, x: number, y: number, w: number, h: number) {
  for (let row = y; row < y + h; row += 1) {
    for (let col = x; col < x + w; col += 1) target.add(`${col},${row}`);
  }
}

export function makeCostField(dynamicObstacles: Point[], inflationRadius: number): CostField {
  const raw = new Set<string>();
  STATIC_RECTS.forEach((rect) => addRectCells(raw, rect.x, rect.y, rect.w, rect.h));
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

  const obstaclePoints = [...raw].map((key) => {
    const [x, y] = key.split(",").map(Number);
    return { x, y };
  });
  const blocked = new Set<string>();
  const costs = Array.from({ length: ROWS }, () => Array.from({ length: COLS }, () => 1));

  for (let y = 0; y < ROWS; y += 1) {
    for (let x = 0; x < COLS; x += 1) {
      let minDistance = Number.POSITIVE_INFINITY;
      for (const obstacle of obstaclePoints) {
        minDistance = Math.min(minDistance, Math.hypot(x - obstacle.x, y - obstacle.y));
      }
      if (minDistance <= inflationRadius) blocked.add(`${x},${y}`);
      const safetyBand = Math.max(0, inflationRadius + 3.2 - minDistance);
      costs[y][x] = raw.has(`${x},${y}`) ? 99 : 1 + safetyBand * safetyBand * 0.65;
    }
  }
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
  const samples = Math.max(2, Math.ceil(distance / 0.18));
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

export function runPlanner(key: PlannerKey, start: Point, goal: Point, field: CostField) {
  if (key === "rrt") return rrtSearch(start, goal, field);
  return gridSearch(key, start, goal, field);
}

export function pointToCanvas(point: Point): Point {
  return { x: point.x * CELL + CELL / 2, y: point.y * CELL + CELL / 2 };
}
