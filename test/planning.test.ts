import assert from "node:assert/strict";
import test from "node:test";

import {
  COLS,
  ROWS,
  configureGrid,
  defaultDynamicObstacles,
  dynamicObstaclesAt,
  makeCostField,
  runPlanner,
  type PlannerKey,
  type Point,
} from "../app/planning.ts";

const algorithms: PlannerKey[] = ["astar", "dijkstra", "rrt", "rrtstar"];
const scenarios: Array<{ start: Point; goal: Point }> = [
  { start: { x: 3, y: 20 }, goal: { x: 32, y: 3 } },
  { start: { x: 2, y: 2 }, goal: { x: 31, y: 20 } },
];

function assertPathIsCollisionFree(path: Point[], blocked: Set<string>) {
  for (let index = 1; index < path.length; index += 1) {
    const from = path[index - 1];
    const to = path[index];
    const samples = Math.max(2, Math.ceil(Math.hypot(to.x - from.x, to.y - from.y) / 0.08));
    for (let sampleIndex = 0; sampleIndex <= samples; sampleIndex += 1) {
      const t = sampleIndex / samples;
      const x = Math.round(from.x + (to.x - from.x) * t);
      const y = Math.round(from.y + (to.y - from.y) * t);
      assert.equal(blocked.has(`${x},${y}`), false);
    }
  }
}

test("cost map expands obstacle cells when inflation grows", () => {
  const obstacles = dynamicObstaclesAt(1_000);
  const compact = makeCostField(obstacles, 0.8);
  const inflated = makeCostField(obstacles, 2.2);

  assert.ok(inflated.blocked.size > compact.blocked.size);
  assert.equal(inflated.raw.size, compact.raw.size);
});

test("dynamic obstacles support regular, random, and user-defined motion seeds", () => {
  const defaults = defaultDynamicObstacles();
  const regularStart = dynamicObstaclesAt(0, defaults, "regular");
  const regularLater = dynamicObstaclesAt(2_000, defaults, "regular");
  const randomLater = dynamicObstaclesAt(2_000, defaults, "random");

  assert.notDeepEqual(regularLater, regularStart);
  assert.notDeepEqual(randomLater, regularLater);

  const custom = dynamicObstaclesAt(1_000, [{ x: 12, y: 10, phase: 0.6 }], "random");
  assert.equal(custom.length, 1);
  assert.ok(custom[0].x >= 2 && custom[0].x <= COLS - 3);
  assert.ok(custom[0].y >= 2 && custom[0].y <= ROWS - 3);
});

for (const [scenarioIndex, scenario] of scenarios.entries()) {
  for (const algorithm of algorithms) {
    test(`${algorithm} finds a collision-free path in scenario ${scenarioIndex + 1}`, () => {
      const field = makeCostField(dynamicObstaclesAt(1_000), 1.45);
      const result = runPlanner(algorithm, scenario.start, scenario.goal, field);

      assert.equal(result.found, true);
      assert.ok(result.steps.length > 0);
      assert.ok(result.path.length > 1);
      assert.deepEqual(result.path[0], scenario.start);
      assert.deepEqual(result.path.at(-1), scenario.goal);
      assertPathIsCollisionFree(result.path, field.blocked);
    });
  }
}

test("RRT* rewiring improves the deterministic RRT route", () => {
  const field = makeCostField(dynamicObstaclesAt(1_000), 1.45);
  const { start, goal } = scenarios[0];
  const rrt = runPlanner("rrt", start, goal, field);
  const rrtStar = runPlanner("rrtstar", start, goal, field);

  assert.ok(rrtStar.pathLength < rrt.pathLength);
});

test("all planners support a user-defined grid size", () => {
  try {
    configureGrid(48, 30);
    assert.equal(COLS, 48);
    assert.equal(ROWS, 30);

    const field = makeCostField(dynamicObstaclesAt(1_000), 1.45);
    assert.equal(field.costs.length, 30);
    assert.equal(field.costs[0].length, 48);

    const start = { x: 4, y: 25 };
    const goal = { x: 44, y: 4 };
    for (const algorithm of algorithms) {
      const result = runPlanner(algorithm, start, goal, field);
      assert.equal(result.found, true);
      assertPathIsCollisionFree(result.path, field.blocked);
    }
  } finally {
    configureGrid(35, 24);
  }
});
