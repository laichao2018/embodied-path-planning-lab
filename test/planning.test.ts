import assert from "node:assert/strict";
import test from "node:test";

import {
  dynamicObstaclesAt,
  makeCostField,
  runPlanner,
  type PlannerKey,
  type Point,
} from "../app/planning.ts";

const algorithms: PlannerKey[] = ["astar", "dijkstra", "rrt"];
const scenarios: Array<{ start: Point; goal: Point }> = [
  { start: { x: 3, y: 20 }, goal: { x: 32, y: 3 } },
  { start: { x: 2, y: 2 }, goal: { x: 31, y: 20 } },
];

test("cost map expands obstacle cells when inflation grows", () => {
  const obstacles = dynamicObstaclesAt(1_000);
  const compact = makeCostField(obstacles, 0.8);
  const inflated = makeCostField(obstacles, 2.2);

  assert.ok(inflated.blocked.size > compact.blocked.size);
  assert.equal(inflated.raw.size, compact.raw.size);
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
      assert.ok(result.path.every((point) => !field.blocked.has(`${Math.round(point.x)},${Math.round(point.y)}`)));
    });
  }
}
