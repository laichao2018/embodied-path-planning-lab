(function () {
  "use strict";

  // The obstacle template uses the original compact coordinate system, while the
  // workspace opens on a more spacious 120 × 100 map.
  const BASE_COLS = 35;
  const BASE_ROWS = 24;
  const DEFAULT_COLS = 120;
  const DEFAULT_ROWS = 100;
  const MAX_RENDER_WIDTH = 1200;
  const MAX_RENDER_HEIGHT = 800;
  const GRID_LIMITS = { minCols: 32, maxCols: 240, minRows: 20, maxRows: 200 };
  let CELL = Math.max(3, Math.min(24, Math.floor(Math.min(MAX_RENDER_WIDTH / DEFAULT_COLS, MAX_RENDER_HEIGHT / DEFAULT_ROWS))));
  let COLS = DEFAULT_COLS;
  let ROWS = DEFAULT_ROWS;
  let WIDTH = COLS * CELL;
  let HEIGHT = ROWS * CELL;
  const PLANNERS = ["astar", "dijkstra", "rrt", "rrtstar"];
  const BASE_STATIC_RECTS = [
    { x: 6, y: 3, w: 4, h: 7 },
    { x: 15, y: 0, w: 3, h: 8 },
    { x: 22, y: 10, w: 9, h: 3 },
    { x: 12, y: 17, w: 3, h: 6 },
    { x: 25, y: 17, w: 3, h: 4 },
    { x: 4, y: 14, w: 5, h: 2 },
  ];

  function staticRectsForGrid(cols, rows) {
    return BASE_STATIC_RECTS.map((rect) => {
      const x = Math.max(1, Math.min(cols - 2, Math.round(rect.x / BASE_COLS * cols)));
      const y = Math.max(0, Math.min(rows - 2, Math.round(rect.y / BASE_ROWS * rows)));
      const w = Math.max(1, Math.min(cols - 1 - x, Math.round(rect.w / BASE_COLS * cols)));
      const h = Math.max(1, Math.min(rows - 1 - y, Math.round(rect.h / BASE_ROWS * rows)));
      return { x, y, w, h };
    });
  }

  let STATIC_RECTS = staticRectsForGrid(COLS, ROWS);

  function configureGrid(cols, rows) {
    COLS = Math.max(GRID_LIMITS.minCols, Math.min(GRID_LIMITS.maxCols, Math.round(cols)));
    ROWS = Math.max(GRID_LIMITS.minRows, Math.min(GRID_LIMITS.maxRows, Math.round(rows)));
    CELL = Math.max(3, Math.min(24, Math.floor(Math.min(MAX_RENDER_WIDTH / COLS, MAX_RENDER_HEIGHT / ROWS))));
    WIDTH = COLS * CELL;
    HEIGHT = ROWS * CELL;
    STATIC_RECTS = staticRectsForGrid(COLS, ROWS);
    return { cols: COLS, rows: ROWS };
  }
  const COLORS = {
    astar: "#00bfff",
    dijkstra: "#ffd400",
    rrt: "#ff4b4b",
    rrtstar: "#a970ff",
  };
  const NAMES = { astar: "A*", dijkstra: "DIJKSTRA", rrt: "RRT", rrtstar: "RRT*" };
  function defaultPointsForGrid(cols, rows) {
    const horizontalInset = Math.max(3, Math.round(3 / BASE_COLS * cols));
    const startBottomInset = Math.max(3, Math.round(4 / BASE_ROWS * rows));
    const goalTopInset = Math.max(2, Math.round(3 / BASE_ROWS * rows));
    return {
      start: { x: horizontalInset, y: rows - startBottomInset },
      goal: { x: cols - horizontalInset, y: goalTopInset },
    };
  }

  const INITIAL_POINTS = defaultPointsForGrid(COLS, ROWS);

  function defaultDynamicObstacles() {
    const scaleX = COLS / BASE_COLS;
    const scaleY = ROWS / BASE_ROWS;
    return [
      { x: 19 * scaleX, y: 14.7 * scaleY, phase: 0 },
      { x: 29.4 * scaleX, y: 5.7 * scaleY, phase: 1.4 },
      { x: 8.5 * scaleX, y: 19.2 * scaleY, phase: 2.2 },
    ];
  }

  const canvas = document.getElementById("map");
  const mapPanel = document.getElementById("map-panel");
  const ctx = canvas.getContext("2d");
  const elements = {
    layerControls: document.getElementById("layer-controls"),
    layerName: document.getElementById("layer-name"),
    gridCols: document.getElementById("grid-cols"),
    gridRows: document.getElementById("grid-rows"),
    applyGrid: document.getElementById("apply-grid"),
    gridSizeStat: document.getElementById("grid-size-stat"),
    inflation: document.getElementById("inflation"),
    inflationValue: document.getElementById("inflation-value"),
    dynamicMode: document.getElementById("dynamic-mode"),
    dynamicCount: document.getElementById("dynamic-count"),
    addDynamic: document.getElementById("add-dynamic"),
    clearDynamic: document.getElementById("clear-dynamic"),
    resetDynamic: document.getElementById("reset-dynamic"),
    selectStart: document.getElementById("select-start"),
    selectGoal: document.getElementById("select-goal"),
    startValue: document.getElementById("start-value"),
    goalValue: document.getElementById("goal-value"),
    resetPoints: document.getElementById("reset-points"),
    speed: document.getElementById("speed"),
    speedValue: document.getElementById("speed-value"),
    runPlanner: document.getElementById("run-planner"),
    runState: document.getElementById("run-state"),
    timelineControl: document.getElementById("timeline-control"),
    timelineSlider: document.getElementById("timeline-slider"),
    timelineValue: document.getElementById("timeline-value"),
    progressBar: document.getElementById("progress-bar"),
    notice: document.getElementById("notice"),
    plannerPickerCount: document.getElementById("planner-picker-count"),
    emptyPlannerStatus: document.getElementById("empty-planner-status"),
    staticMenu: document.getElementById("static-menu"),
    staticMenuTitle: document.getElementById("static-menu-title"),
    staticMenuPosition: document.getElementById("static-menu-position"),
    staticWidth: document.getElementById("static-width"),
    staticHeight: document.getElementById("static-height"),
    saveStatic: document.getElementById("save-static"),
    deleteStatic: document.getElementById("delete-static"),
    closeStaticMenu: document.getElementById("close-static-menu"),
  };

  const state = {
    layer: "cost",
    placement: "start",
    start: { ...INITIAL_POINTS.start },
    goal: { ...INITIAL_POINTS.goal },
    inflation: 1.45,
    speed: 4,
    dynamicMotion: "regular",
    dynamicSeeds: defaultDynamicObstacles(),
    placingDynamic: false,
    enabled: { astar: true, dijkstra: true, rrt: true, rrtstar: true },
    results: {},
    progress: 0,
    running: false,
    executing: false,
    executionCursor: 0,
    draggingMarker: null,
    draggingStatic: null,
    staticMenu: null,
    notice: "拖动地图上的起点或终点图标设置位置",
  };

  function keyOf(point) {
    return `${Math.round(point.x)},${Math.round(point.y)}`;
  }

  function formatCell(point) {
    return `${Math.round(point.x).toString().padStart(2, "0")},${Math.round(point.y).toString().padStart(2, "0")}`;
  }

  function dynamicObstaclesAt(timeMs, seeds = state.dynamicSeeds, motion = state.dynamicMotion) {
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

  function addRectCells(target, x, y, width, height) {
    for (let row = y; row < y + height; row += 1) {
      for (let col = x; col < x + width; col += 1) target.add(`${col},${row}`);
    }
  }

  function makeCostField(dynamicObstacles, inflationRadius, staticRects = STATIC_RECTS) {
    const raw = new Set();
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

    const obstaclePoints = [...raw].map((key) => {
      const [x, y] = key.split(",").map(Number);
      return { x, y };
    }).filter((point) => point.x >= 0 && point.x < COLS && point.y >= 0 && point.y < ROWS);
    const blocked = new Set(raw);
    const costs = Array.from({ length: ROWS }, () => Array.from({ length: COLS }, () => 1));
    obstaclePoints.forEach((point) => { costs[point.y][point.x] = 99; });
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

  function octile(a, b) {
    const dx = Math.abs(a.x - b.x);
    const dy = Math.abs(a.y - b.y);
    return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy);
  }

  function reconstruct(parent, goal) {
    const path = [goal];
    let current = keyOf(goal);
    let guard = 0;
    while (parent.has(current) && guard < COLS * ROWS + 20) {
      current = parent.get(current);
      const [x, y] = current.split(",").map(Number);
      path.push({ x, y });
      guard += 1;
    }
    return path.reverse();
  }

  function pathLength(path) {
    let total = 0;
    for (let i = 1; i < path.length; i += 1) {
      total += Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    }
    return total;
  }

  function gridSearch(planner, start, goal, field) {
    const begin = performance.now();
    const open = [{ point: start, g: 0, priority: planner === "astar" ? octile(start, goal) : 0 }];
    const gScore = new Map([[keyOf(start), 0]]);
    const parent = new Map();
    const closed = new Set();
    const steps = [];
    let generatedCount = 1;
    let finalPath = [];
    const directions = [
      [-1, 0], [1, 0], [0, -1], [0, 1],
      [-1, -1], [1, -1], [-1, 1], [1, 1],
    ];

    while (open.length && steps.length < COLS * ROWS * 3) {
      open.sort((a, b) => a.priority - b.priority || a.g - b.g);
      const current = open.shift();
      const currentKey = keyOf(current.point);
      if (closed.has(currentKey)) continue;
      closed.add(currentKey);

      if (currentKey === keyOf(goal)) {
        steps.push({ pick: current.point, generated: [], open: open.length, closed: closed.size });
        finalPath = reconstruct(parent, goal);
        break;
      }

      const generated = [];
      directions.forEach(([dx, dy]) => {
        const neighbor = { x: current.point.x + dx, y: current.point.y + dy };
        const neighborKey = keyOf(neighbor);
        if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= COLS || neighbor.y >= ROWS) return;
        if (field.blocked.has(neighborKey) || closed.has(neighborKey)) return;
        if (dx !== 0 && dy !== 0) {
          if (
            field.blocked.has(`${current.point.x + dx},${current.point.y}`) ||
            field.blocked.has(`${current.point.x},${current.point.y + dy}`)
          ) return;
        }
        const moveCost = (dx !== 0 && dy !== 0 ? Math.SQRT2 : 1) * field.costs[neighbor.y][neighbor.x];
        const tentative = current.g + moveCost;
        if (tentative >= (gScore.get(neighborKey) ?? Number.POSITIVE_INFINITY)) return;
        gScore.set(neighborKey, tentative);
        parent.set(neighborKey, currentKey);
        const priority = tentative + (planner === "astar" ? octile(neighbor, goal) : 0);
        open.push({ point: neighbor, g: tentative, priority });
        generated.push(neighbor);
        generatedCount += 1;
      });
      steps.push({ pick: current.point, generated, open: open.length, closed: closed.size });
    }

    return {
      key: planner,
      steps,
      path: finalPath,
      expanded: closed.size,
      generated: generatedCount,
      pathLength: pathLength(finalPath),
      durationMs: performance.now() - begin,
      found: finalPath.length > 0,
    };
  }

  function mulberry32(seed) {
    return function random() {
      seed |= 0;
      seed = (seed + 0x6d2b79f5) | 0;
      let value = Math.imul(seed ^ (seed >>> 15), 1 | seed);
      value = (value + Math.imul(value ^ (value >>> 7), 61 | value)) ^ value;
      return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
    };
  }

  function segmentIsFree(a, b, blocked) {
    const distance = Math.hypot(b.x - a.x, b.y - a.y);
    const samples = Math.max(2, Math.ceil(distance / 0.08));
    for (let i = 0; i <= samples; i += 1) {
      const t = i / samples;
      if (blocked.has(keyOf({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }))) return false;
    }
    return true;
  }

  function rrtSearch(start, goal, field) {
    const begin = performance.now();
    const random = mulberry32(Math.round(start.x * 997 + start.y * 131 + goal.x * 67 + goal.y * 17));
    const nodes = [{ ...start, parent: -1 }];
    const steps = [];
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

    const path = [];
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

  function rrtStarSearch(start, goal, field) {
    const begin = performance.now();
    const random = mulberry32(Math.round(start.x * 997 + start.y * 131 + goal.x * 67 + goal.y * 17 + 4049));
    const nodes = [{ ...start, parent: -1, cost: 0 }];
    const steps = [];
    const stepSize = 1.55;
    const neighborRadius = 3.6;
    let bestGoalParent = -1;
    let bestGoalCost = Number.POSITIVE_INFINITY;

    function isAncestor(candidate, index) {
      let current = index;
      let guard = 0;
      while (current >= 0 && guard <= nodes.length) {
        if (current === candidate) return true;
        current = nodes[current].parent;
        guard += 1;
      }
      return false;
    }

    function refreshDescendantCosts(parentIndex) {
      for (let index = 0; index < nodes.length; index += 1) {
        if (nodes[index].parent !== parentIndex) continue;
        nodes[index].cost = nodes[parentIndex].cost + Math.hypot(
          nodes[index].x - nodes[parentIndex].x,
          nodes[index].y - nodes[parentIndex].y,
        );
        refreshDescendantCosts(index);
      }
    }

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
      nearIndices.forEach((candidate) => {
        const node = nodes[candidate.index];
        const candidateCost = node.cost + candidate.distance;
        if (candidateCost >= parentCost || !segmentIsFree(node, next, field.blocked)) return;
        parentIndex = candidate.index;
        parentCost = candidateCost;
      });

      nodes.push({ ...next, parent: parentIndex, cost: parentCost });
      const nextIndex = nodes.length - 1;
      nearIndices.forEach((candidate) => {
        if (candidate.index === 0 || candidate.index === parentIndex) return;
        if (isAncestor(candidate.index, nextIndex)) return;
        const node = nodes[candidate.index];
        const rewiredCost = parentCost + candidate.distance;
        if (rewiredCost + 1e-6 >= node.cost || !segmentIsFree(next, node, field.blocked)) return;
        node.parent = nextIndex;
        node.cost = rewiredCost;
        refreshDescendantCosts(candidate.index);
      });

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
      if (distanceToGoal < 2.5 && goalCost < bestGoalCost && segmentIsFree(next, goal, field.blocked)) {
        bestGoalParent = nextIndex;
        bestGoalCost = goalCost;
      }
    }

    const path = [];
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

  function runPlanner(planner, start, goal, field) {
    if (planner === "rrt") return rrtSearch(start, goal, field);
    if (planner === "rrtstar") return rrtStarSearch(start, goal, field);
    return gridSearch(planner, start, goal, field);
  }

  function pointToCanvas(point) {
    return { x: point.x * CELL + CELL / 2, y: point.y * CELL + CELL / 2 };
  }

  function positionAlongPath(path, cursor) {
    if (!path.length) return null;
    const index = Math.min(Math.floor(cursor), path.length - 1);
    const nextIndex = Math.min(index + 1, path.length - 1);
    const fraction = cursor - index;
    return {
      x: path[index].x + (path[nextIndex].x - path[index].x) * fraction,
      y: path[index].y + (path[nextIndex].y - path[index].y) * fraction,
      heading: Math.atan2(path[nextIndex].y - path[index].y, path[nextIndex].x - path[index].x),
    };
  }

  function resultList() {
    return PLANNERS.map((planner) => state.results[planner]).filter(Boolean);
  }

  function maxSteps() {
    return Math.max(0, ...resultList().map((result) => result.steps.length));
  }

  function clearRun() {
    state.results = {};
    state.progress = 0;
    state.running = false;
    state.executing = false;
    state.executionCursor = 0;
  }

  function setNotice(message) {
    state.notice = message;
    elements.notice.textContent = message;
  }

  function updateInterface() {
    elements.layerControls.querySelectorAll("button").forEach((button) => {
      button.classList.toggle("active", button.dataset.layer === state.layer);
    });
    elements.layerName.textContent = state.layer.toUpperCase();
    elements.gridSizeStat.textContent = `${COLS} × ${ROWS}`;
    elements.inflation.value = String(state.inflation);
    elements.inflationValue.textContent = `${state.inflation.toFixed(2)} cell`;
    elements.speed.value = String(state.speed);
    elements.speedValue.textContent = `${state.speed}×`;
    elements.startValue.textContent = formatCell(state.start);
    elements.goalValue.textContent = formatCell(state.goal);
    elements.selectStart.classList.toggle("selected", state.placement === "start");
    elements.selectGoal.classList.toggle("selected", state.placement === "goal");
    elements.dynamicCount.textContent = `${state.dynamicSeeds.length} 个动态障碍`;
    elements.dynamicMode.querySelectorAll("button").forEach((button) => {
      button.classList.toggle("active", button.dataset.motion === state.dynamicMotion);
    });
    elements.addDynamic.classList.toggle("active", state.placingDynamic);
    elements.addDynamic.textContent = state.placingDynamic ? "结束放置" : "添加障碍";
    elements.notice.textContent = state.notice;
    PLANNERS.forEach((planner) => {
      document.getElementById(`enable-${planner}`).checked = state.enabled[planner];
    });
    const enabledPlanners = PLANNERS.filter((planner) => state.enabled[planner]);
    elements.plannerPickerCount.textContent = `${enabledPlanners.length} / ${PLANNERS.length}`;
    elements.emptyPlannerStatus.hidden = Boolean(enabledPlanners.length);
    updateTelemetry();
  }

  function updateTelemetry() {
    const stepsTotal = maxSteps();
    const percent = stepsTotal ? Math.min(100, state.progress / stepsTotal * 100) : 0;
    elements.progressBar.style.width = `${percent}%`;
    elements.timelineControl.classList.toggle("disabled", !resultList().length);
    elements.timelineSlider.disabled = !resultList().length;
    elements.timelineSlider.max = String(Math.max(stepsTotal, 1));
    elements.timelineSlider.value = String(Math.min(state.progress, Math.max(stepsTotal, 1)));
    elements.timelineValue.textContent = `${Math.round(percent)}%`;
    elements.runState.classList.toggle("is-running", state.running);
    if (state.running) elements.runState.textContent = `SEARCH ${Math.round(percent)}%`;
    else if (state.executing) elements.runState.textContent = "ROBOT EXECUTING";
    else if (state.placingDynamic) elements.runState.textContent = "ADD DYNAMIC OBSTACLE";
    else if (state.draggingStatic) elements.runState.textContent = "DRAG STATIC OBSTACLE";
    else if (state.draggingMarker) elements.runState.textContent = `DRAG ${state.draggingMarker.toUpperCase()}`;
    else elements.runState.textContent = "DRAG MARKERS";

    const hasResults = resultList().length > 0;
    const canResume = hasResults && state.progress < stepsTotal;
    elements.runPlanner.innerHTML = state.running
      ? "暂停推演 <span>Ⅱ</span>"
      : canResume
        ? "继续推演 <span>→</span>"
        : "启动路径规划 <span>→</span>";

    PLANNERS.forEach((planner) => {
      const card = document.querySelector(`[data-card="${planner}"]`);
      const result = state.results[planner];
      const visible = result ? Math.min(Math.floor(state.progress), result.steps.length) : 0;
      const done = Boolean(result && visible >= result.steps.length);
      const currentStep = result ? result.steps[visible - 1] : null;
      const visibleGenerated = result
        ? 1 + result.steps.slice(0, visible).reduce((total, step) => total + step.generated.length, 0)
        : 0;
      card.hidden = !state.enabled[planner];
      card.querySelector("strong").textContent = !state.enabled[planner]
        ? "关闭"
        : !result
          ? ""
          : done
            ? result.found ? "已完成" : "无路径"
            : "计算中";
      card.querySelector("[data-pick]").textContent = String(visible);
      card.querySelector("[data-generated]").textContent = String(visibleGenerated);
      card.querySelector("[data-open]").textContent = String(currentStep ? currentStep.open : 0);
      card.querySelector("[data-closed]").textContent = String(currentStep ? currentStep.closed : 0);
      card.querySelector("[data-time]").textContent = result ? `${result.durationMs.toFixed(1)}ms` : "—";
      card.querySelector("[data-path]").textContent = done && result.found
        ? `${(result.pathLength * 0.25).toFixed(1)}m`
        : "—";
      const statusText = card.querySelector("[data-status-text]");
      statusText.textContent = !result
        ? "等待启动路径规划"
        : done
          ? result.found ? `已找到路径，长度 ${(result.pathLength * 0.25).toFixed(1)}m` : "搜索结束，未找到路径"
          : currentStep ? `正在检查 ${formatCell(currentStep.pick)}，本步新增 ${currentStep.generated.length} 个候选点` : "正在初始化搜索空间";
    });
  }

  function startPlanning() {
    state.placingDynamic = false;
    canvas.classList.remove("is-placing-dynamic");
    elements.addDynamic.classList.remove("active");
    elements.addDynamic.textContent = "添加障碍";
    const stepsTotal = maxSteps();
    if (resultList().length && state.progress < stepsTotal) {
      state.running = !state.running;
      setNotice(state.running ? "继续推演搜索过程" : "推演已暂停");
      updateTelemetry();
      return;
    }
    const selected = PLANNERS.filter((planner) => state.enabled[planner]);
    if (!selected.length) {
      setNotice("请至少选择一种规划算法");
      return;
    }
    const field = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
    if (field.blocked.has(keyOf(state.start)) || field.blocked.has(keyOf(state.goal))) {
      setNotice("起点或终点位于膨胀障碍区，请重新放置");
      return;
    }
    state.results = {};
    selected.forEach((planner) => {
      state.results[planner] = runPlanner(planner, state.start, state.goal, field);
    });
    state.progress = 0;
    state.executionCursor = 0;
    state.executing = false;
    state.running = true;
    setNotice("推演中：亮环表示当前扩展节点，微光点表示本步新生成节点");
    updateTelemetry();
  }

  function pointerGridPosition(event, snap) {
    const rect = canvas.getBoundingClientRect();
    const raw = {
      x: (event.clientX - rect.left) / rect.width * COLS - 0.5,
      y: (event.clientY - rect.top) / rect.height * ROWS - 0.5,
    };
    if (!snap) return raw;
    return {
      x: Math.max(1, Math.min(COLS - 2, Math.round(raw.x))),
      y: Math.max(1, Math.min(ROWS - 2, Math.round(raw.y))),
    };
  }

  function pointInStaticRect(point, rect) {
    return point.x >= rect.x && point.x < rect.x + rect.w && point.y >= rect.y && point.y < rect.y + rect.h;
  }

  function staticRectsOverlap(a, b) {
    return a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
  }

  function staticRectIsAllowed(candidate, index) {
    if (pointInStaticRect(state.start, candidate) || pointInStaticRect(state.goal, candidate)) return false;
    return !STATIC_RECTS.some((rect, rectIndex) => rectIndex !== index && staticRectsOverlap(candidate, rect));
  }

  function closeStaticMenu() {
    state.staticMenu = null;
    elements.staticMenu.hidden = true;
  }

  function openStaticMenu(event) {
    event.preventDefault();
    const point = pointerGridPosition(event, true);
    const index = STATIC_RECTS.findIndex((rect) => pointInStaticRect(point, rect));
    const rect = index >= 0 ? STATIC_RECTS[index] : { x: point.x, y: point.y, w: 3, h: 3 };
    const bounds = mapPanel.getBoundingClientRect();
    const left = Math.max(8, Math.min(bounds.width - 218, event.clientX - bounds.left));
    const top = Math.max(40, Math.min(bounds.height - 164, event.clientY - bounds.top));
    state.placingDynamic = false;
    state.staticMenu = { index: index >= 0 ? index : null, point: { x: rect.x, y: rect.y } };
    elements.staticMenu.style.left = `${left}px`;
    elements.staticMenu.style.top = `${top}px`;
    elements.staticMenuTitle.textContent = index >= 0 ? "编辑静态障碍" : "添加静态障碍";
    elements.staticMenuPosition.textContent = `位置 ${formatCell(rect)}`;
    elements.staticWidth.value = String(rect.w);
    elements.staticHeight.value = String(rect.h);
    elements.staticWidth.max = String(Math.max(1, COLS - 1 - rect.x));
    elements.staticHeight.max = String(Math.max(1, ROWS - 1 - rect.y));
    elements.saveStatic.textContent = index >= 0 ? "保存" : "添加";
    elements.deleteStatic.hidden = index < 0;
    elements.staticMenu.hidden = false;
    canvas.classList.remove("is-placing-dynamic");
    setNotice(index >= 0 ? "可调整该静态障碍的尺寸、删除或拖动位置" : "设置尺寸后可在此处添加静态障碍");
    updateInterface();
  }

  function saveStaticObstacle() {
    if (!state.staticMenu) return;
    const { index, point } = state.staticMenu;
    const x = Math.max(1, Math.min(COLS - 2, point.x));
    const y = Math.max(1, Math.min(ROWS - 2, point.y));
    const candidate = {
      x,
      y,
      w: Math.max(1, Math.min(COLS - 1 - x, Math.round(Number(elements.staticWidth.value)) || 1)),
      h: Math.max(1, Math.min(ROWS - 1 - y, Math.round(Number(elements.staticHeight.value)) || 1)),
    };
    if (!staticRectIsAllowed(candidate, index)) {
      setNotice("静态障碍不能覆盖起终点或与其他静态障碍重叠");
      return;
    }
    clearRun();
    if (index === null) {
      STATIC_RECTS.push(candidate);
      setNotice(`已添加 ${candidate.w} × ${candidate.h} 的静态障碍`);
    } else {
      STATIC_RECTS[index] = candidate;
      setNotice("静态障碍尺寸已更新");
    }
    cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
    closeStaticMenu();
    updateInterface();
  }

  function deleteStaticObstacle() {
    if (!state.staticMenu || state.staticMenu.index === null) return;
    clearRun();
    STATIC_RECTS.splice(state.staticMenu.index, 1);
    cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
    closeStaticMenu();
    setNotice("静态障碍已清除");
    updateInterface();
  }

  function beginMarkerDrag(event) {
    if (event.button !== 0) return;
    closeStaticMenu();
    const raw = pointerGridPosition(event, false);
    const startDistance = Math.hypot(raw.x - state.start.x, raw.y - state.start.y);
    const goalDistance = Math.hypot(raw.x - state.goal.x, raw.y - state.goal.y);
    const marker = Math.min(startDistance, goalDistance) <= 1.15
      ? startDistance <= goalDistance ? "start" : "goal"
      : null;
    if (!marker) {
      const point = pointerGridPosition(event, true);
      const staticIndex = STATIC_RECTS.findIndex((rect) => pointInStaticRect(point, rect));
      if (staticIndex >= 0) {
        const rect = STATIC_RECTS[staticIndex];
        event.preventDefault();
        canvas.setPointerCapture(event.pointerId);
        clearRun();
        state.placingDynamic = false;
        state.draggingStatic = { index: staticIndex, offsetX: point.x - rect.x, offsetY: point.y - rect.y };
        canvas.classList.remove("is-placing-dynamic");
        canvas.classList.add("is-dragging");
        setNotice("正在拖动静态障碍，松开后完成移动");
        updateInterface();
        return;
      }
      if (!state.placingDynamic) return;
      const staticField = makeCostField([], state.inflation);
      if (staticField.blocked.has(keyOf(point))) {
        setNotice("这里是静态障碍或膨胀区，不能放置动态障碍");
        return;
      }
      if (Math.hypot(point.x - state.start.x, point.y - state.start.y) < 2 || Math.hypot(point.x - state.goal.x, point.y - state.goal.y) < 2) {
        setNotice("动态障碍需要与起终点保持至少 2 个栅格");
        return;
      }
      if (state.dynamicSeeds.some((seed) => Math.hypot(seed.x - point.x, seed.y - point.y) < 1.5)) {
        setNotice("此处附近已经有一个动态障碍");
        return;
      }
      if (state.dynamicSeeds.length >= 8) {
        setNotice("最多可放置 8 个动态障碍");
        return;
      }
      event.preventDefault();
      clearRun();
      state.dynamicSeeds.push({ ...point, phase: state.dynamicSeeds.length * 0.83 + 0.4 });
      cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
      setNotice(`已添加动态障碍 D${state.dynamicSeeds.length}，可继续点击地图添加`);
      updateInterface();
      return;
    }
    event.preventDefault();
    canvas.setPointerCapture(event.pointerId);
    clearRun();
    state.placingDynamic = false;
    state.draggingMarker = marker;
    state.draggingStatic = null;
    state.placement = marker;
    canvas.classList.remove("is-placing-dynamic");
    canvas.classList.add("is-dragging");
    setNotice(marker === "start" ? "正在拖动起点图标" : "正在拖动终点图标");
    updateInterface();
  }

  function moveMarker(event) {
    const point = pointerGridPosition(event, true);
    if (state.draggingStatic) {
      const current = STATIC_RECTS[state.draggingStatic.index];
      if (!current) return;
      const candidate = {
        ...current,
        x: Math.max(1, Math.min(COLS - 1 - current.w, point.x - state.draggingStatic.offsetX)),
        y: Math.max(1, Math.min(ROWS - 1 - current.h, point.y - state.draggingStatic.offsetY)),
      };
      if (!staticRectIsAllowed(candidate, state.draggingStatic.index)) {
        setNotice("静态障碍不能覆盖起终点或其他静态障碍");
        return;
      }
      STATIC_RECTS[state.draggingStatic.index] = candidate;
      cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
      setNotice("松开以固定静态障碍位置");
      return;
    }
    if (!state.draggingMarker) return;
    const field = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
    if (field.blocked.has(keyOf(point))) {
      setNotice("此处是障碍或膨胀区，图标不能放置");
      return;
    }
    const other = state.draggingMarker === "start" ? state.goal : state.start;
    if (point.x === other.x && point.y === other.y) {
      setNotice("起点与终点不能重合");
      return;
    }
    if (state.draggingMarker === "start") {
      state.start = point;
    } else {
      state.goal = point;
    }
    setNotice(state.draggingMarker === "start" ? "松开以设置起点" : "松开以设置终点");
    updateInterface();
  }

  function endMarkerDrag(event) {
    if (!state.draggingMarker && !state.draggingStatic) return;
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    if (state.draggingStatic) {
      state.draggingStatic = null;
      canvas.classList.remove("is-dragging");
      setNotice("静态障碍已更新，可以启动路径规划");
      updateInterface();
      return;
    }
    const label = state.draggingMarker === "start" ? "起点" : "终点";
    state.draggingMarker = null;
    canvas.classList.remove("is-dragging");
    setNotice(`${label}已更新，可以启动路径规划`);
    updateInterface();
  }

  function drawMarker(point, color, kind, now) {
    const pixel = pointToCanvas(point);
    const active = state.draggingMarker === kind;
    const markerSize = Math.max(5, Math.min(14, CELL * 0.58));
    const markerScale = markerSize / 14;
    ctx.save();
    ctx.translate(pixel.x, pixel.y);
    if (active) {
      ctx.beginPath();
      ctx.strokeStyle = `${color}88`;
      ctx.lineWidth = 2;
      ctx.arc(0, 0, markerSize + 6 + Math.sin(now / 90) * 2, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.arc(0, 0, markerSize, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.strokeStyle = color;
    if (kind === "start") {
      ctx.beginPath();
      ctx.moveTo(-4 * markerScale, -7 * markerScale);
      ctx.lineTo(8 * markerScale, 0);
      ctx.lineTo(-4 * markerScale, 7 * markerScale);
      ctx.closePath();
      ctx.fill();
    } else {
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(-5 * markerScale, 8 * markerScale);
      ctx.lineTo(-5 * markerScale, -8 * markerScale);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(-4 * markerScale, -7 * markerScale);
      ctx.lineTo(8 * markerScale, -4 * markerScale);
      ctx.lineTo(-4 * markerScale, 1 * markerScale);
      ctx.closePath();
      ctx.fill();
    }
    ctx.restore();
  }

  let cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
  let lastFieldUpdate = 0;
  function draw(now) {
    if (now - lastFieldUpdate > 160) {
      cachedField = makeCostField(dynamicObstaclesAt(now), state.inflation);
      lastFieldUpdate = now;
    }
    const field = cachedField;
    const dynamicObstacles = dynamicObstaclesAt(now);
    ctx.clearRect(0, 0, WIDTH, HEIGHT);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 0, WIDTH, HEIGHT);

    if (state.layer === "cost") {
      for (let y = 0; y < ROWS; y += 1) {
        for (let x = 0; x < COLS; x += 1) {
          if (field.raw.has(`${x},${y}`)) continue;
          const cost = field.costs[y][x];
          if (cost <= 1.05) continue;
          const strength = Math.min(1, (cost - 1) / 8);
          ctx.fillStyle = `rgba(239, 68, 68, ${0.05 + strength * 0.5})`;
          ctx.fillRect(x * CELL, y * CELL, CELL, CELL);
        }
      }
    }
    if (state.layer === "inflation") {
      ctx.fillStyle = "rgba(239, 68, 68, .3)";
      field.blocked.forEach((key) => {
        if (field.raw.has(key)) return;
        const [x, y] = key.split(",").map(Number);
        ctx.fillRect(x * CELL, y * CELL, CELL, CELL);
      });
    }

    // Align one-pixel solid grid strokes to the device pixel grid so cells stay sharp.
    ctx.setLineDash([]);
    ctx.strokeStyle = "rgba(100, 116, 139, .52)";
    ctx.lineWidth = 1;
    for (let x = 0; x <= WIDTH; x += CELL) {
      const position = x === WIDTH ? x - 0.5 : x + 0.5;
      ctx.beginPath();
      ctx.moveTo(position, 0);
      ctx.lineTo(position, HEIGHT);
      ctx.stroke();
    }
    for (let y = 0; y <= HEIGHT; y += CELL) {
      const position = y === HEIGHT ? y - 0.5 : y + 0.5;
      ctx.beginPath();
      ctx.moveTo(0, position);
      ctx.lineTo(WIDTH, position);
      ctx.stroke();
    }

    ctx.fillStyle = "#374151";
    ctx.strokeStyle = "#111827";
    ctx.lineWidth = 1.5;
    STATIC_RECTS.forEach((rect) => {
      ctx.fillRect(rect.x * CELL, rect.y * CELL, rect.w * CELL, rect.h * CELL);
      ctx.strokeRect(rect.x * CELL + 1, rect.y * CELL + 1, Math.max(0, rect.w * CELL - 2), Math.max(0, rect.h * CELL - 2));
    });
    dynamicObstacles.forEach((point, index) => {
      const pixel = pointToCanvas(point);
      if (state.layer !== "raw") {
        ctx.beginPath();
        ctx.fillStyle = "rgba(239, 68, 68, .12)";
        ctx.arc(pixel.x, pixel.y, (state.inflation + 1) * CELL, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "rgba(220, 38, 38, .5)";
        ctx.stroke();
      }
      ctx.beginPath();
      ctx.fillStyle = "#1f2937";
      ctx.arc(pixel.x, pixel.y, Math.max(4, CELL * 0.54), 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#ffffff";
      ctx.font = `700 ${Math.max(6, Math.min(9, CELL * 0.38))}px ui-monospace, monospace`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(`D${index + 1}`, pixel.x, pixel.y + 1);
    });
    ctx.strokeStyle = "#334155";
    ctx.lineWidth = Math.max(2, Math.min(8, CELL / 3));
    ctx.strokeRect(2, 2, WIDTH - 4, HEIGHT - 4);

    resultList().forEach((result) => {
      const visible = Math.min(Math.floor(state.progress), result.steps.length);
      const color = COLORS[result.key];
      const steps = result.steps.slice(0, visible);
      if (result.key === "rrt" || result.key === "rrtstar") {
        ctx.save();
        ctx.setLineDash([5, 5]);
        ctx.lineWidth = 1.2;
        ctx.strokeStyle = `${color}c9`;
        ctx.lineCap = "butt";
        steps.forEach((step) => {
          if (!step.edge) return;
          const a = pointToCanvas(step.edge[0]);
          const b = pointToCanvas(step.edge[1]);
          ctx.beginPath();
          ctx.moveTo(a.x, a.y);
          ctx.lineTo(b.x, b.y);
          ctx.stroke();
        });
        ctx.restore();
      } else {
        steps.forEach((step) => {
          const pick = pointToCanvas(step.pick);
          ctx.fillStyle = `${color}20`;
          ctx.fillRect(pick.x - CELL / 2 + 1, pick.y - CELL / 2 + 1, Math.max(1, CELL - 2), Math.max(1, CELL - 2));
          step.generated.forEach((point) => {
            const generated = pointToCanvas(point);
            ctx.beginPath();
            ctx.fillStyle = `${color}aa`;
            ctx.arc(generated.x, generated.y, 2.2, 0, Math.PI * 2);
            ctx.fill();
          });
        });
      }
      const current = steps[steps.length - 1];
      if (current && visible < result.steps.length) {
        const pick = pointToCanvas(current.pick);
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5;
        ctx.arc(pick.x, pick.y, 8 + Math.sin(now / 120) * 2, 0, Math.PI * 2);
        ctx.stroke();
      }
      if (visible >= result.steps.length && result.path.length) {
        ctx.save();
        ctx.setLineDash([]);
        ctx.beginPath();
        result.path.forEach((point, index) => {
          const pixel = pointToCanvas(point);
          if (index === 0) ctx.moveTo(pixel.x, pixel.y);
          else ctx.lineTo(pixel.x, pixel.y);
        });
        ctx.strokeStyle = color;
        ctx.lineWidth = Math.max(3.5, Math.min(5.5, CELL * 0.22));
        ctx.globalAlpha = 1;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.stroke();
        ctx.restore();
      }
    });

    const executionResult = state.results.astar || state.results.dijkstra || state.results.rrt || state.results.rrtstar;
    const robot = positionAlongPath(executionResult ? executionResult.path : [state.start], state.executionCursor) || { ...state.start, heading: 0 };
    const robotPixel = pointToCanvas(robot);
    ctx.save();
    ctx.translate(robotPixel.x, robotPixel.y);
    ctx.rotate(robot.heading);
    ctx.fillStyle = "#f4fbfb";
    ctx.strokeStyle = "#071016";
    ctx.lineWidth = 2;
    ctx.fillRect(-11, -8, 22, 16);
    ctx.strokeRect(-11, -8, 22, 16);
    ctx.fillStyle = "#00bfff";
    ctx.beginPath();
    ctx.moveTo(13, 0);
    ctx.lineTo(5, -5);
    ctx.lineTo(5, 5);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = "#071016";
    ctx.fillRect(-7, -11, 5, 4);
    ctx.fillRect(-7, 7, 5, 4);
    ctx.fillRect(3, -11, 5, 4);
    ctx.fillRect(3, 7, 5, 4);
    ctx.restore();

    drawMarker(state.start, "#2ed47a", "start", now);
    drawMarker(state.goal, "#ff9f43", "goal", now);
  }

  elements.layerControls.addEventListener("click", (event) => {
    const button = event.target.closest("button[data-layer]");
    if (!button) return;
    state.layer = button.dataset.layer;
    updateInterface();
  });
  function applyGridSize() {
    const next = configureGrid(Number(elements.gridCols.value) || COLS, Number(elements.gridRows.value) || ROWS);
    const points = defaultPointsForGrid(next.cols, next.rows);
    clearRun();
    canvas.width = WIDTH;
    canvas.height = HEIGHT;
    state.start = points.start;
    state.goal = points.goal;
    state.placement = "start";
    state.draggingMarker = null;
    state.draggingStatic = null;
    state.placingDynamic = false;
    state.dynamicSeeds = defaultDynamicObstacles();
    elements.gridCols.value = String(next.cols);
    elements.gridRows.value = String(next.rows);
    closeStaticMenu();
    canvas.classList.remove("is-dragging", "is-placing-dynamic");
    cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
    setNotice(`地图已调整为 ${next.cols} × ${next.rows}，障碍物与任务点已重新布局`);
    updateInterface();
  }
  elements.applyGrid.addEventListener("click", applyGridSize);
  [elements.gridCols, elements.gridRows].forEach((input) => {
    input.addEventListener("keydown", (event) => {
      if (event.key === "Enter") applyGridSize();
    });
  });
  elements.inflation.addEventListener("input", (event) => {
    clearRun();
    state.inflation = Number(event.target.value);
    cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
    updateInterface();
  });
  elements.dynamicMode.addEventListener("click", (event) => {
    const button = event.target.closest("button[data-motion]");
    if (!button) return;
    clearRun();
    state.dynamicMotion = button.dataset.motion;
    setNotice(state.dynamicMotion === "regular" ? "动态障碍已切换为规则往复运动" : "动态障碍已切换为随机平滑运动");
    updateInterface();
  });
  elements.addDynamic.addEventListener("click", () => {
    state.placingDynamic = !state.placingDynamic;
    canvas.classList.toggle("is-placing-dynamic", state.placingDynamic);
    setNotice(state.placingDynamic ? "请在地图空白区域点击放置动态障碍" : "已结束放置动态障碍");
    updateInterface();
  });
  elements.clearDynamic.addEventListener("click", () => {
    clearRun();
    state.dynamicSeeds = [];
    state.placingDynamic = false;
    canvas.classList.remove("is-placing-dynamic");
    cachedField = makeCostField([], state.inflation);
    setNotice("动态障碍已清空");
    updateInterface();
  });
  elements.resetDynamic.addEventListener("click", () => {
    clearRun();
    state.dynamicSeeds = defaultDynamicObstacles();
    state.placingDynamic = false;
    canvas.classList.remove("is-placing-dynamic");
    cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
    setNotice("已恢复默认动态障碍");
    updateInterface();
  });
  elements.selectStart.addEventListener("click", () => {
    state.placingDynamic = false;
    canvas.classList.remove("is-placing-dynamic");
    state.placement = "start";
    setNotice("请拖动地图上的绿色起点图标");
    updateInterface();
  });
  elements.selectGoal.addEventListener("click", () => {
    state.placingDynamic = false;
    canvas.classList.remove("is-placing-dynamic");
    state.placement = "goal";
    setNotice("请拖动地图上的橙色终点图标");
    updateInterface();
  });
  elements.resetPoints.addEventListener("click", () => {
    const points = defaultPointsForGrid(COLS, ROWS);
    clearRun();
    state.start = points.start;
    state.goal = points.goal;
    state.placement = "start";
    state.draggingMarker = null;
    state.draggingStatic = null;
    state.placingDynamic = false;
    closeStaticMenu();
    canvas.classList.remove("is-dragging", "is-placing-dynamic");
    setNotice("起终点已重置，可直接拖动图标调整位置");
    updateInterface();
  });
  PLANNERS.forEach((planner) => {
    document.getElementById(`enable-${planner}`).addEventListener("change", (event) => {
      clearRun();
      state.enabled[planner] = event.target.checked;
      updateInterface();
    });
  });
  elements.speed.addEventListener("input", (event) => {
    state.speed = Number(event.target.value);
    updateInterface();
  });
  elements.timelineSlider.addEventListener("input", (event) => {
    state.running = false;
    state.executing = false;
    state.executionCursor = 0;
    state.progress = Number(event.target.value);
    const percent = Math.round(state.progress / Math.max(maxSteps(), 1) * 100);
    setNotice(`已定位到推演过程 ${percent}%`);
    updateTelemetry();
  });
  elements.runPlanner.addEventListener("click", startPlanning);
  elements.closeStaticMenu.addEventListener("click", closeStaticMenu);
  elements.saveStatic.addEventListener("click", saveStaticObstacle);
  elements.deleteStatic.addEventListener("click", deleteStaticObstacle);
  canvas.addEventListener("pointerdown", beginMarkerDrag);
  canvas.addEventListener("pointermove", moveMarker);
  canvas.addEventListener("pointerup", endMarkerDrag);
  canvas.addEventListener("pointercancel", endMarkerDrag);
  canvas.addEventListener("contextmenu", openStaticMenu);

  let previousFrame = performance.now();
  let progressAccumulator = 0;
  function animate(now) {
    const elapsed = Math.min(100, now - previousFrame);
    previousFrame = now;
    if (state.running) {
      progressAccumulator += elapsed;
      if (progressAccumulator >= 42) {
        const increments = Math.floor(progressAccumulator / 42);
        progressAccumulator %= 42;
        state.progress = Math.min(maxSteps(), state.progress + increments * state.speed);
        if (state.progress >= maxSteps()) {
          state.running = false;
          const executionResult = state.results.astar || state.results.dijkstra || state.results.rrt || state.results.rrtstar;
          if (executionResult && executionResult.path.length) {
            state.executing = true;
            setNotice("全局路径已生成，机器人开始沿优先路径执行");
          } else {
            setNotice("搜索结束：当前地图未找到可执行路径");
          }
        }
        updateTelemetry();
      }
    }
    if (state.executing) {
      const executionResult = state.results.astar || state.results.dijkstra || state.results.rrt || state.results.rrtstar;
      if (!executionResult || !executionResult.path.length) {
        state.executing = false;
      } else {
        const end = executionResult.path.length - 1;
        state.executionCursor = Math.min(end, state.executionCursor + elapsed / 46 * (0.12 + state.speed * 0.018));
        if (state.executionCursor >= end) {
          state.executing = false;
          setNotice("任务完成：机器人已抵达目标点");
          updateTelemetry();
        }
      }
    }
    draw(now);
    window.requestAnimationFrame(animate);
  }

  updateInterface();
  window.requestAnimationFrame(animate);
})();
