(function () {
  "use strict";

  const COLS = 35;
  const ROWS = 24;
  const CELL = 24;
  const WIDTH = COLS * CELL;
  const HEIGHT = ROWS * CELL;
  const PLANNERS = ["astar", "dijkstra", "rrt", "rrtstar"];
  const STATIC_RECTS = [
    { x: 6, y: 3, w: 4, h: 7 },
    { x: 15, y: 0, w: 3, h: 8 },
    { x: 22, y: 10, w: 9, h: 3 },
    { x: 12, y: 17, w: 3, h: 6 },
    { x: 25, y: 17, w: 3, h: 4 },
    { x: 4, y: 14, w: 5, h: 2 },
  ];
  const COLORS = {
    astar: "#00bfff",
    dijkstra: "#ffd400",
    rrt: "#ff4b4b",
    rrtstar: "#a970ff",
  };
  const NAMES = { astar: "A*", dijkstra: "DIJKSTRA", rrt: "RRT", rrtstar: "RRT*" };
  const DEFAULT_START = { x: 3, y: 20 };
  const DEFAULT_GOAL = { x: 32, y: 3 };

  const canvas = document.getElementById("map");
  const ctx = canvas.getContext("2d");
  const elements = {
    layerControls: document.getElementById("layer-controls"),
    layerName: document.getElementById("layer-name"),
    inflation: document.getElementById("inflation"),
    inflationValue: document.getElementById("inflation-value"),
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
    events: document.getElementById("events"),
  };

  const state = {
    layer: "cost",
    placement: "start",
    start: { ...DEFAULT_START },
    goal: { ...DEFAULT_GOAL },
    inflation: 1.45,
    speed: 4,
    enabled: { astar: true, dijkstra: true, rrt: true, rrtstar: true },
    results: {},
    progress: 0,
    running: false,
    executing: false,
    executionCursor: 0,
    draggingMarker: null,
    notice: "拖动地图上的起点或终点图标设置位置",
  };

  function keyOf(point) {
    return `${Math.round(point.x)},${Math.round(point.y)}`;
  }

  function formatCell(point) {
    return `${Math.round(point.x).toString().padStart(2, "0")},${Math.round(point.y).toString().padStart(2, "0")}`;
  }

  function dynamicObstaclesAt(timeMs) {
    const t = timeMs / 1000;
    return [
      { x: 19 + Math.sin(t * 0.72) * 3.4, y: 14.7 },
      { x: 29.4, y: 5.7 + Math.sin(t * 0.58 + 1.4) * 3.1 },
      { x: 8.5 + Math.sin(t * 0.49 + 2.2) * 2.3, y: 19.2 },
    ];
  }

  function addRectCells(target, x, y, width, height) {
    for (let row = y; row < y + height; row += 1) {
      for (let col = x; col < x + width; col += 1) target.add(`${col},${row}`);
    }
  }

  function makeCostField(dynamicObstacles, inflationRadius) {
    const raw = new Set();
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
    const blocked = new Set();
    const costs = Array.from({ length: ROWS }, () => Array.from({ length: COLS }, () => 1));
    for (let y = 0; y < ROWS; y += 1) {
      for (let x = 0; x < COLS; x += 1) {
        let minDistance = Number.POSITIVE_INFINITY;
        obstaclePoints.forEach((obstacle) => {
          minDistance = Math.min(minDistance, Math.hypot(x - obstacle.x, y - obstacle.y));
        });
        if (minDistance <= inflationRadius) blocked.add(`${x},${y}`);
        const safetyBand = Math.max(0, inflationRadius + 3.2 - minDistance);
        costs[y][x] = raw.has(`${x},${y}`) ? 99 : 1 + safetyBand * safetyBand * 0.65;
      }
    }
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
    elements.inflation.value = String(state.inflation);
    elements.inflationValue.textContent = `${state.inflation.toFixed(2)} cell`;
    elements.speed.value = String(state.speed);
    elements.speedValue.textContent = `${state.speed}×`;
    elements.startValue.textContent = formatCell(state.start);
    elements.goalValue.textContent = formatCell(state.goal);
    elements.selectStart.classList.toggle("selected", state.placement === "start");
    elements.selectGoal.classList.toggle("selected", state.placement === "goal");
    elements.notice.textContent = state.notice;
    PLANNERS.forEach((planner) => {
      document.getElementById(`enable-${planner}`).checked = state.enabled[planner];
    });
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
    });

    if (!hasResults) {
      elements.events.innerHTML = "<span><b>地图</b> 障碍与代价数据已载入</span><span><b>操作</b> 设置起终点后启动路径规划</span><span><b>回放</b> 生成结果后可拖动时间轴</span>";
      return;
    }
    elements.events.innerHTML = resultList().map((result) => {
      const index = Math.min(Math.floor(state.progress), result.steps.length) - 1;
      const step = result.steps[index];
      const done = Math.floor(state.progress) >= result.steps.length;
      const message = done
        ? result.found ? `已找到路径，长度 ${(result.pathLength * 0.25).toFixed(1)}m` : "搜索结束，未找到路径"
        : step ? `正在检查 ${formatCell(step.pick)}，新增 ${step.generated.length} 个候选点` : "等待开始推演";
      return `<span><b style="color:${COLORS[result.key]}">${NAMES[result.key]}</b> ${message}</span>`;
    }).join("") + (state.executing ? "<span><b>机器人</b> 正在沿优先路径移动</span>" : "");
  }

  function startPlanning() {
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

  function beginMarkerDrag(event) {
    const raw = pointerGridPosition(event, false);
    const startDistance = Math.hypot(raw.x - state.start.x, raw.y - state.start.y);
    const goalDistance = Math.hypot(raw.x - state.goal.x, raw.y - state.goal.y);
    const marker = Math.min(startDistance, goalDistance) <= 1.15
      ? startDistance <= goalDistance ? "start" : "goal"
      : null;
    if (!marker) return;
    event.preventDefault();
    canvas.setPointerCapture(event.pointerId);
    clearRun();
    state.draggingMarker = marker;
    state.placement = marker;
    canvas.classList.add("is-dragging");
    setNotice(marker === "start" ? "正在拖动起点图标" : "正在拖动终点图标");
    updateInterface();
  }

  function moveMarker(event) {
    if (!state.draggingMarker) return;
    const point = pointerGridPosition(event, true);
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
    if (!state.draggingMarker) return;
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    const label = state.draggingMarker === "start" ? "起点" : "终点";
    state.draggingMarker = null;
    canvas.classList.remove("is-dragging");
    setNotice(`${label}已更新，可以启动路径规划`);
    updateInterface();
  }

  function drawMarker(point, color, kind, now) {
    const pixel = pointToCanvas(point);
    const active = state.draggingMarker === kind;
    ctx.save();
    ctx.translate(pixel.x, pixel.y);
    if (active) {
      ctx.beginPath();
      ctx.strokeStyle = `${color}88`;
      ctx.lineWidth = 2;
      ctx.arc(0, 0, 20 + Math.sin(now / 90) * 2, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.fillStyle = "#071116";
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.arc(0, 0, 14, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.strokeStyle = color;
    if (kind === "start") {
      ctx.beginPath();
      ctx.moveTo(-4, -7);
      ctx.lineTo(8, 0);
      ctx.lineTo(-4, 7);
      ctx.closePath();
      ctx.fill();
    } else {
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(-5, 8);
      ctx.lineTo(-5, -8);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(-4, -7);
      ctx.lineTo(8, -4);
      ctx.lineTo(-4, 1);
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
    ctx.fillStyle = "#091219";
    ctx.fillRect(0, 0, WIDTH, HEIGHT);

    if (state.layer === "cost") {
      for (let y = 0; y < ROWS; y += 1) {
        for (let x = 0; x < COLS; x += 1) {
          if (field.raw.has(`${x},${y}`)) continue;
          const cost = field.costs[y][x];
          if (cost <= 1.05) continue;
          const strength = Math.min(0.5, (cost - 1) / 12);
          ctx.fillStyle = `rgba(255, ${Math.round(154 - strength * 90)}, 73, ${strength})`;
          ctx.fillRect(x * CELL, y * CELL, CELL, CELL);
        }
      }
    }
    if (state.layer === "inflation") {
      ctx.fillStyle = "rgba(255, 135, 80, .24)";
      field.blocked.forEach((key) => {
        if (field.raw.has(key)) return;
        const [x, y] = key.split(",").map(Number);
        ctx.fillRect(x * CELL, y * CELL, CELL, CELL);
      });
    }

    ctx.strokeStyle = "rgba(137, 174, 187, .095)";
    ctx.lineWidth = 1;
    for (let x = 0; x <= WIDTH; x += CELL) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, HEIGHT);
      ctx.stroke();
    }
    for (let y = 0; y <= HEIGHT; y += CELL) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(WIDTH, y);
      ctx.stroke();
    }

    ctx.fillStyle = "#33434c";
    ctx.strokeStyle = "#71828c";
    ctx.lineWidth = 1.5;
    STATIC_RECTS.forEach((rect) => {
      ctx.fillRect(rect.x * CELL, rect.y * CELL, rect.w * CELL, rect.h * CELL);
      ctx.strokeRect(rect.x * CELL + 1, rect.y * CELL + 1, rect.w * CELL - 2, rect.h * CELL - 2);
    });
    dynamicObstacles.forEach((point, index) => {
      const pixel = pointToCanvas(point);
      if (state.layer !== "raw") {
        ctx.beginPath();
        ctx.fillStyle = "rgba(255, 92, 92, .11)";
        ctx.arc(pixel.x, pixel.y, (state.inflation + 1) * CELL, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "rgba(255, 92, 92, .32)";
        ctx.stroke();
      }
      ctx.beginPath();
      ctx.fillStyle = "#f4f7f8";
      ctx.arc(pixel.x, pixel.y, 13, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#172026";
      ctx.font = "700 9px ui-monospace, monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(`D${index + 1}`, pixel.x, pixel.y + 1);
    });
    ctx.strokeStyle = "#748994";
    ctx.lineWidth = 8;
    ctx.strokeRect(4, 4, WIDTH - 8, HEIGHT - 8);

    resultList().forEach((result) => {
      const visible = Math.min(Math.floor(state.progress), result.steps.length);
      const color = COLORS[result.key];
      const steps = result.steps.slice(0, visible);
      if (result.key === "rrt" || result.key === "rrtstar") {
        ctx.lineWidth = 1.15;
        ctx.strokeStyle = `${color}66`;
        steps.forEach((step) => {
          if (!step.edge) return;
          const a = pointToCanvas(step.edge[0]);
          const b = pointToCanvas(step.edge[1]);
          ctx.beginPath();
          ctx.moveTo(a.x, a.y);
          ctx.lineTo(b.x, b.y);
          ctx.stroke();
        });
      } else {
        steps.forEach((step) => {
          const pick = pointToCanvas(step.pick);
          ctx.fillStyle = `${color}12`;
          ctx.fillRect(pick.x - CELL / 2 + 2, pick.y - CELL / 2 + 2, CELL - 4, CELL - 4);
          step.generated.forEach((point) => {
            const generated = pointToCanvas(point);
            ctx.beginPath();
            ctx.fillStyle = `${color}55`;
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
        ctx.beginPath();
        result.path.forEach((point, index) => {
          const pixel = pointToCanvas(point);
          if (index === 0) ctx.moveTo(pixel.x, pixel.y);
          else ctx.lineTo(pixel.x, pixel.y);
        });
        ctx.strokeStyle = color;
        ctx.lineWidth = result.key === "astar" ? 4 : 2.5;
        ctx.globalAlpha = result.key === "astar" ? 1 : 0.86;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.stroke();
        ctx.globalAlpha = 1;
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
  elements.inflation.addEventListener("input", (event) => {
    clearRun();
    state.inflation = Number(event.target.value);
    cachedField = makeCostField(dynamicObstaclesAt(performance.now()), state.inflation);
    updateInterface();
  });
  elements.selectStart.addEventListener("click", () => {
    state.placement = "start";
    setNotice("请拖动地图上的绿色起点图标");
    updateInterface();
  });
  elements.selectGoal.addEventListener("click", () => {
    state.placement = "goal";
    setNotice("请拖动地图上的橙色终点图标");
    updateInterface();
  });
  elements.resetPoints.addEventListener("click", () => {
    clearRun();
    state.start = { ...DEFAULT_START };
    state.goal = { ...DEFAULT_GOAL };
    state.placement = "start";
    state.draggingMarker = null;
    canvas.classList.remove("is-dragging");
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
  canvas.addEventListener("pointerdown", beginMarkerDrag);
  canvas.addEventListener("pointermove", moveMarker);
  canvas.addEventListener("pointerup", endMarkerDrag);
  canvas.addEventListener("pointercancel", endMarkerDrag);

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
