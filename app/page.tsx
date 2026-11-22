"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import {
  CANVAS_HEIGHT,
  CANVAS_WIDTH,
  CELL,
  COLS,
  GRID_LIMITS,
  ROWS,
  STATIC_RECTS,
  configureGrid,
  defaultDynamicObstacles,
  dynamicObstaclesAt,
  makeCostField,
  planDwaStep,
  pointToCanvas,
  runPlanner,
  type PlannerKey,
  type PlannerResult,
  type Point,
  type DynamicMotion,
  type DynamicObstacleSeed,
  type DwaCandidate,
  type DwaPose,
  type StaticRect,
} from "./planning";

type Layer = "raw" | "inflation" | "cost";
type Results = Partial<Record<PlannerKey, PlannerResult>>;
type StaticDrag = { index: number; offsetX: number; offsetY: number };
type StaticMenu = {
  left: number;
  top: number;
  index: number | null;
  point: Point;
  width: number;
  height: number;
};
type DwaRunState = {
  pose: DwaPose;
  trail: Point[];
  candidates: DwaCandidate[];
  chosen: DwaCandidate | null;
  globalIndex: number;
  target: Point;
  status: "tracking" | "waiting" | "complete";
};
const PLANNER_KEYS: PlannerKey[] = ["astar", "dijkstra", "rrt", "rrtstar"];

const COLORS: Record<PlannerKey, string> = {
  astar: "#00bfff",
  dijkstra: "#ffd400",
  rrt: "#ff4b4b",
  rrtstar: "#a970ff",
};

const NAMES: Record<PlannerKey, string> = {
  astar: "A*",
  dijkstra: "DIJKSTRA",
  rrt: "RRT",
  rrtstar: "RRT*",
};

const FORMULAS: Record<PlannerKey, string> = {
  astar: "f(n) = g(n) + h(n)",
  dijkstra: "priority = g(n)",
  rrt: "qnew = STEER(qnear, qrand)",
  rrtstar: "Jnew = min(Jnear + c)",
};

function defaultPointsForGrid(cols: number, rows: number) {
  const horizontalInset = Math.max(3, Math.round((3 / 35) * cols));
  const startBottomInset = Math.max(3, Math.round((4 / 24) * rows));
  const goalTopInset = Math.max(2, Math.round((3 / 24) * rows));
  return {
    start: { x: horizontalInset, y: rows - startBottomInset },
    goal: { x: cols - horizontalInset, y: goalTopInset },
  };
}

const INITIAL_POINTS = defaultPointsForGrid(COLS, ROWS);

function positionAlongPath(path: Point[], cursor: number) {
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

function createDwaRun(path: Point[]): DwaRunState | null {
  if (!path.length) return null;
  const origin = path[0];
  const next = path[1] ?? origin;
  return {
    pose: {
      ...origin,
      heading: Math.atan2(next.y - origin.y, next.x - origin.x),
      velocity: 2.4,
      yawRate: 0,
    },
    trail: [origin],
    candidates: [],
    chosen: null,
    globalIndex: 0,
    target: path[Math.min(path.length - 1, 6)],
    status: "tracking",
  };
}

function formatCell(point: Point) {
  return `${Math.round(point.x).toString().padStart(2, "0")},${Math.round(point.y).toString().padStart(2, "0")}`;
}

export default function Home() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const mapPanelRef = useRef<HTMLElement>(null);
  const [layer, setLayer] = useState<Layer>("cost");
  const [placement, setPlacement] = useState<"start" | "goal">("start");
  const [gridSize, setGridSize] = useState({ cols: COLS, rows: ROWS });
  const [draftCols, setDraftCols] = useState(String(COLS));
  const [draftRows, setDraftRows] = useState(String(ROWS));
  const [start, setStart] = useState<Point>(INITIAL_POINTS.start);
  const [goal, setGoal] = useState<Point>(INITIAL_POINTS.goal);
  const [inflation, setInflation] = useState(1.45);
  const [speed, setSpeed] = useState(4);
  const [dynamicMotion, setDynamicMotion] = useState<DynamicMotion>("regular");
  const [dynamicSeeds, setDynamicSeeds] = useState<DynamicObstacleSeed[]>(() => defaultDynamicObstacles());
  const [placingDynamic, setPlacingDynamic] = useState(false);
  const [staticObstacles, setStaticObstacles] = useState<StaticRect[]>(() => STATIC_RECTS.map((rect) => ({ ...rect })));
  const [draggingStatic, setDraggingStatic] = useState<StaticDrag | null>(null);
  const [staticMenu, setStaticMenu] = useState<StaticMenu | null>(null);
  const [enabled, setEnabled] = useState<Record<PlannerKey, boolean>>({
    astar: true,
    dijkstra: true,
    rrt: true,
    rrtstar: true,
  });
  const [results, setResults] = useState<Results>({});
  const [progress, setProgress] = useState(0);
  const [running, setRunning] = useState(false);
  const [executing, setExecuting] = useState(false);
  const [executionCursor, setExecutionCursor] = useState(0);
  const [dwaState, setDwaState] = useState<DwaRunState | null>(null);
  const [draggingMarker, setDraggingMarker] = useState<"start" | "goal" | null>(null);
  const [notice, setNotice] = useState("拖动地图上的起点或终点图标设置位置");

  const resultList = useMemo(
    () => PLANNER_KEYS.map((key) => results[key]).filter(Boolean) as PlannerResult[],
    [results],
  );
  const maxSteps = Math.max(0, ...resultList.map((result) => result.steps.length));
  const executionResult = results.astar ?? results.dijkstra ?? results.rrt ?? results.rrtstar;

  const clearRun = () => {
    setResults({});
    setProgress(0);
    setRunning(false);
    setExecuting(false);
    setExecutionCursor(0);
    setDwaState(null);
  };

  const applyGridSize = () => {
    const next = configureGrid(Number(draftCols) || gridSize.cols, Number(draftRows) || gridSize.rows);
    const points = defaultPointsForGrid(next.cols, next.rows);
    clearRun();
    setGridSize(next);
    setDraftCols(String(next.cols));
    setDraftRows(String(next.rows));
    setStart(points.start);
    setGoal(points.goal);
    setPlacement("start");
    setDraggingMarker(null);
    setDraggingStatic(null);
    setStaticMenu(null);
    setPlacingDynamic(false);
    setDynamicSeeds(defaultDynamicObstacles());
    setStaticObstacles(STATIC_RECTS.map((rect) => ({ ...rect })));
    setNotice(`地图已调整为 ${next.cols} × ${next.rows}，障碍物与任务点已重新布局`);
  };

  useEffect(() => {
    if (!running || maxSteps === 0) return;
    const timer = window.setInterval(() => {
      setProgress((current) => Math.min(maxSteps, current + speed));
    }, 42);
    return () => window.clearInterval(timer);
  }, [maxSteps, running, speed]);

  useEffect(() => {
    if (!running || maxSteps === 0 || progress < maxSteps) return;
    setRunning(false);
    setExecuting(true);
    setNotice("全局路径已生成，DWA 局部规划开始跟踪并实时绕障");
  }, [maxSteps, progress, running]);

  useEffect(() => {
    if (!executing || !executionResult?.path.length) return;
    setDwaState((current) => current ?? createDwaRun(executionResult.path));
  }, [executing, executionResult]);

  useEffect(() => {
    if (!executing || !executionResult?.path.length) return;
    const timer = window.setInterval(() => {
      const field = makeCostField(
        dynamicObstaclesAt(performance.now(), dynamicSeeds, dynamicMotion),
        inflation,
        staticObstacles,
      );
      setDwaState((current) => {
        const run = current ?? createDwaRun(executionResult.path);
        if (!run || run.status === "complete") return run;
        const localPlan = planDwaStep(run.pose, executionResult.path, field, run.globalIndex);
        if (localPlan.reachedGoal) {
          return { ...run, candidates: localPlan.candidates, chosen: null, globalIndex: localPlan.globalIndex, target: localPlan.target, status: "complete" };
        }
        if (!localPlan.best?.path.length) {
          return { ...run, candidates: localPlan.candidates, chosen: null, globalIndex: localPlan.globalIndex, target: localPlan.target, status: "waiting" };
        }
        const next = localPlan.best.path[0];
        const nextPose: DwaPose = {
          x: next.x,
          y: next.y,
          heading: Math.atan2(next.y - run.pose.y, next.x - run.pose.x),
          velocity: localPlan.best.velocity,
          yawRate: localPlan.best.yawRate,
        };
        return {
          pose: nextPose,
          trail: [...run.trail.slice(-480), next],
          candidates: localPlan.candidates,
          chosen: localPlan.best,
          globalIndex: localPlan.globalIndex,
          target: localPlan.target,
          status: "tracking",
        };
      });
    }, Math.max(55, 155 - speed * 15));
    return () => window.clearInterval(timer);
  }, [dynamicMotion, dynamicSeeds, executing, executionResult, inflation, speed, staticObstacles]);

  useEffect(() => {
    if (!executing || dwaState?.status !== "complete") return;
    setExecuting(false);
    setNotice("DWA 局部规划完成：机器人已安全抵达目标点");
  }, [dwaState?.status, executing]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    let frame = 0;
    let cachedField = makeCostField(
      dynamicObstaclesAt(performance.now(), dynamicSeeds, dynamicMotion),
      inflation,
      staticObstacles,
    );
    let lastFieldUpdate = 0;

    const draw = (now: number) => {
      if (now - lastFieldUpdate > 160) {
        cachedField = makeCostField(dynamicObstaclesAt(now, dynamicSeeds, dynamicMotion), inflation, staticObstacles);
        lastFieldUpdate = now;
      }
      const dynamicObstacles = dynamicObstaclesAt(now, dynamicSeeds, dynamicMotion);
      const field = cachedField;

      ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
      ctx.fillStyle = "#ffffff";
      ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

      if (layer === "cost") {
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

      if (layer === "inflation") {
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
      for (let x = 0; x <= CANVAS_WIDTH; x += CELL) {
        const position = x === CANVAS_WIDTH ? x - 0.5 : x + 0.5;
        ctx.beginPath(); ctx.moveTo(position, 0); ctx.lineTo(position, CANVAS_HEIGHT); ctx.stroke();
      }
      for (let y = 0; y <= CANVAS_HEIGHT; y += CELL) {
        const position = y === CANVAS_HEIGHT ? y - 0.5 : y + 0.5;
        ctx.beginPath(); ctx.moveTo(0, position); ctx.lineTo(CANVAS_WIDTH, position); ctx.stroke();
      }

      ctx.fillStyle = "#374151";
      ctx.strokeStyle = "#111827";
      ctx.lineWidth = 1.5;
      staticObstacles.forEach((rect) => {
        ctx.fillRect(rect.x * CELL, rect.y * CELL, rect.w * CELL, rect.h * CELL);
        ctx.strokeRect(
          rect.x * CELL + 1,
          rect.y * CELL + 1,
          Math.max(0, rect.w * CELL - 2),
          Math.max(0, rect.h * CELL - 2),
        );
      });

      dynamicObstacles.forEach((point, index) => {
        const pixel = pointToCanvas(point);
        if (layer !== "raw") {
          ctx.beginPath();
          ctx.fillStyle = "rgba(239, 68, 68, .12)";
          ctx.arc(pixel.x, pixel.y, (inflation + 1) * CELL, 0, Math.PI * 2);
          ctx.fill();
          ctx.strokeStyle = "rgba(220, 38, 38, .5)";
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.fillStyle = "#1f2937";
        ctx.arc(pixel.x, pixel.y, Math.max(4, CELL * 0.54), 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#ffffff";
        ctx.font = `700 ${Math.max(6, Math.min(9, CELL * 0.38))}px ui-monospace`;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(`D${index + 1}`, pixel.x, pixel.y + 1);
      });

      ctx.strokeStyle = "#334155";
      ctx.lineWidth = Math.max(2, Math.min(8, CELL / 3));
      ctx.strokeRect(2, 2, CANVAS_WIDTH - 4, CANVAS_HEIGHT - 4);

      resultList.forEach((result) => {
        const visible = Math.min(Math.floor(progress), result.steps.length);
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
            ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
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

        const current = steps.at(-1);
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

      if (dwaState && executionResult?.path.length && progress >= maxSteps) {
        const drawPolyline = (points: Point[], color: string, width: number, dash: number[] = []) => {
          if (points.length < 2) return;
          ctx.beginPath();
          points.forEach((point, index) => {
            const pixel = pointToCanvas(point);
            if (index === 0) ctx.moveTo(pixel.x, pixel.y);
            else ctx.lineTo(pixel.x, pixel.y);
          });
          ctx.strokeStyle = color;
          ctx.lineWidth = width;
          ctx.setLineDash(dash);
          ctx.stroke();
        };
        ctx.save();
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        dwaState.candidates.filter((candidate) => candidate.safe).forEach((candidate) => {
          drawPolyline(candidate.path, "rgba(20, 184, 166, .2)", 1, [3, 4]);
        });
        if (dwaState.chosen) drawPolyline(dwaState.chosen.path, "#14b8a6", 2.3);
        drawPolyline(dwaState.trail, "#0f766e", 2.8);
        const target = pointToCanvas(dwaState.target);
        ctx.setLineDash([2, 3]);
        ctx.strokeStyle = "#14b8a6";
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(target.x, target.y, Math.max(5, CELL * 0.72), 0, Math.PI * 2); ctx.stroke();
        ctx.restore();
      }

      const drawMarker = (point: Point, color: string, kind: "start" | "goal") => {
        const pixel = pointToCanvas(point);
        const active = draggingMarker === kind;
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
      };
      const robot = dwaState?.pose ?? positionAlongPath(executionResult?.path ?? [start], executionCursor) ?? { ...start, heading: 0 };
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
      ctx.moveTo(13, 0); ctx.lineTo(5, -5); ctx.lineTo(5, 5); ctx.closePath(); ctx.fill();
      ctx.fillStyle = "#071016";
      ctx.fillRect(-7, -11, 5, 4); ctx.fillRect(-7, 7, 5, 4); ctx.fillRect(3, -11, 5, 4); ctx.fillRect(3, 7, 5, 4);
      ctx.restore();

      drawMarker(start, "#2ed47a", "start");
      drawMarker(goal, "#ff9f43", "goal");

      frame = window.requestAnimationFrame(draw);
    };
    frame = window.requestAnimationFrame(draw);
    return () => window.cancelAnimationFrame(frame);
  }, [draggingMarker, dynamicMotion, dynamicSeeds, dwaState, executionCursor, executionResult, gridSize, inflation, layer, maxSteps, progress, resultList, start, staticObstacles, goal]);

  const plan = () => {
    setPlacingDynamic(false);
    if (resultList.length && progress < maxSteps) {
      setRunning((value) => !value);
      setNotice(running ? "推演已暂停" : "继续推演搜索过程");
      return;
    }
    const selected = PLANNER_KEYS.filter((key) => enabled[key]);
    if (!selected.length) {
      setNotice("请至少选择一种规划算法");
      return;
    }
    const snapshot = dynamicObstaclesAt(performance.now(), dynamicSeeds, dynamicMotion);
    const field = makeCostField(snapshot, inflation, staticObstacles);
    if (field.blocked.has(`${start.x},${start.y}`) || field.blocked.has(`${goal.x},${goal.y}`)) {
      setNotice("起点或终点位于膨胀障碍区，请重新放置");
      return;
    }
    const nextResults: Results = {};
    selected.forEach((key) => { nextResults[key] = runPlanner(key, start, goal, field); });
    setResults(nextResults);
    setProgress(0);
    setExecutionCursor(0);
    setExecuting(false);
    setRunning(true);
    setNotice("推演中：亮环表示当前扩展节点，微光点表示本步新生成节点");
  };

  const pointerGridPosition = (event: React.PointerEvent<HTMLCanvasElement>, snap: boolean) => {
    const canvas = event.currentTarget;
    const rect = canvas.getBoundingClientRect();
    const raw = {
      x: ((event.clientX - rect.left) / rect.width) * COLS - 0.5,
      y: ((event.clientY - rect.top) / rect.height) * ROWS - 0.5,
    };
    if (!snap) return raw;
    return {
      x: Math.max(1, Math.min(COLS - 2, Math.round(raw.x))),
      y: Math.max(1, Math.min(ROWS - 2, Math.round(raw.y))),
    };
  };

  const pointInStaticRect = (point: Point, rect: StaticRect) => (
    point.x >= rect.x && point.x < rect.x + rect.w && point.y >= rect.y && point.y < rect.y + rect.h
  );

  const staticRectsOverlap = (a: StaticRect, b: StaticRect) => (
    a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y
  );

  const staticRectIsAllowed = (candidate: StaticRect, index: number | null) => {
    if (pointInStaticRect(start, candidate) || pointInStaticRect(goal, candidate)) return false;
    return !staticObstacles.some((rect, rectIndex) => rectIndex !== index && staticRectsOverlap(candidate, rect));
  };

  const openStaticMenu = (event: React.MouseEvent<HTMLCanvasElement>) => {
    event.preventDefault();
    const point = pointerGridPosition(event as unknown as React.PointerEvent<HTMLCanvasElement>, true);
    const index = staticObstacles.findIndex((rect) => pointInStaticRect(point, rect));
    const rect = index >= 0 ? staticObstacles[index] : { x: point.x, y: point.y, w: 3, h: 3 };
    const panelBounds = mapPanelRef.current?.getBoundingClientRect();
    const panelWidth = panelBounds?.width ?? 320;
    const panelHeight = panelBounds?.height ?? 600;
    const left = Math.max(8, Math.min(panelWidth - 218, event.clientX - (panelBounds?.left ?? 0)));
    const top = Math.max(40, Math.min(panelHeight - 164, event.clientY - (panelBounds?.top ?? 0)));
    setPlacingDynamic(false);
    setStaticMenu({ left, top, index: index >= 0 ? index : null, point: { x: rect.x, y: rect.y }, width: rect.w, height: rect.h });
    setNotice(index >= 0 ? "可调整该静态障碍的尺寸、删除或拖动位置" : "设置尺寸后可在此处添加静态障碍");
  };

  const applyStaticMenu = () => {
    if (!staticMenu) return;
    const x = Math.max(1, Math.min(COLS - 2, staticMenu.point.x));
    const y = Math.max(1, Math.min(ROWS - 2, staticMenu.point.y));
    const candidate: StaticRect = {
      x,
      y,
      w: Math.max(1, Math.min(COLS - 1 - x, Math.round(staticMenu.width) || 1)),
      h: Math.max(1, Math.min(ROWS - 1 - y, Math.round(staticMenu.height) || 1)),
    };
    if (!staticRectIsAllowed(candidate, staticMenu.index)) {
      setNotice("静态障碍不能覆盖起终点或与其他静态障碍重叠");
      return;
    }
    clearRun();
    if (staticMenu.index === null) {
      setStaticObstacles((current) => [...current, candidate]);
      setNotice(`已添加 ${candidate.w} × ${candidate.h} 的静态障碍`);
    } else {
      setStaticObstacles((current) => current.map((rect, index) => index === staticMenu.index ? candidate : rect));
      setNotice("静态障碍尺寸已更新");
    }
    setStaticMenu(null);
  };

  const deleteStaticMenuObstacle = () => {
    if (staticMenu?.index === null || staticMenu === null) return;
    clearRun();
    setStaticObstacles((current) => current.filter((_, index) => index !== staticMenu.index));
    setStaticMenu(null);
    setNotice("静态障碍已清除");
  };

  const beginMarkerDrag = (event: React.PointerEvent<HTMLCanvasElement>) => {
    if (event.button !== 0) return;
    setStaticMenu(null);
    const raw = pointerGridPosition(event, false);
    const startDistance = Math.hypot(raw.x - start.x, raw.y - start.y);
    const goalDistance = Math.hypot(raw.x - goal.x, raw.y - goal.y);
    const marker = Math.min(startDistance, goalDistance) <= 1.15
      ? startDistance <= goalDistance ? "start" : "goal"
      : null;
    if (!marker) {
      const point = pointerGridPosition(event, true);
      const staticIndex = staticObstacles.findIndex((rect) => pointInStaticRect(point, rect));
      if (staticIndex >= 0) {
        const rect = staticObstacles[staticIndex];
        event.preventDefault();
        event.currentTarget.setPointerCapture(event.pointerId);
        clearRun();
        setPlacingDynamic(false);
        setDraggingStatic({ index: staticIndex, offsetX: point.x - rect.x, offsetY: point.y - rect.y });
        setNotice("正在拖动静态障碍，松开后完成移动");
        return;
      }
      if (!placingDynamic) return;
      const staticField = makeCostField([], inflation, staticObstacles);
      if (staticField.blocked.has(`${point.x},${point.y}`)) {
        setNotice("这里是静态障碍或膨胀区，不能放置动态障碍");
        return;
      }
      if (Math.hypot(point.x - start.x, point.y - start.y) < 2 || Math.hypot(point.x - goal.x, point.y - goal.y) < 2) {
        setNotice("动态障碍需要与起终点保持至少 2 个栅格");
        return;
      }
      if (dynamicSeeds.some((seed) => Math.hypot(seed.x - point.x, seed.y - point.y) < 1.5)) {
        setNotice("此处附近已经有一个动态障碍");
        return;
      }
      if (dynamicSeeds.length >= 8) {
        setNotice("最多可放置 8 个动态障碍");
        return;
      }
      event.preventDefault();
      clearRun();
      setDynamicSeeds((current) => [...current, { ...point, phase: current.length * 0.83 + 0.4 }]);
      setNotice(`已添加动态障碍 D${dynamicSeeds.length + 1}，可继续点击地图添加`);
      return;
    }
    event.preventDefault();
    event.currentTarget.setPointerCapture(event.pointerId);
    clearRun();
    setPlacingDynamic(false);
    setDraggingMarker(marker);
    setDraggingStatic(null);
    setPlacement(marker);
    setNotice(marker === "start" ? "正在拖动起点图标" : "正在拖动终点图标");
  };

  const moveMarker = (event: React.PointerEvent<HTMLCanvasElement>) => {
    const point = pointerGridPosition(event, true);
    if (draggingStatic) {
      const current = staticObstacles[draggingStatic.index];
      if (!current) return;
      const candidate: StaticRect = {
        ...current,
        x: Math.max(1, Math.min(COLS - 1 - current.w, point.x - draggingStatic.offsetX)),
        y: Math.max(1, Math.min(ROWS - 1 - current.h, point.y - draggingStatic.offsetY)),
      };
      if (!staticRectIsAllowed(candidate, draggingStatic.index)) {
        setNotice("静态障碍不能覆盖起终点或其他静态障碍");
        return;
      }
      setStaticObstacles((currentRects) => currentRects.map((rect, index) => index === draggingStatic.index ? candidate : rect));
      setNotice("松开以固定静态障碍位置");
      return;
    }
    if (!draggingMarker) return;
    const field = makeCostField(dynamicObstaclesAt(performance.now(), dynamicSeeds, dynamicMotion), inflation, staticObstacles);
    if (field.blocked.has(`${point.x},${point.y}`)) {
      setNotice("此处是障碍或膨胀区，图标不能放置");
      return;
    }
    const other = draggingMarker === "start" ? goal : start;
    if (point.x === other.x && point.y === other.y) {
      setNotice("起点与终点不能重合");
      return;
    }
    if (draggingMarker === "start") {
      setStart(point);
    } else {
      setGoal(point);
    }
    setNotice(draggingMarker === "start" ? "松开以设置起点" : "松开以设置终点");
  };

  const endMarkerDrag = (event: React.PointerEvent<HTMLCanvasElement>) => {
    if (!draggingMarker && !draggingStatic) return;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
    if (draggingStatic) {
      setDraggingStatic(null);
      setNotice("静态障碍已更新，可以启动路径规划");
      return;
    }
    const label = draggingMarker === "start" ? "起点" : "终点";
    setDraggingMarker(null);
    setNotice(`${label}已更新，可以启动路径规划`);
  };

  const seekProgress = (event: React.ChangeEvent<HTMLInputElement>) => {
    const next = Number(event.target.value);
    setRunning(false);
    setExecuting(false);
    setExecutionCursor(0);
    setProgress(next);
    setNotice(`已定位到推演过程 ${Math.round((next / Math.max(maxSteps, 1)) * 100)}%`);
  };

  const resetPoints = () => {
    const points = defaultPointsForGrid(gridSize.cols, gridSize.rows);
    clearRun();
    setStart(points.start);
    setGoal(points.goal);
    setPlacement("start");
    setDraggingMarker(null);
    setNotice("起终点已重置，可直接拖动图标调整位置");
  };

  const visibleStep = (result: PlannerResult) => result.steps[Math.min(Math.floor(progress), result.steps.length) - 1];
  const enabledPlanners = PLANNER_KEYS.filter((key) => enabled[key]);
  const dwaSafeCount = dwaState?.candidates.filter((candidate) => candidate.safe).length ?? 0;
  const dwaLabel = !dwaState
    ? "待命"
    : dwaState.status === "tracking" ? "局部绕障中" : dwaState.status === "waiting" ? "等待安全窗口" : "已抵达";

  const togglePlanner = (key: PlannerKey) => {
    const next = { ...enabled, [key]: !enabled[key] };
    clearRun();
    setEnabled(next);
  };

  return (
    <main className="lab-page">
      <header className="topbar">
        <div className="brand-lockup">
          <span className="brand-mark">NS</span>
          <div><strong>NAV STUDIO</strong><p>ROBOT PATH PLANNING WORKSPACE</p></div>
        </div>
        <div className="workspace-stats">
          <span>GRID <b>{gridSize.cols} × {gridSize.rows}</b></span>
          <span>PLANNERS <b>04</b></span>
          <span>OBSTACLES <b>LIVE</b></span>
        </div>
        <div className="status-strip"><span className="live-dot" /> SIMULATION <b>20 Hz</b></div>
      </header>

      <section className="lab-grid">
        <aside className="control-panel panel">
          <div className="panel-heading primary"><div><p>CONTROL DECK</p><h2>规划控制</h2></div><i className="panel-signal" /></div>
          <div className="section-label"><span>MAP SIZE</span><b>32–240 列 / 20–200 行</b></div>
          <div className="grid-size-control">
            <label>列<input type="number" min={GRID_LIMITS.minCols} max={GRID_LIMITS.maxCols} value={draftCols} onChange={(event) => setDraftCols(event.target.value)} onKeyDown={(event) => { if (event.key === "Enter") applyGridSize(); }} aria-label="地图列数" /></label>
            <i>×</i>
            <label>行<input type="number" min={GRID_LIMITS.minRows} max={GRID_LIMITS.maxRows} value={draftRows} onChange={(event) => setDraftRows(event.target.value)} onKeyDown={(event) => { if (event.key === "Enter") applyGridSize(); }} aria-label="地图行数" /></label>
            <button type="button" onClick={applyGridSize}>应用</button>
          </div>
          <div className="segmented">
            {(["raw", "inflation", "cost"] as Layer[]).map((item) => (
              <button key={item} type="button" aria-pressed={layer === item} className={layer === item ? "active" : ""} onClick={() => setLayer(item)}>
                {item === "raw" ? "原始" : item === "inflation" ? "膨胀" : "代价"}
              </button>
            ))}
          </div>

          <label className="range-control">
            <span>膨胀半径 <b>{inflation.toFixed(2)} cell</b></span>
            <input type="range" min="0.8" max="2.4" step="0.05" value={inflation} onChange={(event) => { clearRun(); setInflation(Number(event.target.value)); }} />
          </label>

          <div className="section-label"><span>DYNAMIC</span><b>{dynamicSeeds.length} 个动态障碍</b></div>
          <div className="segmented dynamic-mode-control">
            {(["regular", "random"] as DynamicMotion[]).map((mode) => (
              <button key={mode} type="button" className={dynamicMotion === mode ? "active" : ""} onClick={() => { clearRun(); setDynamicMotion(mode); setNotice(mode === "regular" ? "动态障碍已切换为规则往复运动" : "动态障碍已切换为随机平滑运动"); }}>
                {mode === "regular" ? "规则移动" : "随机移动"}
              </button>
            ))}
          </div>
          <div className="dynamic-actions">
            <button type="button" className={placingDynamic ? "active" : ""} onClick={() => { setPlacingDynamic((current) => !current); setNotice(placingDynamic ? "已结束放置动态障碍" : "请在地图空白区域点击放置动态障碍"); }}>{placingDynamic ? "结束放置" : "添加障碍"}</button>
            <button type="button" onClick={() => { clearRun(); setDynamicSeeds([]); setPlacingDynamic(false); setNotice("动态障碍已清空"); }}>清空</button>
            <button type="button" onClick={() => { clearRun(); setDynamicSeeds(defaultDynamicObstacles()); setPlacingDynamic(false); setNotice("已恢复默认动态障碍"); }}>默认</button>
          </div>

          <div className="section-label"><span>LOCAL PLANNER</span><b>全局路径完成后自动接管</b></div>
          <div className={`dwa-card ${dwaState?.status ?? "idle"}`}>
            <div className="dwa-title"><p><i /> DWA 局部避障</p><strong>{dwaLabel}</strong></div>
            <dl>
              <div><dt>安全候选</dt><dd>{dwaSafeCount}/{dwaState?.candidates.length ?? 20}</dd></div>
              <div><dt>线速度</dt><dd>{dwaState ? `${dwaState.pose.velocity.toFixed(1)} c/s` : "—"}</dd></div>
              <div><dt>角速度</dt><dd>{dwaState ? `${dwaState.pose.yawRate.toFixed(2)} rad/s` : "—"}</dd></div>
              <div><dt>目标点</dt><dd>{dwaState ? formatCell(dwaState.target) : "—"}</dd></div>
            </dl>
            <p>评估速度窗口与预测轨迹，避开静态、动态及膨胀障碍。</p>
          </div>

          <div className="section-label"><span>MISSION</span><b>拖拽任务点</b></div>
          <div className="point-readout">
            <button type="button" className={placement === "start" ? "selected" : ""} onClick={() => { setPlacingDynamic(false); setPlacement("start"); setNotice("请拖动地图上的绿色起点图标"); }}><i className="start-swatch" /> 起点 <b>{formatCell(start)}</b></button>
            <button type="button" className={placement === "goal" ? "selected" : ""} onClick={() => { setPlacingDynamic(false); setPlacement("goal"); setNotice("请拖动地图上的橙色终点图标"); }}><i className="goal-swatch" /> 终点 <b>{formatCell(goal)}</b></button>
          </div>
          <p className="drag-hint">按住地图中的图标并拖到自由栅格</p>
          <button className="reset-button" type="button" onClick={resetPoints}>重置起终点</button>

          <div className="section-label"><span>PLANNERS</span><b>算法与搜索状态</b></div>
          <details className="planner-picker">
            <summary>选择规划算法 <b>{enabledPlanners.length} / {PLANNER_KEYS.length}</b></summary>
            <div className="algorithm-list">
              {PLANNER_KEYS.map((key) => (
                <label key={key} className={`planner-option ${key}`}>
                  <input type="checkbox" checked={enabled[key]} onChange={() => togglePlanner(key)} />
                  <i className={key} />{NAMES[key]}<small>{key === "astar" ? "f=g+h" : key === "dijkstra" ? "g only" : key === "rrt" ? "sample" : "rewire"}</small>
                </label>
              ))}
            </div>
          </details>

          <div className="planner-status-view multi">
            {enabledPlanners.length ? enabledPlanners.map((key) => {
              const result = results[key];
              const visible = result ? Math.min(Math.floor(progress), result.steps.length) : 0;
              const done = Boolean(result && visible >= result.steps.length);
              const step = result ? visibleStep(result) : undefined;
              const generated = result ? 1 + result.steps.slice(0, visible).reduce((total, item) => total + item.generated.length, 0) : 0;
              const message = !result
                ? "等待启动路径规划"
                : done
                  ? result.found ? `已找到路径，长度 ${(result.pathLength * 0.25).toFixed(1)}m` : "搜索结束，未找到路径"
                  : step ? `正在检查 ${formatCell(step.pick)}，本步新增 ${step.generated.length} 个候选点` : "正在初始化搜索空间";
              return (
                <div key={key} className={`telemetry-card compact ${key}`}>
                  <div className="telemetry-title"><p>{NAMES[key]}</p><strong>{!result ? "" : done ? result.found ? "已完成" : "无路径" : "计算中"}</strong></div>
                  <div className="algorithm-formula"><span>核心规则</span><b>{FORMULAS[key]}</b></div>
                  <dl>
                    <div><dt>扩展</dt><dd>{visible}</dd></div>
                    <div><dt>生成</dt><dd>{generated}</dd></div>
                    <div><dt>待选</dt><dd>{step?.open ?? 0}</dd></div>
                    <div><dt>已检</dt><dd>{step?.closed ?? 0}</dd></div>
                    <div><dt>耗时</dt><dd>{result ? `${result.durationMs.toFixed(1)}ms` : "—"}</dd></div>
                    <div><dt>路径</dt><dd>{done && result?.found ? `${(result.pathLength * 0.25).toFixed(1)}m` : "—"}</dd></div>
                  </dl>
                  <p className="status-explanation">{message}</p>
                </div>
              );
            }) : <p className="empty-planner-status">请从上方下拉框勾选至少一种算法</p>}
          </div>

          <label className="range-control speed-control">
            <span>推演速度 <b>{speed}×</b></span>
            <input type="range" min="1" max="10" step="1" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
          </label>

          <button className="run-button" type="button" onClick={plan}>
            {running ? "暂停推演" : resultList.length && progress < maxSteps ? "继续推演" : "启动路径规划"}<span>{running ? "Ⅱ" : "→"}</span>
          </button>
        </aside>

        <section className="map-panel panel" ref={mapPanelRef}>
          <div className="map-toolbar">
            <div><span className="pulse" /> PLANNING WORLD <i>/</i> {layer.toUpperCase()}</div>
            <p className={running ? "is-running" : ""}>{running ? `SEARCH ${Math.round((progress / Math.max(maxSteps, 1)) * 100)}%` : executing ? `DWA ${dwaState?.status === "waiting" ? "WAITING" : "TRACKING"}` : placingDynamic ? "ADD DYNAMIC OBSTACLE" : draggingStatic ? "DRAG STATIC OBSTACLE" : draggingMarker ? `DRAG ${draggingMarker.toUpperCase()}` : "DRAG MARKERS"}</p>
          </div>
          <canvas
            ref={canvasRef}
            className={placingDynamic ? "is-placing-dynamic" : draggingMarker || draggingStatic ? "is-dragging" : ""}
            width={CANVAS_WIDTH}
            height={CANVAS_HEIGHT}
            onPointerDown={beginMarkerDrag}
            onPointerMove={moveMarker}
            onPointerUp={endMarkerDrag}
            onPointerCancel={endMarkerDrag}
            onContextMenu={openStaticMenu}
            aria-label="交互式路径规划地图，可拖动起终点和静态障碍，右键编辑或添加静态障碍"
          />
          {staticMenu && (
            <div className="static-menu" style={{ left: staticMenu.left, top: staticMenu.top }} onPointerDown={(event) => event.stopPropagation()}>
              <div><b>{staticMenu.index === null ? "添加静态障碍" : "编辑静态障碍"}</b><button type="button" onClick={() => setStaticMenu(null)} aria-label="关闭静态障碍菜单">×</button></div>
              <p>位置 {formatCell(staticMenu.point)}</p>
              <label>宽度<input type="number" min="1" max={Math.max(1, COLS - 1 - staticMenu.point.x)} value={staticMenu.width} onChange={(event) => setStaticMenu((current) => current ? { ...current, width: Number(event.target.value) } : current)} /></label>
              <label>高度<input type="number" min="1" max={Math.max(1, ROWS - 1 - staticMenu.point.y)} value={staticMenu.height} onChange={(event) => setStaticMenu((current) => current ? { ...current, height: Number(event.target.value) } : current)} /></label>
              <div className="static-menu-actions"><button type="button" onClick={applyStaticMenu}>{staticMenu.index === null ? "添加" : "保存"}</button>{staticMenu.index !== null && <button type="button" className="delete" onClick={deleteStaticMenuObstacle}>清除</button>}</div>
            </div>
          )}
          <div className={`timeline-control ${resultList.length ? "" : "disabled"}`}>
            <span>REPLAY</span>
            <div className="timeline-range">
              <i style={{ width: `${Math.min(100, (progress / Math.max(maxSteps, 1)) * 100)}%` }} />
              <input
                type="range"
                min="0"
                max={Math.max(maxSteps, 1)}
                step="1"
                value={Math.min(progress, Math.max(maxSteps, 1))}
                disabled={!resultList.length}
                onChange={seekProgress}
                aria-label="拖动查看路径规划推演过程"
              />
            </div>
            <b>{Math.round((progress / Math.max(maxSteps, 1)) * 100)}%</b>
          </div>
          <div className="map-legend">
            <span><i className="wall" />边界 / 静态障碍</span>
            <span><i className="dynamic" />动态障碍</span>
            <span><i className="inflated" />膨胀与代价</span>
            <span><i className="pick-ring" />当前扩展节点</span>
            <span><i className="dwa" />DWA 候选 / 局部轨迹</span>
            <span className="legend-note">{notice}</span>
          </div>
        </section>

      </section>
    </main>
  );
}
