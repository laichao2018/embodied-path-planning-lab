"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import {
  CANVAS_HEIGHT,
  CANVAS_WIDTH,
  CELL,
  COLS,
  ROWS,
  STATIC_RECTS,
  dynamicObstaclesAt,
  makeCostField,
  pointToCanvas,
  runPlanner,
  type PlannerKey,
  type PlannerResult,
  type Point,
} from "./planning";

type Layer = "raw" | "inflation" | "cost";
type Results = Partial<Record<PlannerKey, PlannerResult>>;
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

const DESCRIPTIONS: Record<PlannerKey, string> = {
  astar: "f = g + h / 目标导向",
  dijkstra: "priority = g / 等代价扩展",
  rrt: "sample + steer / 树生长",
  rrtstar: "choose parent + rewire / 渐近优化",
};

const FORMULAS: Record<PlannerKey, string> = {
  astar: "f(n) = g(n) + h(n)",
  dijkstra: "priority = g(n)",
  rrt: "qnew = STEER(qnear, qrand)",
  rrtstar: "Jnew = min(Jnear + c)",
};

const DEFAULT_START = { x: 3, y: 20 };
const DEFAULT_GOAL = { x: 32, y: 3 };

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

function formatCell(point: Point) {
  return `${Math.round(point.x).toString().padStart(2, "0")},${Math.round(point.y).toString().padStart(2, "0")}`;
}

export default function Home() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [layer, setLayer] = useState<Layer>("cost");
  const [placement, setPlacement] = useState<"start" | "goal">("start");
  const [start, setStart] = useState<Point>(DEFAULT_START);
  const [goal, setGoal] = useState<Point>(DEFAULT_GOAL);
  const [inflation, setInflation] = useState(1.45);
  const [speed, setSpeed] = useState(4);
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
    setNotice("全局路径已生成，机器人开始沿 A* 优先路径执行");
  }, [maxSteps, progress, running]);

  useEffect(() => {
    if (!executing || !executionResult?.path.length) return;
    const end = executionResult.path.length - 1;
    const timer = window.setInterval(() => {
      setExecutionCursor((current) => {
        const next = Math.min(end, current + 0.12 + speed * 0.018);
        if (next >= end) {
          window.clearInterval(timer);
          setExecuting(false);
          setNotice("任务完成：机器人已抵达目标点");
        }
        return next;
      });
    }, 46);
    return () => window.clearInterval(timer);
  }, [executing, executionResult, speed]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    let frame = 0;
    let cachedField = makeCostField(dynamicObstaclesAt(performance.now()), inflation);
    let lastFieldUpdate = 0;

    const draw = (now: number) => {
      if (now - lastFieldUpdate > 160) {
        cachedField = makeCostField(dynamicObstaclesAt(now), inflation);
        lastFieldUpdate = now;
      }
      const dynamicObstacles = dynamicObstaclesAt(now);
      const field = cachedField;

      ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
      ctx.fillStyle = "#091219";
      ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

      if (layer === "cost") {
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

      if (layer === "inflation") {
        ctx.fillStyle = "rgba(255, 135, 80, .24)";
        field.blocked.forEach((key) => {
          if (field.raw.has(key)) return;
          const [x, y] = key.split(",").map(Number);
          ctx.fillRect(x * CELL, y * CELL, CELL, CELL);
        });
      }

      ctx.strokeStyle = "rgba(137, 174, 187, .095)";
      ctx.lineWidth = 1;
      for (let x = 0; x <= CANVAS_WIDTH; x += CELL) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, CANVAS_HEIGHT); ctx.stroke();
      }
      for (let y = 0; y <= CANVAS_HEIGHT; y += CELL) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(CANVAS_WIDTH, y); ctx.stroke();
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
        if (layer !== "raw") {
          ctx.beginPath();
          ctx.fillStyle = "rgba(255, 92, 92, .11)";
          ctx.arc(pixel.x, pixel.y, (inflation + 1) * CELL, 0, Math.PI * 2);
          ctx.fill();
          ctx.strokeStyle = "rgba(255, 92, 92, .32)";
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.fillStyle = "#f4f7f8";
        ctx.arc(pixel.x, pixel.y, 13, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#172026";
        ctx.font = "700 9px ui-monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(`D${index + 1}`, pixel.x, pixel.y + 1);
      });

      ctx.strokeStyle = "#748994";
      ctx.lineWidth = 8;
      ctx.strokeRect(4, 4, CANVAS_WIDTH - 8, CANVAS_HEIGHT - 8);

      resultList.forEach((result) => {
        const visible = Math.min(Math.floor(progress), result.steps.length);
        const color = COLORS[result.key];
        const steps = result.steps.slice(0, visible);

        if (result.key === "rrt" || result.key === "rrtstar") {
          ctx.lineWidth = 1.15;
          ctx.strokeStyle = `${color}66`;
          steps.forEach((step) => {
            if (!step.edge) return;
            const a = pointToCanvas(step.edge[0]);
            const b = pointToCanvas(step.edge[1]);
            ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
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

      const drawMarker = (point: Point, color: string, kind: "start" | "goal") => {
        const pixel = pointToCanvas(point);
        const active = draggingMarker === kind;
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
      };
      const robot = positionAlongPath(executionResult?.path ?? [start], executionCursor) ?? { ...start, heading: 0 };
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
  }, [draggingMarker, executionCursor, executionResult, inflation, layer, progress, resultList, start, goal]);

  const plan = () => {
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
    const snapshot = dynamicObstaclesAt(performance.now());
    const field = makeCostField(snapshot, inflation);
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

  const beginMarkerDrag = (event: React.PointerEvent<HTMLCanvasElement>) => {
    const raw = pointerGridPosition(event, false);
    const startDistance = Math.hypot(raw.x - start.x, raw.y - start.y);
    const goalDistance = Math.hypot(raw.x - goal.x, raw.y - goal.y);
    const marker = Math.min(startDistance, goalDistance) <= 1.15
      ? startDistance <= goalDistance ? "start" : "goal"
      : null;
    if (!marker) return;
    event.preventDefault();
    event.currentTarget.setPointerCapture(event.pointerId);
    clearRun();
    setDraggingMarker(marker);
    setPlacement(marker);
    setNotice(marker === "start" ? "正在拖动起点图标" : "正在拖动终点图标");
  };

  const moveMarker = (event: React.PointerEvent<HTMLCanvasElement>) => {
    if (!draggingMarker) return;
    const point = pointerGridPosition(event, true);
    const field = makeCostField(dynamicObstaclesAt(performance.now()), inflation);
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
    if (!draggingMarker) return;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
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
    clearRun();
    setStart(DEFAULT_START);
    setGoal(DEFAULT_GOAL);
    setPlacement("start");
    setDraggingMarker(null);
    setNotice("起终点已重置，可直接拖动图标调整位置");
  };

  const visibleStep = (result: PlannerResult) => result.steps[Math.min(Math.floor(progress), result.steps.length) - 1];

  return (
    <main className="lab-page">
      <header className="topbar">
        <div className="brand-lockup">
          <span className="brand-mark">NS</span>
          <div><strong>NAV STUDIO</strong><p>ROBOT PATH PLANNING WORKSPACE</p></div>
        </div>
        <div className="workspace-stats">
          <span>GRID <b>35 × 24</b></span>
          <span>PLANNERS <b>04</b></span>
          <span>OBSTACLES <b>LIVE</b></span>
        </div>
        <div className="status-strip"><span className="live-dot" /> SIMULATION <b>20 Hz</b></div>
      </header>

      <section className="lab-grid">
        <aside className="control-panel panel">
          <div className="panel-heading primary"><div><p>CONTROL DECK</p><h2>规划控制</h2></div><i className="panel-signal" /></div>
          <div className="section-label"><span>MAP MODEL</span><b>地图模型</b></div>
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

          <div className="section-label"><span>MISSION</span><b>拖拽任务点</b></div>
          <div className="point-readout">
            <button type="button" className={placement === "start" ? "selected" : ""} onClick={() => { setPlacement("start"); setNotice("请拖动地图上的绿色起点图标"); }}><i className="start-swatch" /> 起点 <b>{formatCell(start)}</b></button>
            <button type="button" className={placement === "goal" ? "selected" : ""} onClick={() => { setPlacement("goal"); setNotice("请拖动地图上的橙色终点图标"); }}><i className="goal-swatch" /> 终点 <b>{formatCell(goal)}</b></button>
          </div>
          <p className="drag-hint">按住地图中的图标并拖到自由栅格</p>
          <button className="reset-button" type="button" onClick={resetPoints}>重置起终点</button>

          <div className="section-label"><span>PLANNERS</span><b>算法选择</b></div>
          <div className="algorithm-list">
            {PLANNER_KEYS.map((key) => (
              <label key={key} className={`planner-option ${key}`}>
                <input type="checkbox" checked={enabled[key]} onChange={() => { clearRun(); setEnabled((current) => ({ ...current, [key]: !current[key] })); }} />
                <i className={key} />{NAMES[key]}<small>{key === "astar" ? "f=g+h" : key === "dijkstra" ? "g only" : key === "rrt" ? "sample" : "rewire"}</small>
              </label>
            ))}
          </div>

          <label className="range-control speed-control">
            <span>推演速度 <b>{speed}×</b></span>
            <input type="range" min="1" max="10" step="1" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
          </label>

          <button className="run-button" type="button" onClick={plan}>
            {running ? "暂停推演" : resultList.length && progress < maxSteps ? "继续推演" : "启动路径规划"}<span>{running ? "Ⅱ" : "→"}</span>
          </button>
        </aside>

        <section className="map-panel panel">
          <div className="map-toolbar">
            <div><span className="pulse" /> PLANNING WORLD <i>/</i> {layer.toUpperCase()}</div>
            <p className={running ? "is-running" : ""}>{running ? `SEARCH ${Math.round((progress / Math.max(maxSteps, 1)) * 100)}%` : executing ? "ROBOT EXECUTING" : draggingMarker ? `DRAG ${draggingMarker.toUpperCase()}` : "DRAG MARKERS"}</p>
          </div>
          <canvas
            ref={canvasRef}
            className={draggingMarker ? "is-dragging" : ""}
            width={CANVAS_WIDTH}
            height={CANVAS_HEIGHT}
            onPointerDown={beginMarkerDrag}
            onPointerMove={moveMarker}
            onPointerUp={endMarkerDrag}
            onPointerCancel={endMarkerDrag}
            aria-label="交互式路径规划地图，拖动起点箭头或终点旗帜设置位置"
          />
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
            <span className="legend-note">{notice}</span>
          </div>
        </section>

        <aside className="telemetry-panel panel">
          <div className="panel-heading primary"><div><p>ALGORITHM DATA</p><h2>搜索状态</h2></div><i className="panel-signal" /></div>
          {PLANNER_KEYS.map((key) => {
            const result = results[key];
            const visible = result ? Math.min(Math.floor(progress), result.steps.length) : 0;
            const done = result && visible >= result.steps.length;
            const currentStep = result ? visibleStep(result) : undefined;
            const visibleGenerated = result
              ? 1 + result.steps.slice(0, visible).reduce((total, step) => total + step.generated.length, 0)
              : 0;
            return (
              <div key={key} className={`telemetry-card ${key}`}>
                <div className="telemetry-title"><p>{NAMES[key]}</p><strong>{!enabled[key] ? "关闭" : !result ? "" : done ? result.found ? "已完成" : "无路径" : "计算中"}</strong></div>
                <div className="algorithm-formula"><span>核心规则</span><b>{FORMULAS[key]}</b></div>
                <small>{DESCRIPTIONS[key]}</small>
                <dl>
                  <div><dt>扩展</dt><dd>{visible}</dd></div>
                  <div><dt>生成</dt><dd>{visibleGenerated}</dd></div>
                  <div><dt>待选</dt><dd>{currentStep?.open ?? 0}</dd></div>
                  <div><dt>已检</dt><dd>{currentStep?.closed ?? 0}</dd></div>
                  <div><dt>耗时</dt><dd>{result ? `${result.durationMs.toFixed(1)}ms` : "—"}</dd></div>
                  <div><dt>路径</dt><dd>{done && result?.found ? `${(result.pathLength * 0.25).toFixed(1)}m` : "—"}</dd></div>
                </dl>
              </div>
            );
          })}
          <div className="event-log">
            <div className="event-log-heading"><p>推演记录</p><small>“扩展”是当前检查的节点，“生成”是本步发现的候选节点</small></div>
            {resultList.length === 0 ? (
              <><span><b>地图</b> 障碍与代价数据已载入</span><span><b>操作</b> 设置起终点后启动路径规划</span><span><b>回放</b> 生成结果后可拖动时间轴</span></>
            ) : resultList.map((result) => {
              const step = visibleStep(result);
              const done = Math.floor(progress) >= result.steps.length;
              return (
                <span key={result.key}>
                  <b style={{ color: COLORS[result.key] }}>{NAMES[result.key]}</b>
                  {done
                    ? result.found ? `已找到路径，长度 ${(result.pathLength * 0.25).toFixed(1)}m` : "搜索结束，未找到路径"
                    : step ? `正在检查 ${formatCell(step.pick)}，新增 ${step.generated.length} 个候选点` : "等待开始推演"}
                </span>
              );
            })}
            {executing && <span><b>机器人</b> 正在沿优先路径移动</span>}
          </div>
        </aside>
      </section>
    </main>
  );
}
