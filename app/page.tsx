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

const COLORS: Record<PlannerKey, string> = {
  astar: "#43d9ff",
  dijkstra: "#ffb85b",
  rrt: "#ff5d8e",
};

const NAMES: Record<PlannerKey, string> = {
  astar: "A*",
  dijkstra: "DIJKSTRA",
  rrt: "RRT",
};

const DESCRIPTIONS: Record<PlannerKey, string> = {
  astar: "f = g + h / 目标导向",
  dijkstra: "priority = g / 等代价扩展",
  rrt: "sample + steer / 树生长",
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
  });
  const [results, setResults] = useState<Results>({});
  const [progress, setProgress] = useState(0);
  const [running, setRunning] = useState(false);
  const [executing, setExecuting] = useState(false);
  const [executionCursor, setExecutionCursor] = useState(0);
  const [notice, setNotice] = useState("单击地图依次设置起点与终点");

  const resultList = useMemo(
    () => (["astar", "dijkstra", "rrt"] as PlannerKey[]).map((key) => results[key]).filter(Boolean) as PlannerResult[],
    [results],
  );
  const maxSteps = Math.max(0, ...resultList.map((result) => result.steps.length));
  const executionResult = results.astar ?? results.dijkstra ?? results.rrt;

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
          ctx.fillStyle = "rgba(255, 93, 142, .12)";
          ctx.arc(pixel.x, pixel.y, (inflation + 1) * CELL, 0, Math.PI * 2);
          ctx.fill();
          ctx.strokeStyle = "rgba(255, 93, 142, .28)";
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.fillStyle = "#ff5d8e";
        ctx.arc(pixel.x, pixel.y, 13, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#1d0810";
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

        if (result.key === "rrt") {
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

      const drawMarker = (point: Point, color: string, label: string) => {
        const pixel = pointToCanvas(point);
        ctx.beginPath();
        ctx.fillStyle = color;
        ctx.arc(pixel.x, pixel.y, 13, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#061015";
        ctx.font = "800 11px ui-monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(label, pixel.x, pixel.y + 1);
      };
      drawMarker(start, "#7df8cf", "S");
      drawMarker(goal, "#ffb85b", "G");

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
      ctx.fillStyle = "#43d9ff";
      ctx.beginPath();
      ctx.moveTo(13, 0); ctx.lineTo(5, -5); ctx.lineTo(5, 5); ctx.closePath(); ctx.fill();
      ctx.fillStyle = "#071016";
      ctx.fillRect(-7, -11, 5, 4); ctx.fillRect(-7, 7, 5, 4); ctx.fillRect(3, -11, 5, 4); ctx.fillRect(3, 7, 5, 4);
      ctx.restore();

      frame = window.requestAnimationFrame(draw);
    };
    frame = window.requestAnimationFrame(draw);
    return () => window.cancelAnimationFrame(frame);
  }, [executionCursor, executionResult, inflation, layer, progress, resultList, start, goal]);

  const plan = () => {
    if (resultList.length && progress < maxSteps) {
      setRunning((value) => !value);
      setNotice(running ? "推演已暂停" : "继续推演搜索过程");
      return;
    }
    const selected = (["astar", "dijkstra", "rrt"] as PlannerKey[]).filter((key) => enabled[key]);
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
    setNotice("推演中：亮环表示 PICK，微光节点表示 GENERATE");
  };

  const placePoint = (event: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = event.currentTarget;
    const rect = canvas.getBoundingClientRect();
    const point = {
      x: Math.max(1, Math.min(COLS - 2, Math.floor(((event.clientX - rect.left) / rect.width) * COLS))),
      y: Math.max(1, Math.min(ROWS - 2, Math.floor(((event.clientY - rect.top) / rect.height) * ROWS))),
    };
    const field = makeCostField(dynamicObstaclesAt(performance.now()), inflation);
    if (field.blocked.has(`${point.x},${point.y}`)) {
      setNotice("该栅格属于障碍或膨胀区，请选择自由空间");
      return;
    }
    clearRun();
    if (placement === "start") {
      setStart(point);
      setPlacement("goal");
      setNotice("起点已设置，现在请选择终点");
    } else {
      setGoal(point);
      setPlacement("start");
      setNotice("终点已设置，可以启动路径规划");
    }
  };

  const resetPoints = () => {
    clearRun();
    setStart(DEFAULT_START);
    setGoal(DEFAULT_GOAL);
    setPlacement("start");
    setNotice("起终点已重置");
  };

  const visibleStep = (result: PlannerResult) => result.steps[Math.min(Math.floor(progress), result.steps.length) - 1];

  return (
    <main className="lab-page">
      <header className="topbar">
        <div className="brand-lockup">
          <span className="brand-mark">06</span>
          <div><p>具身智能 · 路径规划</p><strong>PATH LAB</strong></div>
        </div>
        <div className="status-strip"><span className="live-dot" /> 仿真环境在线 <b>20 Hz</b></div>
      </header>

      <section className="hero-copy">
        <div>
          <p className="eyebrow">GLOBAL SEARCH / LOCAL REACTION</p>
          <h1>让搜索过程，<em>看得见。</em></h1>
        </div>
        <p>从障碍膨胀到代价传播，再到 Open Set 的生成与 Pick。动态障碍持续运动，路径颜色对应不同规划器。</p>
      </section>

      <section className="lab-grid">
        <aside className="control-panel panel">
          <div className="panel-heading"><span>01</span><div><p>MAP LAYER</p><h2>地图图层</h2></div></div>
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

          <div className="panel-heading compact"><span>02</span><div><p>MISSION POINTS</p><h2>任务点</h2></div></div>
          <div className="point-readout">
            <button type="button" className={placement === "start" ? "selected" : ""} onClick={() => setPlacement("start")}><i className="start-swatch" /> 起点 <b>{formatCell(start)}</b></button>
            <button type="button" className={placement === "goal" ? "selected" : ""} onClick={() => setPlacement("goal")}><i className="goal-swatch" /> 终点 <b>{formatCell(goal)}</b></button>
          </div>
          <button className="reset-button" type="button" onClick={resetPoints}>重置起终点</button>

          <div className="panel-heading compact"><span>03</span><div><p>ALGORITHMS</p><h2>规划器</h2></div></div>
          <div className="algorithm-list">
            {(["astar", "dijkstra", "rrt"] as PlannerKey[]).map((key) => (
              <label key={key}>
                <input type="checkbox" checked={enabled[key]} onChange={() => { clearRun(); setEnabled((current) => ({ ...current, [key]: !current[key] })); }} />
                <i className={key} />{NAMES[key]}<small>{key === "astar" ? "f=g+h" : key === "dijkstra" ? "g only" : "sample"}</small>
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
            <div><span className="pulse" /> WORLD_01 / {layer.toUpperCase()}</div>
            <p className={running ? "is-running" : ""}>{running ? `SEARCH ${Math.round((progress / Math.max(maxSteps, 1)) * 100)}%` : executing ? "ROBOT EXECUTING" : placement === "start" ? "SET START" : "SET GOAL"}</p>
          </div>
          <canvas ref={canvasRef} width={CANVAS_WIDTH} height={CANVAS_HEIGHT} onClick={placePoint} aria-label="交互式路径规划地图，单击放置起点和终点" />
          <div className="progress-rail"><span style={{ width: `${Math.min(100, (progress / Math.max(maxSteps, 1)) * 100)}%` }} /></div>
          <div className="map-legend">
            <span><i className="wall" />边界 / 静态障碍</span>
            <span><i className="dynamic" />动态障碍</span>
            <span><i className="inflated" />膨胀与代价</span>
            <span><i className="pick-ring" />当前 PICK</span>
            <span className="legend-note">{notice}</span>
          </div>
        </section>

        <aside className="telemetry-panel panel">
          <div className="panel-heading"><span>04</span><div><p>SEARCH TELEMETRY</p><h2>搜索遥测</h2></div></div>
          {(["astar", "dijkstra", "rrt"] as PlannerKey[]).map((key) => {
            const result = results[key];
            const visible = result ? Math.min(Math.floor(progress), result.steps.length) : 0;
            const done = result && visible >= result.steps.length;
            return (
              <div key={key} className={`telemetry-card ${key}`}>
                <div className="telemetry-title"><p>{NAMES[key]}</p><span style={{ background: COLORS[key] }} /></div>
                <strong>{!enabled[key] ? "OFF" : !result ? "READY" : done ? result.found ? "PATH" : "FAIL" : "SEARCH"}</strong>
                <small>{DESCRIPTIONS[key]}</small>
                <dl>
                  <div><dt>PICK</dt><dd>{visible}</dd></div>
                  <div><dt>PATH</dt><dd>{done && result?.found ? `${(result.pathLength * 0.25).toFixed(1)}m` : "—"}</dd></div>
                </dl>
              </div>
            );
          })}
          <div className="event-log">
            <p>EVENT STREAM</p>
            {resultList.length === 0 ? (
              <><span><b>00</b> 等待规划任务</span><span><b>01</b> 代价地图已就绪</span><span><b>02</b> 动态感知运行中</span></>
            ) : resultList.map((result) => {
              const step = visibleStep(result);
              return (
                <span key={result.key}>
                  <b style={{ color: COLORS[result.key] }}>{NAMES[result.key]}</b>
                  {step ? `PICK ${formatCell(step.pick)} → GEN ${step.generated.length}` : "初始化 OPEN SET"}
                </span>
              );
            })}
            {executing && <span><b>BOT</b> 局部闭环跟踪中</span>}
          </div>
        </aside>
      </section>

      <section className="method-strip">
        <article><span>01</span><div><h3>GENERATE</h3><p>扩展可达邻居 / 随机采样</p></div></article>
        <article><span>02</span><div><h3>PICK</h3><p>按 g、f 或最近邻选择</p></div></article>
        <article><span>03</span><div><h3>CHECK</h3><p>碰撞检测与代价查询</p></div></article>
        <article><span>04</span><div><h3>EXECUTE</h3><p>局部闭环持续响应动态障碍</p></div></article>
      </section>
    </main>
  );
}
