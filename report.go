package main

import (
	"encoding/json"
	"os"
	"strings"
)

// 以下所有结构仅用于把运行结果序列化进静态 HTML。
type xyPoint struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

type constraintDTO struct {
	ID      string  `json:"id"`
	Kind    string  `json:"kind"`
	I       int     `json:"i"`
	J       int     `json:"j"`
	Culprit bool    `json:"culprit"`
	Zx      float64 `json:"zx"`
	Zy      float64 `json:"zy"`
}

type runDTO struct {
	Name              string          `json:"name"`
	Status            string          `json:"status"`
	OptStatus         string          `json:"optStatus"`
	CulpritID         string          `json:"culpritId"`
	FinalCost         float64         `json:"finalCost"`
	Iterations        int             `json:"iterations"`
	MaxOdomPosErr     float64         `json:"maxOdomPosErr"`
	MaxOdomErrRatio   float64         `json:"maxOdomErrRatio"`
	MaxOdomAngErr     float64         `json:"maxOdomAngErr"`
	SelfIntersections int             `json:"selfIntersections"`
	Notes             []string        `json:"notes"`
	Constraints       []constraintDTO `json:"constraints"`
	Before            []xyPoint       `json:"before"`
	After             []xyPoint       `json:"after"`
	Truth             []xyPoint       `json:"truth"`
	Costs             []float64       `json:"costs"`
	LoopScores        []LoopScore     `json:"loopScores"`
}

func toDTO(r ScenarioRun) runDTO {
	d := r.Diagnosis
	dto := runDTO{
		Name:              r.Scenario.Graph.Name,
		Status:            d.Status,
		OptStatus:         d.OptStatus,
		CulpritID:         d.CulpritID,
		FinalCost:         d.FinalCost,
		Iterations:        r.Optimized.Iter,
		MaxOdomPosErr:     d.MaxOdomPosErr,
		MaxOdomErrRatio:   d.MaxOdomErrRatio,
		MaxOdomAngErr:     d.MaxOdomAngErr,
		SelfIntersections: d.SelfIntersections,
		Notes:             d.Notes,
		Costs:             r.Optimized.Costs,
		LoopScores:        d.LoopScores,
	}
	conv := func(nodes []Pose) []xyPoint {
		out := make([]xyPoint, len(nodes))
		for i, p := range nodes {
			out[i] = xyPoint{X: p.X, Y: p.Y}
		}
		return out
	}
	dto.Before = conv(r.Initial)
	dto.After = conv(r.Optimized.Nodes)
	dto.Truth = conv(r.Scenario.Truth)
	for k := range r.Scenario.Graph.Constraints {
		c := r.Scenario.Graph.Constraints[k]
		dto.Constraints = append(dto.Constraints, constraintDTO{
			ID: c.ID, Kind: string(c.Kind), I: c.I, J: c.J,
			Culprit: c.ID == d.CulpritID, Zx: c.Z.X, Zy: c.Z.Y,
		})
	}
	if dto.Notes == nil {
		dto.Notes = []string{}
	}
	return dto
}

func writeHTMLReport(path string, runs []ScenarioRun) error {
	dtos := make([]runDTO, len(runs))
	for i, r := range runs {
		dtos[i] = toDTO(r)
	}
	data, err := json.MarshalIndent(dtos, "", "  ")
	if err != nil {
		return err
	}
	// json.Marshal 默认把 < > & 转义为 \u003c 等，因此数据中不会出现
	// "</script>" 序列；此处再兜底一次，保证内嵌脚本永不被截断。
	safe := strings.ReplaceAll(string(data), "</", `<\/`)

	html := `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>位姿图优化诊断报告</title>
<style>
  :root {
    --bg:#f5f6f8; --card:#ffffff; --ink:#1d2433; --muted:#5c6675;
    --ok:#1a9e5c; --bad:#d83a34; --warn:#d97706; --accent:#2563eb;
    --border:#dfe3e8;
  }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--ink);
    font-family:-apple-system,"Segoe UI",Roboto,"PingFang SC","Microsoft YaHei",sans-serif; }
  header { padding:28px 32px 8px; }
  h1 { margin:0 0 6px; font-size:24px; }
  header p { margin:0; color:var(--muted); font-size:14px; }
  main { padding:16px 32px 48px; display:grid; gap:24px; }
  .card { background:var(--card); border:1px solid var(--border); border-radius:12px;
    padding:20px 24px; box-shadow:0 1px 3px rgba(20,30,50,.06); }
  .card h2 { margin:0 0 4px; font-size:18px; }
  .badge { display:inline-block; padding:2px 10px; border-radius:999px;
    font-size:12px; font-weight:600; color:#fff; margin-left:8px; vertical-align:middle; }
  .badge.ok { background:var(--ok); }
  .badge.bad_loop { background:var(--bad); }
  .badge.local_minimum { background:var(--warn); }
  .layout { display:grid; grid-template-columns:minmax(360px,1fr) 320px; gap:24px; margin-top:12px; }
  @media (max-width:860px){ .layout{ grid-template-columns:1fr; } }
  .panel { border:1px solid var(--border); border-radius:10px; padding:14px 16px; }
  .panel h3 { margin:0 0 8px; font-size:13px; color:var(--muted);
    text-transform:uppercase; letter-spacing:.06em; }
  canvas { width:100%; display:block; background:#fcfcfd; border-radius:8px; }
  .toggles { display:flex; gap:14px; flex-wrap:wrap; margin:10px 0 4px; font-size:13px; }
  .toggles label { display:flex; gap:5px; align-items:center; cursor:pointer; }
  .swatch { width:14px; height:3px; border-radius:2px; display:inline-block; }
  .kv { display:grid; grid-template-columns:auto 1fr; gap:4px 12px; font-size:13px; margin:0; }
  .kv dt { color:var(--muted); } .kv dd { margin:0; text-align:right; font-variant-numeric:tabular-nums; }
  .notes { margin:10px 0 0; padding-left:18px; font-size:13px; line-height:1.55; }
  .notes li { margin-bottom:4px; }
  table { width:100%; border-collapse:collapse; font-size:12.5px; margin-top:8px; }
  th,td { border-bottom:1px solid var(--border); padding:5px 6px; text-align:right;
    font-variant-numeric:tabular-nums; }
  th:first-child,td:first-child { text-align:left; }
  tr.culprit td { background:#fdeceb; font-weight:600; }
  .culprit-tag { color:var(--bad); font-weight:700; }
  .legend-note { font-size:12px; color:var(--muted); margin-top:6px; }
  .curves { display:grid; grid-template-columns:1fr 1fr; gap:16px; }
  footer { color:var(--muted); font-size:12px; padding:0 32px 32px; }
</style>
</head>
<body>
<header>
  <h1>简化 2D 位姿图优化（Go 标准库实现）— 诊断报告</h1>
  <p>Levenberg–Marquardt 非线性最小二乘 · 角度流形 wrap · Huber 鲁棒核 + leave-one-out 错误回环定位 · 自穿越/残差畸形的局部最优检测。本文件数据与逻辑全部内嵌，可离线打开。</p>
</header>
<main id="root"></main>
<footer>优化前轨迹（灰虚线）＝里程计递推的漂移初值；绿线＝真值；蓝线＝优化结果；蓝色细虚线为正确回环，红色粗虚线为被判定错误的回环。</footer>
<script>
"use strict";
const RUNS = __DATA__;

const STATUS_TEXT = {ok:"正常收敛", bad_loop:"检测到错误回环", local_minimum:"陷入不合理局部最优"};

function el(tag, attrs, children) {
  const e = document.createElement(tag);
  if (attrs) for (const k in attrs) {
    if (k === "class") e.className = attrs[k];
    else if (k === "text") e.textContent = attrs[k];
    else e.setAttribute(k, attrs[k]);
  }
  (children || []).forEach(c => e.appendChild(c));
  return e;
}

// 统一的坐标变换：数据坐标 -> 画布像素（y 轴翻转，留边距）
function makeProjector(points, w, h, pad) {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  points.forEach(p => {
    minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
    minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
  });
  const sx = (w - 2*pad) / Math.max(maxX - minX, 1e-9);
  const sy = (h - 2*pad) / Math.max(maxY - minY, 1e-9);
  const s = Math.min(sx, sy);
  const ox = (w - s*(maxX + minX)) / 2;
  const oy = (h + s*(maxY + minY)) / 2;
  return p => [ox + s*p.x, oy - s*p.y];
}

function drawPath(ctx, proj, pts, color, width, dash) {
  ctx.save();
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []);
  ctx.lineJoin = "round"; ctx.lineCap = "round";
  ctx.beginPath();
  pts.forEach((p, i) => {
    const [x, y] = proj(p);
    i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawTrajectory(canvas, run, show) {
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth, cssH = 380;
  canvas.width = cssW * dpr; canvas.height = cssH * dpr;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, cssW, cssH);

  const all = run.before.concat(run.after).concat(run.truth);
  const proj = makeProjector(all, cssW, cssH, 34);

  // 回环约束画在轨迹下层
  run.constraints.forEach(c => {
    if (c.kind !== "loop") return;
    const useAfter = show.layer === "after";
    const pts = useAfter ? run.after : run.before;
    ctx.save();
    if (c.culprit) {
      ctx.strokeStyle = "#d83a34"; ctx.lineWidth = 2.6;
      ctx.setLineDash([9, 5]);
    } else {
      ctx.strokeStyle = "rgba(37,99,235,.55)"; ctx.lineWidth = 1.3;
      ctx.setLineDash([5, 4]);
    }
    ctx.beginPath();
    let [x1, y1] = proj(pts[c.i]), [x2, y2] = proj(pts[c.j]);
    ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    if (c.culprit) {
      // 红色端点标记
      ctx.setLineDash([]); ctx.fillStyle = "#d83a34";
      [pts[c.i], pts[c.j]].forEach(p => {
        const [x, y] = proj(p);
        ctx.beginPath(); ctx.arc(x, y, 4.5, 0, 7); ctx.fill();
      });
    }
    ctx.restore();
  });

  if (show.before) drawPath(ctx, proj, run.before, "#9aa3af", 1.4, [6, 5]);
  if (show.truth)  drawPath(ctx, proj, run.truth,  "rgba(26,158,92,.8)", 1.6, [2, 3]);
  if (show.layer === "after") drawPath(ctx, proj, run.after, "#2563eb", 2.4, []);

  // 起点/终点
  const pts = show.layer === "after" ? run.after : run.before;
  let [sx, sy] = proj(pts[0]);
  ctx.fillStyle = "#1d2433";
  ctx.beginPath(); ctx.arc(sx, sy, 4, 0, 7); ctx.fill();
}

function drawCostCurve(canvas, costs) {
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth, cssH = 130;
  canvas.width = cssW * dpr; canvas.height = cssH * dpr;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, cssW, cssH);
  if (costs.length < 2) return;
  const pad = 30;
  const lc = costs.map(v => Math.log10(Math.max(v, 1e-20)));
  const lo = Math.min(...lc), hi = Math.max(...lc);
  const X = i => pad + i * (cssW - 2*pad) / (costs.length - 1);
  const Y = v => cssH - pad - (v - lo) * (cssH - 2*pad) / Math.max(hi - lo, 1e-9);
  ctx.strokeStyle = "#2563eb"; ctx.lineWidth = 1.8;
  ctx.beginPath();
  lc.forEach((v, i) => { const x = X(i), y = Y(v); i ? ctx.lineTo(x,y) : ctx.moveTo(x,y); });
  ctx.stroke();
  ctx.fillStyle = "#5c6675"; ctx.font = "11px sans-serif";
  ctx.fillText("log₁₀ cost", 6, 14);
  ctx.fillText("10^" + hi.toFixed(1), 6, Y(hi));
  ctx.fillText("10^" + lo.toFixed(1), 6, Y(lo) + 4);
  ctx.fillText("迭代 " + (costs.length - 1), cssW - 64, cssH - 8);
}

function buildCard(run) {
  const badge = el("span", {class:"badge " + run.status, text: STATUS_TEXT[run.status] || run.status});
  const h2 = el("h2", {text:"场景：" + run.name}, [badge]);

  const canvas = el("canvas", {height:"380"});
  const costCanvas = el("canvas", {height:"130"});

  const show = {before:true, truth:true, layer:"after"};
  let layerTextNode = null;
  function toggle(kind) {
    if (kind === "layer") {
      show.layer = show.layer === "after" ? "before" : "after";
      if (layerTextNode)
        layerTextNode.textContent = show.layer === "after" ? "主图层：优化后" : "主图层：优化前";
    } else {
      show[kind] = !show[kind];
    }
    drawTrajectory(canvas, run, show);
  }

  const mkLabel = (text, color, onclick) => {
    const lab = el("label", {}, []);
    lab.appendChild(el("span", {class:"swatch", style:"background:" + color}));
    const txt = el("span", {text});
    lab.appendChild(txt);
    if (onclick) {
      lab.style.cursor = "pointer";
      lab.addEventListener("click", ev => { ev.preventDefault(); onclick(txt); });
    }
    return {lab, txt};
  };
  const tBefore = mkLabel("优化前（漂移初值）", "#9aa3af", () => toggle("before"));
  const tTruth  = mkLabel("真值", "rgba(26,158,92,.8)", () => toggle("truth"));
  const tLayer  = mkLabel("主图层：优化后", "#2563eb", () => toggle("layer"));
  layerTextNode = tLayer.txt;
  const toggles = el("div", {class:"toggles"}, [tBefore.lab, tTruth.lab, tLayer.lab]);

  const plotPanel = el("div", {class:"panel"}, [
    el("h3", {text:"轨迹对比"}), canvas, toggles,
    el("div", {class:"legend-note",
      text:"红点与红色粗虚线＝定位出的错误回环；蓝细虚线＝正确回环。点击“主图层”可切换回环所附着的轨迹版本。"}),
  ]);

  const kv = el("dl", {class:"kv"}, []);
  const addKV = (k, v) => { kv.appendChild(el("dt",{text:k})); kv.appendChild(el("dd",{text:v})); };
  addKV("LM 状态", run.optStatus === "converged" ? "数值收敛" : "达迭代上限");
  addKV("接受迭代数", String(run.iterations));
  addKV("最终代价 ½Σχ²", run.finalCost.toExponential(3));
  addKV("里程计最大平移残差", run.maxOdomPosErr.toFixed(4) + " m（边长 " +
    (100*run.maxOdomErrRatio).toFixed(1) + "%）");
  addKV("里程计最大角度残差", run.maxOdomAngErr.toFixed(4) + " rad");
  addKV("非相邻段自交数", String(run.selfIntersections));
  if (run.culpritId)
    addKV("错误回环", run.culpritId);

  const notes = el("ul", {class:"notes"}, run.notes.map(n => el("li", {text:n})));

  // 回环评分表
  let tableWrap = null;
  if (run.loopScores.length) {
    const table = el("table", {}, []);
    const head = el("tr", {}, ["回环","节点 i→j","鲁棒残差 r","留一 χ²","删后代价","可疑度"]
      .map(t => el("th", {text:t})));
    table.appendChild(head);
    run.loopScores.forEach(s => {
      const tr = el("tr", {class: s.id === run.culpritId ? "culprit" : ""}, []);
      const nameTd = el("td", {}, [document.createTextNode(s.id)]);
      if (s.id === run.culpritId)
        nameTd.appendChild(el("span", {class:"culprit-tag", text:" ← 罪魁"}));
      tr.appendChild(nameTd);
      tr.appendChild(el("td", {text:s.i + "→" + s.j}));
      tr.appendChild(el("td", {text:s.residualNorm.toFixed(3)}));
      tr.appendChild(el("td", {text:s.ownResidual.toFixed(2)}));
      tr.appendChild(el("td", {text:s.removedCost.toFixed(3)}));
      tr.appendChild(el("td", {text:s.suspicion.toFixed(2)}));
      table.appendChild(tr);
    });
    tableWrap = el("div", {class:"panel"}, [el("h3", {text:"回环可疑度评分（leave-one-out）"}), table]);
  }

  const infoPanel = el("div", {class:"panel"}, [
    el("h3", {text:"诊断结论"}), kv, notes,
  ]);

  const curvePanel = el("div", {class:"panel"}, [
    el("h3", {text:"代价收敛曲线（每次被接受迭代）"}), costCanvas,
  ]);

  const children = [
    h2,
    el("div", {class:"layout"}, [plotPanel, infoPanel]),
  ];
  if (tableWrap) children.push(tableWrap);
  children.push(curvePanel);

  const card = el("section", {class:"card"}, children);
  requestAnimationFrame(() => {
    drawTrajectory(canvas, run, show);
    drawCostCurve(costCanvas, run.costs);
  });
  window.addEventListener("resize", () => {
    drawTrajectory(canvas, run, show);
    drawCostCurve(costCanvas, run.costs);
  });
  return card;
}

const root = document.getElementById("root");
RUNS.forEach(r => root.appendChild(buildCard(r)));
</script>
</body>
</html>
`

	html = strings.ReplaceAll(html, "__DATA__", safe)
	return os.WriteFile(path, []byte(html), 0o644)
}
