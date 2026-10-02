package graphslam

// reportHTMLPrefix/Suffix 构成一个完全自包含的静态 HTML：
// 数据以 window.REPORT_DATA 内嵌，全部图形用原生 SVG + JavaScript 绘制，
// 不引用任何第三方库、字体或网络资源。

const reportHTMLPrefix = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>位姿图优化报告</title>
<style>
  :root {
    color-scheme: light;
    --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e;
    --muted: #898781; --grid: #e1e0d9; --axis: #c3c2b7; --border: rgba(11,11,11,.10);
    --s-initial: #eb6834; --s-opt: #2a78d6; --s-truth: #1baf7a;
    --good-loop: #4a3aa7; --critical: #d03b3b; --good: #0ca30c; --warning: #b97d00;
    --s-trapped: #e87ba4;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      color-scheme: dark;
      --page: #0d0d0d; --surface: #1a1a19; --ink: #ffffff; --ink-2: #c3c2b7;
      --muted: #898781; --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,.10);
      --s-initial: #d95926; --s-opt: #3987e5; --s-truth: #199e70;
      --good-loop: #9085e9; --critical: #e66767; --good: #0ca30c; --warning: #d99a1a;
      --s-trapped: #d55181;
    }
  }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--page); color:var(--ink);
    font:14px/1.55 system-ui,-apple-system,"Segoe UI",sans-serif; }
  header { padding:28px 24px 8px; max-width:1180px; margin:0 auto; }
  h1 { font-size:22px; margin:0 0 4px; }
  .sub { color:var(--ink-2); font-size:13px; }
  main { max-width:1180px; margin:0 auto; padding:12px 24px 48px; }
  .card { background:var(--surface); border:1px solid var(--border); border-radius:12px;
    padding:20px 22px; margin:18px 0; }
  h2 { font-size:17px; margin:0 0 4px; }
  .desc { color:var(--ink-2); margin:0 0 12px; }
  .badges { display:flex; flex-wrap:wrap; gap:8px; margin-bottom:14px; }
  .badge { display:inline-flex; align-items:center; gap:6px; border:1px solid var(--border);
    border-radius:999px; padding:3px 11px; font-size:12.5px; color:var(--ink-2); }
  .badge.ok { color:var(--good); border-color:color-mix(in srgb, var(--good) 40%, transparent); }
  .badge.bad { color:var(--critical); border-color:color-mix(in srgb, var(--critical) 45%, transparent); font-weight:600; }
  .grid2 { display:grid; grid-template-columns: minmax(0,1fr) 320px; gap:18px; }
  @media (max-width:900px){ .grid2 { grid-template-columns:1fr; } }
  .panel { border:1px solid var(--border); border-radius:10px; padding:12px; min-width:0; }
  .panel h3 { margin:0 0 8px; font-size:13px; color:var(--ink-2); font-weight:600; }
  svg { width:100%; height:auto; display:block; }
  .legend { display:flex; flex-wrap:wrap; gap:14px; margin-top:8px; font-size:12.5px; color:var(--ink-2); }
  .legend label { display:inline-flex; align-items:center; gap:6px; cursor:pointer; user-select:none; }
  .swatch { width:18px; height:0; border-top:3px solid; display:inline-block; }
  .swatch.dashed { border-top-style:dashed; }
  .stats { width:100%; border-collapse:collapse; font-size:12.5px; font-variant-numeric:tabular-nums; }
  .stats td { padding:3px 0; } .stats td:last-child { text-align:right; color:var(--ink); }
  table.data { width:100%; border-collapse:collapse; margin-top:12px; font-size:12.5px; }
  table.data th, table.data td { text-align:left; padding:7px 9px; border-bottom:1px solid var(--grid); }
  table.data th { color:var(--muted); font-weight:600; font-size:12px; }
  table.data td.num, table.data th.num { text-align:right; font-variant-numeric:tabular-nums; }
  .tag { display:inline-flex; gap:5px; align-items:center; font-weight:600; }
  .tag.bad { color:var(--critical); } .tag.ok { color:var(--good); }
  .issues { margin:10px 0 0; padding-left:0; list-style:none; }
  .issues li { color:var(--warning); font-size:12.5px; padding:2px 0; }
  .tooltip { position:fixed; pointer-events:none; z-index:10; background:var(--surface);
    border:1px solid var(--axis); border-radius:8px; padding:7px 10px; font-size:12px;
    box-shadow:0 4px 16px rgba(0,0,0,.18); display:none; max-width:260px; }
  .tooltip b { font-variant-numeric:tabular-nums; }
  .hidden-layer { display:none !important; }
  .footer-note { color:var(--muted); font-size:12px; margin-top:24px; }
  text { font-family:system-ui,-apple-system,"Segoe UI",sans-serif; }
</style>
</head>
<body>
<header>
  <h1>位姿图优化对比报告</h1>
  <div class="sub">Levenberg–Marquardt（自研稠密 Cholesky）· 错误回环留一法定位 · 局部最优几何检测 · 生成于 <span id="ts"></span></div>
</header>
<main id="root"></main>
<div class="tooltip" id="tip"></div>
<script>
"use strict";
window.REPORT_DATA = /*@DATA@*/null;
const D = window.REPORT_DATA;
document.getElementById('ts').textContent = D.generatedAt;
const SVGNS = "http://www.w3.org/2000/svg";
const tip = document.getElementById('tip');
function el(tag, attrs, parent) {
  const n = document.createElementNS(SVGNS, tag);
  for (const k in attrs) n.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(n);
  return n;
}
function showTip(html, x, y) { tip.innerHTML = html; tip.style.display = 'block';
  tip.style.left = Math.min(x + 14, innerWidth - 270) + 'px'; tip.style.top = (y + 14) + 'px'; }
function hideTip() { tip.style.display = 'none'; }
function fmt(v) { return (Math.round(v*1000)/1000).toString(); }

// ---- 轨迹图 ----
function trajectory(s) {
  const W = 720, H = 520, M = {l:46,r:14,t:14,b:30};
  const all = s.initial.concat(s.optimized).concat(s.truth || []).concat(s.trapped || []);
  let minX=1e9,maxX=-1e9,minY=1e9,maxY=-1e9;
  for (const p of all) { minX=Math.min(minX,p[0]); maxX=Math.max(maxX,p[0]);
    minY=Math.min(minY,p[1]); maxY=Math.max(maxY,p[1]); }
  const pad = Math.max((maxX-minX),(maxY-minY))*0.06 + 1e-6;
  minX-=pad; maxX+=pad; minY-=pad; maxY+=pad;
  const iw=W-M.l-M.r, ih=H-M.t-M.b;
  const sc=Math.min(iw/(maxX-minX), ih/(maxY-minY));
  const ox=M.l+(iw-(maxX-minX)*sc)/2, oy=M.t+(ih-(maxY-minY)*sc)/2;
  const X=x=>ox+(x-minX)*sc, Y=y=>oy+(maxY-y)*sc;

  const svg = el('svg', {viewBox:@BT@0 0 ${W} ${H}@BT@, role:'img'});
  // 网格与刻度
  for (let g=0; g<=5; g++) {
    const gx = M.l + iw*g/5, gy = M.t + ih*g/5;
    el('line',{x1:gx,y1:M.t,x2:gx,y2:M.t+ih,stroke:'var(--grid)','stroke-width':1},svg);
    el('line',{x1:M.l,y1:gy,x2:M.l+iw,y2:gy,stroke:'var(--grid)','stroke-width':1},svg);
    const tx = minX+(maxX-minX)*g/5, ty = maxY-(maxY-minY)*g/5;
    const t1=el('text',{x:gx,y:H-8,'text-anchor':'middle','font-size':10,fill:'var(--muted)'},svg); t1.textContent=fmt(tx);
    const t2=el('text',{x:M.l-6,y:gy+3,'text-anchor':'end','font-size':10,fill:'var(--muted)'},svg); t2.textContent=fmt(ty);
  }
  el('line',{x1:M.l,y1:M.t,x2:M.l,y2:M.t+ih,stroke:'var(--axis)','stroke-width':1.2},svg);
  el('line',{x1:M.l,y1:M.t+ih,x2:M.l+iw,y2:M.t+ih,stroke:'var(--axis)','stroke-width':1.2},svg);

  function poly(layer, poses, color, width, dash) {
    const g = el('g',{class:'layer layer-'+layer},svg);
    let d='';
    poses.forEach((p,i)=>{ d+=(i?'L':'M')+fmt(X(p[0]))+' '+fmt(Y(p[1]))+' '; });
    el('path',{d:d,fill:'none',stroke:color,'stroke-width':width,'stroke-linejoin':'round',
      'stroke-linecap':'round',...(dash?{'stroke-dasharray':dash}:{})},g);
    poses.forEach((p,i)=>{
      el('circle',{cx:X(p[0]),cy:Y(p[1]),r:3,fill:color,stroke:'var(--surface)','stroke-width':1},g);
      const hit=el('circle',{cx:X(p[0]),cy:Y(p[1]),r:10,fill:'transparent'},g);
      const layerName = {initial:'优化前', opt:'优化后（恢复解）', truth:'真值', trapped:'坏初值困驻点'}[layer];
      hit.addEventListener('mousemove',ev=>showTip(
        @BT@<b>${layerName} 节点 ${i}</b><br>x=${fmt(p[0])}, y=${fmt(p[1])}<br>θ=${fmt(p[2])} rad@BT@,
        ev.clientX, ev.clientY));
      hit.addEventListener('mouseleave',hideTip);
    });
    return g;
  }
  if (s.truth) poly('truth', s.truth, 'var(--s-truth)', 2, '6 5');
  poly('initial', s.initial, 'var(--s-initial)', 2.5, null);
  if (s.trapped) poly('trapped', s.trapped, 'var(--s-trapped)', 2, '2 4');
  poly('opt', s.optimized, 'var(--s-opt)', 2.5, null);

  // 回环边（画在优化后位姿上）
  const eg = el('g',{class:'layer layer-loops'},svg);
  s.edges.filter(e=>e.kind==='loop').forEach(e=>{
    const a=s.optimized[e.i], b=s.optimized[e.j];
    const col = e.bad ? 'var(--critical)' : 'var(--good-loop)';
    const ln = el('line',{x1:X(a[0]),y1:Y(a[1]),x2:X(b[0]),y2:Y(b[1]),
      stroke:col,'stroke-width':e.bad?3:1.6,'stroke-dasharray':e.bad?'8 4':'4 4',
      'stroke-linecap':'round'},eg);
    const hit = el('line',{x1:X(a[0]),y1:Y(a[1]),x2:X(b[0]),y2:Y(b[1]),
      stroke:'transparent','stroke-width':12},eg);
    const label = e.bad ? @BT@⚠ 错误回环 ${e.i}↔${e.j}@BT@ : @BT@正确回环 ${e.i}↔${e.j}@BT@;
    const handler = ev => showTip(@BT@<b style="color:${e.bad?'var(--critical)':'var(--good-loop)'}">${label}</b>@BT@+
      @BT@<br>归一化残差 r=${fmt(e.residual)}σ@BT@, ev.clientX, ev.clientY);
    ln.addEventListener('mousemove',handler); hit.addEventListener('mousemove',handler);
    ln.addEventListener('mouseleave',hideTip); hit.addEventListener('mouseleave',hideTip);
    if (e.bad) {
      const mx=(X(a[0])+X(b[0]))/2, my=(Y(a[1])+Y(b[1]))/2;
      el('circle',{cx:mx,cy:my,r:9,fill:'var(--critical)','fill-opacity':0.18,
        stroke:'var(--critical)','stroke-width':1.5},eg);
      const t=el('text',{x:mx,y:my+3.5,'text-anchor':'middle','font-size':10,
        fill:'var(--critical)','font-weight':700},eg); t.textContent='!';
    }
  });

  // 几何问题标记：自穿越交点
  if (s.geometry) s.geometry.issues.filter(q=>q.kind==='self-intersection').forEach(q=>{
    const a=s.optimized[q.a], b=s.optimized[q.a+1], c=s.optimized[q.b], d2=s.optimized[q.b+1];
    const p=segCross(a,b,c,d2);
    if (p) {
      const g=el('g',{class:'layer layer-issues'},svg);
      el('rect',{x:X(p[0])-6,y:Y(p[1])-6,width:12,height:12,
        fill:'var(--warning)','fill-opacity':0.2,stroke:'var(--warning)',
        'stroke-width':1.5,transform:@BT@rotate(45 ${X(p[0])} ${Y(p[1])})@BT@},g);
    }
  });
  return svg;
}
function segCross(a,b,c,d) {
  const den=(a[0]-b[0])*(c[1]-d[1])-(a[1]-b[1])*(c[0]-d[0]);
  if (Math.abs(den)<1e-12) return null;
  const t=((a[0]-c[0])*(c[1]-d[1])-(a[1]-c[1])*(c[0]-d[0]))/den;
  return [a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])];
}

// ---- 收敛曲线 ----
function convergence(s) {
  const W=300,H=150,M={l:42,r:10,t:12,b:24};
  const hist = s.chi2History, max=Math.max(...hist), min=Math.min(...hist);
  const iw=W-M.l-M.r, ih=H-M.t-M.b;
  // χ² 用对数纵轴，能同时看清首步大跳变与尾部平台。
  const lo=Math.log(Math.max(min*0.8,1e-9)), hi=Math.log(max*1.05+1e-9);
  const X=i=>M.l+iw*i/Math.max(hist.length-1,1);
  const Y=v=>M.t+ih-(Math.log(Math.max(v,1e-9))-lo)/(hi-lo||1)*ih;
  const svg=el('svg',{viewBox:@BT@0 0 ${W} ${H}@BT@});
  for(let g=0;g<=4;g++){
    const gy=M.t+ih*g/4;
    el('line',{x1:M.l,y1:gy,x2:M.l+iw,y2:gy,stroke:'var(--grid)','stroke-width':1},svg);
    const val=Math.exp(hi-(hi-lo)*g/4);
    const t=el('text',{x:M.l-5,y:gy+3,'text-anchor':'end','font-size':9.5,fill:'var(--muted)'},svg);
    t.textContent = val>=100? Math.round(val): fmt(val);
  }
  const tx1=el('text',{x:M.l,y:H-6,'font-size':9.5,fill:'var(--muted)'},svg); tx1.textContent='0';
  const tx2=el('text',{x:M.l+iw,y:H-6,'text-anchor':'end','font-size':9.5,fill:'var(--muted)'},svg);
  tx2.textContent=String(hist.length-1)+' 次迭代';
  let d=''; hist.forEach((v,i)=>{ d+=(i?'L':'M')+fmt(X(i))+' '+fmt(Y(v)); });
  el('path',{d,fill:'none',stroke:'var(--s-opt)','stroke-width':2},svg);
  hist.forEach((v,i)=>el('circle',{cx:X(i),cy:Y(v),r:2.5,fill:'var(--s-opt)'},svg));
  const dot=el('circle',{r:4,fill:'var(--s-initial)',display:'none'},svg);
  svg.addEventListener('mousemove',ev=>{
    const rect=svg.getBoundingClientRect();
    const mx=(ev.clientX-rect.left)/rect.width*W;
    const i=Math.max(0,Math.min(hist.length-1,Math.round((mx-M.l)/iw*(hist.length-1))));
    dot.setAttribute('cx',X(i)); dot.setAttribute('cy',Y(hist[i])); dot.setAttribute('display','');
    showTip(@BT@<b>第 ${i} 次迭代</b><br>χ² = ${fmt(hist[i])}@BT@,ev.clientX,ev.clientY);
  });
  svg.addEventListener('mouseleave',()=>{dot.setAttribute('display','none');hideTip();});
  return svg;
}

// ---- 渲染 ----
D.scenarios.forEach(s=>{
  const card=document.createElement('section'); card.className='card';
  const h=document.createElement('h2'); h.textContent=s.name; card.appendChild(h);
  const d=document.createElement('p'); d.className='desc'; d.textContent=s.description; card.appendChild(d);

  const badges=document.createElement('div'); badges.className='badges';
  const b1=document.createElement('span');
  b1.className='badge '+(s.converged?'ok':'bad');
  b1.textContent=(s.converged?'✓ 数值收敛':'✗ 未收敛')+' · '+s.stopReason+' · '+s.iterations+' 次迭代';
  badges.appendChild(b1);
  if (s.geometry) {
    const oc = s.geometry.outcome;
    const b2=document.createElement('span');
    const ok = oc==='converged-ok';
    b2.className='badge '+(ok?'ok':'bad');
    const ocText = {'converged-ok':'✓ 结局可信（收敛且几何合理）',
      'local-minimum-trap':'⚠ 局部最优陷阱（已到驻点但几何/残差不可信）',
      'not-converged':'✗ 未真正收敛'}[oc] || oc;
    b2.textContent=ocText+'（几何问题 '+s.geometry.issues.length+' 处，χ²/边 '+fmt(s.geometry.chi2PerDof)+'）';
    badges.appendChild(b2);
  }
  if (s.multiStart && s.multiStart.some(m=>!m.plausible)) {
    const b3=document.createElement('span');
    b3.className='badge bad';
    b3.textContent='⚠ 多起点中有 '+s.multiStart.filter(m=>!m.plausible).length+
      ' 个初值落入陷阱，已选择几何可信的最低代价解恢复';
    badges.appendChild(b3);
  }
  card.appendChild(badges);

  const grid=document.createElement('div'); grid.className='grid2';
  const p1=document.createElement('div'); p1.className='panel';
  const t1=document.createElement('h3'); t1.textContent='轨迹对比（悬停节点/回环查看详情）'; p1.appendChild(t1);
  const svgWrap=document.createElement('div');
  const svg=trajectory(s); svgWrap.appendChild(svg); p1.appendChild(svgWrap);

  const layers=[['initial','优化前（坏初值/漂移）','var(--s-initial)',false],
    ['opt','优化后（多起点恢复解）','var(--s-opt)',false],
    ['truth','真值（参考）','var(--s-truth)',true],
    ['loops','回环约束','var(--good-loop)',true],
    ['issues','自穿越标记','var(--warning)',true]];
  if (s.trapped) layers.splice(2,0,['trapped','坏初值直接优化的困驻点','var(--s-trapped)',true]);
  const legend=document.createElement('div'); legend.className='legend';
  layers.forEach(([key,name,col,dash])=>{
    const lab=document.createElement('label');
    const cb=document.createElement('input'); cb.type='checkbox'; cb.checked=true;
    cb.addEventListener('change',()=>{
      svg.querySelectorAll('.layer-'+key).forEach(n=>n.classList.toggle('hidden-layer',!cb.checked));
    });
    const sp=document.createElement('span'); sp.className='swatch'+(dash?' dashed':'');
    sp.style.borderColor=col;
    lab.append(cb,sp,document.createTextNode(name)); legend.appendChild(lab);
  });
  p1.appendChild(legend);

  const p2=document.createElement('div'); p2.className='panel';
  const t2=document.createElement('h3'); t2.textContent='χ² 收敛曲线（对数纵轴）'; p2.appendChild(t2);
  p2.appendChild(convergence(s));
  const st=document.createElement('table'); st.className='stats';
  st.innerHTML=@BT@<tr><td>初始 χ²</td><td>${fmt(s.chi2History[0])}</td></tr>
    <tr><td>最终 χ²</td><td>${fmt(s.finalChi2)}</td></tr>
    <tr><td>下降比例</td><td>${fmt((1-s.finalChi2/s.chi2History[0])*100)}%</td></tr>@BT@;
  p2.appendChild(st);

  if (s.geometry && s.geometry.issues.length) {
    const ul=document.createElement('ul'); ul.className='issues';
    s.geometry.issues.forEach(q=>{
      const li=document.createElement('li');
      const kn={ 'self-intersection':'路径自穿越','node-coincidence':'远节点异常重合','pose-jump':'位姿突变' };
      li.textContent=@BT@⚠ ${kn[q.kind]||q.kind}：节点 ${q.a} ↔ ${q.b} — ${q.detail}@BT@;
      ul.appendChild(li);
    });
    p2.appendChild(ul);
  }
  grid.append(p1,p2); card.appendChild(grid);

  if (s.verdicts && s.verdicts.length) {
    const tb=document.createElement('table'); tb.className='data';
    tb.innerHTML='<thead><tr><th>回环边</th><th>端点</th><th class="num">自身残差 (σ)</th>'+
      '<th class="num">剔除后其余回环最大残差 (σ)</th><th class="num">代价下降</th><th>判定</th></tr></thead>';
    const tbBody=document.createElement('tbody');
    s.verdicts.forEach(v=>{
      const tr=document.createElement('tr');
      const tag=@BT@<span class="tag ${v.suspicious?'bad':'ok'}">${v.suspicious?'⚠ 错误回环':'✓ 正常'}</span>@BT@;
      tr.innerHTML=@BT@<td>#${v.edge}</td><td>${v.i} ↔ ${v.j}</td>
        <td class="num">${fmt(v.residual)}</td><td class="num">${fmt(v.otherMax)}</td>
        <td class="num">${fmt(v.improvement)}</td><td>${tag}</td>@BT@;
      tr.addEventListener('mousemove',ev=>showTip(v.reasons.map(r=>'• '+r).join('<br>'),ev.clientX,ev.clientY));
      tr.addEventListener('mouseleave',hideTip);
      tbBody.appendChild(tr);
    });
    tb.appendChild(tbBody); card.appendChild(tb);
  }

  if (s.multiStart && s.multiStart.length) {
    const h3=document.createElement('h3'); h3.style.margin='16px 0 2px';
    h3.style.fontSize='13px'; h3.style.color='var(--ink-2)';
    h3.textContent='多起点优化（用几何先验在吸引盆地间选择）'; card.appendChild(h3);
    const tb=document.createElement('table'); tb.className='data';
    tb.innerHTML='<thead><tr><th>初值</th><th class="num">最终代价</th>'+
      '<th class="num">几何问题数</th><th>几何可信</th><th>选中</th></tr></thead>';
    const body=document.createElement('tbody');
    s.multiStart.forEach(m=>{
      const tr=document.createElement('tr');
      tr.innerHTML=@BT@<td>${m.name}</td><td class="num">${fmt(m.cost)}</td>
        <td class="num">${m.issues}</td>
        <td>${m.plausible?'<span class="tag ok">✓ 是</span>':'<span class="tag bad">✗ 否</span>'}</td>
        <td>${m.chosen?'<b>★ 采用</b>':''}</td>@BT@;
      body.appendChild(tr);
    });
    tb.appendChild(body); card.appendChild(tb);
  }
  document.getElementById('root').appendChild(card);
});
</script>
</body>
</html>`
