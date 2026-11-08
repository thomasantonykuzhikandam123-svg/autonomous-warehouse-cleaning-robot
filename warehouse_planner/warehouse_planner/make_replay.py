#!/usr/bin/env python3
"""
make_replay.py -- turn recorded runs into a flight-recorder replay you can open
in any browser.

    python3 make_replay.py ~/runs/fixed_3
        -> ~/runs/fixed_3/replay.html

    python3 make_replay.py ~/runs/baseline_1 ~/runs/fixed_3
        -> ~/runs/compare_baseline_1_vs_fixed_3.html   (played side by side)

What it shows, per run, synced at the operator's FIRST command:
  - the robot driving, the floor painting in by pass count
    (cyan = once, violet = twice, pink = 3x, red = 4x or more)
  - every operator line, what the robot said, and what it actually SENT
  - a red flag wherever the words name a zone the command did not
  - live painted area, overlap and distance, plus the recorder's own totals

Everything is embedded, so the HTML is one file: open it on the Mac, send it to
your supervisor, drop it into a slide. No server, no internet, no load on the
VM -- it is built from files coverage_recorder already writes.

Written for the VM's Python 3.10 / numpy 1.21.
"""
import argparse
import base64
import csv
import json
import os

import numpy as np

MAX_POINTS = 24000        # ~40 min at 10 Hz; longer runs are thinned evenly


def load_run(path):
    run = os.path.realpath(os.path.expanduser(path))
    with open(os.path.join(run, 'summary.json')) as f:
        s = json.load(f)
    occ = np.load(os.path.join(run, 'map.npy')).astype(np.int8)

    trail = []
    with open(os.path.join(run, 'trail.csv')) as f:
        rows = csv.reader(f)
        next(rows, None)
        for row in rows:
            try:
                trail.append([round(float(row[0]), 2), round(float(row[2]), 3),
                              round(float(row[3]), 3), round(float(row[4]), 2),
                              round(float(row[1]), 3)])   # wall t, x, y, yaw, sim t
            except (ValueError, IndexError):
                continue
    if len(trail) > MAX_POINTS:
        step = len(trail) / MAX_POINTS
        trail = [trail[int(i * step)] for i in range(MAX_POINTS)]

    events = []
    ev_path = os.path.join(run, 'events.jsonl')
    if os.path.exists(ev_path):
        with open(ev_path) as f:
            for line in f:
                try:
                    e = json.loads(line)
                except ValueError:
                    continue
                if e.get('type') == 'dialogue' and isinstance(e.get('data'), dict):
                    d = e['data']
                    events.append({'t': e.get('t'), 'text': d.get('text', ''),
                                   'reply': d.get('reply', ''), 'sent': d.get('sent'),
                                   'notes': d.get('notes') or [],
                                   'error': d.get('error')})

    return {
        'name': os.path.basename(run),
        'map': {'w': int(occ.shape[1]), 'h': int(occ.shape[0]),
                'res': float(s['map']['resolution']),
                'origin': [float(v) for v in s['map']['origin']],
                'occ': base64.b64encode(occ.tobytes()).decode('ascii')},
        'brush': float(s.get('params', {}).get('clean_width', 0.64)),
        'passGap': float(s.get('params', {}).get('pass_gap', 3.0)),
        'trail': trail,
        'events': events,
        'final': {'coverage': s.get('coverage') or {}, 'driving': s.get('driving') or {}},
    }


def build(runs, title):
    data = json.dumps(runs, separators=(',', ':')).replace('</', '<\\/')
    return (TEMPLATE.replace('__TITLE__', title)
                    .replace('__DATA__', data))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+', help='one or two run folders')
    ap.add_argument('--out', default=None)
    ap.add_argument('--title', default=None)
    a = ap.parse_args()
    if len(a.runs) > 2:
        ap.error('one run, or two to compare')
    runs = [load_run(p) for p in a.runs]
    names = [r['name'] for r in runs]
    if a.out:
        out = os.path.expanduser(a.out)
    elif len(runs) == 1:
        out = os.path.join(os.path.realpath(os.path.expanduser(a.runs[0])), 'replay.html')
    else:
        parent = os.path.dirname(os.path.realpath(os.path.expanduser(a.runs[0])))
        out = os.path.join(parent, f'compare_{names[0]}_vs_{names[1]}.html')
    title = a.title or ('Flight recorder: ' + ' vs '.join(names))
    with open(out, 'w') as f:
        f.write(build(runs, title))
    kb = os.path.getsize(out) / 1024
    print(f'wrote {out}  ({kb:.0f} KB, {sum(len(r["trail"]) for r in runs)} pose samples)')


TEMPLATE = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>__TITLE__</title>
<style>
:root{--bg:#eef1f6;--panel:#ffffff;--ink:#0f172a;--muted:#5b6b82;--line:#dbe2ec;
  --stage:#070b16;--chip:#eef2ff;--chipink:#3730a3;--bad:#dc2626;--badbg:#fef2f2;--good:#059669;
  box-sizing:border-box;padding-top:env(safe-area-inset-top,0px);padding-bottom:env(safe-area-inset-bottom,0px)}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#05070d;--panel:#0d1322;--ink:#e6eaf2;
  --muted:#8b98ad;--line:#1b2437;--chip:#1e1b4b;--chipink:#c7d2fe;--badbg:#2a0f14}}
:root[data-theme="dark"]{--bg:#05070d;--panel:#0d1322;--ink:#e6eaf2;--muted:#8b98ad;--line:#1b2437;
  --chip:#1e1b4b;--chipink:#c7d2fe;--badbg:#2a0f14}
html{scroll-padding-top:env(safe-area-inset-top,0px)}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{padding:18px 18px 6px}
h1{margin:0;font-size:20px;letter-spacing:-.01em}
.sub{color:var(--muted);font-size:12.5px;margin-top:2px}
.bar{position:sticky;top:env(safe-area-inset-top,0px);z-index:5;display:flex;flex-wrap:wrap;gap:8px;align-items:center;
  padding:10px 18px;background:color-mix(in srgb,var(--bg) 88%,transparent);backdrop-filter:blur(8px);border-bottom:1px solid var(--line)}
button{font:inherit;border:1px solid var(--line);background:var(--panel);color:var(--ink);border-radius:9px;padding:6px 12px;cursor:pointer}
button.on{background:var(--ink);color:var(--panel);border-color:var(--ink)}
#play{min-width:74px;font-weight:600}
input[type=range]{flex:1;min-width:160px;accent-color:#22d3ee}
#clock{font-variant-numeric:tabular-nums;color:var(--muted);min-width:118px;text-align:right}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px;padding:14px 18px 28px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;overflow:hidden;display:flex;flex-direction:column}
.card h2{margin:0;padding:12px 14px 8px;font-size:15px;display:flex;justify-content:space-between;align-items:baseline;gap:8px}
.card h2 span{font-weight:500;color:var(--muted);font-size:12px}
.stage{background:var(--stage);position:relative}
canvas{display:block;width:100%;height:auto}
.metrics{display:grid;grid-template-columns:repeat(3,1fr);border-top:1px solid var(--line)}
.m{padding:9px 12px;border-right:1px solid var(--line)}
.m:last-child{border-right:0}
.m b{display:block;font-size:19px;font-variant-numeric:tabular-nums;letter-spacing:-.01em}
.m small{color:var(--muted);font-size:11.5px}
.final{padding:7px 14px;font-size:11.5px;color:var(--muted);border-top:1px solid var(--line)}
.log{border-top:1px solid var(--line);max-height:300px;overflow-y:auto;padding:6px 12px 10px}
.turn{padding:8px 0;border-bottom:1px dashed var(--line);animation:pop .35s ease}
.turn:last-child{border-bottom:0}
@keyframes pop{from{opacity:0;transform:translateY(4px)}to{opacity:1;transform:none}}
.you{font-weight:600}
.you:before{content:"operator  ";color:var(--muted);font-weight:500;font-size:11px;letter-spacing:.04em;text-transform:uppercase}
.bot{margin-top:3px}
.bot:before{content:"robot  ";color:var(--muted);font-size:11px;letter-spacing:.04em;text-transform:uppercase}
.did{display:inline-block;margin-top:5px;background:var(--chip);color:var(--chipink);border-radius:6px;padding:1px 7px;font-size:12px;font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
.flag{margin-top:5px;color:var(--bad);background:var(--badbg);border-radius:6px;padding:3px 7px;font-size:12.5px;font-weight:600}
.model{margin-top:4px;color:var(--muted);font-size:12px;font-style:italic}
.empty{color:var(--muted);font-size:12.5px;padding:8px 0}
.legend{display:flex;gap:10px;flex-wrap:wrap;padding:0 18px;color:var(--muted);font-size:12px}
.legend i{display:inline-block;width:11px;height:11px;border-radius:3px;margin-right:5px;vertical-align:-1px}
</style>
</head>
<body>
<header>
  <h1>__TITLE__</h1>
  <div class="sub" id="sub"></div>
</header>
<div class="legend">
  <span><i style="background:#22d3ee"></i>swept once</span>
  <span><i style="background:#a78bfa"></i>twice</span>
  <span><i style="background:#f472b6"></i>3 times</span>
  <span><i style="background:#ef4444"></i>4+ (wasted driving)</span>
</div>
<div class="bar">
  <button id="play">Play</button>
  <span id="speeds"></span>
  <input id="scrub" type="range" min="0" max="1000" value="0" aria-label="time">
  <span id="clock">0:00</span>
</div>
<div class="grid" id="grid"></div>
<script>
const RUNS = __DATA__;
const COL = [null,[34,211,238],[167,139,250],[244,114,182],[239,68,68]];

function b64i8(s){const b=atob(s),a=new Int8Array(b.length);for(let i=0;i<b.length;i++)a[i]=(b.charCodeAt(i)<<24)>>24;return a;}
function fmt(u){const s=Math.max(0,u),m=Math.floor(s/60),r=Math.floor(s%60);return (u<0?"-":"")+m+":"+String(r).padStart(2,"0");}
function esc(s){return String(s==null?"":s).replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));}

// zones a sentence NAMES, to compare against what was actually sent
function zonesSaid(txt){const z=new Set();const t=String(txt||"");let m;const re=/\baisle[\s_]*([1-3])\b/gi;
  while((m=re.exec(t)))z.add("aisle_"+m[1]);if(/\bfront\b/i.test(t))z.add("front");
  if(/\bback\b/i.test(t)&&!/\bback to the dock\b/i.test(t))z.add("back");return z;}
function stepsOf(sent){if(!sent)return[];return Array.isArray(sent.steps)?sent.steps:[sent];}
function mismatch(ev){const steps=stepsOf(ev.sent).filter(s=>s&&(s.action==="clean"||s.action==="skip"));
  if(!steps.length)return null;const sent=new Set();steps.forEach(s=>(s.zones||[]).forEach(z=>sent.add(z)));
  if(sent.has("all"))return null;const said=[...zonesSaid(ev.reply)].filter(z=>!sent.has(z));
  if(!said.length)return null;const nm=z=>z.replace("_"," ");
  return "says "+said.map(nm).join(", ")+" \u2014 sent "+[...sent].map(nm).join(", ");}

const S = RUNS.map((run,ri)=>{
  const m=run.map,w=m.w,h=m.h,res=m.res,ox=m.origin[0],oy=m.origin[1],occ=b64i8(m.occ);
  const floor=new Uint8Array(w*h);for(let i=0;i<w*h;i++)floor[i]=occ[i]===0?1:0;
  const base=document.createElement("canvas");base.width=w;base.height=h;
  const bx=base.getContext("2d"),bi=bx.createImageData(w,h);
  for(let r=0;r<h;r++)for(let c=0;c<w;c++){const v=occ[r*w+c],p=((h-1-r)*w+c)*4;
    const col=v===0?[9,14,28]:(v>=50?[120,138,166]:[14,20,36]);bi.data[p]=col[0];bi.data[p+1]=col[1];bi.data[p+2]=col[2];bi.data[p+3]=255;}
  bx.putImageData(bi,0,0);
  const heat=document.createElement("canvas");heat.width=w;heat.height=h;
  const hx=heat.getContext("2d"),hi=hx.createImageData(w,h);
  const rb=run.brush/2,n=Math.ceil(rb/res),offs=[];
  for(let dy=-n;dy<=n;dy++)for(let dx=-n;dx<=n;dx++)if((dx*res)**2+(dy*res)**2<=rb*rb)offs.push([dx,dy]);
  const firstEv=run.events.length?run.events[0].t:null;
  const t0=firstEv!=null?firstEv:(run.trail.length?run.trail[0][0]:0);

  const card=document.createElement("section");card.className="card";
  const fc=run.final.coverage||{},fd=run.final.driving||{};
  card.innerHTML='<h2>'+esc(run.name)+'<span>'+run.trail.length+' pose samples</span></h2>'+
    '<div class="stage"><canvas></canvas></div>'+
    '<div class="metrics"><div class="m"><b data-k="area">0.0</b><small>m\u00b2 swept</small></div>'+
    '<div class="m"><b data-k="ovl">\u2013</b><small>overlap (sweeps per cell)</small></div>'+
    '<div class="m"><b data-k="dist">0</b><small>m driven</small></div></div>'+
    '<div class="final">recorder, whole run: overlap '+(fc.overlap_ratio??"\u2013")+
    ' \u00b7 sweep efficiency '+(fd.sweep_efficiency??"\u2013")+
    ' \u00b7 unproductive driving '+(fd.unproductive_fraction!=null?Math.round(fd.unproductive_fraction*100)+"%":"\u2013")+'</div>'+
    '<div class="log"></div>';
  document.getElementById("grid").appendChild(card);
  const log=card.querySelector(".log");
  const items=run.events.map(ev=>{const d=document.createElement("div");d.className="turn";d.style.display="none";
    const st=stepsOf(ev.sent);const did=st.length?st.map(s=>s.action+" \u00b7 "+((s.zones||[]).join(", ")||"\u2013")).join("  \u2192  "):"nothing sent";
    const flag=mismatch(ev);const model=(ev.notes||[]).find(x=>/^model said:/.test(x));
    d.innerHTML='<div class="you">'+esc(ev.text)+'</div>'+
      '<div class="bot">'+esc(ev.error?("agent error: "+ev.error):ev.reply)+'</div>'+
      '<div class="did">SENT '+esc(did)+'</div>'+
      (flag?'<div class="flag">\u26a0 '+esc(flag)+'</div>':'')+
      (model?'<div class="model">'+esc(model.replace(/^model said:\s*/,"model's own words: "))+'</div>':'');
    log.appendChild(d);return {ev,d,shown:false};});
  if(!items.length)log.innerHTML='<div class="empty">No /dialogue in this run.</div>';
  return {run,w,h,res,ox,oy,floor,base,heat,hx,hi,offs,t0,card,items,log,
    cv:card.querySelector("canvas"),vals:card.querySelectorAll("b[data-k]"),
    passes:null,lastT:null,idx:0,painted:0,total:0,dist:0,px:null,py:null,pt:null};
});

function reset(s){s.passes=new Uint16Array(s.w*s.h);s.lastT=new Float64Array(s.w*s.h).fill(-1e9);
  s.hi.data.fill(0);s.idx=0;s.painted=0;s.total=0;s.dist=0;s.px=null;s.py=null;s.pt=null;
  s.items.forEach(it=>{it.d.style.display="none";it.shown=false;});}

function advance(s,u){const tr=s.run.trail,gap=s.run.passGap,w=s.w,h=s.h,res=s.res,d=s.hi.data;let dirty=false;
  while(s.idx<tr.length&&tr[s.idx][0]-s.t0<=u){const x=tr[s.idx][1],y=tr[s.idx][2],t=tr[s.idx][4];
    if(s.px!=null){const dt=t-s.pt,dd=Math.hypot(x-s.px,y-s.py);if(dt>0&&dt<=1&&dd/dt>=0.05)s.dist+=dd;}
    s.px=x;s.py=y;s.pt=t;
    const c=Math.floor((x-s.ox)/res),r=Math.floor((y-s.oy)/res);
    for(const [dx,dy] of s.offs){const cc=c+dx,rr=r+dy;if(cc<0||rr<0||cc>=w||rr>=h)continue;const k=rr*w+cc;
      if(!s.floor[k])continue;if(t-s.lastT[k]>gap){if(s.passes[k]===0)s.painted++;s.passes[k]++;s.total++;
        const col=COL[Math.min(4,s.passes[k])],p=((h-1-rr)*w+cc)*4;d[p]=col[0];d[p+1]=col[1];d[p+2]=col[2];d[p+3]=215;dirty=true;}
      s.lastT[k]=t;}
    s.idx++;}
  if(dirty)s.hx.putImageData(s.hi,0,0);
  s.items.forEach(it=>{const on=it.ev.t-s.t0<=u;if(on!==it.shown){it.d.style.display=on?"":"none";it.shown=on;
    if(on)s.log.scrollTop=s.log.scrollHeight;}});}

function draw(s){const cv=s.cv,dpr=window.devicePixelRatio||1,W=cv.clientWidth||300,H=Math.round(W*s.h/s.w);
  if(cv.width!==Math.round(W*dpr)){cv.width=Math.round(W*dpr);cv.height=Math.round(H*dpr);}
  const x=cv.getContext("2d"),k=cv.width/s.w;x.imageSmoothingEnabled=false;
  x.drawImage(s.base,0,0,cv.width,cv.height);x.drawImage(s.heat,0,0,cv.width,cv.height);
  const tr=s.run.trail;if(!s.idx)return;
  const toC=(px,py)=>[(px-s.ox)/s.res*k,(s.h-(py-s.oy)/s.res)*k];
  const from=Math.max(0,s.idx-90);x.lineCap="round";
  for(let i=from+1;i<s.idx;i++){const a=toC(tr[i-1][1],tr[i-1][2]),b=toC(tr[i][1],tr[i][2]);
    x.strokeStyle="rgba(255,255,255,"+(0.08+0.55*(i-from)/(s.idx-from))+")";x.lineWidth=1.6*dpr;
    x.beginPath();x.moveTo(a[0],a[1]);x.lineTo(b[0],b[1]);x.stroke();}
  const p=tr[s.idx-1],c=toC(p[1],p[2]),rad=(s.run.brush/2)/s.res*k;
  const g=x.createRadialGradient(c[0],c[1],0,c[0],c[1],rad*2.2);g.addColorStop(0,"rgba(255,255,255,.35)");g.addColorStop(1,"rgba(255,255,255,0)");
  x.fillStyle=g;x.beginPath();x.arc(c[0],c[1],rad*2.2,0,Math.PI*2);x.fill();
  x.fillStyle="#fff";x.beginPath();x.arc(c[0],c[1],Math.max(3*dpr,rad*.55),0,Math.PI*2);x.fill();
  x.strokeStyle="#0b1020";x.lineWidth=2*dpr;x.beginPath();x.moveTo(c[0],c[1]);
  x.lineTo(c[0]+Math.cos(p[3])*rad*1.1,c[1]-Math.sin(p[3])*rad*1.1);x.stroke();}

function stats(s){const a=s.painted*s.res*s.res;s.vals[0].textContent=a.toFixed(1);
  s.vals[1].textContent=s.painted?(s.total/s.painted).toFixed(2):"\u2013";s.vals[2].textContent=s.dist.toFixed(0);}

let uMin=Infinity,uMax=-Infinity;
S.forEach(s=>{const tr=s.run.trail;if(!tr.length)return;uMin=Math.min(uMin,tr[0][0]-s.t0);uMax=Math.max(uMax,tr[tr.length-1][0]-s.t0);});
if(!isFinite(uMin)){uMin=0;uMax=1;}uMin=Math.max(uMin,-5);
let u=uMin,playing=false,speed=16,last=0;
document.getElementById("sub").textContent=(S.length>1?"Runs are synced at the operator's first command. ":"")+
  "Floor paints by how many times it was swept. Red flags mark replies that name a zone the command did not.";
const sp=document.getElementById("speeds");
[1,4,16,64].forEach(v=>{const b=document.createElement("button");b.textContent=v+"\u00d7";if(v===speed)b.className="on";
  b.onclick=()=>{speed=v;sp.querySelectorAll("button").forEach(x=>x.className="");b.className="on";};sp.appendChild(b);});
const playBtn=document.getElementById("play"),scrub=document.getElementById("scrub"),clock=document.getElementById("clock");
function seek(v){u=v;S.forEach(s=>{reset(s);advance(s,u);});render();}
function render(){S.forEach(s=>{draw(s);stats(s);});scrub.value=Math.round(1000*(u-uMin)/Math.max(1e-6,uMax-uMin));
  clock.textContent=fmt(u)+" / "+fmt(uMax);}
playBtn.onclick=()=>{if(u>=uMax)seek(uMin);playing=!playing;playBtn.textContent=playing?"Pause":"Play";};
scrub.oninput=()=>{seek(uMin+(uMax-uMin)*scrub.value/1000);};
function frame(ts){const dt=last?Math.min(0.1,(ts-last)/1000):0;last=ts;
  if(playing){u=Math.min(uMax,u+dt*speed);S.forEach(s=>advance(s,u));render();
    if(u>=uMax){playing=false;playBtn.textContent="Replay";}}
  requestAnimationFrame(frame);}
window.addEventListener("resize",render);
seek(uMin);requestAnimationFrame(frame);
</script>
</body>
</html>
'''


if __name__ == '__main__':
    main()
