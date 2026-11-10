"use strict";
const $=id=>document.getElementById(id);
const T=32,WT=48,HT=32,NI=6,NJ=4,VW=800,VH=500,DIRS=[[0,-1],[1,0],[0,1],[-1,0]];
const cv=$("cv"),cx=cv.getContext("2d");
function rng(a){return function(){a|=0;a=a+0x6D2B79F5|0;let t=Math.imul(a^a>>>15,1|a);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296}}
const clamp=(v,lo,hi,d)=>{v=+v;if(!Number.isFinite(v))v=d;return Math.min(hi,Math.max(lo,v))};
const road=(x,y)=>x>=0&&y>=0&&x<WT&&y<HT&&(x%8<2||y%8<2);
const solid=(px,py)=>!road(Math.floor(px/T),Math.floor(py/T));
const nodeXY=n=>[((n%NI)*8+1)*T,(((n/NI)|0)*8+1)*T];
const nbrs=n=>{const i=n%NI,j=(n/NI)|0,r=[];DIRS.forEach(([dx,dy],a)=>{const x=i+dx,y=j+dy;if(x>=0&&y>=0&&x<NI&&y<NJ)r.push([a,y*NI+x])});return r};
const gman=(a,b)=>Math.abs(a%NI-b%NI)+Math.abs(((a/NI)|0)-((b/NI)|0));
const angN=a=>Math.atan2(Math.sin(a),Math.cos(a));

/* ---- Week 2: A* / BFS on the tile grid ---- */
function astar(s,t,mode){
 const M=WT*HT,g=new Float32Array(M).fill(1e9),par=new Int32Array(M).fill(-1),cl=new Uint8Array(M);
 const tx=t%WT,ty=(t/WT)|0,h=i=>mode==="u"?0:Math.abs(i%WT-tx)+Math.abs(((i/WT)|0)-ty);
 const open=[s];let exp=0;g[s]=0;
 while(open.length){let bi=0,bf=1e9;
  for(let k=0;k<open.length;k++){const f=g[open[k]]+h(open[k]);if(f<bf){bf=f;bi=k}}
  const c=open[bi];open[bi]=open[open.length-1];open.pop();if(cl[c])continue;cl[c]=1;exp++;
  if(c===t){const p=[c];let q=c;while(par[q]>=0){q=par[q];p.push(q)}return{path:p.reverse(),exp}}
  const px=c%WT,py=(c/WT)|0;
  for(const [dx,dy] of DIRS){const nx=px+dx,ny=py+dy;if(!road(nx,ny))continue;const n=ny*WT+nx;if(g[c]+1<g[n]){g[n]=g[c]+1;par[n]=c;open.push(n)}}}
 return{path:null,exp}}
function bfs(s,t){const seen=new Uint8Array(WT*HT),d=new Int32Array(WT*HT),q=[s];seen[s]=1;let exp=0,h=0;
 while(h<q.length){const c=q[h++];exp++;if(c===t)return{len:d[c],exp};
  for(const [dx,dy] of DIRS){const nx=c%WT+dx,ny=((c/WT)|0)+dy;if(!road(nx,ny))continue;const n=ny*WT+nx;if(!seen[n]){seen[n]=1;d[n]=d[c]+1;q.push(n)}}}
 return{len:-1,exp}}

/* ---- Week 3: CSP (backtracking) ---- */
function solveCSP(rand,startN){
 const vars=["C1","C2","C3","C4","C5","R"],A={};let as=0,bk=0;
 const ok=(v,c)=>{if(Object.values(A).includes(c)||gman(c,startN)<2)return false;
  for(const u in A)if(gman(c,A[u])<2)return false;return true};
 const bt=k=>{if(k===vars.length)return true;
  const dom=[...Array(NI*NJ).keys()].sort(()=>rand()-.5);
  for(const c of dom)if(ok(vars[k],c)){A[vars[k]]=c;as++;if(bt(k+1))return true;delete A[vars[k]];bk++}return false};
 return{sol:bt(0)?A:null,as,bk}}

/* ---- Week 5: Q-learning on the intersection graph ---- */
function greedy(Q,n){const nb=nbrs(n);let b=nb[0];for(const x of nb)if(Q[n*4+x[0]]>Q[n*4+b[0]])b=x;return b}
function trainQ(targets,eps,rand){
 return targets.map(tg=>{const Q=new Float32Array(NI*NJ*4);
  for(let e=0;e<eps;e++){let n=Math.floor(rand()*NI*NJ);
   for(let st=0;st<60&&n!==tg;st++){const nb=nbrs(n);let a;
    if(rand()<.3)a=nb[Math.floor(rand()*nb.length)][0];else a=greedy(Q,n)[0];
    const n2=nb.find(x=>x[0]===a)[1];let mx=0;
    if(n2!==tg){mx=-1e9;for(const [aa] of nbrs(n2))mx=Math.max(mx,Q[n2*4+aa])}
    Q[n*4+a]+=.5*((n2===tg?50:-1)+.9*mx-Q[n*4+a]);n=n2}}
  return Q})}

/* ---- Physics ---- */
function hits(x,y,a){const c=Math.cos(a),s=Math.sin(a);
 for(const [u,w] of [[13,7],[13,-7],[-13,7],[-13,-7]])if(solid(x+u*c-w*s,y+u*s+w*c))return true;return false}
function drive(c,dt,thr,st,mx){
 if(thr>0)c.v+=320*dt;else if(thr<0)c.v-=(c.v>0?560:240)*dt;else c.v*=1-1.4*dt;
 c.v=Math.max(-mx*.4,Math.min(mx,c.v));
 const na=c.a+st*3.2*dt*Math.max(-1,Math.min(1,c.v/90));if(!hits(c.x,c.y,na))c.a=na;
 const dx=Math.cos(c.a)*c.v*dt,dy=Math.sin(c.a)*c.v*dt;
 if(!hits(c.x+dx,c.y+dy,c.a)){c.x+=dx;c.y+=dy}
 else if(!hits(c.x+dx,c.y,c.a)){c.x+=dx;c.v*=.92}
 else if(!hits(c.x,c.y+dy,c.a)){c.y+=dy;c.v*=.92}
 else c.v*=-.3}

/* ---- Game state ---- */
let P,cops,civs,cash,rival,Qs,world,hp,over,t0,hitCD,copSpd,H="m",nC,dbg=false,rand,csp,lastA=null,maxCops=2,time=0;
const K={up:0,down:0,left:0,right:0,nit:0};
let nitro=1,shake=0,parts=[],level=1,score=0,snd=null,night=true,rain=true;
const dk=document.createElement("canvas");dk.width=VW;dk.height=VH;const dg=dk.getContext("2d");
function fx(o,k){if(parts.length>300)return;const c=Math.cos(o.a),s=Math.sin(o.a),bx=o.x-c*14,by=o.y-s*14,r=Math.random;
 parts.push(k==="f"?{x:bx,y:by,vx:-c*120+(r()-.5)*40,vy:-s*120+(r()-.5)*40,l:.25,c:"#ffb020",r:4}:{x:bx,y:by,vx:(r()-.5)*30,vy:(r()-.5)*30,l:.6,c:"rgba(200,200,210,.5)",r:5})}
function ahead(){const x=P.x+Math.cos(P.a)*Math.max(0,P.v)*1.3,y=P.y+Math.sin(P.a)*Math.max(0,P.v)*1.3,tx=Math.floor(x/T),ty=Math.floor(y/T);return road(tx,ty)?ty*WT+tx:tileOf(P)}
function audio(){if(snd)return;try{const A=new(window.AudioContext||window.webkitAudioContext)(),o=A.createOscillator(),g=A.createGain(),s=A.createOscillator(),sg=A.createGain();
 o.type="sawtooth";g.gain.value=.03;o.connect(g);g.connect(A.destination);o.start();s.type="square";sg.gain.value=0;s.connect(sg);sg.connect(A.destination);s.start();snd={A,o,g,s,sg,mute:false}}catch(e){}}
function sound(){if(!snd)return;const t=snd.A.currentTime,m=snd.mute?0:1;
 snd.o.frequency.setTargetAtTime(45+Math.abs(P.v)*.55+(K.nit&&nitro>0?40:0),t,.05);snd.g.gain.setTargetAtTime(.03*m,t,.05);
 let d=1e9;for(const p of cops)d=Math.min(d,Math.hypot(P.x-p.x,P.y-p.y));
 snd.s.frequency.setTargetAtTime(Math.floor(time*2)%2?900:700,t,.02);snd.sg.gain.setTargetAtTime(over?0:m*Math.max(0,1-d/450)*.02,t,.1)}
function beep(f){if(!snd||snd.mute)return;const o=snd.A.createOscillator(),g=snd.A.createGain();o.frequency.value=f;g.gain.value=.08;o.connect(g);g.connect(snd.A.destination);o.start();o.stop(snd.A.currentTime+.12)}
const tileOf=o=>Math.floor(o.y/T)*WT+Math.floor(o.x/T);
const nodeNear=(x,y)=>Math.min(NJ-1,Math.max(0,Math.round((y/T-1)/8)))*NI+Math.min(NI-1,Math.max(0,Math.round((x/T-1)/8)));
const lane=(n,d,s)=>{const [bx,by]=nodeXY(n),[dx,dy]=DIRS[d];return{x:bx+dx*s-dy*.5*T,y:by+dy*s+dx*.5*T,a:Math.atan2(dy,dx)}};
const nextN=(n,d)=>n+DIRS[d][0]+DIRS[d][1]*NI;

function spawnCop(){const pn=nodeNear(P.x,P.y),c=[];for(let n=0;n<NI*NJ;n++)if(gman(n,pn)>=3)c.push(n);
 const n=c[Math.floor(rand()*c.length)],[x,y]=nodeXY(n);cops.push({x:x+T*.5,y:y+T*.5,a:0,v:0,path:null,pt:rand()*.5})}
function newGame(){
 const seed=Math.trunc(clamp($("seed").value,-1e6,1e6,5));$("seed").value=seed;
 maxCops=nC=Math.trunc(clamp($("nc").value,1,5,2));$("nc").value=nC;
 copSpd=clamp($("cs").value,150,260,200);$("cs").value=copSpd;H=$("heur").value;
 const eps=Math.trunc(clamp($("eps").value,50,3000,600));$("eps").value=eps;
 let sd=seed;
 for(;;sd++){rand=rng(sd);csp=solveCSP(rand,8);if(csp.sol)break}
 const [sx,sy]=nodeXY(8);const cs=CAR_STATS[$("car").value]||CAR_STATS.sport;P={x:sx,y:sy+T*.5,a:0,v:0,k:CAR_STATS[$("car").value]?$("car").value:"sport",mv:cs.v,hpm:cs.hp,col:cs.col};
 cash=["C1","C2","C3","C4","C5"].map(k=>({n:csp.sol[k],by:null}));
 const t1=performance.now();Qs=trainQ(cash.map(c=>c.n),eps,rand);const tq=Math.round(performance.now()-t1);
 world=buildWorld2(rng(seed+99));pk=[];const used=new Set([...cash.map(c=>c.n),csp.sol.R,8]);while(pk.length<4){const n=Math.floor(rand()*NI*NJ);if(!used.has(n)){used.add(n);pk.push({n,t:pk.length%2?"n":"r",on:true,cd:0})}}
 cops=[];for(let i=0;i<nC;i++)spawnCop();
 civs=[];for(let i=0;i<8;i++){const n=Math.floor(rand()*NI*NJ),o=nbrs(n);civs.push({n,d:o[Math.floor(rand()*o.length)][0],s:rand()*8*T,w:0,k:["sedan","suv","taxi","bus","sedan","suv","taxi","bus"][i],col:["#64748b","#0ea5a4","#facc15","#2563eb","#7c3aed","#84cc16","#facc15","#16a34a"][i]})}
 rival={n:csp.sol.R,d:0,s:0};pickRival();
 hp=P.hpm;over=null;hitCD=0;time=0;nitro=1;parts=[];shake=0;tl=150;
 $("s1").innerHTML=`<b>${civs.length}</b> рефлекс-көлік, <b>${cops.length}</b> полиция-агент, 1 рақиб-агент; полиция рөлдері: қуушы және жолды кесуші (болжам бойынша)`;
 $("s3").innerHTML=`<b>${csp.as}</b> меншіктеу, <b>${csp.bk}</b> backtrack, 6 айнымалы`;
 $("s5").innerHTML=`<b>${eps}</b> эпизод × 5 мақсат, ${tq} мс; рақиб үйренген саясатпен жүреді`;
 $("s2").textContent="-";$("log").textContent="";}
function pickRival(){const rem=cash.map((c,i)=>[c,i]).filter(x=>!x[0].by);
 if(!rem.length){rival.stop=true;return}
 rem.sort((a,b)=>gman(rival.n,a[0].n)-gman(rival.n,b[0].n));
 const [c,i]=rem[0];rival.tg=i;rival.s=0;rival.d=rival.n===c.n?rival.d:greedy(Qs[i],rival.n)[0]}

/* ---- World rendering (static layer) ---- */
function buildWorld(r){
 const w=document.createElement("canvas");w.width=WT*T;w.height=HT*T;const g=w.getContext("2d");
 g.fillStyle="#2a2e38";g.fillRect(0,0,w.width,w.height);
 g.strokeStyle="#e5c34a";g.lineWidth=2;g.setLineDash([14,12]);
 for(let j=0;j<NJ;j++){g.beginPath();g.moveTo(0,(j*8+1)*T);g.lineTo(w.width,(j*8+1)*T);g.stroke()}
 for(let i=0;i<NI;i++){g.beginPath();g.moveTo((i*8+1)*T,0);g.lineTo((i*8+1)*T,w.height);g.stroke()}
 g.setLineDash([]);g.fillStyle="#2a2e38";
 for(let i=0;i<NI;i++)for(let j=0;j<NJ;j++)g.fillRect(i*8*T,j*8*T,2*T,2*T);
 for(let bx=0;bx<NI;bx++)for(let by=0;by<NJ;by++){const x=(bx*8+2)*T,y=(by*8+2)*T,S=6*T;
  g.fillStyle="#7d8494";g.fillRect(x,y,S,S);
  if(r()<.2){g.fillStyle="#2f6b3a";g.fillRect(x+6,y+6,S-12,S-12);g.fillStyle="#3f8a4c";
   for(let k=0;k<14;k++){g.beginPath();g.arc(x+20+r()*(S-40),y+20+r()*(S-40),9+r()*6,0,7);g.fill()}continue}
  for(let a=0;a<2;a++)for(let b=0;b<2;b++){const lx=x+a*S/2+8,ly=y+b*S/2+8,L=S/2-16,hue=Math.floor(190+r()*70),li=28+Math.floor(r()*18);
   g.fillStyle="rgba(0,0,0,.35)";g.fillRect(lx+5,ly+5,L,L);
   g.fillStyle=`hsl(${hue},18%,${li}%)`;g.fillRect(lx,ly,L,L);
   g.fillStyle=`hsl(${hue},20%,${li+9}%)`;g.fillRect(lx+8,ly+8,L-16,L-16);
   g.fillStyle=`hsl(${hue},25%,${li-8}%)`;g.fillRect(lx+L/2-6,ly+L/2-6,12,12)}}
 return w}
function drawCar(o,col,siren){cx.save();cx.translate(o.x,o.y);cx.rotate(o.a);
 cx.fillStyle="rgba(0,0,0,.35)";cx.fillRect(-12,-6,28,18);
 cx.fillStyle=col;cx.beginPath();cx.roundRect(-14,-8,28,16,4);cx.fill();
 cx.fillStyle="rgba(180,220,255,.85)";cx.fillRect(1,-6,6,12);cx.fillRect(-10,-6,4,12);
 cx.fillStyle="#fff7c2";cx.fillRect(13,-7,2,4);cx.fillRect(13,3,2,4);
 if(siren){const f=Math.floor(time*8)%2;cx.fillStyle=f?"#ff2b2b":"#2b6bff";cx.fillRect(-3,-8,5,8);cx.fillStyle=f?"#2b6bff":"#ff2b2b";cx.fillRect(-3,0,5,8)}
 cx.restore()}

const CAR_STATS={sport:{v:260,hp:100,col:"#e11d48"},suv:{v:225,hp:150,col:"#16a34a"},taxi:{v:240,hp:120,col:"#facc15"}};let pk=[],tl=150;
/* ---- Winter Astana + new cars ---- */
const CARS={sport:[30,15],police:[29,15],sedan:[28,15],suv:[32,17],taxi:[28,15],bus:[48,18],rival:[34,17]};
function drawCar2(o,k,col,siren){const [L,Wd]=CARS[k],h=L/2,w=Wd/2;cx.save();cx.translate(o.x,o.y);cx.rotate(o.a);
 cx.fillStyle="rgba(0,0,0,.4)";cx.beginPath();cx.roundRect(-h+2,-w+3,L,Wd,5);cx.fill();
 cx.fillStyle="#0b0b0f";for(const sx of[-h*.55,h*.5])for(const sy of[-w-1.5,w-1.5])cx.fillRect(sx-4,sy,8,3);
 cx.fillStyle=col;cx.beginPath();cx.roundRect(-h,-w,L,Wd,k==="bus"?3:7);cx.fill();
 if(k==="police"){cx.fillStyle="#f1f5f9";cx.fillRect(-h*.4,-w,L*.42,Wd)}
 if(k==="sport"){cx.fillStyle="#fff";cx.fillRect(-h+3,-2.4,L-8,1.8);cx.fillRect(-h+3,.6,L-8,1.8);cx.fillStyle="#111";cx.fillRect(-h,-w+1,3,Wd-2)}
 if(k==="rival"){cx.fillStyle="#431407";cx.fillRect(-h+1,-w+2,L*.38,Wd-4)}
 if(k==="bus"){cx.fillStyle="rgba(255,255,255,.28)";cx.fillRect(-h+3,-w+2,L-6,Wd-4);cx.fillStyle="rgba(20,30,50,.75)";for(let i=0;i<5;i++)cx.fillRect(-h+5+i*8,-w+3,6,Wd-6)}
 else{cx.fillStyle="rgba(165,210,250,.92)";cx.fillRect(h*.12,-w+2,h*.34,Wd-4);cx.fillRect(-h*.58,-w+2,h*.2,Wd-4);cx.fillStyle="rgba(255,255,255,.25)";cx.fillRect(-h*.36,-w+2,h*.46,Wd-4)}
 if(k==="suv"){cx.strokeStyle="rgba(0,0,0,.5)";cx.lineWidth=1;cx.strokeRect(-h*.34,-w+3,h*.44,Wd-6)}
 if(k==="taxi"){cx.fillStyle="#111";cx.fillRect(-3,-3,6,6);cx.fillStyle="#fde047";cx.fillRect(-2,-2,4,4)}
 cx.fillStyle="#fff7c2";cx.fillRect(h-2,-w+1.5,2.5,3.5);cx.fillRect(h-2,w-5,2.5,3.5);
 cx.fillStyle="#ef4444";cx.fillRect(-h,-w+1.5,2,3.5);cx.fillRect(-h,w-5,2,3.5);
 if(siren){const f=Math.floor(time*8)%2;cx.fillStyle=f?"#ff2b2b":"#2b6bff";cx.fillRect(-4,-w+1,5,w-1);cx.fillStyle=f?"#2b6bff":"#ff2b2b";cx.fillRect(-4,0,5,w-1)}
 cx.restore()}
function buildWorld2(r){
 const w=document.createElement("canvas");w.width=WT*T;w.height=HT*T;const g=w.getContext("2d");
 const tree=(x,y,s)=>{g.fillStyle="rgba(0,0,0,.25)";g.beginPath();g.arc(x+3,y+3,s,0,7);g.fill();g.fillStyle="#1f5a3a";g.beginPath();g.arc(x,y,s,0,7);g.fill();g.fillStyle="#e8f1f8";g.beginPath();g.arc(x,y,s*.5,0,7);g.fill()};
 g.fillStyle="#323641";g.fillRect(0,0,w.width,w.height);
 for(let i=0;i<500;i++){g.fillStyle=`rgba(255,255,255,${.04+r()*.08})`;g.beginPath();g.ellipse(r()*w.width,r()*w.height,6+r()*18,3+r()*7,r()*3,0,7);g.fill()}
 g.strokeStyle="#f1f5f9";g.lineWidth=2;g.setLineDash([16,14]);
 for(let j=0;j<NJ;j++){g.beginPath();g.moveTo(0,(j*8+1)*T);g.lineTo(w.width,(j*8+1)*T);g.stroke()}
 for(let i=0;i<NI;i++){g.beginPath();g.moveTo((i*8+1)*T,0);g.lineTo((i*8+1)*T,w.height);g.stroke()}
 g.setLineDash([]);
 for(let i=0;i<NI;i++)for(let j=0;j<NJ;j++){const x=i*8*T,y=j*8*T;g.fillStyle="#323641";g.fillRect(x,y,2*T,2*T);g.fillStyle="rgba(240,244,250,.7)";
  for(let k=0;k<6;k++){g.fillRect(x+4+k*10,y-9,6,8);g.fillRect(x+4+k*10,y+2*T+1,6,8);g.fillRect(x-9,y+4+k*10,8,6);g.fillRect(x+2*T+1,y+4+k*10,8,6)}}
 const LM={"3,1":"Бәйтерек","1,2":"Хан Шатыр","4,2":"Нұр Әлем"},HUES=[210,25,350,160,45,280];
 for(let bx=0;bx<NI;bx++)for(let by=0;by<NJ;by++){const x=(bx*8+2)*T,y=(by*8+2)*T,S=6*T,cxm=x+S/2,cym=y+S/2,lm=LM[bx+","+by];
  g.fillStyle="#cfd8e3";g.fillRect(x,y,S,S);
  if(lm){g.fillStyle="#e8eef5";g.fillRect(x+5,y+5,S-10,S-10);
   if(bx===3){for(let a=0;a<12;a++)tree(cxm+Math.cos(a/12*6.28)*70,cym+Math.sin(a/12*6.28)*70,9);
    g.strokeStyle="#6b7280";g.lineWidth=5;g.beginPath();g.moveTo(cxm-48,cym-48);g.lineTo(cxm+48,cym+48);g.moveTo(cxm+48,cym-48);g.lineTo(cxm-48,cym+48);g.stroke();
    g.fillStyle="rgba(0,0,0,.25)";g.beginPath();g.arc(cxm+8,cym+8,26,0,7);g.fill();
    const gr=g.createRadialGradient(cxm-8,cym-8,3,cxm,cym,28);gr.addColorStop(0,"#fff3b0");gr.addColorStop(1,"#d49a10");g.fillStyle=gr;g.beginPath();g.arc(cxm,cym,26,0,7);g.fill()}
   else if(bx===1){g.fillStyle="rgba(0,0,0,.2)";g.beginPath();g.arc(cxm+8,cym+8,80,0,7);g.fill();g.fillStyle="#f8fbff";g.strokeStyle="#9ec5e8";g.lineWidth=2;g.beginPath();g.arc(cxm,cym,80,0,7);g.fill();g.stroke();
    for(let a=0;a<18;a++){g.beginPath();g.moveTo(cxm,cym);g.lineTo(cxm+Math.cos(a/18*6.28)*80,cym+Math.sin(a/18*6.28)*80);g.stroke()}
    g.fillStyle="#7fb4e6";g.beginPath();g.arc(cxm,cym,12,0,7);g.fill()}
   else{g.fillStyle="rgba(0,0,0,.25)";g.beginPath();g.arc(cxm+9,cym+9,66,0,7);g.fill();
    const gr=g.createRadialGradient(cxm-20,cym-20,4,cxm,cym,68);gr.addColorStop(0,"#ffffff");gr.addColorStop(.5,"#a8d0f2");gr.addColorStop(1,"#3b82c4");g.fillStyle=gr;g.beginPath();g.arc(cxm,cym,66,0,7);g.fill()}
   g.font="bold 13px system-ui";g.textAlign="center";g.fillStyle="#0f172a";g.fillText(lm,cxm,y+S-10);continue}
  if(r()<.22){g.fillStyle="#e8eef5";g.fillRect(x+5,y+5,S-10,S-10);for(let k=0;k<16;k++)tree(x+20+r()*(S-40),y+20+r()*(S-40),8+r()*6);continue}
  for(let a=0;a<2;a++)for(let b=0;b<2;b++){const lx=x+a*S/2+8,ly=y+b*S/2+8,L=S/2-16,hue=HUES[Math.floor(r()*HUES.length)],li=38+Math.floor(r()*16);
   g.fillStyle="rgba(0,0,0,.3)";g.fillRect(lx+6,ly+6,L,L);
   g.fillStyle=`hsl(${hue},38%,${li}%)`;g.fillRect(lx,ly,L,L);
   g.fillStyle="#eef3f9";g.fillRect(lx+5,ly+5,L-10,L-10);
   g.fillStyle="#9aa7b8";g.fillRect(lx+10+r()*(L-34),ly+10+r()*(L-34),14,10);g.fillRect(lx+10+r()*(L-34),ly+10+r()*(L-34),9,9)}}
 return w}

/* ---- Update ---- */
function update(dt){
 time+=dt;shake=Math.max(0,shake-dt*1.5);for(const q of parts){q.x+=q.vx*dt;q.y+=q.vy*dt;q.l-=dt}parts=parts.filter(q=>q.l>0);if(over)return;tl-=dt;hitCD=Math.max(0,hitCD-dt);
 const stv=(K.right?1:0)-(K.left?1:0),nb=K.nit&&nitro>0&&P.v>60;
 drive(P,dt,K.up||nb?1:K.down?-1:0,stv,nb?P.mv*1.5:P.mv);
 if(nb){P.v=Math.min(P.mv*1.5,P.v+260*dt);nitro-=dt*.5;fx(P,"f")}else nitro=Math.min(1,nitro+dt*.12);
 if(stv&&Math.abs(P.v)>170){const c=Math.cos(P.a),s=Math.sin(P.a),wg=world.getContext("2d");wg.fillStyle="rgba(0,0,0,.3)";for(const w of[-6,6])wg.fillRect(P.x-c*11-s*w,P.y-s*11+c*w,2,2);if(Math.random()<.3)fx(P,"s")}
 cash.forEach(c=>{if(!c.by){const [x,y]=nodeXY(c.n);if(Math.hypot(P.x-x,P.y-y)<40){c.by="p";if(cops.length<5&&cops.length<maxCops+cash.filter(q=>q.by==="p").length)spawnCop();
  $("log").textContent="Қапшық алынды! Полиция күшейді.";beep(880)}}});
 for(const q of pk){if(!q.on){q.cd-=dt;if(q.cd<=0)q.on=true;continue}const [x,y]=nodeXY(q.n);
  if(Math.hypot(P.x-x,P.y-y)<36){q.on=false;q.cd=20;if(q.t==="r"){hp=Math.min(P.hpm,hp+35);$("log").textContent="🔧 Жөндеу: +35 күй"}else{nitro=1;$("log").textContent="⚡ Нитро толды"}beep(660)}}
 for(const p of cops){p.pt-=dt;
  if(p.pt<=0){p.pt=.5;const r=astar(tileOf(p),cops.indexOf(p)%2?ahead():tileOf(P),H);if(r.path){p.path=r.path;
   if(p===cops[0]){const b=bfs(tileOf(p),tileOf(P));lastA={exp:r.exp,bexp:b.exp,len:r.path.length-1};
    $("s2").innerHTML=`${H==="m"?"A*":"UCS"}: <b>${r.exp}</b> түйін, BFS: <b>${b.exp}</b> түйін, жол <b>${r.path.length-1}</b> тақта`}}}
  let tx=P.x,ty=P.y;
  if(p.path&&p.path.length>3&&Math.hypot(P.x-p.x,P.y-p.y)>70){const w=p.path[Math.min(3,p.path.length-1)];tx=(w%WT+.5)*T;ty=(Math.floor(w/WT)+.5)*T}
  const da=angN(Math.atan2(ty-p.y,tx-p.x)-p.a);
  drive(p,dt,Math.abs(da)>1.3?0:1,Math.max(-1,Math.min(1,da*2.2)),copSpd);
  if(Math.hypot(P.x-p.x,P.y-p.y)<24){if(hitCD<=0){hp-=12;hitCD=1;shake=1;$("log").textContent="Полиция соқты! −12"}
   const m=(P.v+p.v)/2;P.v=-Math.abs(m)*.3;p.v=-Math.abs(m)*.3}}
 for(const c of civs){if(c.w>0){c.w-=dt;continue}
  c.s+=95*dt;if(c.s>=8*T){c.n=nextN(c.n,c.d);c.s=0;const o=nbrs(c.n).filter(x=>x[0]!==(c.d+2)%4),l=o.length?o:nbrs(c.n);c.d=l[Math.floor(rand()*l.length)][0]}
  const q=lane(c.n,c.d,c.s);c.x=q.x;c.y=q.y;c.a=q.a;
  if(Math.hypot(P.x-c.x,P.y-c.y)<24){c.w=1.2;if(hitCD<=0){hp-=5;hitCD=.8;shake=.6;$("log").textContent="Жол көлігіне соқты! −5"}P.v*=.3}}
 if(!rival.stop){rival.s+=140*dt;
  if(rival.s>=8*T){rival.n=nextN(rival.n,rival.d);rival.s=0;
   const c=cash.find(c=>c.n===rival.n&&!c.by);if(c){c.by="r";$("log").textContent="Рақиб қапшық алды!";beep(300)}
   pickRival();if(!rival.stop&&rival.n===cash[rival.tg].n){/* already here */}}
  if(!rival.stop){const q=lane(rival.n,rival.d,rival.s);rival.x=q.x;rival.y=q.y;rival.a=q.a}}
 const pc=cash.filter(c=>c.by==="p").length,rc=cash.filter(c=>c.by==="r").length;
 if(pc>=3){over="🎉 ЖЕҢІС! Келесі деңгей жүктелуде...";score+=500+Math.round(hp)*5+Math.round(nitro*100);setTimeout(()=>{level++;$("nc").value=Math.min(5,+$("nc").value+1);$("cs").value=Math.min(260,+$("cs").value+8);$("seed").value=+$("seed").value+1;newGame()},2200)}else if(rc>=3)over="Рақиб-робот 3 қапшық алды. Ұтылдың.";else if(hp<=0)over="Көлік бұзылды. Ұтылдың.";else if(tl<=0)over="Уақыт бітті. Ұтылдың."}

/* ---- Draw ---- */
function draw(){
 const sh=shake*14,camX=Math.max(0,Math.min(WT*T-VW,P.x-VW/2+(Math.random()-.5)*sh)),camY=Math.max(0,Math.min(HT*T-VH,P.y-VH/2+(Math.random()-.5)*sh));
 cx.setTransform(1,0,0,1,0,0);cx.drawImage(world,camX,camY,VW,VH,0,0,VW,VH);
 cx.setTransform(1,0,0,1,-camX,-camY);
 if(dbg&&cops[0]&&cops[0].path){cx.strokeStyle="#22d3ee";cx.lineWidth=4;cx.beginPath();cops[0].path.forEach((t,i)=>{const x=(t%WT+.5)*T,y=(Math.floor(t/WT)+.5)*T;i?cx.lineTo(x,y):cx.moveTo(x,y)});cx.stroke()}
 cx.font="26px serif";cx.textAlign="center";cx.textBaseline="middle";
 cash.forEach(c=>{if(!c.by){const [x,y]=nodeXY(c.n);cx.fillText("💰",x,y+Math.sin(time*4)*3)}});
 pk.forEach(q=>{if(q.on){const [x,y]=nodeXY(q.n);cx.fillText(q.t==="r"?"🔧":"⚡",x,y+Math.sin(time*4)*3)}});
 parts.forEach(q=>{cx.globalAlpha=Math.min(1,q.l*2);cx.fillStyle=q.c;cx.beginPath();cx.arc(q.x,q.y,q.r,0,7);cx.fill()});cx.globalAlpha=1;
 civs.forEach(c=>{if(c.x!==undefined)drawCar2(c,c.k,c.col,false)});
 if(rival.x!==undefined)drawCar2(rival,"rival","#f59e0b",false);
 cops.forEach(p=>drawCar2(p,"police","#1f2937",true));drawCar2(P,P.k,P.col,false);
 cx.setTransform(1,0,0,1,0,0);
 if(night){dg.globalCompositeOperation="source-over";dg.clearRect(0,0,VW,VH);dg.fillStyle=`rgba(4,7,18,${rain?.78:.7})`;dg.fillRect(0,0,VW,VH);dg.globalCompositeOperation="destination-out";
  const lamp=(o,len)=>{dg.save();dg.translate(o.x-camX,o.y-camY);dg.rotate(o.a);let gr=dg.createRadialGradient(len*.55,0,4,len*.55,0,len*.6);gr.addColorStop(0,"rgba(0,0,0,1)");gr.addColorStop(1,"rgba(0,0,0,0)");dg.fillStyle=gr;dg.fillRect(-len,-len,len*2.5,len*2);
   gr=dg.createRadialGradient(0,0,2,0,0,48);gr.addColorStop(0,"rgba(0,0,0,.9)");gr.addColorStop(1,"rgba(0,0,0,0)");dg.fillStyle=gr;dg.fillRect(-60,-60,120,120);dg.restore()};
  lamp(P,200);cops.forEach(p=>lamp(p,120));civs.forEach(c=>{if(c.x!==undefined)lamp(c,70)});if(rival.x!==undefined)lamp(rival,90);
  cash.forEach(c=>{if(!c.by){const [x,y]=nodeXY(c.n),gr=dg.createRadialGradient(x-camX,y-camY,2,x-camX,y-camY,70);gr.addColorStop(0,"rgba(0,0,0,.9)");gr.addColorStop(1,"rgba(0,0,0,0)");dg.fillStyle=gr;dg.fillRect(x-camX-70,y-camY-70,140,140)}});
  dg.globalCompositeOperation="source-over";cx.drawImage(dk,0,0);
  cops.forEach(p=>{const f=Math.floor(time*8)%2,x=p.x-camX,y=p.y-camY,gr=cx.createRadialGradient(x,y,2,x,y,80);gr.addColorStop(0,f?"rgba(255,40,40,.45)":"rgba(40,100,255,.45)");gr.addColorStop(1,"rgba(0,0,0,0)");cx.fillStyle=gr;cx.fillRect(x-80,y-80,160,160)})}
 if(rain){cx.fillStyle="rgba(255,255,255,.8)";for(let i=0;i<130;i++){const x=((i*97+time*30*(1+i%3)+Math.sin(time+i)*20)%VW+VW)%VW,y=(i*53+time*(60+i%5*25))%VH;cx.fillRect(x,y,2+i%3,2+i%3)}}
 cx.fillStyle="rgba(10,12,20,.7)";cx.fillRect(10,10,300,96);
 cx.fillStyle="#fff";cx.font="bold 13px system-ui";cx.textAlign="left";
 const pc=cash.filter(c=>c.by==="p").length,rc=cash.filter(c=>c.by==="r").length;
 cx.fillText(`Деңгей ${level} · Ұпай ${score} · Қапшық ${pc}/${rc} · ⏱${Math.max(0,Math.ceil(tl))}`,18,28);
 cx.fillText("Көлік күйі",18,48);cx.fillStyle="#333";cx.fillRect(100,40,128,12);
 cx.fillStyle=hp>P.hpm*.4?"#22c55e":"#ef4444";cx.fillRect(100,40,128*Math.max(0,hp)/P.hpm,12);
 cx.fillStyle="#fff";cx.fillText("Іздеу: "+"★".repeat(cops.length)+"  "+Math.round(Math.abs(P.v))+" км/сағ",18,70);cx.fillText("Нитро",18,92);cx.fillStyle="#333";cx.fillRect(100,84,128,10);cx.fillStyle="#38bdf8";cx.fillRect(100,84,128*nitro,10);
 const mw=150,mh=100,mx=VW-mw-10,my=10;cx.globalAlpha=.85;cx.drawImage(world,mx,my,mw,mh);cx.globalAlpha=1;
 const dot=(x,y,c,r)=>{cx.fillStyle=c;cx.beginPath();cx.arc(mx+x/(WT*T)*mw,my+y/(HT*T)*mh,r,0,7);cx.fill()};
 cash.forEach(c=>{if(!c.by){const [x,y]=nodeXY(c.n);dot(x,y,"#facc15",3)}});
 cops.forEach(p=>dot(p.x,p.y,"#3b82f6",2.5));if(rival.x!==undefined)dot(rival.x,rival.y,"#f59e0b",3);dot(P.x,P.y,"#fff",3.5);
 if(over){cx.fillStyle="rgba(0,0,0,.6)";cx.fillRect(0,VH/2-50,VW,100);cx.fillStyle="#fff";cx.font="bold 24px system-ui";cx.textAlign="center";
  cx.fillText(over,VW/2,VH/2-8);cx.font="14px system-ui";cx.fillText("Enter — қайта бастау",VW/2,VH/2+22)}}
let last=performance.now();
function loop(n){const dt=Math.min(.05,(n-last)/1000);last=n;update(dt);sound();draw();requestAnimationFrame(loop)}

/* ---- Tests ---- */
function runTests(){
 const out=[];let pass=0,tot=0;const Tt=(n,f)=>{tot++;try{if(f()){pass++;out.push("✔ "+n)}else out.push("✘ "+n)}catch(e){out.push("✘ "+n+" ("+e.message+")")}};
 Tt("A* жолы BFS-пен бірдей ұзындықта (15 жұп)",()=>{const r=rng(9);const rt=[];for(let i=0;i<WT*HT;i++)if(road(i%WT,(i/WT)|0))rt.push(i);
  for(let k=0;k<15;k++){const s=rt[Math.floor(r()*rt.length)],t=rt[Math.floor(r()*rt.length)];
   const a=astar(s,t,"m"),b=bfs(s,t);if(a.path.length-1!==b.len)return false}return true});
 Tt("A* (Manhattan) UCS-тен аз түйін зерттейді",()=>{const s=0,t=WT*HT-1-WT*4-6;let tt=t;while(!road(tt%WT,(tt/WT)|0))tt--;return astar(s,tt,"m").exp<=astar(s,tt,"u").exp});
 Tt("CSP шектеулері орындалған",()=>{const v=[...cash.map(c=>c.n),csp.sol.R];if(new Set(v).size!==6)return false;
  for(const a of v){if(gman(a,8)<2)return false;for(const b of v)if(a!==b&&gman(a,b)<2)return false}return true});
 Tt("Q-саясат кез келген түйінен әр мақсатқа жетеді",()=>{for(let i=0;i<cash.length;i++)for(let n=0;n<NI*NJ;n++){let c=n,st=0;while(c!==cash[i].n&&st<40){c=greedy(Qs[i],c)[1];st++}if(c!==cash[i].n)return false}return true});
 Tt("Қате енгізу: сан емес мән әдепкіге ауысады",()=>clamp("abc",1,5,2)===2&&clamp(99,1,5,2)===5);
 $("log").textContent=`Тест нәтижесі: ${pass}/${tot}\n`+out.join("\n")}

/* ---- Events ---- */
const km={ArrowUp:"up",w:"up",ArrowDown:"down",s:"down",ArrowLeft:"left",a:"left",ArrowRight:"right",d:"right"};
addEventListener("keydown",e=>{audio();if(e.key==="m"&&snd)snd.mute=!snd.mute;if(e.key==="Shift"||e.key===" "){K.nit=1;e.preventDefault()}if(km[e.key]){K[km[e.key]]=1;e.preventDefault()}if(e.key==="Enter"){level=1;score=0;newGame()}});
addEventListener("keyup",e=>{if(km[e.key])K[km[e.key]]=0;if(e.key==="Shift"||e.key===" ")K.nit=0});
document.querySelectorAll(".pad button").forEach(b=>{const k=b.dataset.k;
 b.addEventListener("pointerdown",e=>{e.preventDefault();K[k]=1});["pointerup","pointerleave","pointercancel"].forEach(ev=>b.addEventListener(ev,()=>K[k]=0))});
$("new").onclick=()=>{level=1;score=0;newGame()};$("nt").onclick=()=>{night=!night};$("rn").onclick=()=>{rain=!rain};$("tests").onclick=runTests;
$("dbg").onclick=()=>{dbg=!dbg};
$("theme").onclick=()=>{const r=document.documentElement;r.dataset.theme=r.dataset.theme==="dark"?"light":"dark"};
newGame();requestAnimationFrame(loop);
