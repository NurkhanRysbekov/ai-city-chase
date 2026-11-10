// Браузерсіз тест: node tests/headless.js
// DOM мен canvas жалған (stub) объектілермен ауыстырылады, ойын логикасы іске қосылады.
const fs = require("fs");
const path = require("path");

const vals = { car: "sport", seed: "5", nc: "2", cs: "200", heur: "m", eps: "600" };
const els = {};
const mk = (id) => {
  const o = { value: vals[id] || "", dataset: {}, style: {}, innerHTML: "", textContent: "" };
  return new Proxy(o, {
    get(t, p) { return p in t ? t[p] : () => mk("x"); },
    set(t, p, v) { t[p] = v; return true; },
  });
};
const ctx = new Proxy({}, {
  get(t, p) {
    if (p in t) return t[p];
    if (p === "createRadialGradient") return () => ({ addColorStop() {} });
    if (p === "measureText") return () => ({ width: 10 });
    return () => {};
  },
  set(t, p, v) { t[p] = v; return true; },
});
global.document = {
  getElementById: (id) => els[id] || (els[id] = Object.assign(mk(id), { getContext: () => ctx })),
  createElement: () => ({ getContext: () => ctx, width: 0, height: 0 }),
  documentElement: { dataset: {} },
  querySelectorAll: () => [],
};
global.addEventListener = () => {};
global.requestAnimationFrame = () => {};
global.window = global;

const code = fs.readFileSync(path.join(__dirname, "..", "js", "game.js"), "utf8") + `
runTests();
const logText = document.getElementById("log").textContent;
console.log(logText);
// 60 секундтық симуляция
K.up = 1; K.nit = 1;
for (let i = 0; i < 3600; i++) { K.right = (i % 90 < 20) ? 1 : 0; update(1 / 60); if (i % 300 === 0) draw(); }
newGame(); K.up = 0; K.right = 0;
for (let i = 0; i < 7200 && !over; i++) update(1 / 60);
draw();
console.log("Симуляция аяқталды:", over);
if (!/(\\d+)\\/\\1/.test(logText)) { console.error("ТЕСТ ҚАТЕСІ"); process.exit(1); }
`;
eval(code);
