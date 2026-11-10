# AI City Chase — Қысқы Астана

QAIRU AI Fundamentals Final Project 2026. Браузерде ойналатын көлік жарысы: қапшықтағы ақшаны рақиб-роботтан бұрын жинап, полициядан қашу керек.

## AI тақырыптары
| Апта | Тақырып | Ойындағы орны | Код (`js/game.js`) |
|---|---|---|---|
| 1 | Интеллектуалды агенттер | Қала көліктері (рефлекс), полиция (мақсатқа бағытталған), рақиб (үйренетін) | `update()` |
| 2 | A* іздеу | Полиция ойыншыға дейінгі жолды табады; BFS-пен салыстыру | `astar()`, `bfs()` |
| 3 | CSP | Ақша мен рақиб старты қиылыстарға орналасады | `solveCSP()` |
| 5 | Q-learning | Рақиб әр ақшаға баратын жолды үйренеді | `trainQ()`, `greedy()` |

## Іске қосу
Браузерде `index.html` файлын тікелей ашуға болады, бірақ жергілікті сервер ыңғайлырақ:

```bash
python -m http.server 8000     # http://localhost:8000
# немесе
npx serve .
```

VS Code-та: **Live Server** кеңейтімін орнатып, `index.html` үстінде оң жақ батырма → *Open with Live Server*.

## Басқару
WASD / көрсеткілер — жүргізу, Shift немесе Бос орын — нитро, M — дыбыс, Enter — қайта бастау.

## Тесттер
- Ойын ішінде: **«Тесттер»** батырмасы (5 тест).
- Терминалда (Node.js 18+): `npm test`

## Құрылымы
```
index.html        бет
css/style.css     стиль
js/game.js        ойын және барлық AI алгоритмдері
tests/headless.js браузерсіз тест
.github/workflows/test.yml   GitHub Actions
```

## GitHub-қа жүктеу
```bash
git init
git add .
git commit -m "AI City Chase: first version"
git branch -M main
git remote add origin https://github.com/<логин>/ai-city-chase.git
git push -u origin main
```
GitHub Pages: репозиторий → Settings → Pages → Branch: `main` / root. Сайт `https://<логин>.github.io/ai-city-chase/` мекенжайында ашылады.

## Шектеулер
- Полиция қозғалыс физикасын A* жолымен ғана басқарады, сондықтан бұрыштарда кейде ілініп қалуы мүмкін.
- Q-learning тек қиылыстар графында үйренеді, көлік физикасын білмейді.
- Дыбыс браузерде алдымен бір перне басқаннан кейін қосылады.

## AI қолдану туралы
Код Claude (Anthropic) көмегімен жасалды. 

## Автор
Рысбеков Нұрхан

## Лицензия
MIT, `LICENSE` файлын қараңыз.
