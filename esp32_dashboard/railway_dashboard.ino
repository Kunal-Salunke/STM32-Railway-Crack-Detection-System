/*
 * ================================================================
 *  railway_dashboard.ino — Single-file ESP32 Dashboard
 *
 *  STM32 USART3  ->  ESP32 UART2
 *    STM32 PB10 (TX)  ->  ESP32 GPIO16 (RX2)
 *    STM32 PC11 (RX)  <-  ESP32 GPIO17 (TX2)
 *    GND              ->  GND  (MANDATORY)
 *    Baud: 9600
 *
 *  Dashboard: http://<ESP32_IP>:8000
 *
 *  Required: ArduinoJson v7.x  (Sketch > Include Library > Manage Libraries)
 *  Board: ESP32 Dev Module
 * ================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

const char* WIFI_SSID     = "Kunal's iphone";
const char* WIFI_PASSWORD = "123456789";

#define UART_RX  16
#define UART_TX  17
#define UART_BR  9600

WebServer server(8000);

struct SensorState {
  int      distance_cm     = 999;
  bool     ir_crack        = false;
  bool     obstacle_alert  = false;
  bool     crack_alert     = false;
  String   motor_state     = "STOPPED";
  uint32_t total_obstacles = 0;
  uint32_t total_cracks    = 0;
  uint32_t last_update_ms  = 0;
  uint32_t obs_alert_ms    = 0;
  uint32_t crk_alert_ms    = 0;
  int      history[20];
  int      hist_idx        = 0;
  bool     hist_full       = false;
} g;

String uartLine = "";

void parseLine(const String& ln) {
  g.last_update_ms = millis();
  if (ln.indexOf("Distance:") >= 0) {
    String rest = ln.substring(ln.indexOf("Distance:") + 9);
    rest.trim();
    if (rest.startsWith("< 2"))  g.distance_cm = 1;
    else if (rest.startsWith("999")) g.distance_cm = 999;
    else {
      int sp = rest.indexOf(" cm");
      if (sp > 0) g.distance_cm = rest.substring(0, sp).toInt();
    }
    g.history[g.hist_idx] = (g.distance_cm == 999) ? 0 : g.distance_cm;
    g.hist_idx = (g.hist_idx + 1) % 20;
    if (g.hist_idx == 0) g.hist_full = true;
  }
  if (ln.indexOf("IR: 1") >= 0) { g.ir_crack = false; g.motor_state = "FORWARD"; }
  if (ln.indexOf("IR: 0") >= 0) { g.ir_crack = true;  g.motor_state = "STOPPED"; }
  if (ln.indexOf("OBSTACLE DETECTED") >= 0) {
    if (!g.obstacle_alert) g.total_obstacles++;
    g.obstacle_alert = true;
    g.obs_alert_ms = millis();
  }
  if (ln.indexOf("CRACK DETECTED") >= 0) {
    if (!g.crack_alert) g.total_cracks++;
    g.crack_alert = true;
    g.crk_alert_ms = millis();
  }
}

void clearStaleAlerts() {
  uint32_t now = millis();
  if (g.obstacle_alert && (now - g.obs_alert_ms) > 3000) g.obstacle_alert = false;
  if (g.crack_alert    && (now - g.crk_alert_ms) > 3000) g.crack_alert = false;
}

void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache");
  JsonDocument doc;
  doc["distance_cm"]     = g.distance_cm;
  doc["ir_crack"]        = g.ir_crack;
  doc["obstacle_alert"]  = g.obstacle_alert;
  doc["crack_alert"]     = g.crack_alert;
  doc["motor_state"]     = g.motor_state;
  doc["uptime_s"]        = millis() / 1000;
  doc["total_obstacles"] = g.total_obstacles;
  doc["total_cracks"]    = g.total_cracks;
  doc["rssi"]            = WiFi.RSSI();
  doc["stale"]           = (millis() - g.last_update_ms) > 5000;
  JsonArray hist = doc["history"].to<JsonArray>();
  int count = g.hist_full ? 20 : g.hist_idx;
  int start = g.hist_full ? g.hist_idx : 0;
  for (int i = 0; i < count; i++) hist.add(g.history[(start + i) % 20]);
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ── HTML stored in PROGMEM ────────────────────────────────────
// NOTE: JavaScript uses "var x=function(){}" syntax instead of
// "function x(){}" to avoid Arduino IDE 1.8.x pre-parser bug.
const char PAGE[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>Defence System Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&display=swap" rel="stylesheet">
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{--bg:#070d1a;--card:#0f1829;--border:#1a3050;--blue:#38bdf8;--purple:#818cf8;--muted:#475569}
body{background:var(--bg);color:#e2e8f0;font-family:'Inter',system-ui,sans-serif;min-height:100vh}
.topbar{background:linear-gradient(90deg,#0d1520,#111d2e);padding:14px 24px;display:flex;align-items:center;gap:12px;border-bottom:1px solid var(--border);position:sticky;top:0;z-index:99;backdrop-filter:blur(12px)}
.tb-title{font-size:1.1rem;font-weight:800;background:linear-gradient(135deg,#38bdf8,#818cf8,#c084fc);-webkit-background-clip:text;-webkit-text-fill-color:transparent;flex:1}
.pill{display:flex;align-items:center;gap:6px;background:rgba(34,197,94,.08);border:1px solid rgba(34,197,94,.3);border-radius:20px;padding:5px 14px;font-size:.72rem;font-weight:600;color:#4ade80;transition:all .3s}
.pill.stale{background:rgba(245,158,11,.08);border-color:rgba(245,158,11,.3);color:#fbbf24}
.pill.off{background:rgba(239,68,68,.08);border-color:rgba(239,68,68,.3);color:#fca5a5}
.dot{width:8px;height:8px;border-radius:50%;background:#22c55e;animation:blink 1.2s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.15}}
.upt{font-size:.72rem;color:var(--muted)}
.alert{display:none;margin:10px 18px;padding:14px 20px;border-radius:14px;font-weight:700;font-size:.9rem;border:2px solid;align-items:center;gap:10px;animation:flash .65s infinite alternate}
.alert.show{display:flex}
.a-obs{background:rgba(239,68,68,.06);border-color:#ef4444;color:#fca5a5}
.a-crk{background:rgba(245,158,11,.06);border-color:#f59e0b;color:#fcd34d}
@keyframes flash{from{opacity:1}to{opacity:.45}}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:14px;padding:18px}
.card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:18px;transition:all .25s ease;position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;background:linear-gradient(90deg,transparent,var(--blue),var(--purple),transparent);opacity:0;transition:opacity .3s}
.card:hover{transform:translateY(-4px);box-shadow:0 12px 32px rgba(56,189,248,.1)}.card:hover::before{opacity:1}
.lbl{font-size:.7rem;text-transform:uppercase;letter-spacing:.12em;color:var(--muted);margin-bottom:12px;font-weight:600}
.val{font-size:2rem;font-weight:800;background:linear-gradient(135deg,#38bdf8,#818cf8);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{font-size:.75rem;color:var(--muted);margin-top:6px}
.gauge-w{position:relative;width:120px;height:120px;margin:0 auto}
.gauge-w svg{transform:rotate(-90deg)}
.gc{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center}
.gnum{font-size:1.4rem;font-weight:800;color:#38bdf8}
.gunit{font-size:.6rem;color:var(--muted);margin-top:2px}
.badge{display:inline-block;padding:6px 16px;border-radius:20px;font-size:.8rem;font-weight:700;margin-top:8px;transition:all .3s}
.b-fwd{background:rgba(34,197,94,.1);color:#4ade80;border:1px solid rgba(34,197,94,.3)}
.b-stp{background:rgba(148,163,184,.06);color:#94a3b8;border:1px solid rgba(148,163,184,.15)}
.ir-ok{color:#22c55e;font-size:1.8rem;font-weight:800}
.ir-crk{color:#f59e0b;font-size:1.8rem;font-weight:800;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}
.section{margin:0 18px 18px;background:var(--card);border:1px solid var(--border);border-radius:16px;padding:18px}
.sec-lbl{font-size:.8rem;color:#94a3b8;font-weight:600;margin-bottom:14px}
canvas{display:block;width:100%!important}
.log{background:#050a12;border-radius:10px;padding:12px;height:130px;overflow-y:auto;font-family:'Courier New',monospace;font-size:.72rem;scrollbar-width:thin;scrollbar-color:#1a3050 transparent}
.l-d{color:#38bdf8}.l-i{color:#a78bfa}.l-o{color:#f87171;font-weight:700}.l-c{color:#fbbf24;font-weight:700}.l-m{color:#475569}
.footer{text-align:center;padding:16px;color:#1a3050;font-size:.68rem;border-top:1px solid #0d1520}
@media(max-width:540px){.grid{grid-template-columns:1fr 1fr;padding:10px;gap:10px}.section{margin:0 10px 12px}.topbar{padding:10px 14px}}
</style></head><body>
<div class="topbar">
  <span class="tb-title">&#9741; Defence System Dashboard</span>
  <span class="upt" id="upt">--</span>
  <span class="pill" id="pill"><span class="dot" id="dot"></span><span id="ltxt">Connecting</span></span>
</div>
<div class="alert a-obs" id="a-obs">&#9888; OBSTACLE DETECTED - Distance critical!</div>
<div class="alert a-crk" id="a-crk">&#9888; WARNING - CRACK / SURFACE BREAK DETECTED!</div>
<div class="grid">
  <div class="card">
    <div class="lbl">Ultrasonic Distance</div>
    <div class="gauge-w">
      <svg width="120" height="120" viewBox="0 0 120 120">
        <circle cx="60" cy="60" r="50" fill="none" stroke="#1a3050" stroke-width="8"/>
        <circle cx="60" cy="60" r="50" fill="none" stroke="url(#gg)" stroke-width="8"
          stroke-dasharray="314" stroke-dashoffset="314" stroke-linecap="round" id="arc"/>
        <defs><linearGradient id="gg" x1="0%" y1="0%" x2="100%" y2="0%">
          <stop offset="0%" stop-color="#38bdf8"/><stop offset="100%" stop-color="#818cf8"/>
        </linearGradient></defs>
      </svg>
      <div class="gc"><span class="gnum" id="dval">--</span><span class="gunit" id="dunit">cm</span></div>
    </div>
    <div class="sub" style="text-align:center;margin-top:8px" id="dst">Waiting...</div>
  </div>
  <div class="card">
    <div class="lbl">IR Surface Sensor</div>
    <div class="ir-ok" id="irv">--</div>
    <div class="sub" id="irs">Waiting for data</div>
  </div>
  <div class="card">
    <div class="lbl">Motor State</div>
    <span class="badge b-stp" id="mbadge">STOPPED</span>
    <div class="sub" style="margin-top:10px">L298N drive control</div>
  </div>
  <div class="card">
    <div class="lbl">Obstacle Events</div>
    <div class="val" id="ocnt">0</div>
    <div class="sub">Total since boot</div>
  </div>
  <div class="card">
    <div class="lbl">Crack Events</div>
    <div class="val" id="ccnt">0</div>
    <div class="sub">Total since boot</div>
  </div>
  <div class="card">
    <div class="lbl">WiFi Signal</div>
    <div class="val" id="rssi">--</div>
    <div class="sub">RSSI (dBm)</div>
  </div>
</div>
<div class="section">
  <div class="sec-lbl">&#128200; Distance History (last 20 readings)</div>
  <canvas id="chart" height="150"></canvas>
</div>
<div class="section">
  <div class="sec-lbl">&#9654; Live STM32 Log</div>
  <div class="log" id="logbox"></div>
</div>
<div class="footer">STM32F446RE USART3 (PB10/PC11) &harr; ESP32 GPIO16/17 &bull; 9600 baud &bull; Port 8000</div>
<script>
var cvs=document.getElementById('chart'),ctx=cvs.getContext('2d');
var drawChart=function(data){
  var W=cvs.offsetWidth||500,H=150;cvs.width=W;cvs.height=H;
  if(!data||data.length<2)return;
  var MX=Math.max.apply(null,data.concat([30])),p={t:10,r:10,b:28,l:44};
  var cw=W-p.l-p.r,ch=H-p.t-p.b;
  ctx.clearRect(0,0,W,H);
  var fracs=[0,.25,.5,.75,1];
  for(var fi=0;fi<fracs.length;fi++){
    var f=fracs[fi],y=p.t+ch*(1-f);
    ctx.strokeStyle='#162030';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(p.l+cw,y);ctx.stroke();
    ctx.fillStyle='#334155';ctx.font='10px monospace';
    ctx.fillText(Math.round(MX*f)+'cm',2,y+4);
  }
  var X=function(i){return p.l+i*(cw/(data.length-1));};
  var Y=function(v){return p.t+ch*(1-Math.min(v,MX)/MX);};
  var g2=ctx.createLinearGradient(0,p.t,0,p.t+ch);
  g2.addColorStop(0,'rgba(56,189,248,.25)');g2.addColorStop(1,'rgba(129,140,248,0)');
  ctx.beginPath();
  for(var i=0;i<data.length;i++){if(i===0)ctx.moveTo(X(i),Y(data[i]));else ctx.lineTo(X(i),Y(data[i]));}
  ctx.lineTo(X(data.length-1),p.t+ch);ctx.lineTo(p.l,p.t+ch);ctx.closePath();
  ctx.fillStyle=g2;ctx.fill();
  ctx.beginPath();ctx.lineWidth=2.5;ctx.lineJoin='round';
  var lg=ctx.createLinearGradient(0,0,W,0);
  lg.addColorStop(0,'#38bdf8');lg.addColorStop(1,'#818cf8');
  ctx.strokeStyle=lg;
  for(var i=0;i<data.length;i++){if(i===0)ctx.moveTo(X(i),Y(data[i]));else ctx.lineTo(X(i),Y(data[i]));}
  ctx.stroke();
  for(var i=0;i<data.length;i++){ctx.beginPath();ctx.arc(X(i),Y(data[i]),3.5,0,Math.PI*2);ctx.fillStyle='#818cf8';ctx.fill();ctx.strokeStyle='#0f1829';ctx.lineWidth=1.5;ctx.stroke();}
};
var lb=document.getElementById('logbox');
var logs=[];
var getTs=function(){var d=new Date();return d.toLocaleTimeString();};
var addLog=function(txt,cls){
  logs.push('<div class="'+cls+'"><span style="color:#334155">['+getTs()+']</span> '+txt+'</div>');
  if(logs.length>60)logs.shift();
  lb.innerHTML=logs.join('');lb.scrollTop=lb.scrollHeight;
};
var pObs=false,pCrk=false,pDist=null,pIr=null,fails=0;
var poll=function(){
  var xhr=new XMLHttpRequest();
  xhr.open('GET','/api/data',true);
  xhr.onload=function(){
    if(xhr.status!==200)return;
    try{
    var d=JSON.parse(xhr.responseText);fails=0;
    var cm=d.distance_cm===999?null:d.distance_cm;
    document.getElementById('dval').textContent=cm!==null?cm:'N/A';
    document.getElementById('dunit').textContent=cm!==null?'cm':'';
    document.getElementById('arc').setAttribute('stroke-dashoffset',314*(1-Math.min((cm||0)/400,1)));
    var dstEl=document.getElementById('dst');
    if(cm===null){dstEl.textContent='Out of range';dstEl.style.color='#475569';}
    else if(cm<2){dstEl.textContent='Blind zone';dstEl.style.color='#f59e0b';}
    else if(cm<20){dstEl.textContent='Very close!';dstEl.style.color='#ef4444';}
    else if(cm<80){dstEl.textContent='Approaching';dstEl.style.color='#f59e0b';}
    else{dstEl.textContent='Clear';dstEl.style.color='#22c55e';}
    if(cm!==pDist){addLog('Distance: '+(cm!==null?cm+' cm':'out of range'),'l-d');pDist=cm;}
    var iv=document.getElementById('irv'),is2=document.getElementById('irs');
    if(d.ir_crack){iv.textContent='CRACK';iv.className='ir-crk';is2.textContent='Surface break detected';}
    else{iv.textContent='OK';iv.className='ir-ok';is2.textContent='Surface intact';}
    if(d.ir_crack!==pIr){addLog('IR: '+(d.ir_crack?'0 CRACK':'1 OK'),d.ir_crack?'l-c':'l-i');pIr=d.ir_crack;}
    var mb=document.getElementById('mbadge');
    mb.textContent=d.motor_state;
    mb.className='badge '+(d.motor_state==='FORWARD'?'b-fwd':'b-stp');
    document.getElementById('ocnt').textContent=d.total_obstacles;
    document.getElementById('ccnt').textContent=d.total_cracks;
    document.getElementById('rssi').textContent=d.rssi!==undefined?d.rssi+' dBm':'--';
    var u=d.uptime_s||0;
    document.getElementById('upt').textContent='Uptime: '+(u>=3600?Math.floor(u/3600)+'h ':'')+(u%3600>=60?Math.floor((u%3600)/60)+'m ':'')+u%60+'s';
    document.getElementById('a-obs').className='alert a-obs'+(d.obstacle_alert?' show':'');
    document.getElementById('a-crk').className='alert a-crk'+(d.crack_alert?' show':'');
    if(d.obstacle_alert&&!pObs)addLog('*** OBSTACLE DETECTED ***','l-o');
    if(d.crack_alert&&!pCrk)addLog('*** WARNING!! CRACK DETECTED ***','l-c');
    pObs=d.obstacle_alert;pCrk=d.crack_alert;
    var pill=document.getElementById('pill');
    pill.className='pill'+(d.stale?' stale':'');
    document.getElementById('dot').style.background=d.stale?'#f59e0b':'#22c55e';
    document.getElementById('ltxt').textContent=d.stale?'No STM32 data':'Live';
    if(d.history&&d.history.length>1)drawChart(d.history);
    }catch(e){fails++;}
  };
  xhr.onerror=function(){
    fails++;
    document.getElementById('pill').className='pill off';
    document.getElementById('dot').style.background='#ef4444';
    document.getElementById('ltxt').textContent='Disconnected';
    if(fails<=3)addLog('Connection error','l-m');
  };
  xhr.send();
};
poll();setInterval(poll,1000);window.addEventListener('resize',poll);
</script></body></html>)rawhtml";

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", PAGE);
}

void setup() {
  Serial.begin(115200);
  memset(g.history, 0, sizeof(g.history));
  Serial2.begin(UART_BR, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("\n[ESP32] UART2 ready on GPIO16(RX)/GPIO17(TX) @ 9600");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[ESP32] Connecting to WiFi: %s ", WIFI_SSID);
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t++ < 30) { delay(500); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[OK] Dashboard -> http://%s:8000\n", WiFi.localIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("DefenceSystem-AP", "defence123");
    Serial.println("\n[AP] WiFi failed - Hotspot created");
    Serial.println("[AP] Connect to: DefenceSystem-AP  |  Password: defence123");
    Serial.println("[AP] Dashboard -> http://192.168.4.1:8000");
  }
  server.on("/",         HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.onNotFound([]() { server.send(404, "text/plain", "Not Found"); });
  server.begin();
  Serial.println("[ESP32] HTTP server running on port 8000");
}

void loop() {
  server.handleClient();
  clearStaleAlerts();
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      uartLine.trim();
      if (uartLine.length() > 0) {
        Serial.println("[STM32] " + uartLine);
        parseLine(uartLine);
      }
      uartLine = "";
    } else if (c != '\r') {
      if (uartLine.length() < 128) uartLine += c;
    }
  }
}