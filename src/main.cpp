#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <QTRSensors.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>
#include <Servo.h>
#include <PIDino.hpp>

Servo M1, M2;
QTRSensors qtr;
AsyncWebServer server(80);

bool driveEnabled = false;
int baseSpeed = 1000;

float kp = 1.0;
float ki = 0.0;
float kd = 0.0;
float setP = 500;

PIDino PID1(setP, kp, ki, kd, 1000);

constexpr uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

const char* ssid = "Fezaro";
const char* password = "autoskap";


String htmlPage() {
    String html = R"rawliteral(
    <!DOCTYPE html><html><head><title>PID Control</title></head>
    <body>

    <h2>Enable Robot</h2>
    <label class="switch">
        <input type="checkbox" onchange="toggleDrive(this)" id="drive" %STATE%>
        <span class="slider"></span>
    </label>

    <h3>Base Speed: <span id="speedVal">%SPEED%</span></h3>
    <input type="range" min="1000" max="2000" value="%SPEED%"
           id="speedSlider" oninput="updateSpeed(this.value)">

    <h3>Kp: <span id="kpVal">%KP%</span></h3>
    <input type="range" min="0" max="300" value="%KPR%" oninput="updateKp(this.value)">

    <h3>Ki: <span id="kiVal">%KI%</span></h3>
    <input type="range" min="0" max="300" value="%KIR%" oninput="updateKi(this.value)">

    <h3>Kd: <span id="kdVal">%KD%</span></h3>
    <input type="range" min="0" max="300" value="%KDR%" oninput="updateKd(this.value)">

    <h3>SetPoint: <span id="spVal">%SP%</span></h3>
    <input type="range" min="0" max="7000" value="%SP%" oninput="updateSP(this.value)">

<script>

function toggleDrive(el){
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/toggle?state=" + (el.checked ? 1 : 0), true);
    xhr.send();
}

function updateSpeed(val){
    document.getElementById("speedVal").innerText = val;
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/setSpeed?value=" + val, true);
    xhr.send();
}

function updateKp(val){
    document.getElementById("kpVal").innerText = (val/100).toFixed(2);
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/setKp?value=" + val, true);
    xhr.send();
}

function updateKi(val){
    document.getElementById("kiVal").innerText = (val/100).toFixed(2);
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/setKi?value=" + val, true);
    xhr.send();
}

function updateKd(val){
    document.getElementById("kdVal").innerText = (val/100).toFixed(2);
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/setKd?value=" + val, true);
    xhr.send();
}

function updateSP(val){
    document.getElementById("spVal").innerText = val;
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/setSP?value=" + val, true);
    xhr.send();
}

</script>

<style>
body { font-family: sans-serif; padding: 20px; }
.switch { position: relative; display: inline-block; width: 60px; height: 34px; }
.switch input { opacity: 0; width: 0; height: 0; }
.slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0;
         background-color: #ccc; transition: .4s; border-radius: 34px; }
.slider:before { position: absolute; content: ""; height: 26px; width: 26px;
                 left: 4px; bottom: 4px; background-color: white; transition: .4s; border-radius: 50%; }
input:checked + .slider { background-color: #2196F3; }
input:checked + .slider:before { transform: translateX(26px); }
</style>

    </body></html>
    )rawliteral";

    html.replace("%STATE%", driveEnabled ? "checked" : "");
    html.replace("%SPEED%", String(baseSpeed));

    html.replace("%KP%", String(kp, 2));
    html.replace("%KI%", String(ki, 2));
    html.replace("%KD%", String(kd, 2));

    html.replace("%KPR%", String((int)(kp * 100)));
    html.replace("%KIR%", String((int)(ki * 100)));
    html.replace("%KDR%", String((int)(kd * 100)));

    html.replace("%SP%", String(setP));

    return html;
}

void setup()
{
    Serial.begin(115200);

    M1.attach(0);
    M2.attach(2);

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){1, 3}, SensorCount);

    delay(500);
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    for (uint16_t i = 0; i < 200; i++)
    {
        qtr.calibrate();
    }
    digitalWrite(8, HIGH);

    // WiFi
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", htmlPage());
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", htmlPage());
    });

    server.on("/toggle", HTTP_GET, [](AsyncWebServerRequest *request){
        driveEnabled = request->getParam("state")->value() == "1";
        request->send(200, "text/plain", "OK");
    });

    server.on("/setSpeed", HTTP_GET, [](AsyncWebServerRequest *request){
        baseSpeed = request->getParam("value")->value().toInt();
        request->send(200, "text/plain", "OK");
    });

    server.on("/setKp", HTTP_GET, [](AsyncWebServerRequest *request){
        kp = request->getParam("value")->value().toInt() / 100.0;
        PID1.setKp(kp);
        request->send(200, "text/plain", "OK");
    });

    server.on("/setKi", HTTP_GET, [](AsyncWebServerRequest *request){
        ki = request->getParam("value")->value().toInt() / 100.0;
        PID1.setKi(ki);
        request->send(200, "text/plain", "OK");
    });

    server.on("/setKd", HTTP_GET, [](AsyncWebServerRequest *request){
        kd = request->getParam("value")->value().toInt() / 100.0;
        PID1.setKd(kd);
        request->send(200, "text/plain", "OK");
    });

    server.on("/setSP", HTTP_GET, [](AsyncWebServerRequest *request){
        setP = request->getParam("value")->value().toFloat();
        PID1.setSetPoint(setP);
        request->send(200, "text/plain", "OK");
    });

    server.begin();
}

void loop(){
    if (!driveEnabled) {
        M1.writeMicroseconds(1000);
        M2.writeMicroseconds(1000);
        return;
    }

    const uint16_t input = qtr.readLineBlack(sensorValues);

    const float pidOut = PID1.compute(input);

    int m1 = baseSpeed + (pidOut - 1000);
    int m2 = baseSpeed + (1000 - pidOut);

    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);

    M1.writeMicroseconds(m1);
    M2.writeMicroseconds(m2);
}
