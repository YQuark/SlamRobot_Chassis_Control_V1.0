#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <math.h>
#include <string.h>

static const char *AP_SSID = "F407-ESP01S";
static const char *AP_PASS = "12345678";
static const uint32_t WIFI_CFG_MAGIC = 0xA55A5AA5UL;
static const uint32_t SERIAL_BAUD = 115200UL;
static const uint16_t EVENT_COUNT = 10U;
static const uint16_t EVENT_LINE_LEN = 72U;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000UL;

static const uint8_t PROTO_HEAD0 = 0xA5U;
static const uint8_t PROTO_HEAD1 = 0x5AU;
static const uint8_t CMD_SET_VELOCITY = 0x01U;
static const uint8_t CMD_ESTOP = 0x02U;
static const uint8_t CMD_STATUS = 0x81U;
static const uint8_t VELOCITY_PAYLOAD_LEN = 10U;
static const uint8_t ESTOP_PAYLOAD_LEN = 1U;
static const uint8_t STATUS_PAYLOAD_LEN = 45U;
static const uint8_t MAX_PAYLOAD_LEN = 64U;

typedef struct
{
  uint32_t magic;
  char ssid[33];
  char pass[65];
  uint16_t checksum;
} wifi_cfg_t;

typedef struct
{
  float left_speed;
  float right_speed;
  int32_t left_encoder;
  int32_t right_encoder;
  float battery_voltage;
  float left_current;
  float right_current;
  int16_t imu_accel[3];
  int16_t imu_gyro[3];
  uint32_t error_flags;
  uint8_t control_mode;
  uint32_t last_status_ms;
} robot_status_t;

typedef enum
{
  RX_WAIT_HEAD0 = 0,
  RX_WAIT_HEAD1,
  RX_WAIT_LEN,
  RX_WAIT_BODY
} rx_state_t;

ESP8266WebServer server(80);
static robot_status_t robot_status = {0};
static rx_state_t rx_state = RX_WAIT_HEAD0;
static uint8_t rx_buf[MAX_PAYLOAD_LEN + 3U] = {0};
static uint8_t rx_len = 0U;
static uint8_t rx_index = 0U;
static char events[EVENT_COUNT][EVENT_LINE_LEN] = {{0}};
static uint8_t event_head = 0U;
static char sta_target_ssid[33] = {0};
static bool sta_connecting = false;
static uint32_t sta_connect_started_ms = 0U;

static uint16_t checksum16(const uint8_t *data, uint16_t len)
{
  uint16_t sum = 0U;
  for (uint16_t i = 0U; i < len; ++i)
  {
    sum = (uint16_t)(sum + data[i]);
  }
  return sum;
}

static uint8_t checksum8(const uint8_t *data, uint16_t len)
{
  static const uint8_t crc8_table[256] = {
    0x00U, 0x5EU, 0xBCU, 0xE2U, 0x61U, 0x3FU, 0xDDU, 0x83U,
    0xC2U, 0x9CU, 0x7EU, 0x20U, 0xA3U, 0xFDU, 0x1FU, 0x41U,
    0x9DU, 0xC3U, 0x21U, 0x7FU, 0xFCU, 0xA2U, 0x40U, 0x1EU,
    0x5FU, 0x01U, 0xE3U, 0xBDU, 0x3EU, 0x60U, 0x82U, 0xDCU,
    0x23U, 0x7DU, 0x9FU, 0xC1U, 0x42U, 0x1CU, 0xFEU, 0xA0U,
    0xE1U, 0xBFU, 0x5DU, 0x03U, 0x80U, 0xDEU, 0x3CU, 0x62U,
    0xBEU, 0xE0U, 0x02U, 0x5CU, 0xDFU, 0x81U, 0x63U, 0x3DU,
    0x7CU, 0x22U, 0xC0U, 0x9EU, 0x1DU, 0x43U, 0xA1U, 0xFFU,
    0x46U, 0x18U, 0xFAU, 0xA4U, 0x27U, 0x79U, 0x9BU, 0xC5U,
    0x84U, 0xDAU, 0x38U, 0x66U, 0xE5U, 0xBBU, 0x59U, 0x07U,
    0xDBU, 0x85U, 0x67U, 0x39U, 0xBAU, 0xE4U, 0x06U, 0x58U,
    0x19U, 0x47U, 0xA5U, 0xFBU, 0x78U, 0x26U, 0xC4U, 0x9AU,
    0x65U, 0x3BU, 0xD9U, 0x87U, 0x04U, 0x5AU, 0xB8U, 0xE6U,
    0xA7U, 0xF9U, 0x1BU, 0x45U, 0xC6U, 0x98U, 0x7AU, 0x24U,
    0xF8U, 0xA6U, 0x44U, 0x1AU, 0x99U, 0xC7U, 0x25U, 0x7BU,
    0x3AU, 0x64U, 0x86U, 0xD8U, 0x5BU, 0x05U, 0xE7U, 0xB9U,
    0x8CU, 0xD2U, 0x30U, 0x6EU, 0xEDU, 0xB3U, 0x51U, 0x0FU,
    0x4EU, 0x10U, 0xF2U, 0xACU, 0x2FU, 0x71U, 0x93U, 0xCDU,
    0x11U, 0x4FU, 0xADU, 0xF3U, 0x70U, 0x2EU, 0xCCU, 0x92U,
    0xD3U, 0x8DU, 0x6FU, 0x31U, 0xB2U, 0xECU, 0x0EU, 0x50U,
    0xAFU, 0xF1U, 0x13U, 0x4DU, 0xCEU, 0x90U, 0x72U, 0x2CU,
    0x6DU, 0x33U, 0xD1U, 0x8FU, 0x0CU, 0x52U, 0xB0U, 0xEEU,
    0x32U, 0x6CU, 0x8EU, 0xD0U, 0x53U, 0x0DU, 0xEFU, 0xB1U,
    0xF0U, 0xAEU, 0x4CU, 0x12U, 0x91U, 0xCFU, 0x2DU, 0x73U,
    0xCAU, 0x94U, 0x76U, 0x28U, 0xABU, 0xF5U, 0x17U, 0x49U,
    0x08U, 0x56U, 0xB4U, 0xEAU, 0x69U, 0x37U, 0xD5U, 0x8BU,
    0x57U, 0x09U, 0xEBU, 0xB5U, 0x36U, 0x68U, 0x8AU, 0xD4U,
    0x95U, 0xCBU, 0x29U, 0x77U, 0xF4U, 0xAAU, 0x48U, 0x16U,
    0xE9U, 0xB7U, 0x55U, 0x0BU, 0x88U, 0xD6U, 0x34U, 0x6AU,
    0x2BU, 0x75U, 0x97U, 0xC9U, 0x4AU, 0x14U, 0xF6U, 0xA8U,
    0x74U, 0x2AU, 0xC8U, 0x96U, 0x15U, 0x4BU, 0xA9U, 0xF7U,
    0xB6U, 0xE8U, 0x0AU, 0x54U, 0xD7U, 0x89U, 0x6BU, 0x35U,
  };
  uint8_t crc = 0U;

  for (uint16_t i = 0U; i < len; ++i)
  {
    crc = crc8_table[crc ^ data[i]];
  }
  return crc;
}

static void logEvent(const char *text)
{
  if (text == nullptr)
  {
    return;
  }
  snprintf(events[event_head], EVENT_LINE_LEN, "[%lus] %s", millis() / 1000UL, text);
  event_head = (uint8_t)((event_head + 1U) % EVENT_COUNT);
}

static void writeU32LE(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value & 0xFFU);
  out[1] = (uint8_t)((value >> 8) & 0xFFU);
  out[2] = (uint8_t)((value >> 16) & 0xFFU);
  out[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint32_t readU32LE(const uint8_t *in)
{
  return ((uint32_t)in[0]) |
         ((uint32_t)in[1] << 8) |
         ((uint32_t)in[2] << 16) |
         ((uint32_t)in[3] << 24);
}

static int32_t readI32LE(const uint8_t *in)
{
  return (int32_t)readU32LE(in);
}

static int16_t readI16LE(const uint8_t *in)
{
  return (int16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static void writeFloatLE(uint8_t *out, float value)
{
  uint32_t raw = 0U;
  memcpy(&raw, &value, sizeof(raw));
  writeU32LE(out, raw);
}

static float readFloatLE(const uint8_t *in)
{
  uint32_t raw = readU32LE(in);
  float value = 0.0f;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

static uint16_t buildFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint16_t out_len)
{
  uint8_t cmd_len = (uint8_t)(1U + payload_len);
  uint16_t frame_len = (uint16_t)cmd_len + 4U;

  if (out == nullptr || out_len < frame_len || payload_len > MAX_PAYLOAD_LEN)
  {
    return 0U;
  }
  out[0] = PROTO_HEAD0;
  out[1] = PROTO_HEAD1;
  out[2] = cmd_len;
  out[3] = cmd;
  if (payload_len > 0U && payload != nullptr)
  {
    memcpy(&out[4], payload, payload_len);
  }
  out[frame_len - 1U] = checksum8(&out[2], (uint16_t)cmd_len + 1U);
  return frame_len;
}

static bool sendVelocity(float linear_x, float angular_z, uint8_t enable)
{
  uint8_t payload[VELOCITY_PAYLOAD_LEN] = {0};
  uint8_t frame[VELOCITY_PAYLOAD_LEN + 8U] = {0};
  uint16_t frame_len;

  writeFloatLE(&payload[0], linear_x);
  writeFloatLE(&payload[4], angular_z);
  payload[8] = enable;
  payload[9] = 0U;
  frame_len = buildFrame(CMD_SET_VELOCITY, payload, sizeof(payload), frame, sizeof(frame));
  if (frame_len == 0U)
  {
    return false;
  }
  return Serial.write(frame, frame_len) == frame_len;
}

static bool sendEstop(uint8_t enabled)
{
  uint8_t payload[ESTOP_PAYLOAD_LEN] = {enabled};
  uint8_t frame[ESTOP_PAYLOAD_LEN + 8U] = {0};
  uint16_t frame_len = buildFrame(CMD_ESTOP, payload, sizeof(payload), frame, sizeof(frame));

  if (frame_len == 0U)
  {
    return false;
  }
  return Serial.write(frame, frame_len) == frame_len;
}

static void parseStatus(const uint8_t *payload, uint8_t payload_len)
{
  uint8_t offset = 0U;
  if (payload == nullptr || payload_len != STATUS_PAYLOAD_LEN)
  {
    return;
  }

  robot_status.left_speed = readFloatLE(&payload[offset]); offset = (uint8_t)(offset + 4U);
  robot_status.right_speed = readFloatLE(&payload[offset]); offset = (uint8_t)(offset + 4U);
  robot_status.left_encoder = readI32LE(&payload[offset]); offset = (uint8_t)(offset + 4U);
  robot_status.right_encoder = readI32LE(&payload[offset]); offset = (uint8_t)(offset + 4U);
  robot_status.battery_voltage = readFloatLE(&payload[offset]); offset = (uint8_t)(offset + 4U);
  robot_status.left_current = readFloatLE(&payload[offset]); offset = (uint8_t)(offset + 4U);
  robot_status.right_current = readFloatLE(&payload[offset]); offset = (uint8_t)(offset + 4U);
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    robot_status.imu_accel[i] = readI16LE(&payload[offset]);
    offset = (uint8_t)(offset + 2U);
  }
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    robot_status.imu_gyro[i] = readI16LE(&payload[offset]);
    offset = (uint8_t)(offset + 2U);
  }
  robot_status.error_flags = readU32LE(&payload[offset]); offset = (uint8_t)(offset + 4U);
  robot_status.control_mode = payload[offset];
  robot_status.last_status_ms = millis();
}

static void handleFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  if (cmd == CMD_STATUS)
  {
    parseStatus(payload, payload_len);
  }
}

static void resetParser(void)
{
  rx_state = RX_WAIT_HEAD0;
  rx_len = 0U;
  rx_index = 0U;
}

static void processByte(uint8_t byte)
{
  switch (rx_state)
  {
    case RX_WAIT_HEAD0:
      if (byte == PROTO_HEAD0)
      {
        rx_state = RX_WAIT_HEAD1;
      }
      break;
    case RX_WAIT_HEAD1:
      rx_state = (byte == PROTO_HEAD1) ? RX_WAIT_LEN : RX_WAIT_HEAD0;
      break;
    case RX_WAIT_LEN:
      if (byte == 0U || byte > (uint8_t)(MAX_PAYLOAD_LEN + 1U))
      {
        logEvent("收到非法长度帧");
        resetParser();
      }
      else
      {
        rx_buf[0] = byte;
        rx_len = byte;
        rx_index = 0U;
        rx_state = RX_WAIT_BODY;
      }
      break;
    case RX_WAIT_BODY:
      rx_index++;
      rx_buf[rx_index] = byte;
      if (rx_index >= (uint8_t)(rx_len + 1U))
      {
        uint8_t recv_sum = rx_buf[rx_index];
        uint8_t calc_sum = checksum8(rx_buf, (uint16_t)rx_len + 1U);
        if (recv_sum == calc_sum)
        {
          handleFrame(rx_buf[1], &rx_buf[2], (uint8_t)(rx_len - 1U));
        }
        else
        {
          logEvent("状态帧校验失败");
        }
        resetParser();
      }
      break;
    default:
      resetParser();
      break;
  }
}

static void pollSerial(void)
{
  while (Serial.available() > 0)
  {
    processByte((uint8_t)Serial.read());
  }
}

static bool loadWifiCfg(wifi_cfg_t *cfg)
{
  if (cfg == nullptr)
  {
    return false;
  }
  EEPROM.get(0, *cfg);
  if (cfg->magic != WIFI_CFG_MAGIC)
  {
    return false;
  }
  if (cfg->checksum != checksum16((const uint8_t *)cfg, (uint16_t)(sizeof(wifi_cfg_t) - sizeof(uint16_t))))
  {
    return false;
  }
  cfg->ssid[32] = '\0';
  cfg->pass[64] = '\0';
  return cfg->ssid[0] != '\0';
}

static void saveWifiCfg(const char *ssid, const char *pass)
{
  wifi_cfg_t cfg = {0};
  cfg.magic = WIFI_CFG_MAGIC;
  strncpy(cfg.ssid, ssid, sizeof(cfg.ssid) - 1U);
  strncpy(cfg.pass, pass, sizeof(cfg.pass) - 1U);
  cfg.checksum = checksum16((const uint8_t *)&cfg, (uint16_t)(sizeof(wifi_cfg_t) - sizeof(uint16_t)));
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

static void startStaConnect(const char *ssid, const char *pass)
{
  if (ssid == nullptr || ssid[0] == '\0')
  {
    return;
  }
  strncpy(sta_target_ssid, ssid, sizeof(sta_target_ssid) - 1U);
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, pass);
  sta_connecting = true;
  sta_connect_started_ms = millis();
  logEvent("开始连接外部 Wi-Fi");
}

static const char *controlModeText(uint8_t mode)
{
  switch (mode)
  {
    case 1U: return "USART3 上位机";
    case 2U: return "PS2 手柄";
    case 3U: return "ESP01S 网页";
    case 4U: return "USART1 调试";
    default: return "无控制源";
  }
}

static void sendStatusJson(void)
{
  char json[560];
  uint32_t age_ms = (robot_status.last_status_ms == 0U) ? 0xFFFFFFFFUL : (millis() - robot_status.last_status_ms);
  snprintf(json, sizeof(json),
           "{\"left_speed\":%.3f,\"right_speed\":%.3f,"
           "\"left_encoder\":%ld,\"right_encoder\":%ld,"
           "\"battery_voltage\":%.3f,\"left_current\":%.3f,\"right_current\":%.3f,"
           "\"imu_accel\":[%d,%d,%d],\"imu_gyro\":[%d,%d,%d],"
           "\"error_flags\":%lu,\"control_mode\":%u,\"control_mode_text\":\"%s\","
           "\"status_age_ms\":%lu}",
           robot_status.left_speed,
           robot_status.right_speed,
           (long)robot_status.left_encoder,
           (long)robot_status.right_encoder,
           robot_status.battery_voltage,
           robot_status.left_current,
           robot_status.right_current,
           robot_status.imu_accel[0],
           robot_status.imu_accel[1],
           robot_status.imu_accel[2],
           robot_status.imu_gyro[0],
           robot_status.imu_gyro[1],
           robot_status.imu_gyro[2],
           (unsigned long)robot_status.error_flags,
           robot_status.control_mode,
           controlModeText(robot_status.control_mode),
           (unsigned long)age_ms);
  server.send(200, "application/json; charset=utf-8", json);
}

static void sendEventsJson(void)
{
  String json = "[";
  for (uint8_t i = 0U; i < EVENT_COUNT; ++i)
  {
    uint8_t index = (uint8_t)((event_head + i) % EVENT_COUNT);
    if (events[index][0] == '\0')
    {
      continue;
    }
    if (json.length() > 1U)
    {
      json += ",";
    }
    json += "\"";
    json += events[index];
    json += "\"";
  }
  json += "]";
  server.send(200, "application/json; charset=utf-8", json);
}

static void handleCmd(void)
{
  if (!server.hasArg("v") || !server.hasArg("w"))
  {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"msg\":\"缺少 v 或 w\"}");
    return;
  }
  float linear_x = server.arg("v").toFloat();
  float angular_z = server.arg("w").toFloat();
  bool ok = sendVelocity(linear_x, angular_z, 1U);
  logEvent(ok ? "网页遥控指令已发送" : "网页遥控指令发送失败");
  server.send(ok ? 200 : 500,
              "application/json; charset=utf-8",
              ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

static void handleStop(void)
{
  bool ok = sendVelocity(0.0f, 0.0f, 0U);
  logEvent(ok ? "网页发送停车指令" : "停车指令发送失败");
  server.send(ok ? 200 : 500,
              "application/json; charset=utf-8",
              ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

static void handleEstop(void)
{
  uint8_t enabled = (server.hasArg("on") && server.arg("on").toInt() != 0) ? 1U : 0U;
  bool ok = sendEstop(enabled);
  logEvent(enabled != 0U ? "网页设置急停" : "网页清除急停");
  server.send(ok ? 200 : 500,
              "application/json; charset=utf-8",
              ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

static void handleWifiSet(void)
{
  if (!server.hasArg("ssid"))
  {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"msg\":\"缺少 SSID\"}");
    return;
  }
  String ssid_arg = server.arg("ssid");
  String pass_arg = server.hasArg("pass") ? server.arg("pass") : String("");
  char ssid[33] = {0};
  char pass[65] = {0};
  ssid_arg.toCharArray(ssid, sizeof(ssid));
  pass_arg.toCharArray(pass, sizeof(pass));
  if (ssid[0] == '\0')
  {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"msg\":\"SSID 不能为空\"}");
    return;
  }
  saveWifiCfg(ssid, pass);
  startStaConnect(ssid, pass);
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

static void handleWifiStatus(void)
{
  char json[420];
  IPAddress ap_ip = WiFi.softAPIP();
  IPAddress sta_ip = WiFi.localIP();
  bool connected = WiFi.status() == WL_CONNECTED;
  snprintf(json, sizeof(json),
           "{\"ap_ssid\":\"%s\",\"ap_ip\":\"%u.%u.%u.%u\","
           "\"sta_ssid\":\"%s\",\"sta_connected\":%s,\"sta_ip\":\"%u.%u.%u.%u\","
           "\"rssi\":%d,\"clients\":%u}",
           AP_SSID,
           ap_ip[0], ap_ip[1], ap_ip[2], ap_ip[3],
           sta_target_ssid,
           connected ? "true" : "false",
           sta_ip[0], sta_ip[1], sta_ip[2], sta_ip[3],
           connected ? WiFi.RSSI() : 0,
           (unsigned)WiFi.softAPgetStationNum());
  server.send(200, "application/json; charset=utf-8", json);
}

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>F407 无线调试台</title>
  <style>
    :root{--bg:#f4f7fb;--fg:#172033;--muted:#5a667c;--line:#d9e1eb;--panel:#fff;--accent:#0f766e;--warn:#b45309;--danger:#b91c1c}
    *{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font-family:"Segoe UI","Microsoft YaHei",sans-serif}
    main{max-width:1080px;margin:0 auto;padding:16px;display:grid;gap:14px}
    header{display:flex;justify-content:space-between;gap:12px;align-items:end;border-bottom:1px solid var(--line);padding-bottom:12px}
    h1,h2{margin:0}h1{font-size:24px}h2{font-size:18px}
    .grid{display:grid;gap:14px}.two{grid-template-columns:repeat(2,minmax(0,1fr))}
    section{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px}
    .stats{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
    .stat{border:1px solid var(--line);border-radius:8px;padding:10px}
    .k{color:var(--muted);font-size:13px}.v{font-size:18px;font-weight:700;margin-top:4px}
    .pad{display:grid;grid-template-columns:repeat(3,72px);gap:10px;justify-content:center;margin:14px 0}
    button{height:52px;border:0;border-radius:8px;background:var(--accent);color:#fff;font-size:18px;font-weight:700}
    button.stop{background:var(--danger)}button.soft{background:#334155;font-size:14px}
    .row{display:flex;gap:10px;flex-wrap:wrap}.row>*{flex:1 1 140px}
    input{height:40px;border:1px solid var(--line);border-radius:8px;padding:0 10px;font-size:14px}
    pre{margin:0;background:#111827;color:#e5e7eb;border-radius:8px;padding:12px;min-height:180px;max-height:260px;overflow:auto;white-space:pre-wrap}
    .tag{display:inline-flex;align-items:center;border:1px solid var(--line);border-radius:999px;padding:4px 10px;font-size:13px;background:#fff}
    .warn{color:var(--warn)}.danger{color:var(--danger)}
    @media(max-width:820px){.two{grid-template-columns:1fr}.stats{grid-template-columns:1fr}header{display:block}.pad{grid-template-columns:repeat(3,64px)}}
  </style>
</head>
<body>
  <main>
    <header>
      <div><h1>F407 无线调试台</h1></div>
      <div class="tag" id="link">等待状态帧</div>
    </header>
    <div class="grid two">
      <section>
        <h2>遥控</h2>
        <div class="pad">
          <span></span><button id="up">↑</button><span></span>
          <button id="left">←</button><button id="stop" class="stop">■</button><button id="right">→</button>
          <span></span><button id="down">↓</button><span></span>
        </div>
        <div class="row">
          <button id="estop" class="stop">急停</button>
          <button id="clear-estop" class="soft">清除急停</button>
        </div>
      </section>
      <section>
        <h2>系统信息</h2>
        <div class="stats">
          <div class="stat"><div class="k">当前控制源</div><div class="v" id="src">--</div></div>
          <div class="stat"><div class="k">错误标志</div><div class="v" id="err">--</div></div>
          <div class="stat"><div class="k">左右速度</div><div class="v" id="speed">--</div></div>
          <div class="stat"><div class="k">电池电压</div><div class="v" id="battery">--</div></div>
          <div class="stat"><div class="k">左右电流</div><div class="v" id="current">--</div></div>
          <div class="stat"><div class="k">编码器</div><div class="v" id="enc">--</div></div>
        </div>
      </section>
    </div>
    <div class="grid two">
      <section>
        <h2>网络</h2>
        <div class="row">
          <input id="ssid" placeholder="路由器 SSID">
          <input id="pass" type="password" placeholder="路由器密码">
          <button id="wifi" class="soft">连接</button>
        </div>
        <div style="margin-top:12px" id="wifi-state">--</div>
      </section>
      <section>
        <h2>调试日志</h2>
        <pre id="events"></pre>
      </section>
    </div>
  </main>
  <script>
    const speed = {v:0.25,w:0.6};
    let holdTimer = null;
    const $ = (id) => document.getElementById(id);
    async function getJson(url, init){ return fetch(url, Object.assign({cache:"no-store"}, init||{})).then(r=>r.json()); }
    function hold(v,w){
      stopHold(false);
      sendCmd(v,w);
      holdTimer = setInterval(()=>sendCmd(v,w), 120);
    }
    function stopHold(sendStop=true){
      if(holdTimer){ clearInterval(holdTimer); holdTimer=null; }
      if(sendStop){ fetch("/api/stop",{cache:"no-store"}).catch(()=>{}); }
    }
    function sendCmd(v,w){ fetch(`/api/cmd?v=${v}&w=${w}`,{cache:"no-store"}).catch(()=>{}); }
    [["up",speed.v,0],["down",-speed.v,0],["left",0,speed.w],["right",0,-speed.w]].forEach(([id,v,w])=>{
      const el=$(id); ["pointerdown"].forEach(evt=>el.addEventListener(evt,e=>{e.preventDefault();hold(v,w);}));
      ["pointerup","pointercancel","pointerleave"].forEach(evt=>el.addEventListener(evt,e=>{e.preventDefault();stopHold(true);}));
    });
    $("stop").onclick=()=>stopHold(true);
    $("estop").onclick=()=>fetch("/api/estop?on=1",{cache:"no-store"});
    $("clear-estop").onclick=()=>fetch("/api/estop?on=0",{cache:"no-store"});
    $("wifi").onclick=()=>fetch("/api/wifi",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},
      body:`ssid=${encodeURIComponent($("ssid").value)}&pass=${encodeURIComponent($("pass").value)}`});
    window.addEventListener("blur",()=>stopHold(true));
    document.addEventListener("visibilitychange",()=>{if(document.hidden)stopHold(true);});
    async function refresh(){
      try{
        const s=await getJson("/api/status");
        $("link").textContent=s.status_age_ms<1000?"状态在线":"状态超时";
        $("src").textContent=s.control_mode_text;
        $("err").textContent="0x"+Number(s.error_flags||0).toString(16).toUpperCase();
        $("speed").textContent=`${Number(s.left_speed).toFixed(3)} / ${Number(s.right_speed).toFixed(3)} m/s`;
        $("battery").textContent=`${Number(s.battery_voltage).toFixed(2)} V`;
        $("current").textContent=`${Number(s.left_current).toFixed(2)} / ${Number(s.right_current).toFixed(2)} A`;
        $("enc").textContent=`${s.left_encoder} / ${s.right_encoder}`;
      }catch(_){}
      try{
        const w=await getJson("/api/wifi");
        $("wifi-state").textContent=`AP ${w.ap_ssid} (${w.ap_ip}) | STA ${w.sta_connected?"已连接":"未连接"} ${w.sta_ip}`;
      }catch(_){}
      try{
        const e=await getJson("/api/events");
        $("events").textContent=e.join("\n");
      }catch(_){}
      setTimeout(refresh,1200);
    }
    refresh();
  </script>
</body>
</html>
)HTML";

static void handleRoot(void)
{
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void serviceStaState(void)
{
  if (!sta_connecting)
  {
    return;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    sta_connecting = false;
    logEvent("外部 Wi-Fi 已连接");
  }
  else if ((millis() - sta_connect_started_ms) > WIFI_CONNECT_TIMEOUT_MS)
  {
    sta_connecting = false;
    logEvent("外部 Wi-Fi 连接超时");
  }
}

void setup(void)
{
  Serial.begin(SERIAL_BAUD);
  EEPROM.begin(sizeof(wifi_cfg_t));
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  logEvent("ESP01S 网页控制台启动");

  wifi_cfg_t cfg = {0};
  if (loadWifiCfg(&cfg))
  {
    startStaConnect(cfg.ssid, cfg.pass);
  }

  server.on("/", handleRoot);
  server.on("/api/status", HTTP_GET, sendStatusJson);
  server.on("/api/events", HTTP_GET, sendEventsJson);
  server.on("/api/cmd", HTTP_GET, handleCmd);
  server.on("/api/stop", HTTP_GET, handleStop);
  server.on("/api/estop", HTTP_GET, handleEstop);
  server.on("/api/wifi", HTTP_POST, handleWifiSet);
  server.on("/api/wifi", HTTP_GET, handleWifiStatus);
  server.begin();
}

void loop(void)
{
  server.handleClient();
  pollSerial();
  serviceStaState();
  delay(1);
}
