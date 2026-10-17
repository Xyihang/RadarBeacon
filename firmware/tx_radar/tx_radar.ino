/*
 * ============================================================
 *  RadarBeacon · 发射端
 *  ESP32-C3 SuperMini + HLK-LD2410C-P（24GHz 人体存在雷达）
 * ============================================================
 *
 * 做的事：
 *   读雷达的 OUT 脚（GPIO5），判断"有人/没人"，
 *   用 ESP-NOW 单播把状态发给接收端那块板子。
 *
 * ★★★ 烧录前必须改这里 ★★★
 *   把 peerMac[] 改成你自己接收端的 MAC。
 *   怎么读：接收端烧好 rx_alert 之后，串口启动横幅会打印
 *          [RX] 本机 MAC = XX:XX:XX:XX:XX:XX
 *   把那 6 组十六进制抄过来，冒号改逗号、每段前面加 0x 即可。
 *   填错的表现：接收端日志里一直是 原始包=0。
 *
 * 发包规则：
 *   type = 1（有人）   上升沿连发 3 次，之后每 1.5 秒心跳 1 次
 *   type = 0（没人）   下降沿连发 3 次
 *
 * 接线：
 *   雷达 VCC  → 开发板 5V
 *   雷达 GND  → 开发板 G
 *   雷达 OUT  → 开发板 GPIO5（丝印 5）
 *   雷达 TX/RX → 悬空，套热缩管
 *
 * 供电：5V USB-C 电池直插开发板 Type-C 口
 *
 * 【烧录前必须确认】Arduino IDE：
 *   工具 → USB CDC On Boot → Enabled
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>

// ============================================================
//  ★ 接收端 MAC —— 改成你自己的（见文件头说明）
// ============================================================

uint8_t peerMac[] = {0xE8, 0x3D, 0xC1, 0x95, 0x0B, 0xB8};

// ============================================================
//  用户可调参数
// ============================================================

#define RADAR_OUT        5        // 雷达 OUT 接的引脚
#define WIFI_CHANNEL     6        // 两端必须一致

// 不对称防抖：认定"有人"要快（灯要跟手），认定"没人"要慢（抗雷达瞬时抖动）
#define DEBOUNCE_ON_MS   120
#define DEBOUNCE_OFF_MS  400

#define HEARTBEAT_MS     1000UL   // 有人期间的心跳间隔（越短越能扛射频衰落）
#define HEARTBEAT_LOG_MS 10000UL  // 心跳日志每 10 秒才打一行，免得刷屏

// 每条状态变化重复发几次。单播有链路层 ACK，但实测 ACK 状态不可靠，
// 连发几次成本极低，能兜住偶发丢包。
#define BURST_COUNT      3
#define BURST_GAP_MS     15

// 主循环看门狗：这么多毫秒没有转完一圈就重启板子
// 专治"某个库或串口把 loop() 卡死"。设成 0 关闭。
#define LOOP_WDT_MS      8000

// ============================================================
//  通信协议
// ============================================================

#define PROTO_MAGIC      0x5242u  // 'R''B'，用来丢掉同信道上别人的 ESP-NOW 包
#define PROTO_VER        1

#define MSG_ABSENT       0        // 没人
#define MSG_PRESENT      1        // 有人

typedef struct {
  uint16_t magic;    // 必须等于 PROTO_MAGIC
  uint8_t  ver;      // 协议版本
  uint8_t  type;     // 1 = 有人，0 = 没人
  uint32_t seq;      // 发送端 millis()；连发 3 次时这 3 个包共用同一个 seq
  uint32_t bootId;   // 本次上电的随机数，接收端靠它识别"发射端重启过"
} RadioMsg;

esp_now_peer_info_t g_peer = {};
uint32_t g_bootId = 0;
volatile uint32_t g_loopTick = 0;   // 每转一圈 +1，看门狗任务盯着它

// 链路层 ACK 统计。ACK 状态本身不可靠（实测经常报没收到但数据送达），
// 但成功率能看出射频通路的好坏：一直 0 说明帧根本没到对面。
volatile uint32_t g_ackOk   = 0;
volatile uint32_t g_ackFail = 0;

void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) g_ackOk++;
  else                                g_ackFail++;
}

// ============================================================
//  主循环看门狗
// ============================================================
// 注意：这里绝不能打印。loop() 卡住时串口很可能也卡着，
// 在里面 print 会把自己也堵死，所以只判断、只重启。

void watchdogTask(void *) {
  uint32_t lastTick = 0;
  uint32_t stuckSec = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (g_loopTick == lastTick) {
      if (++stuckSec >= (LOOP_WDT_MS / 1000)) esp_restart();
    } else {
      stuckSec = 0;
      lastTick = g_loopTick;
    }
  }
}

// ============================================================
//  发送
// ============================================================

// burst：同一个包连发几次（状态变化用 3，心跳用 1）
void sendState(uint8_t type, uint8_t burst) {
  RadioMsg m;
  m.magic  = PROTO_MAGIC;
  m.ver    = PROTO_VER;
  m.type   = type;
  m.seq    = millis();
  m.bootId = g_bootId;

  uint8_t ok = 0;
  for (uint8_t i = 0; i < burst; i++) {
    if (esp_now_send(peerMac, (uint8_t *)&m, sizeof(m)) == ESP_OK) ok++;
    if (i + 1 < burst) delay(BURST_GAP_MS);
  }

  // 发送回调的 ACK 状态不可靠，这里只报"驱动层有没有把包收下"。
  // 发不出去通常意味着队列忙，下一轮心跳会补上。
  if (Serial && ok < burst) {
    Serial.print("[TX] 警告：只发出 ");
    Serial.print(ok);
    Serial.print("/");
    Serial.print(burst);
    Serial.println(" 个包");
  }
}

// 雷达 OUT 的每一次原始翻转都记下来。
// 这是排查"人回来了灯却不亮"的关键证据：
//   有这行 → 雷达确实重新拉高了，问题在链路或接收端
//   没这行 → 雷达自己没重新触发，该去调雷达的距离门 / 灵敏度
void logEdge(bool high) {
  if (!Serial) return;
  Serial.print("[TX] OUT → ");
  Serial.print(high ? "高（探到人）" : "低（没人）");
  Serial.print("   t=");
  Serial.println(millis());
}

// ============================================================
//  setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  // 串口写满时不阻塞。USB CDC 的 TX 缓冲满后 print 会卡死整个 loop()，
  // 表现是"程序像死了，不发包也不打印"。设成 0 = 写不下就丢掉。
  Serial.setTxTimeoutMs(0);
  delay(2000);

  pinMode(RADAR_OUT, INPUT_PULLDOWN);
  g_bootId = esp_random();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);              // 关键：不关 modem sleep 会漏包
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    if (Serial) Serial.println("[TX] ESP-NOW 初始化失败，3 秒后重启");
    delay(3000);
    ESP.restart();
  }

  memcpy(g_peer.peer_addr, peerMac, 6);
  g_peer.channel = WIFI_CHANNEL;
  g_peer.encrypt = false;
  if (esp_now_add_peer(&g_peer) != ESP_OK) {
    if (Serial) Serial.println("[TX] 添加对端失败，检查 peerMac 是否填错");
    delay(3000);
    ESP.restart();
  }

  // esp_now_init() 可能把信道重置回 STA 默认值，这里必须再设一次
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  esp_now_register_send_cb(onSent);

  if (Serial) {
    uint8_t ch = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    Serial.println();
    Serial.println("[TX] ===== RadarBeacon 发射端 =====");
    Serial.print("[TX] 本机 MAC   = ");
    Serial.println(WiFi.macAddress());
    Serial.print("[TX] 目标 MAC   = ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", peerMac[i]);
      Serial.print(i < 5 ? ":" : "\n");
    }
    Serial.print("[TX] 信道       = ");
    Serial.println(ch);
    Serial.print("[TX] bootId     = 0x");
    Serial.println(g_bootId, HEX);
    Serial.print("[TX] 开机 OUT   = ");
    Serial.println(digitalRead(RADAR_OUT) == HIGH ? "高" : "低");
    Serial.println("[TX] 等待雷达触发...");
  }

#if LOOP_WDT_MS > 0
  xTaskCreate(watchdogTask, "loopWdt", 3072, nullptr, 1, nullptr);
#endif
}

void loop() {
  g_loopTick++;

  static bool     lastRaw      = false;  // 上一次读到的原始电平
  static uint32_t lastRawMs    = 0;      // 原始电平最后一次翻转的时刻
  static bool     present      = false;  // 防抖后的稳定状态
  static uint32_t lastBeatMs   = 0;      // 上一次发心跳的时刻
  static uint32_t lastBeatLog  = 0;      // 上一次打心跳日志的时刻

  // ---- 1. 读电平，记录翻转时刻 ----
  bool raw = (digitalRead(RADAR_OUT) == HIGH);
  if (raw != lastRaw) {
    lastRaw   = raw;
    lastRawMs = millis();
    logEdge(raw);
  }

  // ---- 2. 防抖后确认状态翻转，立刻连发 3 次 ----
  uint32_t need = raw ? DEBOUNCE_ON_MS : DEBOUNCE_OFF_MS;
  if (millis() - lastRawMs >= need && raw != present) {
    present = raw;
    if (present) {
      sendState(MSG_PRESENT, BURST_COUNT);
      lastBeatMs = millis();
      if (Serial) Serial.println("[TX] 有人 → 已发送");
    } else {
      sendState(MSG_ABSENT, BURST_COUNT);
      if (Serial) Serial.println("[TX] 没人 → 已发送");
    }
  }

  // ---- 3. 有人期间发心跳 ----
  if (present && millis() - lastBeatMs >= HEARTBEAT_MS) {
    lastBeatMs = millis();
    sendState(MSG_PRESENT, 1);         // 心跳丢一个不要紧，下一拍会补上
    if (millis() - lastBeatLog >= HEARTBEAT_LOG_MS) {
      lastBeatLog = millis();
      if (Serial) Serial.println("[TX] 持续有人（心跳中）");
    }
  }

  // ---- 4. 每 10 秒报一次状态（不管有没有在发，都能看出链路好坏）----
  static uint32_t lastStat = 0;
  if (millis() - lastStat >= 10000UL) {
    lastStat = millis();
    if (Serial) {
      Serial.print("[TX] 状态 present=");
      Serial.print(present ? 1 : 0);
      Serial.print(" OUT=");
      Serial.print(raw ? 1 : 0);
      Serial.print(" ACK成功=");
      Serial.print((unsigned long)g_ackOk);
      Serial.print(" ACK失败=");
      Serial.println((unsigned long)g_ackFail);
    }
  }

  delay(20);
}
