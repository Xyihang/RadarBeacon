/*
 * ============================================================
 *  RadarBeacon · 接收端
 *  ESP32-C3 SuperMini + 一颗 LED
 * ============================================================
 *
 * 做的事：
 *   监听空气里的 ESP-NOW 广播包，只认 RadarBeacon 发射端发的（靠 magic 过滤），
 *   按状态驱动 GPIO1 上的 LED。
 *
 * 灯的行为：
 *   type = 1（有人）   → 一直快闪（亮 300ms / 灭 300ms）
 *   type = 0（没人）   → 立刻灭
 *   8 秒收不到任何包   → 自动灭（发射端掉线 / 断电 / 连续丢包兜底）
 *
 * 因为发射端用广播，这块板子**不需要知道发射端的 MAC**，烧完就能用。
 * 同一信道上如果有多个发射端在广播，它会跟着最后收到的那个状态走。
 *
 * 接线：
 *   GPIO1 ── 220Ω ── LED 长脚(+)
 *   GND   ─────────  LED 短脚(-)
 *
 * 供电：5V USB-C 电池直插开发板 Type-C 口
 *
 * 【烧录前必须确认】Arduino IDE：
 *   工具 → USB CDC On Boot → Enabled
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================
//  用户可调参数
// ============================================================

#define ALERT_LED        1        // 提示灯。别用 GPIO8（板载 LED + strapping）
#define WIFI_CHANNEL     6        // 必须和发射端一致

#define BLINK_PERIOD_MS  300UL    // 快闪周期：亮 300ms + 灭 300ms
#define LINK_TIMEOUT_MS  8000UL   // 这么久没收到包就认为发射端失联
#define STATUS_LOG_MS    30000UL  // "持续有人中" 日志间隔

// 主循环看门狗：这么多毫秒没有转完一圈就重启板子。设成 0 关闭。
#define LOOP_WDT_MS      8000

// ============================================================
//  通信协议（必须和发射端完全一致）
// ============================================================

#define PROTO_MAGIC      0x5242u  // 'R''B'
#define PROTO_VER        1

#define MSG_ABSENT       0
#define MSG_PRESENT      1

typedef struct {
  uint16_t magic;
  uint8_t  ver;
  uint8_t  type;
  uint32_t seq;
  uint32_t bootId;
} RadioMsg;

// ============================================================
//  中断上下文与主循环之间传状态
// ============================================================

volatile bool     g_present     = false;
volatile uint32_t g_lastRecvMs  = 0;
volatile uint32_t g_lastGapMs   = 0;
volatile uint32_t g_lastSeq     = 0;
volatile uint32_t g_lastBootId  = 0;
volatile uint8_t  g_lastType    = MSG_ABSENT;
volatile int8_t   g_lastRssi    = 0;
volatile bool     g_newPkt      = false;
volatile bool     g_bootChanged = false;
volatile uint32_t g_pktCount    = 0;
volatile uint32_t g_loopTick    = 0;

// ============================================================
//  主循环看门狗
// ============================================================
// 绝不能在这里打印：loop() 卡住时串口很可能也卡着，print 会把自己也堵死。

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
//  收包回调
// ============================================================
// 回调跑在 WiFi 任务里，这里只置标志位，一律不打印。

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(RadioMsg)) return;

  RadioMsg m;
  memcpy(&m, data, sizeof(m));

  if (m.magic != PROTO_MAGIC) return;                       // 不是 RadarBeacon 的包
  if (m.ver != PROTO_VER) return;                           // 协议版本对不上
  if (m.type != MSG_PRESENT && m.type != MSG_ABSENT) return;

  // 发射端每条状态变化会连发 3 次，这 3 个包共用同一个 seq。
  // 只认第一次，否则日志会刷 3 行、也无谓地反复重置看门狗。
  if (m.seq == g_lastSeq && m.bootId == g_lastBootId) return;

  // 第一次收到包时 g_lastBootId 还是 0，那次不算"重启"
  if (g_lastBootId != 0 && m.bootId != g_lastBootId) g_bootChanged = true;

  uint32_t now  = millis();
  g_lastGapMs   = now - g_lastRecvMs;
  g_lastRecvMs  = now;
  g_lastSeq     = m.seq;
  g_lastBootId  = m.bootId;
  g_lastType    = m.type;
  g_lastRssi    = info->rx_ctrl->rssi;
  g_present     = (m.type == MSG_PRESENT);
  g_pktCount++;
  g_newPkt      = true;
}

// ============================================================
//  setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  // 串口写满时不阻塞。USB CDC 的 TX 缓冲满后 print 会卡死整个 loop()，
  // 表现是"灯停在某个状态不动、串口也没输出"。设成 0 = 写不下就丢掉。
  Serial.setTxTimeoutMs(0);
  delay(2000);

  pinMode(ALERT_LED, OUTPUT);
  digitalWrite(ALERT_LED, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);              // 关键：不关 modem sleep 会漏包
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    if (Serial) Serial.println("[RX] ESP-NOW 初始化失败，3 秒后重启");
    delay(3000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);

  // esp_now_init() 可能把信道重置回 STA 默认值，这里必须再设一次
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (Serial) {
    uint8_t ch = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    Serial.println();
    Serial.println("[RX] ===== RadarBeacon 接收端 =====");
    Serial.print("[RX] 本机 MAC = ");
    Serial.println(WiFi.macAddress());
    Serial.print("[RX] 信道     = ");
    Serial.println(ch);
    Serial.println("[RX] 监听方式 = 广播（不需要填发射端 MAC）");
    Serial.println("[RX] 等待广播...");
  }

#if LOOP_WDT_MS > 0
  xTaskCreate(watchdogTask, "loopWdt", 3072, nullptr, 1, nullptr);
#endif
}

void loop() {
  g_loopTick++;

  static bool     wasPresent = false;
  static uint32_t ledStart   = 0;     // 本次报警的起点，闪烁相位从这里算
  static uint32_t lastStat   = 0;     // 上一次打"持续有人中"的时刻

  // ---- 1. 打印新收到的包 ----
  if (g_newPkt) {
    g_newPkt = false;
    if (Serial) {
      Serial.print("[RX] 收到包 type=");
      Serial.print(g_lastType);
      Serial.print(" seq=");
      Serial.print((unsigned long)g_lastSeq);
      Serial.print(" gap=");
      Serial.print((unsigned long)g_lastGapMs);
      Serial.print("ms rssi=");
      Serial.print(g_lastRssi);
      Serial.print("dBm 累计=");
      Serial.println((unsigned long)g_pktCount);
    }
  }

  // ---- 2. 发射端重启过？ ----
  if (g_bootChanged) {
    g_bootChanged = false;
    if (Serial) {
      Serial.print("[RX] 发射端重启过（新 bootId=0x");
      Serial.print((unsigned long)g_lastBootId, HEX);
      Serial.println("）");
    }
  }

  // ---- 3. 链路看门狗 ----
  if (g_present && millis() - g_lastRecvMs > LINK_TIMEOUT_MS) {
    uint32_t dead = millis() - g_lastRecvMs;
    g_present = false;
    if (Serial) {
      Serial.print("[RX] 超过 ");
      Serial.print((unsigned long)(LINK_TIMEOUT_MS / 1000));
      Serial.print(" 秒收不到包 → 灯灭（实际 ");
      Serial.print((unsigned long)dead);
      Serial.println(" ms 无包）");
    }
  }

  // ---- 4. 状态翻转 ----
  if (g_present != wasPresent) {
    wasPresent = g_present;
    lastStat   = millis();
    if (g_present) {
      ledStart = millis();           // 相位归零 → 灯立刻亮，不用等半个周期
      if (Serial) {
        Serial.print("[RX] 有人 → 开始快闪 seq=");
        Serial.println((unsigned long)g_lastSeq);
      }
    } else {
      if (Serial) Serial.println("[RX] 没人 → 灯灭");
    }
  }

  // ---- 5. 驱动 LED ----
  if (!g_present) {
    digitalWrite(ALERT_LED, LOW);
    delay(20);
    return;
  }

  // 相位从 ledStart 算，不是从全局 millis() 算 ——
  // 否则灯亮起时可能正好落在"灭"的那半个周期，要等最多 BLINK_PERIOD_MS 才第一次亮。
  uint32_t ph = (millis() - ledStart) / BLINK_PERIOD_MS;
  digitalWrite(ALERT_LED, (ph % 2 == 0) ? HIGH : LOW);

  // ---- 6. 周期性状态日志 ----
  if (millis() - lastStat >= STATUS_LOG_MS) {
    lastStat = millis();
    if (Serial) {
      Serial.print("[RX] 持续有人中 seq=");
      Serial.println((unsigned long)g_lastSeq);
    }
  }

  delay(20);
}
