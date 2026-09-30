/*
 * 接收端 · ESP32-C3 SuperMini
 * ------------------------------------------------------------
 * 收 ESP-NOW 包 → GPIO1 上的 LED：
 *   type = 1（有人）    一直快闪
 *   type = 0（人已离开）立刻灭
 *   超过 10 秒收不到任何包 → 自动灭（发射端断电 / 掉线 / 连续丢包兜底）
 *
 * 接线：
 *   GPIO1 ── 220Ω ── LED 长脚(+)
 *   GND   ─────────  LED 短脚(-)
 *
 * 【烧录前必须确认】Arduino IDE：
 *   工具 → USB CDC On Boot → Enabled
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ALERT_LED        1          // 报警灯，别用 GPIO8（板载 RGB + strapping）
#define WIFI_CHANNEL     6          // 两端必须一致

#define BLINK_PERIOD_MS  300UL      // 快闪一个亮灭周期：亮 150ms + 灭 150ms
#define LINK_TIMEOUT_MS  10000UL    // 这么久没收到包就当发射端失联

#define MSG_PRESENT      1
#define MSG_ABSENT       0

typedef struct {
  uint8_t  type;                    // 1 = 有人，0 = 人已离开
  uint32_t seq;                     // 发送端的 millis()，排查丢包用
} RadioMsg;

volatile bool     present  = false;
volatile uint32_t lastRecv = 0;
volatile uint32_t lastSeq  = 0;
volatile uint8_t  lastType = 0;
volatile int8_t   lastRssi = 0;
volatile bool     newPkt   = false;

// 打印当前实际信道，用来排查"两端不在同一信道"
void printChannel() {
  uint8_t ch = 0;
  wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  if (Serial) {
    Serial.print("[RX] 当前信道 = ");
    Serial.println(ch);
  }
}

// ESP32 核心 3.0+ 的回调签名，第一参数是 esp_now_recv_info_t*
// 回调里只置标志位、不打印，避免串口阻塞影响实时性
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(RadioMsg)) return;

  RadioMsg m;
  memcpy(&m, data, sizeof(m));
  if (m.type != MSG_PRESENT && m.type != MSG_ABSENT) return;

  lastSeq  = m.seq;
  lastType = m.type;
  lastRssi = info->rx_ctrl->rssi;
  lastRecv = millis();
  present  = (m.type == MSG_PRESENT);
  newPkt   = true;
}

void setup() {
  Serial.begin(115200);
  // 串口写满时不阻塞：USB CDC 的 TX 缓冲写满后 print 会卡死整个 loop()，
  // 表现是"灯停在某个状态不动、串口也没输出了"。设成 0 = 写不下就丢。
  Serial.setTxTimeoutMs(0);
  delay(2000);

  pinMode(ALERT_LED, OUTPUT);
  digitalWrite(ALERT_LED, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);             // 关键：关掉 modem sleep，否则 ESP-NOW 会漏包
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    if (Serial) Serial.println("[RX] ESP-NOW 初始化失败，3 秒后重启");
    delay(3000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);

  // esp_now_init() 可能把信道重置回 STA 默认值，所以这里必须再设一次
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (Serial) {
    Serial.println();
    Serial.println("[RX] 接收端就绪");
    Serial.print("[RX] 本机 MAC = ");
    Serial.println(WiFi.macAddress());
    printChannel();
    Serial.println("[RX] 等待报警...");
  }
}

void loop() {
  static bool     wasPresent = false;
  static uint32_t lastBeat   = 0;

  // 每收到一个包就打一行，带 RSSI —— 排查链路问题全靠它
  if (newPkt) {
    newPkt = false;
    if (Serial) {
      Serial.print("[RX] 收到包 type=");
      Serial.print(lastType);
      Serial.print(" seq=");
      Serial.print((unsigned long)lastSeq);
      Serial.print(" rssi=");
      Serial.print(lastRssi);
      Serial.println(" dBm");
    }
  }

  // 看门狗：太久没收到包，认为发射端失联，灯灭
  if (present && millis() - lastRecv > LINK_TIMEOUT_MS) {
    present = false;
    if (Serial) Serial.println("[RX] 超过 10 秒没收到包 → 灯灭");
  }

  // 只在状态翻转时打印，避免主循环被串口拖住
  if (present != wasPresent) {
    wasPresent = present;
    lastBeat   = millis();          // 重置心跳计时，避免刚进入就打印
    if (Serial) {
      if (present) {
        Serial.print("[RX] 有人 → 开始快闪 seq=");
        Serial.println((unsigned long)lastSeq);
      } else {
        Serial.println("[RX] 人已离开 → 灯灭");
      }
    }
  }

  if (!present) {
    digitalWrite(ALERT_LED, LOW);
    delay(20);
    return;
  }

  // 一直快闪，直到收到"离开"包或超时
  digitalWrite(ALERT_LED, ((millis() / BLINK_PERIOD_MS) % 2 == 0) ? HIGH : LOW);

  // 心跳日志：每 30 秒报一次，证明链路还活着
  if (millis() - lastBeat >= 30000UL) {
    lastBeat = millis();
    if (Serial) {
      Serial.print("[RX] 持续有人中 seq=");
      Serial.println((unsigned long)lastSeq);
    }
  }

  delay(20);
}
