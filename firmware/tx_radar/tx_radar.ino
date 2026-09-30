/*
 * 发射端 · ESP32-C3 SuperMini + HLK-LD2410C-P
 * ------------------------------------------------------------
 * 读雷达 OUT 脚(GPIO5) → 200ms 防抖 → ESP-NOW 通知接收端
 *
 * 两种包：
 *   type = 1（有人）    上升沿发一次，之后每 2 秒心跳重发
 *   type = 0（人已离开）下降沿发一次
 *
 * 接收端拿到这两种包就知道"人在不在"，而且心跳让它可以对丢包兜底。
 *
 * 接线：
 *   雷达 VCC → 5V         雷达 GND → GND
 *   雷达 OUT → GPIO5      雷达 TX / RX 悬空
 *
 * 串口会打印发送回执：ESP-NOW 的发送回调只有在**对端 ACK 了**才报成功，
 * 所以只接这一块板子、接收端仅供电，也能验证链路通不通。
 *
 * 【烧录前必须确认】Arduino IDE：
 *   工具 → USB CDC On Boot → Enabled
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define RADAR_OUT       5
#define WIFI_CHANNEL    6
#define DEBOUNCE_MS     200
#define HEARTBEAT_MS    2000UL   // 有人期间的重发间隔，接收端靠它做超时兜底

#define MSG_PRESENT     1        // 有人
#define MSG_ABSENT      0        // 人已离开

// 接收端 MAC（由 rx_mac 实测读出：E8:3D:C1:95:0C:94）
uint8_t peerMac[] = {0xE8, 0x3D, 0xC1, 0x95, 0x0C, 0x94};

typedef struct {
  uint8_t  type;               // 1 = 有人，0 = 人已离开
  uint32_t seq;                // 本机 millis()
} RadioMsg;

esp_now_peer_info_t peerInfo = {};   // = {} 保证 ifidx / lm_* 等字段为 0

// -1 = 还没结果，0 = 失败，1 = 成功
volatile int8_t lastSendStatus = -1;

// 回调在 WiFi 任务里跑，这里只记状态，打印放到 loop()
void onSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
  lastSendStatus = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;
}

void sendMsg(uint8_t type) {
  RadioMsg msg;
  msg.type = type;
  msg.seq  = millis();
  esp_now_send(peerMac, (uint8_t *)&msg, sizeof(msg));
}

void setup() {
  Serial.begin(115200);
  // 串口写满时不阻塞：USB CDC 的 TX 缓冲写满后 print 会卡死整个 loop()，
  // 表现是"程序像死了一样，既不发包也不打印"。设成 0 = 写不下就丢。
  Serial.setTxTimeoutMs(0);
  delay(2000);

  pinMode(RADAR_OUT, INPUT_PULLDOWN);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    if (Serial) Serial.println("[TX] ESP-NOW 初始化失败，3 秒后重启");
    delay(3000);
    ESP.restart();
  }

  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    if (Serial) Serial.println("[TX] 添加对端失败，检查 peerMac 是否填错");
    delay(3000);
    ESP.restart();
  }

  esp_now_register_send_cb(onSent);

  // esp_now_init() 可能把信道重置回 STA 默认值，所以这里必须再设一次
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (Serial) {
    Serial.println();
    Serial.println("[TX] 发射端就绪");
    Serial.print("[TX] 本机 MAC = ");
    Serial.println(WiFi.macAddress());
    Serial.print("[TX] 目标 MAC = ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", peerMac[i]);
      Serial.print(i < 5 ? ":" : "\n");
    }
    uint8_t ch = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    Serial.print("[TX] 当前信道 = ");
    Serial.println(ch);
    Serial.println("[TX] 等待雷达触发...");
  }
}

void loop() {
  static bool     lastRaw    = false;   // 上一次读到的原始电平
  static uint32_t lastRawMs  = 0;       // 原始电平最后一次翻转的时刻
  static bool     present    = false;   // 防抖后的稳定状态
  static uint32_t lastSendMs = 0;       // 上一次发送"有人"的时刻

  // 打印上一次的发送回执
  // 注意：这个状态只反映"链路层 ACK 有没有回来"。实测在这套板子上 ACK 经常收不到，
  // 但接收端照样 100% 收到包（接收端日志 rssi=-55 左右、每包必到）。
  // 所以这里不要写"发送失败"，否则会误导排查方向。真正的判据是接收端的串口日志。
  int8_t st = lastSendStatus;
  if (st != -1) {
    lastSendStatus = -1;
    if (Serial) {
      Serial.print("[TX] 对端 ACK：");
      Serial.println(st == 1 ? "已收到" : "没收到（不代表包没送到，以接收端日志为准）");
    }
  }

  bool raw = (digitalRead(RADAR_OUT) == HIGH);
  if (raw != lastRaw) {
    lastRaw   = raw;
    lastRawMs = millis();
  }

  // 电平必须稳定超过 DEBOUNCE_MS 才认可
  if (millis() - lastRawMs >= DEBOUNCE_MS && raw != present) {
    present = raw;
    if (present) {
      sendMsg(MSG_PRESENT);
      lastSendMs = millis();
      if (Serial) Serial.println("[TX] 有人 → 已发送报警");
    } else {
      sendMsg(MSG_ABSENT);
      if (Serial) Serial.println("[TX] 人已离开 → 已通知接收端");
    }
  }

  // 有人期间周期性重发：接收端漏掉"离开"包也能靠超时兜底
  if (present && millis() - lastSendMs >= HEARTBEAT_MS) {
    sendMsg(MSG_PRESENT);
    lastSendMs = millis();
    if (Serial) Serial.println("[TX] 持续有人（心跳重发）");
  }

  delay(20);
}
