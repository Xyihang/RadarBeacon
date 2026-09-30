/*
 * 接收端 · 第 1 步：读出这块板子的 MAC 地址
 * ------------------------------------------------------------
 * 板子：ESP32-C3 SuperMini（接收端那一块）
 * 用途：把串口打印出来的 MAC 抄进 tx_radar.ino 的 peerMac[]
 * 读完这个程序就会被 rx_alert.ino 覆盖，不用保留
 *
 * 【烧录前必须确认】Arduino IDE：
 *   工具 → USB CDC On Boot → Enabled
 * 否则串口监视器一片空白 —— C3 SuperMini 板上没有 USB 转串口芯片，
 * 串口只能走芯片自带的 USB CDC。
 */

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(2000);                 // 等电脑枚举 USB CDC，否则开头几行会丢

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  if (Serial) {
    Serial.println();
    Serial.println("============== 接收端 MAC ==============");
    Serial.print("RX MAC = ");
    Serial.println(WiFi.macAddress());
    Serial.println("----------------------------------------");
    Serial.println("把上面那行抄下来，改成这种格式：");
    Serial.println("  {0x7C, 0x87, 0xCE, 0x4A, 0x12, 0x3B}");
    Serial.println("填进 tx_radar.ino 顶部的 peerMac[]");
    Serial.println("========================================");
    Serial.println("这块板子接下来要烧 rx_alert.ino");
  }
}

void loop() {
  delay(1000);
}
