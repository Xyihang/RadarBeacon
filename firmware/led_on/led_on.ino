/*
 * LED 接线测试 · GPIO1 恒亮
 * ------------------------------------------------------------
 * 用途：LED 焊/接好之后，确认极性、限流电阻、接头接触都正常。
 *
 * 预期：烧完复位，LED 应该一直亮。
 *   不亮 → 依次查：LED 极性（长脚是正极）→ 接头松紧 → 电阻好坏
 *   半亮/闪烁 → 接触不良
 *
 * ★ 测完记得把正式程序刷回去：
 *     接收端板 → rx_alert
 *     发射端板 → tx_radar
 */

#define TEST_LED 1

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(TEST_LED, OUTPUT);
  digitalWrite(TEST_LED, HIGH);

  if (Serial) {
    Serial.println();
    Serial.println("[TEST] GPIO1 已置高，LED 应该恒亮");
    Serial.println("[TEST] 不亮就查：LED 极性 / 接头松紧 / 限流电阻");
    Serial.println("[TEST] 测完把 rx_alert 或 tx_radar 刷回来");
  }
}

void loop() {
  delay(1000);
}
