/*
 * 诊断 · GPIO5 电平监视
 * ------------------------------------------------------------
 * 用途：雷达 OUT 疑似没信号时，直接盯 GPIO5 的实际电平，排除固件干扰。
 * 每秒打印一次当前电平 + 累计翻转次数，变化时立即打印。
 *
 * 判读：
 *   一直 = 0，翻转 0 次        → 雷达没驱动 OUT（没供电 / 接错脚 / 没共地 / 雷达坏）
 *   一直 = 1，翻转 0 次        → OUT 恒高（距离门太远、灵敏度太高，或接错到了电源脚）
 *   在 0 / 1 之间翻转           → 接线正常，问题在别处
 *
 * ★ 测完记得把 tx_radar 刷回发射端
 */

#define RADAR_OUT 5

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(RADAR_OUT, INPUT_PULLDOWN);

  if (Serial) {
    Serial.println();
    Serial.println("[DIAG] 开始监视 GPIO5（雷达 OUT）");
    Serial.println("[DIAG] 每秒打印一次电平，变化时立刻打印");
  }
}

void loop() {
  static int      last  = -1;
  static uint32_t flips = 0;
  static uint32_t tick  = 0;

  int v = digitalRead(RADAR_OUT);

  if (v != last) {
    if (last != -1) flips++;
    last = v;
    if (Serial) {
      Serial.print("[DIAG] GPIO5 变为 ");
      Serial.println(v);
    }
  }

  if (millis() - tick >= 1000) {
    tick = millis();
    if (Serial) {
      Serial.print("[DIAG] 当前电平=");
      Serial.print(v);
      Serial.print("  累计翻转=");
      Serial.println((unsigned long)flips);
    }
  }

  delay(20);
}
