// dai4_0_led_check（第4回：外付けLEDの配線を確かめる）
// 【新規作成の教材・実機未検証】
//
// ボード：Arduino UNO R4 WiFi
// 配線：赤 D2　黄 D3　青 D4　緑 D5　白 D6（それぞれ 抵抗 → LED の＋、LED の− → GND）
//
// 赤 → 黄 → 青 → 緑 → 白 の順に、1個ずつ 0.5秒 光ります。
// LED マトリクスには、いま光らせているピンの番号（2〜6）が出ます。
// 光らない色があったら、その色の LED の向き・抵抗・GND の線を確かめます。
// 確かめたら、dai4_ai_remote を書き込みます。

#include "Arduino_LED_Matrix.h"

const int LED_PINS[5] = {2, 3, 4, 5, 6};  // 赤 黄 青 緑 白
ArduinoLEDMatrix matrix;

// 数字（横3 × 縦5）
const char* DIGITS[10][5] = {
  {"###", "#.#", "#.#", "#.#", "###"}, {".#.", "##.", ".#.", ".#.", "###"},
  {"###", "..#", "###", "#..", "###"}, {"###", "..#", "###", "..#", "###"},
  {"#.#", "#.#", "###", "..#", "..#"}, {"###", "#..", "###", "..#", "###"},
  {"###", "#..", "###", "#.#", "###"}, {"###", "..#", "..#", "..#", "..#"},
  {"###", "#.#", "###", "#.#", "###"}, {"###", "#.#", "###", "..#", "###"}
};

// 1けたの数字を LED マトリクスのまん中に出す
void showDigit(int d) {
  uint8_t frame[8][12] = {0};
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 3; x++) {
      if (DIGITS[d][y][x] == '#') frame[y + 1][x + 5] = 1;
    }
  }
  matrix.renderBitmap(frame, 8, 12);
}

void setup() {
  matrix.begin();
  for (int i = 0; i < 5; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
}

void loop() {
  for (int i = 0; i < 5; i++) {
    showDigit(LED_PINS[i]);             // ピンの番号を表示
    digitalWrite(LED_PINS[i], HIGH);    // 光らせる
    delay(500);
    digitalWrite(LED_PINS[i], LOW);     // 消す
  }
}
