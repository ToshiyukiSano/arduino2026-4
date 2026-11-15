// dai4_ai_remote（第4回：手話やジェスチャーをAIで見分けて、Arduino を操作する）
// 【教材・実機未検証】
//
// ボード：Arduino UNO R4 WiFi（UNO R3・R4 Minima では動きません）
// 必要なライブラリ：Servo、ArduinoBLE
//   （UNO R4 WiFi の通信用モジュールのファームウェアが 0.2.0 以上であること）
//
// 配線（基本）：
//   ステージ1：外付けLED 1個　D2 → 抵抗 → LED の＋（長い足）、LED の−（短い足）→ GND
//   ステージ2：サーボ　信号 → D9、電源（＋）→ 5V、GND（−）→ GND（第3回と同じ）
//   ステージ3：本体の LED マトリクス（配線なし）
// 配線（発展・時間が余った人）：
//   LED を追加　D3・D4・D5・D6（それぞれ 抵抗 → LED → GND）
//
// しくみ：
//   PC（Chrome で開いた index.html）の AI が、カメラの画像のクラスを見分ける
//   → 操作ページで決めた1文字を Bluetooth（BLE）で送る → Arduino が動く。
//     O = LED 点灯　F = LED 消灯　　　　　　　　（ステージ1）
//     L = サーボ左　C = サーボ中央　R = サーボ右　（ステージ2）
//     1・2・3 = LED マトリクスの絵　　　　　　　（ステージ3）
//     S = LED を順番に点灯　　　　　　　　　　　（発展）
//     N = 待機（何も変えない）
//   AI は PC の中で動いています。Arduino が受け取るのは1文字だけです。
//   書き込むときは PC と USB でつなぐ（書き込みと電源）。
//   書き込んだあとは PC の USB を外し、USB 充電器などから Arduino の USB 端子へ給電する。
//
// 生徒が変えるのは「★ここを変える」の中だけです（発展では handleCommand に1行足します）。

#include <Servo.h>               // サーボを動かすための機能を読み込む
#include <ArduinoBLE.h>          // Bluetooth（BLE）を使うための機能を読み込む
#include "Arduino_LED_Matrix.h"  // 本体の LED マトリクスを使うための機能を読み込む

// LED のピン（ここは変えない）
const int RED    = 2;   // 基本の LED（ステージ1）
const int YELLOW = 3;   // 発展
const int BLUE   = 4;   // 発展
const int GREEN  = 5;   // 発展
const int WHITE  = 6;   // 発展

// =================== ★ここを変える（ここから）===================

// ① 機器名：先生から割り当てられた自分の番号にする（NGS-01 〜 NGS-11）
const char DEVICE_NAME[] = "NGS-01";

// ② ステージ2：サーボの角度（0〜180）
const int ANGLE_LEFT   = 180;  // 左
const int ANGLE_CENTER = 90;   // 中央
const int ANGLE_RIGHT  = 0;    // 右

// ③ ステージ3：LED マトリクスの絵（横12 × 縦8。「#」が光る、「.」が消える）
//    1行は必ず12文字。行は8行。
const char* PICTURE_1[8] = {   // 絵1：ハート
  "............",
  "..##....##..",
  ".####..####.",
  ".##########.",
  "..########..",
  "...######...",
  ".....##.....",
  "............"
};

const char* PICTURE_2[8] = {   // 絵2：にこにこ
  "....####....",
  "..##....##..",
  ".#..#..#..#.",
  ".#........#.",
  ".#.#....#.#.",
  ".#..####..#.",
  "..##....##..",
  "....####...."
};

const char* PICTURE_3[8] = {   // 絵3：上向きの矢印（余裕があれば使う）
  ".....##.....",
  "....####....",
  "...######...",
  "..##.##.##..",
  ".....##.....",
  ".....##.....",
  ".....##.....",
  "............"
};

// =================== ★ここを変える（ここまで）===================

static_assert(ANGLE_LEFT >= 0 && ANGLE_LEFT <= 180, "ANGLE_LEFT は 0〜180 にしてください");
static_assert(ANGLE_CENTER >= 0 && ANGLE_CENTER <= 180, "ANGLE_CENTER は 0〜180 にしてください");
static_assert(ANGLE_RIGHT >= 0 && ANGLE_RIGHT <= 180, "ANGLE_RIGHT は 0〜180 にしてください");

// ---- ここから下は、発展で handleCommand に1行足すとき以外は変更しない ----
const int SERVO_PIN = 9;  // サーボの信号の線（D9）
const int LED_PINS[5] = {RED, YELLOW, BLUE, GREEN, WHITE};

// BLE の設定（index.html と同じ番号。第3回とは別の番号）
BLEService remoteService("2fb61ef3-3619-447d-8e6c-58a330ada446");
BLEByteCharacteristic commandChar("a757a3b8-787c-44bb-a768-95c597baf42b",
                                  BLEWrite | BLEWriteWithoutResponse);
BLEStringCharacteristic statusChar("67fc9eee-e535-4917-99f6-222dc95351ce",
                                   BLERead | BLENotify, 32);

Servo servo;
ArduinoLEDMatrix matrix;
bool bleReady = false;
char lastCommand = 'N';
int lastAngle = 90;
bool redOn = false;
char pendingCmd = 0;  // Bluetooth で届いた指示（loop の中で動かす）

// つながったときの絵（小さなチェック）
const char* PICTURE_READY[8] = {
  "............", "............", "..........#.", ".........#..",
  "..#.....#...", "...#...#....", "....#.#.....", ".....#......"
};

// 数字（横3 × 縦5）：待っている間、機器名の番号を LED マトリクスに表示する
const char* DIGITS[10][5] = {
  {"###", "#.#", "#.#", "#.#", "###"}, {".#.", "##.", ".#.", ".#.", "###"},
  {"###", "..#", "###", "#..", "###"}, {"###", "..#", "###", "..#", "###"},
  {"#.#", "#.#", "###", "..#", "..#"}, {"###", "#..", "###", "..#", "###"},
  {"###", "#..", "###", "#.#", "###"}, {"###", "..#", "..#", "..#", "..#"},
  {"###", "#.#", "###", "#.#", "###"}, {"###", "#.#", "###", "..#", "###"}
};

// 絵（8行 × 12文字）を LED マトリクスに表示する
void showPicture(const char* picture[8]) {
  uint8_t frame[8][12] = {0};
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 12 && picture[y][x] != '\0'; x++) {
      if (picture[y][x] == '#') frame[y][x] = 1;
    }
  }
  matrix.renderBitmap(frame, 8, 12);
}

// 機器名の最後の2文字（番号）を LED マトリクスに表示する
void showNumber() {
  uint8_t frame[8][12] = {0};
  int len = strlen(DEVICE_NAME);
  for (int k = 0; k < 2; k++) {
    int i = len - 2 + k;
    if (i < 0) continue;
    char c = DEVICE_NAME[i];
    if (c < '0' || c > '9') continue;
    int left = (k == 0) ? 2 : 7;
    for (int y = 0; y < 5; y++) {
      for (int x = 0; x < 3; x++) {
        if (DIGITS[c - '0'][y][x] == '#') frame[y + 1][left + x] = 1;
      }
    }
  }
  matrix.renderBitmap(frame, 8, 12);
}

// 外付け LED をすべて消す
void ledsOff() {
  for (int i = 0; i < 5; i++) digitalWrite(LED_PINS[i], LOW);
  redOn = false;
}

// 発展：指定した LED だけを光らせる（ほかは消す）
void ledOnly(int pin) {
  ledsOff();
  digitalWrite(pin, HIGH);
  redOn = (pin == RED);
}

// 発展：LED を D2 → D6 の順に1回ずつ光らせる
void ledSequence() {
  for (int i = 0; i < 5; i++) {
    ledOnly(LED_PINS[i]);
    delay(200);
    BLE.poll();  // 光らせている間も Bluetooth の処理を続ける
  }
  ledsOff();
}

// PC へ今の状態を知らせる（例：「L:0:180:NGS-01」＝最後の文字:LED:角度:機器名）
void report() {
  char text[32];
  snprintf(text, sizeof(text), "%c:%d:%d:%s", lastCommand, redOn ? 1 : 0, lastAngle, DEVICE_NAME);
  Serial.println(text);
  if (bleReady) statusChar.writeValue(text);
}

// 指示を受け取ったときの動き
void handleCommand(char cmd) {
  if (cmd == 'O') {               // ステージ1：O なら LED 点灯
    digitalWrite(RED, HIGH);
    redOn = true;
  } else if (cmd == 'F') {        // ステージ1：F なら LED 消灯
    digitalWrite(RED, LOW);
    redOn = false;
  } else if (cmd == 'L') {        // ステージ2：L ならサーボを左へ
    servo.write(ANGLE_LEFT);
    lastAngle = ANGLE_LEFT;
    // 発展：ここに ledOnly(YELLOW); などを1行足すと、LED も切り替わる
  } else if (cmd == 'C') {        // ステージ2：C ならサーボを中央へ
    servo.write(ANGLE_CENTER);
    lastAngle = ANGLE_CENTER;
  } else if (cmd == 'R') {        // ステージ2：R ならサーボを右へ
    servo.write(ANGLE_RIGHT);
    lastAngle = ANGLE_RIGHT;
  } else if (cmd == '1') {        // ステージ3：1 なら絵1
    showPicture(PICTURE_1);
  } else if (cmd == '2') {        // ステージ3：2 なら絵2
    showPicture(PICTURE_2);
  } else if (cmd == '3') {        // ステージ3：3 なら絵3
    showPicture(PICTURE_3);
  } else if (cmd == 'S') {        // 発展：S なら LED を順番に点灯
    ledSequence();
  } else if (cmd == 'N') {        // 待機：何も変えない
  } else {
    return;  // それ以外（改行など）は何もしない
  }
  lastCommand = cmd;
  report();
}

// Bluetooth で指示が届いたとき
void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  pendingCmd = (char)commandChar.value();  // ここでは受け取るだけ。動かすのは loop の中
}

// PC とつながったとき
void onConnected(BLEDevice central) {
  Serial.print("connected: ");
  Serial.println(central.address());
  showPicture(PICTURE_READY);  // つながったらチェックの絵
  report();
}

// PC との接続が切れたとき
void onDisconnected(BLEDevice central) {
  Serial.println("disconnected");
  ledsOff();
  showNumber();     // 番号の表示に戻す（サーボはその角度のまま）
  BLE.advertise();  // もう一度つなげるように待つ
}

void setup() {
  Serial.begin(9600);
  matrix.begin();

  for (int i = 0; i < 5; i++) pinMode(LED_PINS[i], OUTPUT);
  ledsOff();

  servo.attach(SERVO_PIN);  // サーボの信号を出すピン（D9）
  servo.write(90);          // 電源を入れたら 90°（中央）へ
  lastAngle = 90;

  if (BLE.begin()) {
    BLE.setLocalName(DEVICE_NAME);
    BLE.setDeviceName(DEVICE_NAME);
    BLE.setAdvertisedService(remoteService);
    remoteService.addCharacteristic(commandChar);
    remoteService.addCharacteristic(statusChar);
    BLE.addService(remoteService);
    commandChar.setEventHandler(BLEWritten, onCommandWritten);
    BLE.setEventHandler(BLEConnected, onConnected);
    BLE.setEventHandler(BLEDisconnected, onDisconnected);
    bleReady = true;
    report();
    BLE.advertise();
    showNumber();  // 番号が出たら「接続を待っている」
    Serial.print("BLE OK: ");
    Serial.println(DEVICE_NAME);
  } else {
    // Bluetooth を始められなかった：LED マトリクスに「×」を出す
    const char* PICTURE_ERROR[8] = {
      "............", "...#....#...", "....#..#....", ".....##.....",
      ".....##.....", "....#..#....", "...#....#...", "............"
    };
    showPicture(PICTURE_ERROR);
    Serial.println("BLE start failed. Serial Monitor (9600): O F L C R 1 2 3 S N");
  }
}

void loop() {
  if (bleReady) BLE.poll();  // Bluetooth の処理（止めずに毎回まわす）

  if (pendingCmd != 0) {     // Bluetooth で指示が届いていたら動かす
    char cmd = pendingCmd;
    pendingCmd = 0;
    handleCommand(cmd);
  }

  // 予備：シリアルモニター（9600）から O・F・L・C・R・1・2・3・S・N を送っても動く
  while (Serial.available() > 0) {
    handleCommand((char)Serial.read());
  }
}
