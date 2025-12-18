// ## 設定構造体とクラス定義
struct SteppingMotorConf
{
  int pinA1;
  int pinA2;
  int pinB1;
  int pinB2;
  float doorPositionLimit;
  float frontStep;
  float backStep;
};

class SteppingMotor
{
private:
  int count = 3; // 0~3
  int pinA1;
  int pinA2;
  int pinB1;
  int pinB2;
  float doorPosition = 0;
  float doorPositionLimit;
  float frontStep;
  float backStep;

  void io(int pin1, int pin2, int state)
  {
    switch (state)
    {
    case -1:
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      break;
    case 1:
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      break;
    default:
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      break;
    }
  }

  void move(int a, int b)
  { // -1 or 0 or 1
    io(pinA1, pinA2, a);
    io(pinB1, pinB2, b);
  }

public:
  enum Direction
  {
    Front,
    Back,
  };

  SteppingMotor(const SteppingMotorConf &conf)
  {
    pinA1 = conf.pinA1;
    pinA2 = conf.pinA2;
    pinB1 = conf.pinB1;
    pinB2 = conf.pinB2;
    pinMode(pinA1, OUTPUT);
    pinMode(pinA2, OUTPUT);
    pinMode(pinB1, OUTPUT);
    pinMode(pinB2, OUTPUT);
    doorPositionLimit = conf.doorPositionLimit;
    frontStep = conf.frontStep;
    backStep = conf.backStep;
  }

  bool stepWithLimit(SteppingMotor::Direction direction) // スコープ指定がないとコンパイルエラーになる
  {
    // 超過した値を修正
    if (doorPosition > doorPositionLimit)
    {
      doorPosition = doorPositionLimit;
    }
    else if (doorPosition < 0)
    {
      doorPosition = 0;
    }
    // モーター動作と位置を加算
    if (direction == Front && doorPosition < doorPositionLimit)
    {
      step(Front);
      doorPosition += frontStep;
      if (doorPosition >= doorPositionLimit)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else if (direction == Back && doorPosition > 0)
    {
      step(Back);
      doorPosition += backStep;
      if (doorPosition <= 0)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  void step(SteppingMotor::Direction direction)
  {
    // count
    if (direction == Front)
    {
      count += 1;
    }
    else
    {
      count -= 1;
    }
    if (count == 4)
    {
      count = 0;
    }
    else if (count == -1)
    {
      count = 3;
    }
    // move
    switch (count)
    {
    case 0:
      move(1, 1);
      break;
    case 1:
      move(1, -1);
      break;
    case 2:
      move(-1, -1);
      break;
    case 3:
      move(-1, 1);
      break;
    }
  }

  void powerStop()
  {
    switch (count)
    {
    case 0:
      move(1, 1);
      break;
    case 1:
      move(1, -1);
      break;
    case 2:
      move(-1, -1);
      break;
    case 3:
      move(-1, 1);
      break;
    }
  }

  void freeStop()
  {
    move(0, 0);
  }
};

struct DistanceSensorConf
{
  int pinTrig;
  int pinEcho;
  int pinLed;
  int sampleTimes;
  float sampleTolerance;
};

class DistanceSensor
{
private:
  int pinTrig;
  int pinEcho;
  int pinLed;

  int triggerCount = 0; // 0,1,2
  unsigned long triggerTime_us = 0;

  unsigned long startTime;
  unsigned long endTime;
  int lastEchoState;

  static const int sampleTimesMax = 20;
  int sampleTimes;
  int sampleCount;
  float samples[sampleTimesMax];
  float result;
  float sampleTolerance; // 外れ値判定用の許容範囲(中央値に対する割合)

  enum SensorMode
  {
    WaitStart,
    Trigger,
    Measuring,
    WaitNextMeasure,
    Completed,
  };
  SensorMode sensorMode = WaitStart;

  void startMeasure()
  {
    lastEchoState = LOW;
    triggerCount = 0; // 安全のため
    sensorMode = Trigger;
  }

  void trigger()
  {
    switch (triggerCount)
    {
    case 0:
      digitalWrite(pinTrig, LOW);
      triggerTime_us = micros();
      triggerCount = 1;
      break;
    case 1:
      if (micros() - triggerTime_us >= 2)
      {
        digitalWrite(pinTrig, HIGH);
        triggerTime_us = micros();
        triggerCount = 2;
      }
      break;
    case 2:
      if (micros() - triggerTime_us >= 10)
      {
        digitalWrite(pinTrig, LOW);
        triggerCount = 0;
        sensorMode = Measuring;
      }
    }
  }

  bool captureEcho()
  {
    // 計測完了でtrueを返す
    int state = digitalRead(pinEcho);
    if (lastEchoState == LOW && state == HIGH)
    {
      // LOW → HIGH（立ち上がり）
      startTime = micros();
      lastEchoState = HIGH;
      return false;
    }
    else if (lastEchoState == HIGH && state == LOW)
    {
      // HIGH → LOW（立ち下がり）
      endTime = micros();
      lastEchoState = LOW;
      return true;
    }
    else
    {
      return false;
    }
  }

  float getDistance()
  {
    long duration = endTime - startTime;
    float distance = duration * 0.0343 / 2.0;
    if (distance >= 2 && distance <= 400)
    {
      return distance;
    }
    else
    {
      return -1;
    }
  }

  float filter()
  {
    // 配列のコピーを作ってソート
    float sorted[sampleTimesMax];
    memcpy(sorted, samples, sampleTimes * sizeof(float));
    for (int i = 0; i < sampleTimes - 1; i++)
    {
      for (int j = i + 1; j < sampleTimes; j++)
      {
        if (sorted[i] > sorted[j])
        {
          float tmp = sorted[i];
          sorted[i] = sorted[j];
          sorted[j] = tmp;
        }
      }
    }
    // 中央値
    float median = sorted[sampleTimes / 2];
    // 外れ値除外(中央値 ± 15%)
    float sum = 0;
    int count = 0;
    for (size_t i = 0; i < sampleTimes; i++)
    {
      if (fabs(samples[i] - median) < median * sampleTolerance)
      {
        sum += samples[i];
        count++;
      }
    }
    // 平均値計算
    if (count == 0)
    {
      return -1; // 全て外れ値 → 失敗
    }
    return sum / count;
  }

public:
  DistanceSensor(const DistanceSensorConf &conf)
  {
    pinTrig = conf.pinTrig;
    pinEcho = conf.pinEcho;
    pinLed = conf.pinLed;
    pinMode(pinTrig, OUTPUT);
    pinMode(pinEcho, INPUT);
    pinMode(pinLed, OUTPUT);
    sampleTimes = conf.sampleTimes;
    sampleTolerance = conf.sampleTolerance;
  }

  void led(bool status)
  {
    switch (status)
    {
    case true:
      digitalWrite(pinLed, HIGH);
      break;
    case false:
      digitalWrite(pinLed, LOW);
      break;
    }
  }

  void clock()
  {
    switch (sensorMode)
    {
    case Trigger:
      trigger();
      break;
    case Measuring:
      bool isEnd = captureEcho();
      if (isEnd)
      {
        samples[sampleCount] = getDistance();
        sensorMode = WaitNextMeasure;
        sampleCount += 1;
        if (sampleCount == sampleTimes)
        {
          // 平均を計算
          result = filter();
          sensorMode = Completed;
        }
        else
        {
          startMeasure();
        }
      }
      break;
    }
  }

  void start()
  {
    if (sensorMode == WaitStart)
    {
      sampleCount = 0;
      startMeasure();
    }
  }

  float getResult()
  {
    if (sensorMode == Completed)
    {
      sensorMode = WaitStart;
      return result;
    }
    else
    {
      return -1;
    }
  }
};

struct ManualControlConf
{
  int pinA;
  int pinB;
};
class ManualControl
{
  // プルアップを介すため、LOWがON、HIGHがOFFになる
private:
  int pinA;
  int pinB;

public:
  enum Status
  {
    Front,
    Back,
    Stop,
  };

  ManualControl(const ManualControlConf &conf)
  {
    pinA = conf.pinA;
    pinB = conf.pinB;
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
  }

  bool isManualMode()
  {
    int stateA = digitalRead(pinA);
    int stateB = digitalRead(pinB);
    if (stateA == HIGH && stateB == HIGH)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  ManualControl::Status status()
  {
    int stateA = digitalRead(pinA);
    int stateB = digitalRead(pinB);
    if (stateA == LOW && stateB == LOW)
    {
      return ManualControl::Stop;
    }
    else if (stateA == LOW && stateB == HIGH)
    {
      return ManualControl::Front;
    }
    else if (stateA == HIGH && stateB == LOW)
    {
      return ManualControl::Back;
    }
  }
};

// ## enum,プロトタイプ宣言(ないとコンパイルエラーになる)
enum MainMode
{
  Close,
  Opening,
  Open,
  Closing,
  Manual,
};
void changeMode(MainMode newMode, SteppingMotor &motor);

// ## 変数定義(ユーザー変更可能エリア)
SteppingMotorConf motorConf = {
    // 17, 14, 15, 16, 1600
    .pinA1 = 17,
    .pinA2 = 14,
    .pinB1 = 15,
    .pinB2 = 16,
    .doorPositionLimit = 1450, // ドア全開位置の1/4ステップ数
    .frontStep = 1,
    .backStep = -1.05,
};
DistanceSensorConf distanceAConf = {
    .pinTrig = 2,
    .pinEcho = 3,
    .pinLed = 4,
    .sampleTimes = 10,       // 1度の距離算出のためのセンサー測定回数
    .sampleTolerance = 0.15, // 外れ値判定用の許容範囲(中央値に対する割合)
};
DistanceSensorConf distanceBConf = {
    .pinTrig = 8,
    .pinEcho = 9,
    .pinLed = 10,
    .sampleTimes = 10,       // 1度の距離算出のためのセンサー測定回数
    .sampleTolerance = 0.15, // 外れ値判定用の許容範囲(中央値に対する割合)
};
ManualControlConf manualConf = {
    .pinA = 11,
    .pinB = 12,
};
constexpr unsigned long sensorMeasureInterval_us = 50000; // センサー測定間隔(μs) 小さすぎると測定に失敗しやすい
constexpr unsigned int detectDistanceMin_cm = 5;          // センサー検出距離下限(cm)
constexpr unsigned int detectDistanceMax_cm = 70;         // センサー検出距離上限(cm)
constexpr unsigned long motorStepDurationFront_us = 5500; // 開く時のモーター1/4ステップ間隔(μs)
constexpr unsigned long motorStepDurationBack_us = 5500;  // 閉じる時のモーター1/4ステップ間隔(μs)
constexpr unsigned long openCountLimit_ms = 1000;         // ドア全開時間(ms)
bool enableManualMode = true;                             // マニュアル制御モード有効フラグ

// ## インスタンス生成,グローバル変数定義
SteppingMotor motor(motorConf);
DistanceSensor distanceA(distanceAConf);
DistanceSensor distanceB(distanceBConf);
ManualControl manual(manualConf);
unsigned long lastTrigTime_us = 0;
unsigned long lastStepTime_us = 0;
unsigned long openCount = 0;
MainMode mode = Close;
MainMode lastMode;
int usingSensor = 0; // 0:A, 1:B

// ## 関数定義
void changeMode(MainMode newMode, SteppingMotor &motor, DistanceSensor &sensorA, DistanceSensor &sensorB)

{
  switch (newMode)
  {
  case Open:
    openCount = openCountLimit_ms;
    motor.freeStop();
    sensorA.led(false);
    sensorB.led(false);
    break;
  case Opening:
    sensorA.led(true);
    sensorB.led(true);
    break;
  case Close:
    motor.freeStop();
    sensorA.led(false);
    sensorB.led(false);
    break;
  case Closing:
    sensorA.led(true);
    sensorB.led(true);
    break;
  }
  mode = newMode;
  switch (mode)
  {
  case Opening:
    Serial.println("mode:Opening");
    break;
  case Open:
    Serial.println("mode:Open");
    break;
  case Closing:
    Serial.println("mode:Closing");
    break;
  case Close:
    Serial.println("mode:Close");
    break;
  case Manual:
    Serial.println("mode:Manual");
    break;
  }
}

// ## Arduino標準初期処理
void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("--Initialized--");
  motor.freeStop();
}

// ## Arduino標準ループ処理
void loop()
{

  unsigned long now_us = micros();
  distanceA.clock();
  distanceB.clock();
  // 距離センサーによる検出処理
  if (mode == Close || mode == Closing || mode == Open)
  {
    // センサー起動
    if (now_us - lastTrigTime_us >= sensorMeasureInterval_us)
    {
      lastTrigTime_us = now_us;
      switch (usingSensor)
      {
      case 0:
        usingSensor = 1;
        distanceA.start();
        break;
      case 1:
        usingSensor = 0;
        distanceB.start();
        break;
      }
    }
    // 結果取得
    float resultA = distanceA.getResult();
    float resultB = distanceB.getResult();
    float result = (resultA >= 0) ? resultA : resultB;
    // 検出判定
    if (result >= detectDistanceMin_cm && result <= detectDistanceMax_cm)
    {
      Serial.println("distanceDetect:" + String(result));
      if (mode != Open)
      {
        changeMode(Opening, motor, distanceA, distanceB);
      }
      else
      {
        openCount = openCountLimit_ms; // open時に時間延長
      }
    }
  }
  // モーター動作
  switch (mode)
  {
  case Opening:
    if (now_us - lastStepTime_us >= motorStepDurationFront_us)
    {
      // 実行頻度が高いため、ここにSerial.println()を入れると距離センサー系の機能がうまく動かない
      lastStepTime_us = now_us;
      bool isEnd = motor.stepWithLimit(SteppingMotor::Front);
      if (isEnd)
      {
        changeMode(Open, motor, distanceA, distanceB);
      }
    }
    break;
  case Closing:
    if (now_us - lastStepTime_us >= motorStepDurationBack_us)
    {
      lastStepTime_us = now_us;
      bool isEnd = motor.stepWithLimit(SteppingMotor::Back);
      if (isEnd)
      {
        changeMode(Close, motor, distanceA, distanceB);
      }
    }
    break;
  case Open:
    if (now_us - lastStepTime_us >= 1000 && openCount > 0) // 1msごとにトリガー
    {
      lastStepTime_us = now_us;
      openCount -= 1;
      if (openCount == 0)
      {
        changeMode(Closing, motor, distanceA, distanceB);
      }
    }
    break;
  case Manual:
    ManualControl::Status status = manual.status();
    switch (status)
    {
    case ManualControl::Front:
      if (now_us - lastStepTime_us >= motorStepDurationFront_us)
      {
        lastStepTime_us = now_us;
        motor.step(SteppingMotor::Front);
      }
      break;
    case ManualControl::Back:
      if (now_us - lastStepTime_us >= motorStepDurationBack_us)
      {
        lastStepTime_us = now_us;
        motor.step(SteppingMotor::Back);
      }
      break;
    case ManualControl::Stop:
      motor.powerStop();
      break;
    }
    break;
  }
  // マニュアル制御確認
  if (enableManualMode)
  {
    if (manual.isManualMode() && mode != Manual)
    {
      lastMode = mode;
      changeMode(Manual, motor, distanceA, distanceB);
    }
    else if (!manual.isManualMode() && mode == Manual)
    {
      changeMode(lastMode, motor, distanceA, distanceB);
    }
  }
}