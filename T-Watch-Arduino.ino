#include <TinyGPS++.h>
#include <SD.h>
#include <Wire.h>
#include <ADXL345.h>
#define USE_ARDUINO_INTERRUPTS false
#include <PulseSensorPlayground.h>
#include <NeoSWSerial.h>

const int t_delay PROGMEM = 3000;

unsigned int distance = 0;
unsigned int duration = 0;
unsigned int t_current, t_updated = 0;
float old_latitude, old_longitude;
char current_training[8] = "";

typedef struct TrainingBlock
{
  char _id[24];
  int distance;
  int duration;
  int maxHR;
  int minHR;
  float maxSpeed;
  float minSpeed;
  int altitude;
  TrainingBlock *next = NULL;
} TrainingBlock;

typedef struct
{
  char _id[24];
  char date[29];
  char maxDate[29];
  char type[10];
  TrainingBlock *trainingBlocks = NULL;
  TrainingBlock *current = NULL;
} Training;

PulseSensorPlayground pulseSensor;
ADXL345 adxl;
TinyGPSPlus gps;
NeoSWSerial BT(5, 6);

void setup()
{
  Serial.begin(9600);
  while (!Serial) continue;

  // BT
  BT.begin(9600);

  // SD
  while (!SD.begin()) {
  }

  // I2C
  Wire.begin();

  if (!SD.exists(F("t")))
  {
    SD.mkdir(F("t"));
  }

  if (!SD.exists(F("r")))
  {
    SD.mkdir(F("r"));
  }


  // Pulse Sensor
  pulseSensor.analogInput(0);
  pulseSensor.setThreshold(550);
  if (pulseSensor.begin()) {
    // Serial.println(F("pulseSensor"));
  }

  // IMU
  adxl.init();

  t_updated = millis();

  sendTrainingsI2C();
  Serial.println(F("Ready!"));
}
void loop()
{
  pulseSensor.sawNewSample();

  if (strcmp(current_training, "") != 0) {
    t_current = millis();

    while (Serial.available())
    {
      if (gps.encode(Serial.read()) && t_current >= (t_updated + t_delay))
      {
        if (!gps.satellites.value()) {
          // Serial.println(F("GPS without signal"));
          t_updated = millis();
          break;
        }
        if (!gps.location.isValid() || !gps.speed.isValid() || !gps.date.isValid() || !gps.time.isValid()) {
          // Serial.println(F("GPS Data not valid"));
          t_updated = millis();
          break;
        }
        duration += (t_current - t_updated) / 1000;
        saveTBResult(current_training, '#');
        t_updated = millis();
      }
    }
    return;
  }

  if (BT.available())
  {
    if (BT.readString() == F("empezar"))
    {
      // Serial.println(F("Send results started"));
      sendResultsBT();
    }

    if (BT.readString() == F("trainings"))
    {
      // Serial.println(F("receiveTrainingsBT started"));
      receiveTrainingsBT();
    }
  }

  /*Training *t = readTraining(F("5ABB83F.txt"));
    printTraining(t);
    freeTraining(t);*/
}

void sendTrainingsI2C() {
  File trainings = SD.open(F("/t"));
  if (!trainings) {
    return;
  }

  while (true) {
    File f =  trainings.openNextFile();
    if (!f || !f.available()) {
      f.close();
      break;
    }

    Wire.beginTransmission(1);
    Wire.write(f.readStringUntil('\n').substring(0, 7).c_str());
    Wire.write("#");
    Wire.write(f.readStringUntil('\n').c_str());
    Wire.endTransmission();

    f.close();
  }

  trainings.close();
}

float axisAccel(char axis) {
  float a = adxl.AxisDigitalAccelerometerRead(5, axis);
  return acos(a) * 180 / (PI);
}

void saveTBResult(String trainingID, char end)
{
  float latitude, longitude;
  boolean samePoint = gps.location.lat() == old_latitude && old_longitude == gps.location.lng();
  File logFile = SD.open((String)F("/r/") + trainingID + (String)F(".txt"), FILE_WRITE);

  if (!logFile) {
    // Serial.println(F("Error saving data"));
    return;
  }

  if (old_latitude != 0 & old_longitude != 0 && !samePoint ) {
    distance += gps.distanceBetween(latitude, longitude, old_latitude, old_longitude);
  }

  old_latitude = gps.location.lat();
  old_longitude = gps.location.lng();

  // GPS
  logFile.print(gps.location.lat(), 6);
  logFile.print(F(";"));
  logFile.print(gps.location.lng(), 6);
  logFile.print(F(";"));

  // Date
  logFile.print(gps.time.hour());
  logFile.print(F(":"));
  logFile.print(gps.time.minute());
  logFile.print(F(":"));
  logFile.print(gps.time.second());
  logFile.print(F(";"));
  logFile.print(gps.date.day());
  logFile.print(F("/"));
  logFile.print(gps.date.month());
  logFile.print(F("/"));
  logFile.print(gps.date.year());
  logFile.print(F(";"));

  logFile.print(gps.altitude.meters());
  logFile.print(F(";"));

  logFile.print(gps.course.deg());
  logFile.print(F(";"));

  logFile.print(samePoint ? 0 : (60 / gps.speed.kmph()), 6);
  logFile.print(F(";"));
  logFile.print(distance);

  logFile.print(F("~"));
  logFile.print(axisAccel('X'));
  logFile.print(F(";"));
  logFile.print(axisAccel('Y'));
  logFile.print(F(";"));
  logFile.print(axisAccel('Z'));

  logFile.print(F("~"));
  logFile.print(pulseSensor.getBeatsPerMinute());
  logFile.println(end);

  logFile.close();
}

void  sendResultsBT() {
  File results = SD.open(F("/r"));
  if (!results) {
    // Serial.println(F("e-r"));
    results.close();
    BT.print(F("&"));
    waitForACK();
    sendACK();
    return;
  }

  File logFile = results.openNextFile();

  if (!logFile || !logFile.available()) {
    // Serial.println(F("No results"));
    logFile.close();
    results.close();
    BT.print(F("&"));
    waitForACK();
    sendACK();
    return;
  }

  while (logFile && logFile.available()) {
    BT.print(F("resultado entrenamiento"));
    waitForACK();

    String s = logFile.readStringUntil('\n');
    if (!logFile.available()) {
      logFile.close();
      logFile = results.openNextFile();
      if (logFile && logFile.available()) {
        s.replace(F("&"), F("¡"));
      }
    }
    BT.print(s);
    waitForACK();
  }

  sendACK();
  logFile.close();
  results.close();
}

void receiveTrainingsBT() {
  sendACK();
  while (true) {
    if (!BT.available()) {
      continue;
    }

    String msg = BT.readString();
    if (msg == F("nuevo entrenamiento")) {
      sendACK();
      trainingToSD();
    }

    if (msg == F("fin")) {
      sendACK();
      break;
    }
  }
}

void waitForACK() {
  while (true) {
    if (BT.available() > 0 ) //Si hay datos en el BTSerial
    {
      if (BT.readString() == F("vale")) {
        break;
      }
    }
  }
}

void sendACK() {
  BT.print(F("vale"));
}

void sendError() {
  Serial.println(F("BT error"));
  BT.print(F("error"));
}

void trainingToSD() {
  File training;
  String msg;
  while (true) {
    if (!BT.available()) {
      continue;
    }
    msg = BT.readString();
    if (SD.exists((String)F("/t/") + msg.substring(0, 7) + (String)F(".txt"))) {
      SD.remove((String)F("/t/") + msg.substring(0, 7) + (String)F(".txt"));
    }
    training = SD.open((String)F("/t/") + msg.substring(0, 7) + (String)F(".txt"), FILE_WRITE);
    break;
  }

  if (!training) {
    sendError();
    return;
  }

  training.println(msg);
  sendACK();

  while (true) {
    if (!BT.available()) {
      continue;
    }

    msg = BT.readString();

    if (msg == F("fin")) {
      sendACK();
      break;
    }

    training.println(msg);
    sendACK();
  }
  training.close();
}

Training* readTraining(String fileName) {
  File dataFile = SD.open((String)F("/t/") + fileName, FILE_READ);

  if (!dataFile) {
    // Serial.println(F("error opening training.txt"));
    return NULL;
  }

  Training *t = (Training*)malloc(sizeof (Training));
  TrainingBlock *tb;

  t->trainingBlocks = ((TrainingBlock*)malloc(sizeof (TrainingBlock)));
  t->current = t->trainingBlocks;

  tb = t->trainingBlocks;

  if (dataFile.available()) {
    dataFile.readStringUntil('\n').toCharArray(t->_id, 24);
    dataFile.readStringUntil('\n').toCharArray(t->date, 29);
    dataFile.readStringUntil('\n').toCharArray(t->maxDate, 29);
    dataFile.readStringUntil('\n').toCharArray(t->type, 10);
    dataFile.readStringUntil('\n'); //Remove #
  }


  while (dataFile.available()) {
    String msg = dataFile.readStringUntil('\n');

    if (msg.startsWith("*")) {
      tb->next = ((TrainingBlock*)malloc(sizeof (TrainingBlock)));
      tb = tb->next;
      continue;
    }

    msg.toCharArray(tb->_id, 24);
    msg = dataFile.readStringUntil('\n');
    tb->distance = msg.toInt();
    msg = dataFile.readStringUntil('\n');
    tb->duration = msg.toInt();
    msg = dataFile.readStringUntil('\n');
    tb->maxHR = msg.toInt();
    msg = dataFile.readStringUntil('\n');
    tb->minHR = msg.toInt();
    msg = dataFile.readStringUntil('\n');
    tb->maxSpeed = msg.toFloat();
    msg = dataFile.readStringUntil('\n');
    tb->minSpeed = msg.toFloat();
    msg = dataFile.readStringUntil('\n');
    tb->altitude = msg.toInt();
    tb->next = NULL;
  }
  dataFile.close();
  return t;
}

void printTraining(Training *t) {
  if (t == NULL) {
    return;
  }
  Serial.println(t->_id);
  Serial.println(t->date);
  Serial.println(t->maxDate);
  Serial.println(t->type);
  Serial.println(F("#"));
  TrainingBlock *tb = t->trainingBlocks;
  while (tb != NULL) {
    Serial.println(tb->_id);
    Serial.println(tb->distance);
    Serial.println(tb->duration);
    Serial.println(F("*"));
    tb = tb->next;
  }
}

void freeTraining(Training *t) {
  if (t == NULL) {
    return;
  }
  TrainingBlock *tb = t->trainingBlocks;
  while (tb != NULL) {
    TrainingBlock *previous = tb;
    tb = tb->next;
    free(previous);
  }
  free(t);
}

TrainingBlock* nextTrainingBlock(Training *t) {
  if (t == NULL) {
    return NULL;
  }
  TrainingBlock* tb = t->current;
  t->current = tb->next;
  return t->current;
}

int freeRAM () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
