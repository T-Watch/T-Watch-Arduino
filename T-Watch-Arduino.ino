#include <TinyGPS++.h>
#include <Wire.h>
#include <SD.h>
#include <ADXL345.h>

const int t_delay PROGMEM = 3000;

unsigned int distance = 0;
unsigned int duration = 0;
byte BPM = 0;
unsigned int t_current, t_updated = 0;
float old_latitude, old_longitude;

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

ADXL345 adxl;
TinyGPSPlus gps;
Training *current_training = NULL;

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  Serial.print(F("Serial..."));
  Serial1.begin(9600);
  while (!Serial1);
  Serial.print(F("GPS..."));
  Serial2.begin(9600);
  while (!Serial2);
  Serial.print(F("BT..."));

  // SD
  while (!SD.begin());
  Serial.print(F("SD..."));

  if (!SD.exists(F("t")))
  {
    SD.mkdir(F("t"));
  }

  if (!SD.exists(F("r")))
  {
    SD.mkdir(F("r"));
  }

  // IMU
  adxl.init();
  Serial.print(F("IMU..."));

  t_updated = millis();
  Serial.println(F("Ready!"));

  current_training = readTraining(F("5ABB83F"));
}

void loop()
{
  // Training mode
  if (current_training != NULL) {
    t_current = millis();

    if (t_current >= (t_updated + 60000)) {
      BPM = 0;
    }

    if (analogRead(0) > 550) {
      BPM++;
    }

    while (Serial1.available())
    {
      if (gps.encode(Serial1.read()) && t_current >= (t_updated + t_delay))
      {

        if (!gps.satellites.value()) {
          Serial.println(F("GPS without signal"));
          t_updated = millis();
          break;
        }

        if (!gps.location.isValid() || !gps.speed.isValid() || !gps.date.isValid() || !gps.time.isValid()) {
          Serial.println(F("GPS Data not valid"));
          t_updated = millis();
          break;
        }

        int maxDuration = current_training->current->duration;
        int maxDistance = current_training->current->distance;

        Serial.println(maxDuration);
        Serial.println(duration);
        Serial.println(maxDistance);
        Serial.println(distance);
        Serial.println("*");

        if ((maxDuration != -1 && duration >= maxDuration) || (maxDistance != -1 && distance >= maxDistance)) {
          char c;
          if (nextTrainingBlock(current_training) == NULL) {
            c = '&';
            freeTraining(current_training);
            Serial.println(F("Finished training"));
          } else {
            c = '$';
          }
          duration += (t_current - t_updated) / 1000;
          saveTBResult(String(current_training->_id).substring(0, 7), c);
          duration = 0;
          distance = 0;
          BPM = 0;
        } else {
          duration += (t_current - t_updated) / 1000;
          saveTBResult(String(current_training->_id).substring(0, 7), '#');
        }
        t_updated = millis();
      }
    }
    return;
  }

  if (Serial2.available())
  {
    if (Serial2.readString() == F("empezar"))
    {
      Serial.println(F("Send results started"));
      sendResultsBT();
    }

    if (Serial2.readString() == F("trainings"))
    {
      Serial.println(F("receiveTrainingsBT started"));
      receiveTrainingsBT();
    }
  }
}

/*void sendTrainingsI2C() {
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
  }*/

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
    Serial.println(F("Error saving data"));
    return;
  }

  if (old_latitude != 0 && old_longitude != 0 && !samePoint ) {
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
  logFile.print(BPM);
  logFile.println(end);

  logFile.close();
}

void  sendResultsBT() {
  File results = SD.open(F("/r"));
  if (!results) {
    Serial.println(F("Error opening results directory"));
    results.close();
    Serial2.print(F("&"));
    waitForACK();
    sendACK();
    return;
  }

  File logFile = results.openNextFile();

  if (!logFile || !logFile.available()) {
    Serial.println(F("No results"));
    logFile.close();
    results.close();
    Serial2.print(F("&"));
    waitForACK();
    sendACK();
    return;
  }

  while (logFile && logFile.available()) {
    Serial2.print(F("resultado entrenamiento"));
    waitForACK();

    String s = logFile.readStringUntil('\n');
    if (!logFile.available()) {
      logFile.close();
      logFile = results.openNextFile();
      if (logFile && logFile.available()) {
        s.replace(F("&"), F("¡"));
      }
    }
    Serial2.print(s);
    waitForACK();
  }

  sendACK();
  logFile.close();
  results.close();
}

void receiveTrainingsBT() {
  sendACK();
  while (true) {
    if (!Serial2.available()) {
      continue;
    }

    String msg = Serial2.readString();
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
    if (Serial2.available() > 0 ) //Si hay datos en el BTSerial
    {
      if (Serial2.readString() == F("vale")) {
        break;
      }
    }
  }
}

void sendACK() {
  Serial2.print(F("vale"));
}

void sendError() {
  Serial.println(F("BT error"));
  Serial2.print(F("error"));
}

void trainingToSD() {
  File training;
  String msg;
  while (true) {
    if (!Serial2.available()) {
      continue;
    }
    msg = Serial2.readString();
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
    if (!Serial2.available()) {
      continue;
    }

    msg = Serial2.readString();

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
  File dataFile = SD.open((String)F("/t/") + fileName + (String)(".txt"), FILE_READ);

  if (!dataFile) {
    Serial.println((String)F("Error opening Training") + fileName);
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

  String null_string = "null";
  while (dataFile.available()) {
    String msg = dataFile.readStringUntil('\n');

    if (msg.startsWith("*")) {
      tb->next = ((TrainingBlock*)malloc(sizeof (TrainingBlock)));
      tb = tb->next;
      continue;
    }

    msg.toCharArray(tb->_id, 24);
    msg = dataFile.readStringUntil('\n');
    tb->distance = msg.startsWith(null_string) ? -1 : msg.toInt();
    msg = dataFile.readStringUntil('\n');
    tb->duration = msg.startsWith(null_string) ? -1 : msg.toInt();
    msg = dataFile.readStringUntil('\n');
    tb->maxHR = msg.startsWith(null_string) ? -1 : msg.toInt();
    msg = dataFile.readStringUntil('\n');
    tb->minHR = msg.startsWith(null_string) ? -1 : msg.toInt();
    msg = dataFile.readStringUntil('\n');
    tb->maxSpeed = msg.startsWith(null_string) ? -1 : msg.toFloat();
    msg = dataFile.readStringUntil('\n');
    tb->minSpeed = msg.startsWith(null_string) ? -1 : msg.toFloat();
    msg = dataFile.readStringUntil('\n');
    tb->altitude = msg.startsWith(null_string) ? -1 : msg.toInt();
    tb->next = NULL;
  }
  dataFile.close();
  return t;
}

TrainingBlock* nextTrainingBlock(Training *t) {
  if (t == NULL) {
    return NULL;
  }
  TrainingBlock* tb = t->current;
  t->current = tb->next;
  return t->current;
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
    previous = NULL;
  }
  free(t);
  t = NULL;
}

