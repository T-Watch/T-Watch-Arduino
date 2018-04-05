#include <SoftwareSerial.h>//incluimos SoftwareSerial
#include <TinyGPS.h>//incluimos TinyGPS
#include <SD.h>
#include <Wire.h>
#include <ADXL345.h>
#include <math.h>
#define USE_ARDUINO_INTERRUPTS false
#include <PulseSensorPlayground.h>

typedef struct TrainingBlock
{
  String _id;
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
  String _id;
  String date;
  String maxDate;
  String type;
  TrainingBlock *trainingBlocks = NULL;
  TrainingBlock *current = NULL;
} Training;

PulseSensorPlayground pulseSensor;
ADXL345 adxl;
TinyGPS gps;
SoftwareSerial BT(5, 6);

void setup()
{
  //Imprimimos:
  pinMode(9, OUTPUT);
  Serial.begin(9600);
  while (!Serial) continue;

  BT.begin(9600);
  Serial.println(F("Bluetooth Inicializado"));

  // Initialize SD library
  while (!SD.begin(9)) {
    Serial.println(F("Failed to initialize SD library"));
    delay(1000);
  }

  if (!SD.exists(F("t")))
  {
    Serial.println(F("Carpeta trainings creada"));
    SD.mkdir(F("t"));
  }

  if (!SD.exists(F("r")))
  {
    Serial.println(F("Carpeta results creada"));
    SD.mkdir(F("r"));
  }

  pulseSensor.analogInput(0);
  pulseSensor.setThreshold(550);

  // Double-check the "pulseSensor" object was created and "began" seeing a signal.
  if (pulseSensor.begin()) {
    Serial.println(F("We created a pulseSensor Object !"));
  }

  //SD.remove(F("sensores.txt"));
  adxl.init();
}
void loop()
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  float myInts[12];
  while (Serial.available())
  {
    if (gps.encode(Serial.read()))
    {
      float latitude, longitude, latitude2, longitude2;
      gps.f_get_position(&latitude, &longitude);
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);

      delay(3000);

      gps.f_get_position(&latitude2, &longitude2);
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);
      float distance = gps.distance_between(latitude, longitude, latitude2, longitude2);

      myInts[0] = latitude2;
      myInts[1] = longitude2;
      myInts[2] = day;
      myInts[3] = month;
      myInts[4] = year;
      myInts[5] = (hour + 1);
      myInts[6] = minute;
      myInts[7] = second;
      myInts[8] = gps.f_altitude();
      myInts[9] = gps.f_course();
      myInts[10] = gps.f_speed_kmph();
      myInts[11] = distance;
      Serial.print(F("Escribiendo en sd.."));
      guardarDatos(myInts);
    }
  }

  if (BT.available() > 0 ) //Si hay datos en el BTSerial
  {
    if (BT.readString() == F("empezar"))
    {
      Serial.println(F("Send results started"));
      sendResultsBT();
    }

    if (BT.readString() == F("trainings"))
    {
      Serial.println(F("receiveTrainingsBT started"));
      receiveTrainingsBT();
    }
  }
  /*Training *t = readTraining(F("t1.txt"));
    printTraining(t);
    freeTraining(t);*/
  delay(3000);
}

float axisAccel(char axis) {
  float a = adxl.AxisDigitalAccelerometerRead(5, axis);
  return acos(a) * 180 / (PI);
}


void guardarDatos(float myInts[12])
{
  File logFile = SD.open(F("sensores.txt"), FILE_WRITE);

  if (logFile) {
    Serial.println(F("JJ"));
    int pulsaciones;

    pulsaciones = pulseSensor.getBeatsPerMinute();;

    logFile.print(F("*"));
    logFile.print(myInts[0], 3);
    logFile.print(F(";"));
    logFile.print(myInts[1], 3);
    logFile.print(F(";"));
    logFile.print((int)myInts[5]);
    logFile.print(F(":"));
    logFile.print((int)myInts[6]);
    logFile.print(F(":"));
    logFile.print((int)myInts[7]);
    logFile.print(F(";"));
    logFile.print((int)myInts[2]);
    logFile.print(F("/"));
    logFile.print((int)myInts[3]);
    logFile.print(F("/"));
    logFile.print((int)myInts[4]);
    logFile.print(F(";"));

    logFile.print((int)myInts[8]);
    logFile.print(F(";"));
    logFile.print((int)myInts[9]);
    logFile.print(F(";"));
    logFile.print(myInts[10], 2);
    logFile.print(F(";"));
    logFile.print((int)myInts[11]);

    logFile.print(F("~"));
    logFile.print(axisAccel('X'));
    logFile.print(F(";"));
    logFile.print(axisAccel('Y'));
    logFile.print(F(";"));
    logFile.print(axisAccel('Z'));

    logFile.print(F("~"));
    logFile.print(pulsaciones);
    logFile.println(F("#")); //comprobar correcto ln

    logFile.close();

  }
  else {
    Serial.println(F("Error al abrir el archivo"));
  }
}

void  sendResultsBT() {
  File logFile = SD.open(F("sensores.txt"), FILE_READ);

  if (!logFile) {
    sendError();
    Serial.print(F("The text file cannot be opened"));
    return;
  }

  if (!logFile.available()) {
    Serial.print(F("No results to send"));
    BT.print("&");
    return;
  }

  while (logFile.available()) {
    BT.write("resultado entrenamiento");
    waitForACK();
    BT.print(logFile.readStringUntil('\n'));
    waitForACK();
  }
  sendACK();
  Serial.println("JJ");

  logFile.close();
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
  BT.print("vale");
}

void sendError() {
  Serial.println(F("BT error"));
  BT.print("error");
}

void trainingToSD() {
  File training;
  String msg;
  while (true) {
    if (!BT.available()) {
      continue;
    }
    msg = BT.readString();
    training = SD.open(msg.substring(8) + F(".txt"), FILE_WRITE);
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
      delay(100);
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
  File dataFile = SD.open("/t/" + fileName, FILE_READ);

  if (!dataFile) {
    Serial.println(F("error opening training.txt"));
    return NULL;
  }

  Training *t = (Training*)malloc(sizeof (Training));
  TrainingBlock *tb;

  t->trainingBlocks = ((TrainingBlock*)malloc(sizeof (TrainingBlock)));
  t->current = t->trainingBlocks;

  tb = t->trainingBlocks;

  if (dataFile.available()) {
    t->_id = dataFile.readStringUntil('\n');
    t->date = dataFile.readStringUntil('\n');
    t->maxDate = dataFile.readStringUntil('\n');
    t->type = dataFile.readStringUntil('\n');
    dataFile.readStringUntil('\n'); //Remove #
  }

  while (dataFile.available()) {
    String msg = dataFile.readStringUntil('\n');

    if (msg.startsWith("*")) {
      tb->next = ((TrainingBlock*)malloc(sizeof (TrainingBlock)));
      tb = tb->next;
      continue;
    }

    tb->_id = msg;
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
