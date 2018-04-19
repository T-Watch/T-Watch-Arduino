#include <TinyGPS++.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Time.h>
#include <TimeLib.h>
#include <ADXL345.h>
#include <epd1in54.h>
#include <epdpaint.h>
#include "imagedata.h"


const int t_delay PROGMEM = 3000;

unsigned int distance = 0;
unsigned int duration = 0;
byte BPM = 0;
unsigned int t_current, t_updated = 0;
float old_latitude, old_longitude;

//SCREEN
Epd epd;
unsigned char imagen[1024];
Paint paint(imagen, 0, 0);



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

//Trainings in screen
//TrainingBlock tb[5]; //Fijar un maximo
//char training_type[10];
int nextTB = 0;

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

  //SPI
  pinMode(4, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(4, HIGH);
  //  SPI.begin();

  //BUTTONS
  pinMode(11, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  
  //SCREEN
  setTime(0, 0, 0, 27, 4, 2018);
  digitalWrite(4, LOW);

  if (epd.Init(lut_full_update) != 0) {
    Serial.print(F("e-Paper init failed"));
    return;
  }
  epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
  epd.DisplayFrame();
  epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
  epd.DisplayFrame();
  if (epd.Init(lut_partial_update) != 0) {
    Serial.print(F("e-Paper init failed"));
    return;
  }
  epd.SetFrameMemory(IMAGE_DATA);
  epd.DisplayFrame();
  epd.SetFrameMemory(IMAGE_DATA);
  epd.DisplayFrame();
  digitalWrite(4, HIGH);

  // SD
  digitalWrite(10, LOW);

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

  digitalWrite(10, HIGH);


  // IMU
  adxl.init();
  Serial.print(F("IMU..."));

  t_updated = millis();
  Serial.println(F("Ready!"));

  //current_training = readTraining(F("5ABB83F"));
}

void loop()
{
  paintTime();

  /*just for Testing*/
  if (nextTB == 1) {
    paintTrainingBlock(current_training->current);
  }
  /*just for Testing*/
  if (nextTB == 2) {
    Serial.println(F("FIN"));
    paintEndTraining();
  }

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
      if (digitalRead(11) == 0) {
        Serial.println(F("Boton 12!"));
        trainingsOnScreen();

      }
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
            paintEndTraining();
          } else {
            c = '$';
            paintTrainingBlock(current_training->current);
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

void trainingsOnScreen() {
  digitalWrite(4, HIGH);
  digitalWrite(10, LOW);

  Training tr[8];
  String aux;
  int cont = 0;
  char copy_id[24];
  char copy_date[29];

  //Read Trainings ID and DATE
  File trainings = SD.open(F("/t"));
  while (!trainings) {
    Serial.println("Opening SD");
    trainings = SD.open(F("/t"));
  }

  while (true) {
    File f =  trainings.openNextFile();
    if (!f || !f.available()) {
      f.close();
      break;
    }
    aux = f.readStringUntil('\n').substring(0, 7);
    aux.toCharArray(tr[cont]._id, 24);
    aux = f.readStringUntil('\n').substring(0, 15);
    aux.toCharArray(tr[cont].date, 29);
    f.readStringUntil('\n'); //Remove #
    cont++;
    f.close();
  }

  trainings.close();

  //Paint trainings (date)
  digitalWrite(10, HIGH);
  digitalWrite(4, LOW);

  int position = 20;
  int position_aux = 20;
  int select_training = 0;
  int i = 0;

  paint.SetWidth(200);
  paint.SetHeight(20);
  paint.Clear(0);
  //paint.DrawRectangle(0, 20, 200, 20, 0);
  paint.DrawStringAt(20, 4, "ENTRENAMIENTOS", &Font16, 1);
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());

  for (i = 0; i < 9; i++) {
    paint.Clear(1);
    //paint.DrawRectangle(0, position, 200, position, 1);
    if (strlen(tr[i]._id) != 0) {
      paint.DrawStringAt(0, 4, tr[i].date, &Font16, 0);
    }
    epd.SetFrameMemory(paint.GetImage(), 0, position, paint.GetWidth(), paint.GetHeight());
    position += 20;
  }
  epd.DisplayFrame();


  while (1) {
    digitalWrite(10, HIGH);
    digitalWrite(4, LOW);
    paint.SetWidth(200);
    paint.SetHeight(20);
    if (digitalRead(2) == 0) {
      digitalWrite(10, LOW);
      digitalWrite(4, HIGH);
      current_training = readTraining(tr[select_training]._id);
      digitalWrite(10, HIGH);
      digitalWrite(4, LOW);
      paintTrainingBlock(current_training->current);
      return;
    }
    if (digitalRead(3) == 0) {
      paint.Clear(0);
      paint.DrawStringAt(20, 4, "ENTRENAMIENTOS", &Font16, 1);
      epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
      position = 20;
      for (i = 0; i < 9; i++) {
        paint.Clear(1);
        //paint.DrawRectangle(0, position, 200, position, 1);
        if (strlen(tr[i]._id) != 0) {
          paint.DrawStringAt(0, 4, tr[i].date, &Font16, 0);
        }
        epd.SetFrameMemory(paint.GetImage(), 0, position, paint.GetWidth(), paint.GetHeight());
        position += 20;
      }
      if (position_aux != 20) {
        select_training++;
      }
      paint.Clear(0);
      paint.DrawStringAt(0, 4, tr[select_training].date, &Font16, 1);
      epd.SetFrameMemory(paint.GetImage(), 0, position_aux, 200, 24);
      position_aux += 20;
      if (strlen(tr[select_training + 1]._id) == 0) {
        position_aux = 20;
        select_training = 0;
      }
      epd.DisplayFrame();
    }
     if (digitalRead(11) == 0) {
      return;
    }
  }
}

void paintEndTraining() {
  digitalWrite(4, LOW);
  paint.SetWidth(200);
  paint.SetHeight(20);
  paint.Clear(0);
  paint.DrawStringAt(0, 4, "Fin Entrenamiento", &Font16, 1);
  epd.ClearFrameMemory(0xFF);
  epd.SetFrameMemory(paint.GetImage(), 0, 90, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
  delay(3000);
  return;
}

void paintTime() {
  time_t t = now();
  int h = 0;
  int m = 0;
  int s = 0;
  digitalWrite(4, LOW);
  String hora = " ";
  char  buf[50];
  String stringAux =  String();

  while (1) {
    String hora = " ";
    paint.SetWidth(200);
    paint.SetHeight(24);
    paint.Clear(1);
    //paint.DrawRectangle(0,20,200,20,1);
    paint.DrawStringAt(20, 4, "T-WATCH", &Font24, 0);
    epd.ClearFrameMemory(0xFF);
    epd.SetFrameMemory(paint.GetImage(), 20, 20, paint.GetWidth(), paint.GetHeight());


    paint.Clear(1);
    paint.DrawStringAt(20, 4, "27/04/2018", &Font16, 0);
    epd.SetFrameMemory(paint.GetImage(), 20, 50, paint.GetWidth(), paint.GetHeight());
    paint.Clear(1);

    t = now();
    stringAux =  String(hour(t));
    hora.concat(stringAux);
    hora.concat(":");
    stringAux =  String(minute(t));
    hora.concat(stringAux);
    hora.concat(":");
    stringAux =  String(second(t));
    hora.concat(stringAux);
    hora.toCharArray(buf, 50);
    paint.DrawStringAt(0, 4, buf, &Font24, 0);
    epd.SetFrameMemory(paint.GetImage(), 20, 120, paint.GetWidth(), paint.GetHeight());
    paint.Clear(0);
    epd.DisplayFrame();


    if (digitalRead(3) == 0)
    {
      h++;
      s = 0;
      if (h == 24)h = 0;
      Serial.println("SUMA SEG");
      setTime(h, m, s, 0, 0, 0);

    }
    if (digitalRead(2) == 0) {
      s = 0;
      m = m + 1;
      if (m == 60) {
        m = 0;
        h++;
      }
      Serial.println("SUMA MIN");
      setTime(h, m, s, 0, 0, 0);
    }
    if (digitalRead(11) == 0) {
      trainingsOnScreen();
      break;
    }
  }

  digitalWrite(4, HIGH);
  return;
}


void paintTrainingBlock(TrainingBlock* tb) {
  char distance[15];
  char duration[15];
  char m[2] = "m";
  char s[2] = "s";
  digitalWrite(10, HIGH);
  digitalWrite(4, LOW);
  for (int i = 0; i < 2; i++) {
    paint.SetWidth(200);
    paint.SetHeight(40);
    epd.ClearFrameMemory(0xFF);
    paint.Clear(0);
    paint.DrawStringAt(35, 10, current_training->type, &Font20, 1);
    paint.DrawRectangle(0, 40, 200, 40, 0);
    epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
    paint.SetWidth(200);
    paint.SetHeight(20);
    if (i == 0) {
      paint.Clear(1);
      paint.DrawStringAt(0, 4, "Iniciar Entrenamiento", &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 70, paint.GetWidth(), paint.GetHeight());
    }
    if (tb->distance != -1) {
      itoa(tb->distance, distance, 10);
      strcat(distance, m);
      paint.Clear(1);
      paint.DrawStringAt(60, 4, distance, &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 120, paint.GetWidth(), paint.GetHeight());
    }
    if (tb->duration != -1) {
      itoa(tb->duration, duration, 10);
      strcat(duration, s);
      paint.Clear(1);
      paint.DrawStringAt(60, 4, duration, &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 140, paint.GetWidth(), paint.GetHeight());
    }
    epd.DisplayFrame();
    while (1) {
      if (digitalRead(2) == 0) {
          /*INITIALIZE TRAINING*/
        break;
      }
    }
  }
  nextTB++; //Just for testing
  current_training->current = nextTrainingBlock(current_training);
  digitalWrite(4, HIGH);

  return;
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

  while (!dataFile) {
    dataFile = SD.open((String)F("/t/") + fileName + (String)(".txt"), FILE_READ);
    Serial.println((String)F("Opening Training") + fileName);
    //  return NULL;
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

