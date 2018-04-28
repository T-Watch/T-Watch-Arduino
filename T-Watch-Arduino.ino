#include <TinyGPS++.h>
#include <SD.h>
#include <Time.h>
#include <TimeLib.h>
#include <ADXL345.h>
#include <epd1in54.h>
#include <epdpaint.h>
#include "imagedata.h"
#include "smallimagedata.h"


const int t_delay PROGMEM = 3000;

unsigned int distance = 0;
unsigned int duration = 0;
byte BPM = 0;
unsigned int t_current, t_updated, t_updated2 = 0;
float old_latitude, old_longitude;

//SCREEN
Epd epd;
unsigned char imagen[1024];
Paint paint(imagen, 0, 0);



typedef struct TrainingBlock
{
  char _id[25];
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
  char _id[25];
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
  delay(2000);
  Serial1.println(F("$PUBX,40,RMC,0,0,0,0*47")); //RMC OFF
  delay(100);
  Serial1.println(F("$PUBX,40,VTG,0,0,0,0*5E")); //VTG OFF
  delay(100);
  Serial1.println(F("$PUBX,40,GGA,0,0,0,0*5A")); //CGA OFF
  delay(100);
  Serial1.println(F("$PUBX,40,GSA,0,0,0,0*4E")); //GSA OFF
  delay(100);
  Serial1.println(F("$PUBX,40,GSV,0,0,0,0*59")); //GSV OFF
  delay(100);
  Serial1.println(F("$PUBX,40,GLL,0,0,0,0*5C")); //GLL OFF
  delay(1000);
  Serial.print(F("GPS..."));

  Serial2.begin(9600);
  while (!Serial2);
  Serial.print(F("BT..."));

  //SPI
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(6, HIGH);

  //BUTTONS
  pinMode(11, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  //SCREEN
  setTime(0, 0, 0, 27, 4, 2018);
  enableScreen();

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

  // SD
  enableSD();
  while (!SD.begin(6));
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
}

void loop()
{

  paintTime();
  if (current_training != NULL) {
    Serial.println(F("Training mode"));
    t_updated = millis();
  }
  while (current_training != NULL) {
    t_current = millis();

    if (digitalRead(11) == 0) {
      freeTraining(&current_training);
      break;
    }

    if (t_current >= (t_updated2 + 60000)) {
      BPM = 0;
      t_updated2 = millis();
    }

    if (analogRead(0) > 550) {
      BPM++;
    }

    if (t_current >= (t_updated + t_delay)) {
      duration += (t_current - t_updated) / 1000;
      t_updated = millis();
      Serial1.println(F("$EIGPQ,RMC*3A"));
    }

    while (Serial1.available())
    {
      if (gps.encode(Serial1.read())) {
        if (!gps.satellites.value()) {
          Serial.println(F("GPS without signal"));
          // break;
        }

        if (!gps.location.isValid() || !gps.speed.isValid() || !gps.date.isValid() || !gps.time.isValid()) {
          Serial.println(F("GPS Data not valid"));
          // break;
        }

        int maxDuration = current_training->current->duration;
        int maxDistance = current_training->current->distance;

        if ((maxDuration != -1 && duration >= maxDuration) || (maxDistance != -1 && distance >= maxDistance)) {
          if (nextTrainingBlock(current_training) == NULL) {
            saveTBResult(current_training, '&');
            t_updated = 0;
            t_updated2 = 0;
            t_current = 0;
            freeTraining(&current_training);
            Serial.println(F("Finished training"));
            paintEndTraining();
          } else {
            saveTBResult(current_training, '$');
            paintTrainingBlock(current_training->current);
          }
          duration = 0;
          distance = 0;
          BPM = 0;
        } else {
          saveTBResult(current_training, '#');
          paintTrainingBlock2(current_training->current);
        }
        break;
      }
    }
  }
}

void enableScreen() {
  digitalWrite(6, HIGH);
  digitalWrite(10, LOW);
}

void enableSD() {
  digitalWrite(10, HIGH);
  digitalWrite(6, LOW);
}

void loadScreen() {
  epd.ClearFrameMemory(0xFF);
  epd.DisplayFrame();
  paint.SetWidth(200);
  paint.SetHeight(20);
  paint.Clear(1);
  paint.DrawStringAt(35, 4, "Cargando", &Font16, 0);
  epd.ClearFrameMemory(0xFF);
  epd.SetFrameMemory(paint.GetImage(), 30, 80, paint.GetWidth(), paint.GetHeight());
  paint.Clear(1);
  drawCenteredString(4, F("Entrenamientos"), &Font16, 0);
  epd.SetFrameMemory(paint.GetImage(), 5, 110, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}

void bluetoothScreen() {
  paint.SetWidth(200);
  paint.SetHeight(20);
  paint.Clear(1);
  paint.DrawStringAt(20, 4, "Sincronizando", &Font16, 0);
  epd.ClearFrameMemory(0xFF);
  epd.SetFrameMemory(paint.GetImage(), 10, 90, paint.GetWidth(), paint.GetHeight());
  paint.Clear(1);
  paint.DrawStringAt(20, 4, "Bluetooth", &Font16, 0);
  epd.SetFrameMemory(paint.GetImage(), 25, 110, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}

void trainingsOnScreen() {
  enableScreen();
  loadScreen();
  enableSD();

  Training tr[8];
  String aux;
  int cont = 0;
  char copy_id[24];
  char copy_date[29];

  //Read Trainings ID and DATE
  File trainings = SD.open(F("/t"));
  if (!trainings) {
    Serial.println(F("Error reading trainings for screen"));
    return;
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
  enableScreen();

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
    paint.Clear(i == 0 ? 0 : 1);
    //paint.DrawRectangle(0, position, 200, position, 1);
    if (strlen(tr[i]._id) != 0) {
      paint.DrawStringAt(0, 4, tr[i].date, &Font16, i == 0 ? 1 : 0);
    }
    epd.SetFrameMemory(paint.GetImage(), 0, position, paint.GetWidth(), paint.GetHeight());
    position += 20;
  }
  epd.DisplayFrame();


  while (1) {
    enableScreen();
    paint.SetWidth(200);
    paint.SetHeight(20);
    if (digitalRead(3) == 0) {
      current_training = readTraining(tr[select_training]._id);
      paintTrainingBlock(current_training->current);
      return;
    }
    if (digitalRead(2) == 0) {
      select_training++;
      position_aux += 20;
      if (strlen(tr[select_training]._id) == 0) {
        position_aux = 20;
        select_training = 0;
      }
      paint.Clear(0);
      paint.DrawStringAt(20, 4, "ENTRENAMIENTOS", &Font16, 1);
      epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
      position = 20;
      for (i = 0; i < 9; i++) {
        paint.Clear(1);
        if (strlen(tr[i]._id) != 0) {
          paint.DrawStringAt(0, 4, tr[i].date, &Font16, 0);
        }
        epd.SetFrameMemory(paint.GetImage(), 0, position, paint.GetWidth(), paint.GetHeight());
        position += 20;
      }
      paint.Clear(0);
      paint.DrawStringAt(0, 4, tr[select_training].date, &Font16, 1);
      epd.SetFrameMemory(paint.GetImage(), 0, position_aux, 200, 24);
      epd.DisplayFrame();
    }
    if (digitalRead(11) == 0) {
      return;
    }
  }
}

void paintEndTraining() {
  enableScreen();
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
  enableScreen();
  String hora = " ";
  char  buf[50];
  String stringAux =  String();

  while (1) {
    String hora = " ";


    /* paint.SetWidth(200);
      paint.SetHeight(200);
      paint.Clear(1);
      paint.DrawRectangle(0,0,0,0,1);
      epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
    */

    epd.ClearFrameMemory(0xFF);
    epd.SetFrameMemory(SMALL_IMAGE_DATA);
    paint.SetWidth(200);
    paint.SetHeight(24);
    paint.Clear(1);
    paint.DrawStringAt(30, 4, "27/04/2018", &Font16, 0);
    epd.SetFrameMemory(paint.GetImage(), 20, 80, paint.GetWidth(), paint.GetHeight());
    paint.Clear(1);
    t = now();
    stringAux =  String(hour(t));
    if (hour(t) < 10) {
      hora.concat(F("0"));
    }
    hora.concat(stringAux);
    hora.concat(":");
    stringAux =  String(minute(t));
    if (minute(t) < 10) {
      hora.concat(F("0"));
    }
    hora.concat(stringAux);
    hora.concat(":");
    stringAux =  String(second(t));
    if (second(t) < 10) {
      hora.concat(F("0"));
    }
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

    // BT
    if (Serial2.available())
    {
      if (Serial2.readString() == F("empezar"))
      {
        enableScreen();
        bluetoothScreen();
        Serial.println(F("Send results started"));
        sendResultsBT();
      }

      if (Serial2.readString() == F("trainings"))
      {
        enableScreen();
        bluetoothScreen();
        Serial.println(F("receiveTrainingsBT started"));
        receiveTrainingsBT();
      }
    }
  }

  return;
}

void drawCenteredString(int y, String s, _tFont *font, int colored) {
  int pos = s.length() * (font->Width);
  pos = (200 - pos) / 2;
  paint.DrawStringAt(pos, y, s.c_str(), font, colored);
}


void paintTrainingBlock(TrainingBlock* tb) {
  enableScreen();
  String s;
  for (int i = 0; i < 2; i++) {
    paint.SetWidth(200);
    paint.SetHeight(40);
    epd.ClearFrameMemory(0xFF);
    paint.Clear(0);
    drawCenteredString(10, String(current_training->type).substring(0, strlen(current_training->type) - 1), &Font20, 1);
    paint.DrawRectangle(0, 40, 200, 40, 0);
    epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
    if (i == 0) {
      paint.SetWidth(200);
      paint.SetHeight(30);
      paint.Clear(1);
      paint.DrawStringAt(60, 4, ((String)F("Iniciar")).c_str(), &Font16, 0);
      paint.DrawStringAt(35, 15, ((String)F("Entrenamiento")).c_str(), &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 50, paint.GetWidth(), paint.GetHeight());
    } else {
      s = String(distance / duration) + (String)F(" m/s");
      paint.Clear(1);
      drawCenteredString(4, s, &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 80, paint.GetWidth(), paint.GetHeight());
    }

    paint.SetWidth(200);
    paint.SetHeight(20);

    if (tb->distance != -1) {
      s = (String)F("Distancia: ") + String(tb->distance) + (String)F(" m");
      paint.Clear(1);
      paint.DrawStringAt(10, 4, s.c_str(), &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 100, paint.GetWidth(), paint.GetHeight());
    }
    if (tb->duration != -1) {
      s = (String)F("Duracion: ");
      if ((tb->duration / 60) < 1) {
        s += String(tb->duration) + (String)F(" seg");
      } else {
        s += String(tb->duration / 60) + (String)F(" min");
      }
      paint.Clear(1);
      paint.DrawStringAt(10, 4, s.c_str(), &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 120, paint.GetWidth(), paint.GetHeight());
    }
    epd.DisplayFrame();
    if (i == 0) {
      while (true) {
        if (digitalRead(3) == 0) {
          break;
        }
        if (digitalRead(11) == 0) {
          freeTraining(&current_training);
          trainingsOnScreen();
          break;
        }
      }
    }
  }

  return;
}

void paintTrainingBlock2(TrainingBlock* tb) {
  enableScreen();
  String s;
  for (int i = 0; i < 2; i++) {
    paint.SetWidth(200);
    paint.SetHeight(40);
    epd.ClearFrameMemory(0xFF);
    paint.Clear(0);
    drawCenteredString(10, String(current_training->type).substring(0, strlen(current_training->type) - 1), &Font20, 1);
    paint.DrawRectangle(0, 40, 200, 40, 0);
    epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());

    s = String(distance / duration) + (String)F(" m/s");
    paint.Clear(1);
    drawCenteredString(4, s, &Font16, 0);
    epd.SetFrameMemory(paint.GetImage(), 0, 80, paint.GetWidth(), paint.GetHeight());

    paint.SetWidth(200);
    paint.SetHeight(20);

    if (tb->distance != -1) {
      s = (String)F("Distancia: ") + String(tb->distance) + (String)F(" m");
      paint.Clear(1);
      paint.DrawStringAt(10, 4, s.c_str(), &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 100, paint.GetWidth(), paint.GetHeight());
    }
    if (tb->duration != -1) {
      s = (String)F("Duracion: ");
      if ((tb->duration / 60) < 1) {
        s += String(tb->duration) + (String)F(" seg");
      } else {
        s += String(tb->duration / 60) + (String)F(" min");
      }
      paint.Clear(1);
      paint.DrawStringAt(10, 4, s.c_str(), &Font16, 0);
      epd.SetFrameMemory(paint.GetImage(), 0, 120, paint.GetWidth(), paint.GetHeight());
    }
    epd.DisplayFrame();
  }

  return;
}

float axisAccel(char axis) {
  float a = adxl.AxisDigitalAccelerometerRead(5, axis);
  return acos(a) * 180 / (PI);
}

void saveTBResult(Training *training, char end)
{
  enableSD();
  boolean emptyFile = !SD.exists((String)F("/r/") + String(training->_id).substring(0, 7) + (String)F(".txt"));
  boolean samePoint = gps.location.lat() == old_latitude && old_longitude == gps.location.lng();
  File logFile = SD.open((String)F("/r/") + String(training->_id).substring(0, 7) + (String)F(".txt"), FILE_WRITE);

  if (!logFile) {
    Serial.println(F("Error saving data"));
    return;
  }

  if (emptyFile) {
    logFile.print(String(training->current->_id).substring(0, 24));
    logFile.println(F("º"));
  }

  if (old_latitude != 0 && old_longitude != 0 && !samePoint ) {
    distance += gps.distanceBetween(gps.location.lat(), gps.location.lng(), old_latitude, old_longitude);
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

  logFile.print(samePoint ? 0 : (distance / duration), 6);
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

  if (end == '$') {
    logFile.print(String(training->current->_id).substring(0, 24));
    logFile.println(F("º"));
  }

  logFile.close();
}

void  sendResultsBT() {
  enableSD();
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
      String file_name = logFile.name();
      logFile.close();
      SD.remove((String)F("/r/") + file_name);
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
  enableSD();
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
  enableSD();

  File dataFile = SD.open((String)F("/t/") + fileName + (String)(".txt"), FILE_READ);

  if (!dataFile || !dataFile.available()) {
    Serial.println((String)F("Error opening Training") + fileName);
    return NULL;
  }

  Training *t = (Training*)malloc(sizeof (Training));
  TrainingBlock *tb;

  t->trainingBlocks = ((TrainingBlock*)malloc(sizeof (TrainingBlock)));
  t->current = t->trainingBlocks;

  tb = t->trainingBlocks;

  if (dataFile.available()) {
    dataFile.readStringUntil('\n').toCharArray(t->_id, 25);
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

    msg.toCharArray(tb->_id, 25);
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

void freeTraining(Training **t) {
  if (t == NULL) {
    return;
  }
  TrainingBlock *tb = (*t)->trainingBlocks;
  while (tb != NULL) {
    TrainingBlock *previous = tb;
    tb = tb->next;
    free(previous);
    previous = NULL;
  }
  free(*t);
  *t = NULL;
}

