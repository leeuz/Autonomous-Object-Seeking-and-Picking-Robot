// pin defs
#define TRIG1 2
#define ECHO1 3

#define TRIG2 4
#define ECHO2 5

#define TRIG3 6
#define ECHO3 7

// sensor geometry 
const float spacing = 3.0;     // cm between sensors
const float angle1 = -40.0;    // S1 points 40° left
const float angle2 = 0.0;      // S2 straight
const float angle3 = 40.0;     // S3 points 40° right

// convert degrees→ radians
float rad(float deg) { return deg * 3.14159265 / 180.0; }


// setup
void setup() {
  Serial.begin(9600);

  pinMode(TRIG1, OUTPUT);  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);  pinMode(ECHO3, INPUT);
}


// distance Reading Function
float readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);  // timeout 30ms
  float distance = duration * 0.034 / 2;       // convert to cm
  return distance;
}


// convert polar reading (distance + angle)→ X/Y point
void sensorToXY(float distance, float angleDeg, float sensorX, float* outX, float* outY)
{
  float a = rad(angleDeg);
  *outX = sensorX + distance * sin(a);
  *outY = distance * cos(a);
}


// MAIN LOOP
void loop() {

  // all 3 sensors
  float d1 = readUltrasonic(TRIG1, ECHO1);
  float d2 = readUltrasonic(TRIG2, ECHO2);
  float d3 = readUltrasonic(TRIG3, ECHO3);

  // sensor X positions
  float x1 = -spacing;  // left sensor
  float x2 = 0;         // center sensor
  float x3 = spacing;   // right sensor

  // convert to XY coordinates
  float x_1, y_1;
  float x_2, y_2;
  float x_3, y_3;

  sensorToXY(d1, angle1, x1, &x_1, &y_1);
  sensorToXY(d2, angle2, x2, &x_2, &y_2);
  sensorToXY(d3, angle3, x3, &x_3, &y_3);

  // avg the estimates
  float X = (x_1 + x_2 + x_3) / 3.0;
  float Y = (y_1 + y_2 + y_3) / 3.0;


  // output
   Serial.print("S1 = "); Serial.print(d1);    Serial.print(" cm   ");
  Serial.print("S2 = "); Serial.print(d2); Serial.print(" cm   ");
  Serial.print("S3 = "); Serial.print(d3); Serial.println(" cm");

  Serial.print("OBJECT Position: X = ");
  Serial.print(X); 
  Serial.print(" cm,   Y = ");
  Serial.print(Y);
  Serial.println(" cm\n");

  delay(200);
}
