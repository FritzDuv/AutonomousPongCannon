#include <Servo.h>

Servo mainBodyServo;
Servo ESC;
char inputBuffer[16];
unsigned int val; 

const unsigned long servointerval = 500;
const unsigned long lidarinterval = 500;
unsigned long currentMillis = 0;
unsigned long prevservo = 0;
int lastState = HIGH;
int currentState;
bool cont = false;

void setup() {
     Serial.begin(9600);
     // put your setup code here, to run once:
     mainBodyServo.attach(9);
     Serial1.begin(115200);
     ESC.attach(10,544,2400); //Connect ESC
     delay(1);
     ESC.write(0);
     delay(5000); //Allow ESC to arm
     ESC.write(1500);
     ESC.write(1000);
     Serial.println("Starting");
     pinMode(11, INPUT_PULLUP);
}

int getdist() {
  bool esc = false;
  int dist;
  int uart[9];
  int j;
  int check;
  const int HEADER=0x59;
  while(!esc) {
    if (Serial1.available() >= 9){
      if (Serial1.read() == HEADER) {
        uart[0] = HEADER;
        if (Serial1.read() == HEADER) {
          uart[1] = HEADER;
          for (j = 2; j < 9; j++) {
            uart[j] = Serial1.read();
          }
          byte checksum = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] == checksum){
            dist = uart[2] + uart[3] * 256;
            //Serial.println(dist);
            return dist;
          }
        }
      }
    }
  }
}

void tableSweep(int arr[],int n, double range, int center, int filter) {//Swivels servo while sampling LUNA Lidar to create 2D scan of solo cups on table. n = len(arr), range = angle both sides of 90 degrees for the sweep
  //filter to ignore measured values above filter, temporary solution to sensing lag problem on second sweep
  double pwmperang = 11.1111111; //PWM change corresponding to 1 angle (deg).
  int servomin = round(center - pwmperang*range);
  int servomax = round(center + pwmperang*range);
  int inc = round(2*pwmperang*range/(n-1)); //This needs to organically be a whole number to not lose information
  long prevscantime = 5;//This offset is the time it takes the servo to turn to the next angle in the sweep, calculated using .12sec/60deg at 6V.
  //Look at increasing prevscantime here^
  //Serial.println("move wait time");
  //Serial.println(prevscantime);
  bool cont = true;
  int currpos = servomin;
  int index = 0;
  mainBodyServo.writeMicroseconds(currpos);
  delay(prevscantime);
  while(cont) {
    unsigned long current = millis();
    int currentdist = getdist();
    if ( current - prevservo >= servointerval ){
      prevservo = current;
      if ( currpos > servomax ) {
        cont = false;
      }
      else {
        //Serial.println("currpos");
        //Serial.println(currpos);
        currpos += inc;
        mainBodyServo.writeMicroseconds(currpos);
        if ( current - prevscantime >= lidarinterval ) {
          prevscantime = current;
          //Serial.println("scandist/newmeasurement");
          Serial.println(currpos);
          Serial.println(currentdist);
          arr[index] = currentdist;
          //Serial.println(getdist());
          //Serial.println("verify");
          //Serial.println(arr[index]);
          //Serial.println("index");
          //Serial.println(index);
          index++;
        }
      }
    }
  }
  Serial.println("Done with sweep!");
}

void nLowestAngles(int angs[], int lowdists[], int dists[], int angsize, int distsize){ //Iterates through Array of Distance measurements n = len(angs) times, returning the angles. 
  for(int k = 0; k < angsize; ++k) { //Defaults the lowest distances to the first angsize ones of dists because C is garbage and doesn't support slicing by default 
    lowdists[k] = dists[k];
    angs[k] = k;
  }
  for(int i = angsize-1; i < distsize; ++i ) {
    int replaceMax = 0;
    int highdex = -1;
    int highdist = -10000;
    for(int j = 0; j < angsize; ++j) { //Simultaneously finds the max of the current lowest angsize angles and determines if the max should be substituted
      if (dists[i] < lowdists[j]) {
          replaceMax = 1;
      }
      if (lowdists[j] > highdist) {
          highdist = lowdists[j];
          highdex = j;
      }
    }
    if (replaceMax == 1) {
        angs[highdex] = i;
        lowdists[highdex] = dists[i];
    }
  }
}

int distToPWM(int dist, double vpwm, double h, double k){
  return (sqrt(dist - k) + h)/vpwm;
}
void filterAimShoot(int angs[], int lowdists[], int angsize, int len, double range){ //Uses results of initial table sweep to estimate the closest point on the cup. Then performs a second sweep based around this point and uses that sweep to fit the circle
  double pwmperang = 11.1111111;
  int servomin = round(1500-pwmperang*range);
  int inc = round(2*pwmperang*range/(len-1));
  //first approximate the closest point on the cup by averaging out the smallest points recorded in the sweep
  int smallangles[angsize] = {};
  int lowval = 10000; //Arbitrarily large
  int repeatcount = 0;
  for( int i = 0; i < angsize; ++i ){
    if (lowdists[i] < lowval && angs[i] != 0) {
      lowval = lowdists[i];
      repeatcount = 0;
      smallangles[repeatcount] = angs[i]; 
    }
    else if (lowdists[i] == lowval){
      repeatcount++;
      smallangles[repeatcount] = angs[i];
    }
  }
  double closeang = 0; //variable containing the approximated angle of the closest point
  for( int j = 0; j < repeatcount + 1; ++j ){
    closeang += smallangles[j];       
  }
  float floatlowdist = float(lowval);
  float cupradcm = 3.5;
  closeang = closeang/(repeatcount+1); //average index value
  //double singlecupangle = atan((cupradcm/floatlowdist))*(180/PI); //Defines the range of the second sweep about the approximated closest point on the cup (deg)
  int pwmcloseang = round(closeang*inc + servomin); //convert closeang to pwm value for tablesweep
  mainBodyServo.writeMicroseconds(pwmcloseang);
  delay(1000);
  int shootpwm = distToPWM(lowval+cupradcm+4,9.73480626e-03,-6.02763336e+00,-3.01438917e+02);     
  ESC.writeMicroseconds(shootpwm);
  delay(1000);
  ESC.writeMicroseconds(1000);
  mainBodyServo.writeMicroseconds(round(1500-range*pwmperang));
}
void loop() {
  // put your main code here, to run repeatedly:
  double pwmperang = 11.1111111;
  int points = 26;
  int angsize = 5;
  int sweep[points] = {};
  double angle = 9;
  int finalangsize = 3;
  mainBodyServo.writeMicroseconds(1500-pwmperang*angle);
  delay(1000);
  tableSweep(sweep,points,angle,1500,300); //sweeps the table recording distances at different angles
  int angs[angsize];
  int lowdists[angsize];
  nLowestAngles(angs, lowdists, sweep, angsize, points); //finds the n angles with lowest corresponding distances from the sweep
  filterAimShoot(angs, lowdists, angsize, points, angle); //aims for closest point and shoots, no filter yet :/
  while (cont == false){
    currentState = digitalRead(11);
    if(lastState == LOW && currentState == HIGH)
      cont = true;
     lastState = currentState;
  }
  cont = false;
}
