#include <Servo.h>
Servo mainBodyServo;

const unsigned long servointerval = 1000;
const unsigned long lidarinterval = 1000;
unsigned long currentMillis = 0;
unsigned long prevservo = 0;

void setup() {
     // put your setup code here, to run once:
     Serial.begin(9600);
     while(!Serial);
     mainBodyServo.attach(9);
     Serial1.begin(115200);
     Serial.println("Starting");
}

bool in_array(const int store[], const int storeSize, const int query) {
   for (size_t i=0; i<storeSize; ++i) {
      if (store[i] == query) {
         return true;
      }
   }
   return false;
}

int getdist() { //Reads in dsitance data over Hardware Serial communication from TFLuna Lidar module used for measuring distance.
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
  int inc = round(2*pwmperang*range/(n-1)); //This needs to be a whole number to not lose information
  Serial.println("min/max/inc");
  Serial.println(servomin);
  Serial.println(servomax);
  Serial.println(inc);
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
    //Serial.println(currentdist);
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
        Serial.println(currpos);
        Serial.println(currentdist);
        currpos += inc;
        mainBodyServo.writeMicroseconds(currpos);
        if ( current - prevscantime >= lidarinterval ) {
          prevscantime = current;
          //Serial.println("scandist/newmeasurement");
          //Serial.println(currentdist);
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

void secondSweep(int angs[], int lowdists[], int angsize, int finalfitangs[], int finalfitdists[], int finalangsize, int len, double range){ //Uses results of initial table sweep to estimate the closest point on the cup. Then performs a second sweep based around this point and uses that sweep to fit the circle
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
  Serial.println("Small Angles");
  for( int i = 0; i < angsize; ++i){
    Serial.println(smallangles[i]);
  }
  double closeang = 0; //variable containing the approximated angle of the closest point
  for( int j = 0; j < repeatcount + 1; ++j ){
    closeang += smallangles[j];       
  }
  Serial.println("lowval");
  Serial.println(lowval);
  float floatlowdist = float(lowval);
  float cupradcm = 2;
  closeang = closeang/(repeatcount+1); //average index value
  double singlecupangle = atan((cupradcm/floatlowdist))*(180/PI); //Defines the range of the second sweep about the approximated closest point on the cup (deg)
  Serial.println("closeang");
  Serial.println(closeang);
  Serial.println("sweep2ang");
  Serial.println(180/PI);
  Serial.println(singlecupangle,6);
  Serial.println("closest ang");
  Serial.println(closeang*inc + servomin);
  int pwmcloseang = round(closeang*inc + servomin); //convert closeang to pwm value for tablesweep
  mainBodyServo.writeMicroseconds(pwmcloseang);
  delay(10000); 
  tableSweep(finalfitdists,finalangsize,singlecupangle,pwmcloseang,250); //Collects distances to be used in circle fitting
  int k = 0;
  for( int pwmang = round(closeang - pwmperang*singlecupangle); pwmang <= round(closeang + pwmperang*singlecupangle); pwmang += round(2*pwmperang*singlecupangle) ){ //Fill in the pwm values in finalfitangs
    finalfitangs[k] = pwmang;
    k++;
  }  
  Serial.println("Lowest Angle/Dist Pairs 2: ");
  for( int i = 0; i < finalangsize; ++i){
    Serial.println(i);
    Serial.println(finalfitdists[i]);
  }
}

void convertToCartesian(int angs[], int lowdists[], int angsize, double xs[], double ys[], int len, double range, bool pwminput) { //little bit of trig here, len is the number of samples of the entire sweep
  double pwmperang = 11.1111111;
  int servomin = round(1500-pwmperang*range);
  int inc = round(2*pwmperang*range/(len-1));
  double angle = 0;
  Serial.println("finalfitangles");
  for(int i = 0; i < angsize; ++i ) { 
    if (!pwminput){
      double angle = (servomin + angs[i]*inc)/pwmperang - 45.0;
    }
    else{
      double angle = angs[i]/pwmperang - 45.0;
    }
    Serial.println(int(round((angle*pwmperang)))+500);
    //mainBodyServo.writeMicroseconds(int(round((angle*pwmperang)))+500);
    //delay(10000);
    xs[i] = lowdists[i]*cos(angle*PI/180);
    ys[i] = lowdists[i]*sin(angle*PI/180);
  }
}
 

void loop() {
  // put your main code here, to run repeatedly:
  double pwmperang = 11.1111111;
  int points = 26; // Number of points in initial sweep
  int angsize = 10; // lowest distance points from the sweep, soon to be optimized for cup diam
  int sweep[points] = {};
  double angle = 9; // half of the total angle swept from center position of robot
  int finalangsize = 3; //Number of points used in optimized circle fit
  tableSweep(sweep,points,angle,1500,300);
  Serial.println("Sweep Results: ");
  for( int i = 0; i < sizeof(sweep)/sizeof(sweep[0]); ++i){
    Serial.println(sweep[i]);
  }

  int angs[angsize];
  int lowdists[angsize];
  nLowestAngles(angs, lowdists, sweep, angsize, points);
  Serial.println("Lowest Angle/Dist Pairs: ");
  for( int i = 0; i < angsize; ++i){
    Serial.println(angs[i]);
    Serial.println(lowdists[i]);
  }
  double xs[finalangsize]; //Contain the cartesian points used in circle fit.
  double ys[finalangsize];

  int finalfitangs[finalangsize] = {}; //Contains the pwm angles of the points used in the second sweep
  int finalfitdists[finalangsize] = {}; //Contains the distance values of the points used in the second sweep

  secondSweep(angs, lowdists, angsize, finalfitangs, finalfitdists, finalangsize, points, angle);
  Serial.println("Second Sweep Angle/Dist Pairs: ");
  for( int i = 0; i < finalangsize; ++i){
    Serial.println(finalfitangs[i]);
    Serial.println(finalfitdists[i]);
  }
  
  convertToCartesian(finalfitangs, finalfitdists, finalangsize, xs, ys, points, angle,true);
  
  for( int i = 0; i < angsize; ++i){
    Serial.println("Cartesian Pair");
    Serial.println(xs[i]);
    Serial.println(ys[i]);
  }
  //Now fits circle using Taubin's algorithm and points roboto towards its center
  Data testdata = Data(angsize,xs,ys);
  Circle testcirc = CircleFitByTaubin(testdata);
  double a = testcirc.a;
  double b = testcirc.b;
  double dist = sqrt(pow(a,2)+pow(b,2));
  double r = testcirc.r;
  double theta = atan2(b,a); //basic trig
  //Now all that remains is to convert our angle into a PWM signal and point at it
  int pwmtheta = round((theta*180*pwmperang/PI) + 500);
  Serial.println("dist, theta, pwmtheta, radius, a, b");
  Serial.println(dist);
  Serial.println(theta);
  Serial.println(pwmtheta);
  Serial.println(r);
  Serial.println(a);
  Serial.println(b);
  //mainBodyServo.writeMicroseconds(pwmtheta);
  mainBodyServo.writeMicroseconds(1500-pwmperang*angle);
  //mainBodyServo.detach();
  while( true == true ) {
    delay(1);
  }
}
