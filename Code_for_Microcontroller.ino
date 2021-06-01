#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Servo.h>

Servo Servo_1; //creating an object in class Servo to control servo motor 1
Servo Servo_2; //creating an object in class Servo to control servo motor 2

//Global variables: To be accessed from all functions
  int List[2]; 
  int ITmax, Iter, Irt, Nrt, Nsuccess, Count;
  int Ipar, Npar, Move, Nmove, Nepoch, Index, Direction, Lpar;
  float Umin, Umax, R, Yo, Ynew, Yopt;
  float Ytemp,Xtemp;
  float XXmin[10],XXmax[10], Lstep[10];
  float XX[10], XXo[10]; 
  float XXnew[10], XXopt[10]; 
  float Xinit, Yinit, Xtarget, Ytarget, Xend, Yend, Xpred, Ypred;
  float Link1, Link2, Theta1, Theta2, Angle1, Angle2;
  float Crit, Crit_init, Crit_new, Sum2, Dist;
  float XXinp[3], Ymod;
  float Rbuf;
  char symbol;


//Function to calculate end positions using forward kinematics:
void Forward(float Link1, float Link2, float Angle1, float Angle2, float &Xend, float &Yend) {

// --- This function 

//   Local Definitions   
     float Theta1, Theta2, X1, Y1, X2, Y2;
//  ----- Calculating the Coordinate of the End-Effector of the Robot Manipulator
        Theta1 = Angle1/57.29579; // Theta1 is in Radians !
        Theta2 = Angle2/57.29579; // Theta2 is in Radians ! 
 //  printf("    Angle1 = %6.2f Degrees;   Angle2 = %6.2f Degrees \n", Angle1, Angle2);
          //  --- Forward Kinematics Calculation:
        // ---- ATTENTION: this is a Concrete Structure of the Manipulator
        // ---- It can be chaged by the User !!
        
            X1 = Link1*cos(Theta1); // X Coordinate of first joint point
            Y1 = Link1*sin(Theta1); // Y Coordinate of first joint point
        //  ------------------------
            X2 = Link2*cos(Theta2); // X Coordinate of the Second Joint (from Origin) 
            Y2 = Link2*sin(Theta2); // Y Coordinate of the second Joint (from Origin)
        //  ------------------------ 
            Xend = X1 + X2;  // X-coordinate of the END EFFECTOR
            Yend = Y1 + Y2;  // Y-coordinate of the END EFFECTOR

            //to access the reached positison from another function they are assigned to global variables.
            Xtemp=Xend;
            Ytemp=Yend;
  //    ----------------------------------------------    
  
//    printf("    Xend = %6.2f cm  and Yend = %6.2f cm \n", Xend, Yend);  
// ---- END of the Function FORWARD Kinematics            
}

void IntRandom(int lower, int upper, int &num) 
  {
     num = (rand() % (upper - lower + 1)) + lower;  
  } 


void RandomList(int lower, int upper, int List[3375]) { 
       int i, num, count, index, key;
       count = upper - lower + 1;
      index = 0;
    num = (rand() % (upper - lower + 1)) + lower;
    List[0] = num;
    
    for(index = 1; index < count; index++){
// --- Generate one new Integer number "num":     
     num = (rand() % (upper - lower + 1)) + lower;  
      
      key = 1; 
      while(key == 1) {       
// ---- Check if there is repetition or not:  
         for(i=0; i<index; i++) {
            
              if(List[i] == num) {
                 key = 1;    
                 break; 
          } else {
            key = 0;  
          } 
    // -- to the next generation of the new integer number "num"  
         }  
                    
         if(key == 0) {
   // ---- This is a NEW integer number: save it in List:       
           List[index] = num;
       }  else {
  // --- There exist repetition in the numbrers: 
  // --- Generate another random number "num"     
            num = (rand() % (upper - lower + 1)) + lower;       
       }     
// ---- END of WHILE 
        }
// -- To the generation of the next random number "index" in the List:     
   } 
// --- End of the Function   
}


//Here (into setup) goes the code to be run by Arduino only ONCE!
void setup() {
  // put your setup code here, to run once:
  Servo_2.attach(10);  // attaches the servo on pin 10 to the servo object 
  Servo_1.attach(7);   // attaches the servo on pin 7 to the servo object
  
  Serial.begin(9600);
  srand (time(0)); 
//----------------------------------------
  //Configuration for the robotic manipulator:
  Link1 = 12.0;  //Length Lower Arm
  Link2 = 13.5;  //Length Upper Arm
  
  //Range of Rotation angles:
  //For Link1 
    //Umin=0;
    //Umax=150;
    XXmin[0] = 45.0;
    XXmax[0] = 135.;
    
  //For Link2 
   // Umax=165;
    XXmin[1] = 0.0;
    XXmax[1] = 90.0;
//----------------------------------------
    Npar = 2;
    

    Lstep[0] = 0.5;
    Lstep[1] = 0.5;
    
   
  //scanf("%f %f", &Xtarget, &Ytarget);
  //printf("    Xtarget = %6.2f;   Ytarget = %6.2f \n", Xtarget, Ytarget);

}

// Main code goes here, to run repeatedly:
void loop() {
  
   Serial.println();
   Serial.println("Enter X.");  
   while (Serial.available() == 0){}
   Xtarget=Serial.parseFloat(); 
   Serial.print("Targeted X: ");
   Serial.println(Xtarget);
        
   Serial.println("Enter Y."); 
   while (Serial.available() == 0) {}
   Ytarget=Serial.parseFloat();
   Serial.print("Targeted Y: ");
   Serial.println(Ytarget);
   Serial.println();
   Serial.println("Calculating the rotation angles...");
   Serial.println();
   RandomList(1, Npar, List);
   
   for(Ipar=0; Ipar<Npar; Ipar++) {
           R = (float)rand()/RAND_MAX; // This is a Random number between: 0.0   and 1.0
           XXo[Ipar] = XXmin[Ipar] + (XXmax[Ipar] - XXmin[Ipar])*R; 
  //   printf(" %6.2f ", XXo[Ipar]);       
     }


     Angle1 = XXo[0];
     Angle2 = XXo[1]; 
     Forward(Link1, Link2, Angle1, Angle2, Xinit, Yinit);

     Sum2 = (Xtarget-Xinit)*(Xtarget-Xinit) +  (Ytarget-Yinit)*(Ytarget-Yinit);
     Crit_init = sqrt(Sum2);
     Yo = Crit_init;

for(Ipar=0; Ipar<Npar; Ipar++) {
        XXopt[Ipar] = XXo[Ipar];
   }
   Yopt = Yo; // The current Optimmum Yopt is equal to the Initial Value Yo  

   Iter = 0;
   Nepoch = 0;
   
   do {
    Nepoch = Nepoch + 1;
    Nmove = 0;
 //   printf(" --- Epoch = %2d --- \n", Nepoch);
    for(Direction = 0; Direction < Npar; Direction++) {
// ---- Finding the Local Optimum for this Direction:       
      
    for(Ipar=0; Ipar<Npar; Ipar++) {
                XXnew[Ipar] = XXopt[Ipar];  
        }     
        
        
      Lpar = List[Direction] - 1;     
      XXnew[Lpar] = XXmin[Lpar];
      
      Move = 0;
      while (XXnew[Lpar] <= XXmax[Lpar]) {
// ---- Calculaing the Criterio for this Point in the space:          
        Angle1 = XXnew[0];
    Angle2 = XXnew[1]; 
        Forward(Link1, Link2, Angle1, Angle2, Xinit, Yinit);    
        Sum2 = (Xtarget-Xinit)*(Xtarget-Xinit) +  (Ytarget-Yinit)*(Ytarget-Yinit);
        Crit_new = sqrt(Sum2);
 // -- Calculating the New Criterion        
        Ynew = Crit_new;  
      Iter = Iter + 1;
//    The local Optimim for this direction is found:
//      Check it with the previous Optimimum:
           if(Ynew < Yopt)  {
// --- A New BETTER location has been found pdate the value of the Optimum:
              Move = 1;
//     Update the value of the Optimum:  
            Yopt = Ynew;
// --- Update the current Optimal Coordinates for this Optimum        
            for(Ipar=0; Ipar<Npar; Ipar++) {
                XXopt[Ipar] = XXnew[Ipar];  
              }     
           } 
 
// ----- Continue the Scanning Process for this Direction       
           XXnew[Lpar] = XXnew[Lpar] + Lstep[Lpar];
// --- To the Next turn of the SCANNING Loop WHILE:      
        }       
        
   //     printf(" -- Lpar = %2d;  Iter = %4d;  Yopt = %6.3f; Move = %2d \n", Lpar, Iter, Yopt, Move);
        
// ----------------------     
      if(Move == 1) {
         Nmove = Nmove + 1;
     }    
       
// --- to the search of the Local Optimum for the Next Direction:       
  }
  
//    printf("    Nmove = %2d ! \n", Nmove); 
    
// ===== END of the BIG DO WHILE Loop  
} while (Nmove > 0);

        Angle1 = XXopt[0]; 
        Angle2 = XXopt[1];
        Forward(Link1, Link2, Angle1, Angle2, Xpred, Ypred);

        
        //Assigning the angles to servo motors. Note: The values in teh brackets are the resukt of the calibration.
        Servo_2.write(abs(Angle1+10-180));
        Servo_1.write(abs(Angle2+10+135-Angle1));


        //Printing the results: ---------------------
        Serial.print("Reached X: ");
        Serial.println(Xtemp);
        Serial.print("Reached Y: ");
        Serial.println(Ytemp); 
        Serial.print("Theta1: ");
        Serial.println(Angle1);
        Serial.print("Theta2: ");
        Serial.println(Angle2);
        //------------------------------------------
        
        delay(1000); //delay of 1 second

}
