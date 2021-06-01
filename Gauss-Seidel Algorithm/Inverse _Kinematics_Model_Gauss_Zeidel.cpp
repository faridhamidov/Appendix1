#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

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


int main(){
//  This the MAIN Program for Calculating the Inverse Kinematocs 
//  of a Two-Link Robootic Manipulator with given structure  
//  The Target Location [Xtarget, Ytarget] is given and the 
//  Program finds the respective Rotation Angles: Theta1 and Theta2 
//  by Using the GAUSS-ZEIDEL OPtimizatio Agloritm. 
//     ***  May, 2021 - Baku 

// ---------------------  The Program -----------------------------------
    int List[2]; 
    int ITmax, Iter, Irt, Nrt, Nsuccess, Count;
    int Ipar, Npar, Move, Nmove, Nepoch, Index, Direction, Lpar;
	
    float Umin, Umax, R, Yo, Ynew, Yopt;
    float XXmin[10],XXmax[10], Lstep[10];
    float XX[10], XXo[10]; 
	float XXnew[10], XXopt[10]; 
	float Xinit, Yinit, Xtarget, Ytarget, Xend, Yend, Xpred, Ypred;
	float Link1, Link2, Theta1, Theta2, Angle1, Angle2;
	float Crit, Crit_init, Crit_new, Sum2, Dist;
// ----------	
    
    float Error, SumErr, RMSE, RMSEo, RMSEnew, Rbuf, RMSEopt;
    float XXinp[3], Ymod;

    char symbol;
//  char Name1[30], Name2[30], Name3[30];
//  FILE *fp1, *fp2, *fp3;

  printf("\n    ***   INVERSE_KINEMATICS MODEL_Gauss_Zeidel   *** \n");
  printf("    === This Program uses GAUSS_ZEIDEL Optimization Algorithm \n");
  printf("      to calculate the Rotaion Angles: Theta1  and  Theta2 \n");
  printf("      of the Two Links of the Robotic Maniulator, so that \n");
  printf("      it reaches the predefined Location [Xo, Yo]  \n\n");
  printf("    --- The Program works with a given Model for the Forward \n");
  printf("        Kinematics of the Concrete Two-Link Robotic Manipulator \n");
  printf("    ----------------------------------------------------------- \n"); 
// ----------------------------------------------------------------------------- 
    srand (time(0)); 
    
// Length of the links/joints
    Link1 = 12.0;
    Link2 = 13.5;    

// ======== GAUSS_ZEIDEL OPTIMIZATION ALGORITHM =========
    
	printf("\n ==== GAUSS_ZEIDEL_OPTIMIZATION ALGORITHM ==== \n"); 
    printf(" Two Parameters for Optimization are considered: \n");
	printf(" Theta1 (degrees)  and  Theta2 (degrees) \n"); 
	

    printf("\n  # Enter the Target Coordinate: Xtarget Ytarget --> ");
       scanf("%f %f", &Xtarget, &Ytarget);
       printf("    Xtarget = %6.2f;   Ytarget = %6.2f \n", Xtarget, Ytarget);
 
 // --- The number of the Parameters to optimize is: Npar = 2       		
	Npar = 2;
// --- Randomizing the Order of the Dimensions (Parameters)
//    printf("  -- Randomization of all %3d Dimensions: \n", Npar);      
    RandomList(1, Npar, List);
    /********************************
    for(Ipar=0; Ipar<Npar; Ipar++) { 
    	printf(" %2d ", List[Ipar]);
	}
	printf(" \n");    
	********************************/
    	  
    printf("\n --- ENTER the RANGES [Min Max] for the Two Angles (Degrees) --- \n");
		printf("    - Range of Angle Theta1 (degrees): ");
		scanf("%f %f", &Umin, &Umax);
		XXmin[0] = Umin;
		XXmax[0] = Umax;
		
		printf("    = Range of Angle Theta2 (degrees): ");
		scanf("%f %f", &Umin, &Umax);  
		XXmin[1] = Umin;
		XXmax[1] = Umax;


//************************************************/

	printf("\n * Enter the Step (Degrees) for the Search: Lstep = ");
	scanf("%f", &Rbuf);  
	printf("   Lstep = %5.3f \n", Rbuf);	   
// --- This step is equal for All Paramegters (All SIngletons)	
       	Lstep[0] = Rbuf; 
		Lstep[1] = Rbuf;

// ------- Defining the INITIAL POINT for OPTIMIZATION: 		 
  //  printf("\n --- Random INITIAL Angles Theta1 and Theta2: ");
       for(Ipar=0; Ipar<Npar; Ipar++) {
       	   R = (float)rand()/RAND_MAX; // This is a Random number between: 0.0   and 1.0
           XXo[Ipar] = XXmin[Ipar] + (XXmax[Ipar] - XXmin[Ipar])*R; 
	//	 printf(" %6.2f ", XXo[Ipar]);    	 
	   }
    //     printf(" \n");
// ---------------
// ----- Calculaing of the Criterion: The Distance between the points:
//       [Xned, Yend] and [Xo, Yo]
        Angle1 = XXo[0];
		Angle2 = XXo[1]; 
        Forward(Link1, Link2, Angle1, Angle2, Xinit, Yinit); 

        printf("\n  RANDOM Initial Angles: Theta1 = %6.2f and Theta2 = %6.2f  \n", Angle1, Angle2);
        printf("  Respective Initial Location: Xinit = %6.2f and Yinit = %6.2f  \n", Xinit, Yinit);
// ----Calculating the Initial Criterion: Crit_init
        
        Sum2 = (Xtarget-Xinit)*(Xtarget-Xinit) +  (Ytarget-Yinit)*(Ytarget-Yinit);
        Crit_init = sqrt(Sum2);
        Yo = Crit_init;
        printf("  - Initial Criterion: Crit_init = %7.4f \n\n", Crit_init);
      
//  ----- END of Calcuating the Initial RMSE for the random generated Singletons

// ===== The Essential Part of the GAUSS-ZEIDEL ALGORITHM  
// Initializations:
// -- The current Optimal Coordinate is the Initial Point
   for(Ipar=0; Ipar<Npar; Ipar++) {
   	   XXopt[Ipar] = XXo[Ipar];
   }
   Yopt = Yo; // The current Optimmum Yopt is equal to the Initial Value Yo  

   Iter = 0;
   Nepoch = 0;
   
   do {
   	Nepoch = Nepoch + 1;
   	Nmove = 0;
   	printf(" --- Epoch = %2d --- \n", Nepoch);
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
//   	The local Optimim for this direction is found:
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
        
        printf(" -- Lpar = %2d;  Iter = %4d;  Yopt = %6.3f; Move = %2d \n", Lpar, Iter, Yopt, Move);
        
// ----------------------   	
	    if(Move == 1) {
       	 Nmove = Nmove + 1;
	   }    
       
// --- to the search of the Local Optimum for the Next Direction:   		
	}
	
    printf("    Nmove = %2d ! \n", Nmove); 
		
// ===== END of the BIG DO WHILE Loop  
} while (Nmove > 0);


    printf("\n ------ The OPTIMAL RESULTS ------ \n");
    printf(" TARGET: Xtarget = %6.2f cm;  Ytarget = %6.2f cm \n", Xtarget, Ytarget);
    printf("\n ROTATION ANGLES: Theta1 and Theta2 (Degrees) \n");
    printf(" Theta1 = %6.2f Degrees and Theta2 = %6.2f Degrees \n", 
	        XXopt[0], XXopt[1]);
    printf("\n The Best Criterion (MINIMAL Distance) is: Yopt = %7.3f \n", Yopt);
    Angle1 = XXopt[0]; 
	Angle2 = XXopt[1];
    Forward(Link1, Link2, Angle1, Angle2, Xpred, Ypred);
    printf(" Predicted Arrival Location: Xpred = %6.2f cm;  Ypred = %6.2f cm \n", 
	         Xpred, Ypred);
// --------- END of tthe Algorithm --------    
    printf("\n ** Enter any Symbol to Exit: --> ");
	scanf("%s",&symbol);
// --------   END of the Program: INVERSE_KINEMATICS_MODEL_GAUSS_ZEIDEL -----
}

