//These are the algorithms for resp and HRV
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


//RLS initialization
	
static double lambda=.999999;
float w_n=0;
float P_n=10;
float pi_n=10;
float k_n=10;
float e_n=10;


//.Initialize Kalman Filter Values


int kcount=0;

int kcount2=0;

typedef struct Kalm{
	float Xo;
	float Po;
	float H;
	float Q;
  float R;	
	float A;
  float Xn;
	float K;
	float Pn;	
}Kalm_Filt;

Kalm_Filt kalman={60,90,1,9,41,1,60,1,1};
Kalm_Filt kalman2={800,600,1,60,1100,1,200,200,1};
Kalm_Filt kalman3={800,600,1,60,1100,1,200,200,1};
Kalm_Filt kalman4={10,20,1,2,31,1,10,1,1};

//Peak Detection
typedef struct peak_detect{
	signed int max;
	signed int min;
	signed int maxmax;
	signed int minmin;	
	bool find_max;
}PEAK;
PEAK peak={-100,100,0,0,true};
PEAK peak2={-100,100,0,0,true};
PEAK peak3={-100,100,0,0,true};
PEAK peak4={-100,100,0,0,true};

int deltad=800;
int deltad2=100;
int deltad3=100;
int deltad4=100;

//Function Initialozation
signed int Kalman(signed int LED_val,signed int LED_val1,int order);
signed int Kalman2(signed int LED_val,signed int LED_val1,int order);
signed int Peak_Detect(signed int LED_val,int min_max);
signed int Peak_Detect2(signed int LED_val,int min_max);
signed int Peak_Detect3(signed int LED_val,int min_max);
signed int Peak_Detect4(signed int LED_val,int min_max);
signed char  physical_activity_count(signed char x,signed char  y,signed char  z);
signed int RLS(signed int LED_val,signed char accel);


//H is observation variable, A is state variable,	Q is covarience for state noise, R is observation noise, x_n is state and Posterior mean, 
//P_n is covariance of x and posterior covarience 
//Kalman Filter for HR
signed int Kalman(signed int LED_val,signed int LED_val1,int order){

if(kcount==0){

kalman2.Xo=LED_val1;
kalman.Xo=LED_val;	
	
}	
kcount=1;	
//compute state
kalman2.Xn=kalman2.A*kalman2.Xo;
kalman.Xn=kalman.A*kalman.Xo;
//compute covarience of state
kalman2.Pn=kalman2.A*kalman2.Po*kalman2.A+kalman2.Q;
kalman.Pn=kalman.A*kalman.Po*kalman.A+kalman.Q;
//compute Kalman Gain
kalman2.K=kalman2.Pn*kalman2.H/(kalman2.H*kalman2.Pn*kalman2.H+(kalman2.R));
kalman.K=kalman.Pn*kalman.H/(kalman.H*kalman.Pn*kalman.H+(kalman.R));
//compute Posterior Covarience
kalman2.Pn=(1-kalman2.K*kalman2.H)*kalman2.Pn;
kalman.Pn=(1-kalman.K*kalman.H)*kalman.Pn;
//Compute Posterior state
kalman2.Xn=kalman2.Xn+kalman2.K*(LED_val1-kalman2.H*kalman2.Xn);
kalman.Xn=kalman.Xn+kalman.K*(LED_val-kalman.H*kalman.Xn);
//update
kalman2.Xo=kalman2.Xn;
kalman.Xo=kalman.Xn;
kalman2.Po=kalman2.Pn;
kalman.Po=kalman.Pn;

if(order){

return (signed int)kalman.Xn;
}
	else

	return (signed int)kalman2.Xn;
	
}
//Kalmna Filter for RR
signed int Kalman2(signed int LED_val,signed int LED_val1,int order){

if(kcount2==0){

	kalman3.Xo=LED_val1;
	kalman4.Xo=LED_val;
}	
kcount2=1;	
//compute state
kalman3.Xn=kalman3.A*kalman3.Xo;
kalman4.Xn=kalman4.A*kalman4.Xo;
//compute covarience of state
kalman3.Pn=kalman3.A*kalman3.Po*kalman3.A+kalman3.Q;
kalman4.Pn=kalman4.A*kalman4.Po*kalman4.A+kalman4.Q;
//compute Kalman Gain
kalman3.K=kalman3.Pn*kalman3.H/(kalman3.H*kalman3.Pn*kalman3.H+(kalman3.R));
kalman4.K=kalman4.Pn*kalman4.H/(kalman4.H*kalman4.Pn*kalman4.H+(kalman4.R));
//compute Posterior Covarience
kalman3.Pn=(1-kalman3.K*kalman3.H)*kalman3.Pn;
kalman4.Pn=(1-kalman4.K*kalman4.H)*kalman4.Pn;
//Compute Posterior state
kalman3.Xn=kalman3.Xn+kalman3.K*(LED_val1-kalman3.H*kalman3.Xn);
kalman4.Xn=kalman4.Xn+kalman4.K*(LED_val-kalman4.H*kalman4.Xn);
//update
kalman3.Xo=kalman3.Xn;
kalman4.Xo=kalman4.Xn;
kalman3.Po=kalman3.Pn;
kalman4.Po=kalman4.Pn;

if(order){
	return (signed int)kalman3.Xn;
}
	else
	return (signed int)kalman4.Xn;
	
}

//Peak Detection Algorithm
	
signed int Peak_Detect(signed int LED_val,int min_max){

//initially should always be true
 if (LED_val>peak.max){
      peak.max=LED_val;
   
  }
else if (LED_val<peak.min){
      peak.min=LED_val;
    
}
else if (peak.find_max==true  ) { //check to see if min or max was last found if (find_max==true){ 
  
  if (LED_val<peak.max-deltad){//If value is less than last peak value and a step size, step helps control variability
   //Create a counter to track time between beats here, or possibly in individual function
    peak.maxmax=peak.max;  
		
    peak.min=LED_val; //update min, needed to update max
   
    peak.find_max=false;
    }
 }
 else   if(LED_val>peak.min+deltad){
   
      peak.minmin=peak.min;
			
      peak.max=LED_val; // update max 
    
      peak.find_max=true;
    }
 
	if(min_max){

return peak.maxmax;
	}
	
	else
		return peak.minmin;

}	


//Peak Detection Algorithm
	
signed int Peak_Detect2(signed int LED_val,int min_max){

//initially should always be true
 if (LED_val>peak2.max){
      peak2.max=LED_val;
   
  }
else if (LED_val<peak2.min){
      peak2.min=LED_val;
    
}
else if (peak2.find_max==true  ) { //check to see if min or max was last found if (find_max==true){ 
  
  if (LED_val<peak2.max-deltad2){//If value is less than last peak value and a step size, step helps control variability
   //Create a counter to track time between beats here, or possibly in individual function
    peak2.maxmax=peak2.max;  
		
    peak2.min=LED_val; //update min, needed to update max
   
    peak2.find_max=false;
    }
 }
 else   if(LED_val>peak2.min+deltad2){
   
      peak2.minmin=peak2.min;
			
      peak2.max=LED_val; // update max 
    
      peak2.find_max=true;
    }
 
	if(min_max){

return peak2.maxmax;
	}
	
	else
		return peak2.minmin;

}	


//Peak Detection Algorithm
	
signed int Peak_Detect3(signed int LED_val,int min_max){

//initially should always be true
 if (LED_val>peak3.max){
      peak3.max=LED_val;
   
  }
else if (LED_val<peak3.min){
      peak3.min=LED_val;
    
}
else if (peak3.find_max==true  ) { //check to see if min or max was last found if (find_max==true){ 
  
  if (LED_val<peak3.max-deltad3){//If value is less than last peak value and a step size, step helps control variability
   //Create a counter to track time between beats here, or possibly in individual function
    peak3.maxmax=peak3.max;  
		
    peak3.min=LED_val; //update min, needed to update max
   
    peak3.find_max=false;
    }
 }
 else if(LED_val>peak3.min+deltad3){
   
      peak3.minmin=peak3.min;
			
      peak3.max=LED_val; // update max 
    
      peak3.find_max=true;
    }
 
	if(min_max){

return peak3.maxmax;
	}
	
	else
		return peak3.minmin;

}	



//Peak Detection Algorithm
	
signed int Peak_Detect4(signed int LED_val,int min_max){

//initially should always be true
 if (LED_val>peak4.max){
      peak4.max=LED_val;
   
  }
else if (LED_val<peak4.min){
      peak4.min=LED_val;
    
}
else if (peak4.find_max==true  ) { //check to see if min or max was last found if (find_max==true){ 
  
  if (LED_val<peak4.max-deltad4){//If value is less than last peak value and a step size, step helps control variability
   //Create a counter to track time between beats here, or possibly in individual function
    peak4.maxmax=peak4.max;  
		
    peak4.min=LED_val; //update min, needed to update max
   
    peak4.find_max=false;
    }
 }
 else if(LED_val>peak4.min+deltad4){
   
      peak4.minmin=peak4.min;
			
      peak4.max=LED_val; // update max 
    
      peak4.find_max=true;
    }
 
	if(min_max){

return peak4.maxmax;
	}
	
	else
		return peak4.minmin;

}	

signed char  physical_activity_count(signed char x,signed char  y,signed char  z){

	//if xyz are less then a cetain threshold then return 0
	
	//Activity Count Vector Magnitude
return sqrt((x*x)+(z*z)+(y*y));

}	


signed int RLS(signed int LED_val,signed char accel){

	e_n=LED_val-w_n*accel;
	k_n=(accel)/(lambda*P_n*P_n+accel*accel);
	P_n=lambda*P_n*P_n+accel*accel;
	w_n=w_n+k_n*e_n;
	return (signed int)e_n;
	
}











