#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>

#define LoopTime 1 // millisec
#define ENCODERA 23
#define ENCODERB 24
int ENC2REDGEAR = 216;

#define MOTOR1 20
#define MOTOR2 21

#define PGAIN 1000
#define IGAIN 0.01
#define DGAIN 4

const float Pi = 3.141592;

int n;
int encA;
int encB;
int encPosition = 0;
float redGearPosition = 0;
float refPos;
float errPos = 0;
float InterrPos = 0;
float beferrPos = 0;
float D_Loss = 0;
float t;

float PID;
float Speed;

unsigned int startTime;
unsigned int checkTime;
unsigned int checkTimeBefore;

void funEncA(){
	encA = digitalRead(ENCODERA);
	encB = digitalRead(ENCODERB);
	
	if(encA == HIGH){
		if(encB == LOW){
			encPosition ++;
			}
		else{
			encPosition --;
			}
		}
	else{
		if(encB == LOW){
			encPosition --;
			}
		else{
			encPosition ++;
			}
		}
	redGearPosition = (float)encPosition/ENC2REDGEAR;
	printf("A : %d , B : %d , encPos : %d , gearPos :%f\n", encA, encB, encPosition, redGearPosition);
	}

void funEncB(){
	encA = digitalRead(ENCODERA);
	encB = digitalRead(ENCODERB);
	
	if(encB == HIGH){
		if(encA == LOW){
			encPosition --;
			}
		else{
			encPosition ++;
			}
		}
	else{
		if(encA == LOW){
			encPosition ++;
			}
		else{
			encPosition --;
			}
		}
	redGearPosition = (float)encPosition/ENC2REDGEAR;
	printf("A : %d , B : %d , encPos : %d , gearPos :%f\n", encA, encB, encPosition, redGearPosition);
	}

void Enc1X(){
	ENC2REDGEAR = ENC2REDGEAR/4;
	wiringPiISR(ENCODERA, INT_EDGE_RISING, funEncA);
	}

void Enc2X(){
	ENC2REDGEAR = ENC2REDGEAR/2;
	wiringPiISR(ENCODERA, INT_EDGE_BOTH, funEncA);
	}

void Enc4X(){
	ENC2REDGEAR = ENC2REDGEAR;
	wiringPiISR(ENCODERA, INT_EDGE_BOTH, funEncA);
	wiringPiISR(ENCODERB, INT_EDGE_BOTH, funEncB);
	}

float Speed_Lim(float speed){
	if(speed > 100){
		speed = 100;
		}
	return speed;
	}

int main(void){
	FILE *fp = fopen("/home/pi/MJ_meca/PID_data/1-2/Enc/1X.txt","w");
	int loop = 1;
	wiringPiSetupGpio();
	softPwmCreate(MOTOR1, 0,100);
	softPwmCreate(MOTOR2, 0,100);
	pinMode(ENCODERA, INPUT);
	pinMode(ENCODERB, INPUT);
	
	startTime = millis();
	printf("select mode 1 , 2 , 4 : ");
	scanf("%d", &n);
	
	if(n == 1){
		Enc1X();
		}
	else if(n==2){
		Enc2X();
		}
	else if(n==4){
		Enc4X();
		}
	else{
		printf("Error : input error. input should be 1,2 or 4");
		}
	
	checkTimeBefore = millis();
	while(1){
		checkTime = millis();
		if(checkTime - checkTimeBefore > LoopTime){
			t = 0.001*LoopTime*(loop-1); // time(sec)
			if(t==10){
				softPwmWrite(MOTOR1, 0);
				softPwmWrite(MOTOR2, 0);
				break;
				}
			refPos = sin(2*Pi*t/5);
			errPos = refPos - redGearPosition;
			PID = fabs(errPos*PGAIN + InterrPos*IGAIN + D_Loss*DGAIN);
			Speed = Speed_Lim(PID);
			if(errPos>0){
				softPwmWrite(MOTOR1, 0);
				softPwmWrite(MOTOR2,Speed);
				}
			else{
				softPwmWrite(MOTOR1,Speed);
				softPwmWrite(MOTOR2, 0);
				}
			InterrPos = InterrPos + ((errPos+beferrPos)/2)*LoopTime;
			D_Loss = (errPos - beferrPos)/LoopTime;
			beferrPos = errPos;
			fprintf(fp, "%d\t%f\n", loop, redGearPosition);
			checkTimeBefore = checkTime;
			loop++;
			}
		
		}
	return 0;
}
