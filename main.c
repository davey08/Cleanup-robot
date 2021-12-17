#include <ev3.h>
#include <stdbool.h>
int setPoint = 30;
#define DEG_TO_MOVE_ONE_FOOT 1400
#define QUARTER_TURN 80
#define SPEED 20

bool greenGoal = false;
bool redGoal = false;

int leftTurnCount = 0;
int turns = 0;
/*wander - robot will move through free space until it is within 1 foot of an "object"*/
/*Hopefully we can use all sensors*/

//code above needs to be in its own function
//stopMoving();

// detecting distance parameter in mm
// 915mm = 36in
// 508mm = 20in
// 406mm = 16in
// 305mm = 12in
// 254mm = 10in
// 203mm = 8 in
#define OBJECT_SONAR_DIST_MAX 758 //30 inches
#define OBJECT_SONAR_DIST_MIN 203 //8 inches

typedef enum _colors
{
	transparent = 0,
	black = 1,
	blue = 2,
	green = 3,
	yellow = 4,
	red = 5,
	white = 6,
	brown = 7
}colors;
void rotateCCW(void);

//bool readSonarSensor(int prev)
//{
//	setAllSensorMode(COL_REFLECT, COL_COLOR, US_DIST_MM, COL_COLOR);
//	// returns distance in mm from 0 to 2550mm
//	int dist = readSensor(IN_3);
//	if ( (dist < OBJECT_SONAR_DIST_MIN) || (prev >= (dist - 100) ) )
//	{
//		playFinishingTone();
//		rotateCCW();
//		forwardOneSquare();
//
//		if (5 == readSensor(IN_1))
//			// true means it has found an object and we turn towards it
//			return true;
//	}
//	// false, means it did not find an object yet
//	return false;
//}

void initOurSensors(void)
{
	// Inputs7
	// IN_1 IN_2 IN_3 IN_4
	setAllSensorMode(COL_REFLECT, COL_COLOR, US_DIST_MM, COL_COLOR);
}

void rotateCCW(void)
{
	setAllSensorMode(COL_REFLECT, COL_COLOR, US_DIST_MM, COL_COLOR);
	Wait(1000);

	ResetRotationCount(OUT_A);
	ResetRotationCount(OUT_D);


	OnRevReg(OUT_A, SPEED/2);
	OnFwdReg(OUT_D, SPEED/2);
	Wait(3300);
//	while (MotorRotationCount(OUT_A) <= -(QUARTER_TURN-3));
//	//while (MotorRotationCount(OUT_D) >= -(QUARTER_TURN+6));
	Off(OUT_AD);
	return;
}

void rotateCW(void)
{
	Wait(1000);
	ResetRotationCount(OUT_A);
	ResetRotationCount(OUT_D);

	OnFwdReg(OUT_A, SPEED/2);
	OnRevReg(OUT_D, SPEED/2);
	Wait(3000);
//	while (MotorRotationCount(OUT_D) <= (QUARTER_TURN-3));
//	while (MotorRotationCount(OUT_D) <= QUARTER_TURN-10);

	Off(OUT_AD);

	return;
}

void grapple(void)
{
	Wait(1000);
	ResetRotationCount(OUT_B);
	PlaySound(SOUND_DOUBLE_BEEP);
	Wait(20);


	OnRevReg(OUT_B, 40);
	Wait(2000);

	Off(OUT_B);
	Wait(1000);
	return;
}

void release(void)
{
	Wait(1000);
	ResetRotationCount(OUT_B);
	PlaySound(SOUND_DOUBLE_BEEP);
	Wait(20);

	OnFwdReg(OUT_B, 40);
	Wait(2000);

	Off(OUT_B);
	Wait(1000);
	return;
}
void wallFound(void)
{
	int x = readSensor(IN_4);

	if(x == 2)
	{
		PlaySound(SOUND_LOW_BEEP);
		OnRevReg(OUT_A, SPEED/2);
		OnRevReg(OUT_D, SPEED/2);
		Wait(1200);
		Off(OUT_AD);
		Wait(500);

		rotateCCW();
		Off(OUT_AD);

		return;
	}
	return;
}

bool wallFoundCounter(void)
{
	int x = readSensor(IN_4);

	if(x == 2)
	{
		return true;
	}
	return false;
}

void search(void)
{
	setAllSensorMode(COL_REFLECT, COL_COLOR, US_DIST_MM, COL_COLOR);
	//Wait(1000);
	ResetRotationCount(OUT_A);
	Wait(20);
//	int x = readSensor(IN_4);
	int sonarDistance = 1000; //sonar distance
	int time = 0;
	int counter = 0;

	sonarDistance = readSensor(IN_3);
	while(sonarDistance > 200)
	{
		if(time > 1000) // time before rotate/search
		{
			time = 0;
			turns = turns % 3;
			while(counter < 500) //rotation counter
			{
				sonarDistance = readSensor(IN_3);



				if (turns > 0)
				{
					OnRevReg(OUT_A, SPEED/2);
					OnFwdReg(OUT_D, SPEED/2);
				}
				else
				{
					OnFwdReg(OUT_A, SPEED/2);
					OnRevReg(OUT_D, SPEED/2);
				}

				Wait(10);
				if(sonarDistance <= 600)
				{
					PlaySound(SOUND_DOUBLE_BEEP);
					counter = 0;
					break;
				}
				else if (counter == 498)
				{
					counter == 0;
					break;
				}
				counter++;
			}

			counter = 0;
			Wait(50);
			Off(OUT_AD);
			Wait(50);
		}
		turns++;
		wallFound();

		sonarDistance = readSensor(IN_3);
		OnFwdSync(OUT_AD, SPEED+10);
		Wait(10);
		time++;
	}
	Off(OUT_AD);
	Wait(500);


//	while (sonarDistance > 200)//if loop breaks we found object
//	{
//		x = readSensor(IN_4); //left color sensor
//		sonarDistance = readSensor(IN_3);
//    	 wallFound()
//		OnFwdSync(OUT_AD, SPEED);
//		Wait(10);
//	}
//	Wait(500);
//	Off(OUT_AD);
//	rotateCCW();
	return;
}


void wallFollowing(int check)
{
    //initOurSensors();
	setAllSensorMode(COL_REFLECT, COL_COLOR, US_DIST_MM, COL_COLOR);
	int kp = 100;
    int totalPower = 20;
    int error = 0;
    int lastError = 0;
    int integral = 0;
    int derivative = 0;
    int y = readSensor(IN_1);
    int x = readSensor(IN_4);
    int sonarDistance = 1000;
    int correction = 0;
	//sonar distance to wander
	y = readSensor(IN_1);
	x = readSensor(IN_4);
	error = setPoint - y;
	integral += error;
	derivative = error - lastError;
	correction = (error*kp);
	correction /= 100;

	if(correction > 0)
	{
		OnFwdReg(OUT_D, totalPower);
		OnFwdReg(OUT_A, 0);
	}
	else if (correction < 0)
	{
		OnFwdReg(OUT_A, totalPower);
		OnFwdReg(OUT_D, 0);
	}
	lastError = error;
	Wait(5);

	x = readSensor(IN_4);

	wallFound(); //checks if there is a wall there and rotates
	if(x == 3 && check == 1)
	{
		return;
	}
	else if(x == 5 && check == 0)
	{
		return;
	}
	return;
}


void searchWallFollowing(int check)
{
    //initOurSensors();
	setAllSensorMode(COL_REFLECT, COL_COLOR, US_DIST_MM, COL_COLOR);
	int kp = 100;
    int totalPower = 20;
    int error = 0;
    int lastError = 0;
    int integral = 0;
    int derivative = 0;
    int y = readSensor(IN_1);
    int x = readSensor(IN_4);
    int sonarDistance = 1000;
    int correction = 0;
	//sonar distance to wander
	y = readSensor(IN_1);
	x = readSensor(IN_4);
	error = setPoint - y;
	integral += error;
	derivative = error - lastError;
	correction = (error*kp);
	correction /= 100;

	if(correction > 0)
	{
		OnFwdReg(OUT_D, totalPower);
		OnFwdReg(OUT_A, 0);
	}
	else if (correction < 0)
	{
		OnFwdReg(OUT_A, totalPower);
		OnFwdReg(OUT_D, 0);
	}
	lastError = error;
	Wait(5);

	x = readSensor(IN_4);

	if (wallFoundCounter)
	{
		leftTurnCount++;
	}
	leftTurnCount = leftTurnCount % 3;
	if (leftTurnCount > 0)
	{
		PlaySound(SOUND_LOW_BEEP);
		OnRevReg(OUT_A, SPEED/2);
		OnRevReg(OUT_D, SPEED/2);
		Wait(1200);
		Off(OUT_AD);
		Wait(500);

		rotateCCW();
		Off(OUT_AD);
	}
	else
	{
		PlaySound(SOUND_LOW_BEEP);
		OnRevReg(OUT_A, SPEED/2);
		OnRevReg(OUT_D, SPEED/2);
		Wait(1200);
		Off(OUT_AD);
		Wait(500);

		rotateCW();
		Off(OUT_AD);
	}





	if(x == 3 && check == 1)
	{
		return;
	}
	else if(x == 5 && check == 0)
	{
		return;
	}
	return;
}


void deliverObject()
{
	colors blockColor = readSensor(IN_2); //to check to see which color the block is
	colors goalColor = readSensor(IN_4);
	while(blockColor != green && blockColor != red)
	{
		goalColor = readSensor(IN_4);

		wallFound();//checking for blue wall and rotating

		blockColor = readSensor(IN_2); //checking for block color
		OnFwdSync(OUT_AD, SPEED); //going forward until robot is at block
		Wait(10);
	}
	Off(OUT_AD);
	Wait(500);
	grapple(); //grab object
	goalColor = readSensor(IN_4);
	while(goalColor != blue) //go forward until robot sees wall so that it can initiate wall following
	{
		goalColor = readSensor(IN_4);
		OnFwdSync(OUT_AD, SPEED);
		Wait(10);
	}
	//code below will rotate robot
	OnRevReg(OUT_A, SPEED/2);
	OnRevReg(OUT_D, SPEED/2);
	Wait(1200);
	Off(OUT_AD);
	Wait(100);
	rotateCCW();

	//blockColor = readSensor(IN_2);
	//wall follow until at proper goal
	switch(blockColor)
	{
		case green:
			while(goalColor != black)
			{
				goalColor = readSensor(IN_4);
				wallFollowing(1); //follow wall until at goal
//				searchWallFollowing(1);
//				PlaySound(SOUND_DOUBLE_BEEP);
//				Wait(500);
			}
			PlaySound(SOUND_CLICK);
			Off(OUT_AD);
			Wait(500);
			release();

			OnRevReg(OUT_A, SPEED/2);
			OnRevReg(OUT_D, SPEED/2);
			Wait(5000);
			Off(OUT_AD);
			Wait(500);
			rotateCCW();
			Wait(50);
			greenGoal = true; //set flag to true
			break;

		case red:
			while(goalColor != red)
			{
				goalColor = readSensor(IN_4);
				wallFollowing(0);
//				searchWallFollowing(0);
//				PlaySound(SOUND_DOUBLE_BEEP);
//				Wait(500);
			}
			PlaySound(SOUND_CLICK);
			PlaySound(SOUND_CLICK);
			Off(OUT_AD);
			Wait(500);
			release();

			OnRevReg(OUT_A, SPEED/2);
			OnRevReg(OUT_D, SPEED/2);
			Wait(3000);
			Off(OUT_AD);
			Wait(500);
			rotateCCW();
			Wait(50);
			redGoal = true;
			break;
		default:
			break;
	}
}

int main(void)
{
    InitEV3();
	initOurSensors();
    while(!greenGoal || !redGoal)
    {
    	search();
    	deliverObject();
    }

    PlaySound(SOUND_DOWN);

    Off(OUT_AD);

    FreeEV3();
}
