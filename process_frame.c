/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)

const int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT/2;

int TextColor;
int avgDxy[3][IMG_SIZE];
//GaussFilter[13] = {1, 4, 11, 27, 50, 72, 82, 72, 50, 27, 11, 4, 1};
GaussFilter[13] = {1, 4, 8, 32, 64, 64, 128, 64, 64, 32, 8, 4, 1};
int Border = 6;
int helpBuf[IMG_SIZE];
int k = 5;

void LocalMaximum();

int Mc[IMG_SIZE];

int Mc[IMG_SIZE];
int McLocalMax[IMG_SIZE];
int absmax;

int c,r = 0;

const int SizeBox = 5;
const int Oshift = 6;

void AvgDeriv(int Index);
void CalcDeriv(void);

void ResetProcess()
{
	//called when "reset" button is pressed
	if(TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}


void ProcessFrame()
{
	uint32 t1, t2;
	char Text[] = "hallo world";
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
		data.ipc.state.nThreshold = 15;
	} else {


		//example for time measurement
		t1 = OscSupCycGet();
		//example for copying sensor image to background image
		memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG], IMG_SIZE);

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));


	}
}

