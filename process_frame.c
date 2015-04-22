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
		CalcDeriv();
		AvgDeriv(0);
		AvgDeriv(1);
		AvgDeriv(2);

		absmax = 0;
			//for (int i = 0; i < IMG_SIZE; i++) {
			for (int r = 7 * nc; r < nr * nc - 7 * nc; r += nc) {/* we skip the first and last line */
				for (int c = 7; c < nc - 7; c++) {
					int i = r + c;
					Mc[i] = ((avgDxy[0][i] >> Oshift) * (avgDxy[1][i] >> Oshift)
							- (avgDxy[2][i] >> Oshift) * (avgDxy[2][i] >> Oshift))
							- ((5
									* ((avgDxy[0][i] >> Oshift)
											+ (avgDxy[1][i] >> Oshift))
									* ((avgDxy[0][i] >> Oshift)
											+ (avgDxy[1][i] >> Oshift))) >> 7);
					absmax = MAX(Mc[i], absmax);
				}
			}

			absmax = (absmax/100*data.ipc.state.nThreshold);


			t1 = OscSupCycGet();
			LocalMaximum();

		/*
		 for(r=nc *(Border +1);r<nr*nc -nc *(Border + 1);r+= nc){
		        for(c = Border +1;c< nc- (Border +1);c++){
		            int dIx2 = (avgDxy[0][r+c] >> 7);
		            int dIy2 = (avgDxy[1][r+c] >> 7);
		            int dIxy = (avgDxy[2][r+c] >> 7);

		            avgDxy[2][r+c] = ((dIx2*dIy2)-dIxy*dIxy)-((5*(dIx2+dIy2)*(dIx2+dIy2))>>7);
		//          data.u8TempImage[BACKGROUND][r+c] = MAX(0,MIN(255,(avgDxy[2][r+c]>>10)));


*/
		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));


	}
}

void CalcDeriv()
{
    int c, r;
    for(r = nc; r < nr*nc-nc; r+= nc) {/* we skip the first and last line */
        for(c = 1; c < nc-1; c++) {
            /* do pointer arithmetics with respect to center pixel location */
            unsigned char* p = &data.u8TempImage[SENSORIMG][r+c];
            /* implement Sobel filter  */
            int dx =     -(int) *(p-nc-1) + (int) *(p-nc+1)
                         -2* (int) *(p-1) + 2* (int) *(p+1)
                             -(int) *(p+nc-1) + (int) *(p+nc+1);

            int dy = 	 -(int) *(p - nc - 1) - 2 * (int) *(p - nc)
							- (int) *(p - nc + 1) + (int) *(p + nc - 1)
							+ 2 * (int) *(p + nc) + (int) *(p + nc + 1);

            avgDxy[0][r+c] = dx*dx;
            avgDxy[1][r+c] = dy*dy;
            avgDxy[2][r+c] = dx*dy;

             // data.u8TempImage[BACKGROUND][r+c] = (uint8) MIN(255, MAX(0, (dx*dx) >> 10));
             // data.u8TempImage[BACKGROUND][r+c] = (uint8) MIN(255, MAX(0, (dy*dy) >> 10));
            //data.u8TempImage[BACKGROUND][r+c] = (uint8) MIN(255, MAX(0, 128+dx));
        }
    }
}


void AvgDeriv(int Index)
{
    //do average in x-direction
    int c, r;
    for(r = nc; r < nr*nc-nc; r+= nc) {/* we skip first and last lines (empty) */
        for(c = Border+1; c < nc-(Border+1); c++) {/* +1 because we have one empty border column */
            /* do pointer arithmetics with respect to center pixel location */
            int* p = &avgDxy[Index][r+c];
            int sx =    (*(p-6) + *(p+6))  + ((*(p-5) + *(p+5)) << 2) + ((*(p-4) + *(p+4)) << 3) +
                  ((*(p-3) + *(p+3)) << 5) + ((*(p-2) + *(p+2)) << 6) + ((*(p-1) + *(p+1)) << 6) + (*p << 7);
            //now averaged
            helpBuf[r+c] = (sx >> 8);
        }
    }
    //do average in y-direction
    for(r=nc*(Border+1);r<nr*nc-nc*(Border+1); r+=nc){ /*skip border lines*/
            for(c=Border+1; c<nc-(Border+1);c++){/*+1 because we have one empty border column*/
                /*do pointer arithmetics with respect to center pixel location*/
                int *p = &helpBuf[r+c];
                int sy = (*(p-6*nc) + *(p+6*nc))*1 + (*(p-5*nc) + *(p+5*nc))*4 + (*(p-4*nc) + *(p+4*nc))*11 +
                    (*(p-3*nc) + *(p+3*nc))*27 + (*(p-2*nc) + *(p+2*nc))*50 + (*(p-1*nc) + *(p+1*nc))*72 + (*p)*82;
                //now averaged

            avgDxy[Index][r+c] = (sy >> 8);
        }
    }
}

void LocalMaximum() {
	//memset(McLocalMax, 0, sizeof(McLocalMax));
	int c, r;

	for (r = 7 * nc; r < nr * nc - 7 * nc; r += nc) {/* we skip the first and last line */
		for (c = 7; c < nc - 7; c++) {
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &Mc[c + r];
			/* implement Sobel filter */
			int localMax = 0;
			for (int i = -6; i < 7; i++) {
				for (int j = -6; j < 7; j++) {
					if (localMax <= *(p + nc * i + j)) {
						//	McLocalMax[c+r+iHelp*nc+jHelp] = 0;
						localMax = *(p + i * nc + j);
					}
					//	else if(localMax > *(p+nc*i+j)){
					//	McLocalMax[c+r+i*nc+j] = 0;

					//}
				}

			}
			//McLocalMax[c+r+iHelp*nc+jHelp] = *(p+nc*iHelp+jHelp);
			if (localMax == *p && *p > absmax) {
				DrawBoundingBox(c - SizeBox, r / nc + SizeBox, c + SizeBox,
						r / nc - SizeBox, false, GREEN);

				//OscLog(INFO, "c = %d ; r = %d\n",c,r);
			}
		}
	}
}

