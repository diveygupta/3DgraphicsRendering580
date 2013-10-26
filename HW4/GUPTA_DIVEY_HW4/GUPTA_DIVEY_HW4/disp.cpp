/*   CS580 HW   */
#include    "stdafx.h"  
#include	"Gz.h"
#include	"disp.h"


int GzNewFrameBuffer(char** framebuffer, int width, int height)
{
/* create a framebuffer:
 -- allocate memory for framebuffer : (sizeof)GzPixel x width x height
 -- pass back pointer 
 -- NOTE: this function is optional and not part of the API, but you may want to use it within the display function.
*/

	//GzPixel *myGzPixel=(GzPixel *)malloc(sizeof(GzPixel);
	*framebuffer=(char *)malloc((sizeof(GzPixel))*width*height);
	//*framebuffer=(char *)malloc(3*sizeof(char)*width*height);
	if(framebuffer=NULL)
	return GZ_FAILURE;
	else
	return GZ_SUCCESS;
}

int GzNewDisplay(GzDisplay	**display, int xRes, int yRes)
{
*display=(GzDisplay *)malloc(sizeof(GzDisplay));
	if( *display ==NULL)
                return GZ_FAILURE;

	if(xRes>MAXXRES) 
	(*display)->xres=MAXXRES;
	if(yRes>MAXYRES)
	(*display)->yres=MAXYRES;
	if(xRes<0)
	(*display)->xres=0;
	else
	(*display)->xres=xRes;

	
	if(yRes<0)
	(*display)->yres=0;
	else
	(*display)->yres=yRes;

//	(*display)->fbuf = new GzPixel  ((*display)->yres)[((*display)->xres) *];
	(*display)->fbuf = (GzPixel *)malloc(sizeof(GzPixel)*((*display)->xres) * ((*display)->yres));
	if((*display)->fbuf==NULL)
	return GZ_FAILURE;

	return GZ_SUCCESS;
}


int GzFreeDisplay(GzDisplay	*display)
{
/* clean up, free memory */
	free(display->fbuf);
	free(display);
	return GZ_SUCCESS;
}


int GzGetDisplayParams(GzDisplay *display, int *xRes, int *yRes)
{
/* pass back values for a display */
	if (display == NULL)
	return GZ_FAILURE;
	*xRes=display->xres;
	*yRes=display->yres;
	return GZ_SUCCESS;
}


int GzInitDisplay(GzDisplay	*display)
{
/* set everything to some default values - start a new frame */
	if (display == NULL)
	return GZ_FAILURE;
	int arr=0;	
	for(int i=0;i<display->yres;i++)
		for(int j=0;j<display->xres;j++)
		{
			arr=ARRAY(j,i);//(j,i)
			display->fbuf[arr].blue=1600;
			display->fbuf[arr].green=1600;			
			display->fbuf[arr].red=1600;
			display->fbuf[arr].alpha=1;
			display->fbuf[arr].z=0;
		}
		
		return GZ_SUCCESS;
}


int GzPutDisplay(GzDisplay *display, int i, int j, GzIntensity r, GzIntensity g, GzIntensity b, GzIntensity a, GzDepth z)
{
/* write pixel values into the display */
		if(i<0 || j<0) return GZ_FAILURE;
			if(i>display->xres || j>display->yres) return GZ_FAILURE;
			int arr=ARRAY(i,j);
			
			if (r < 0)
			display->fbuf[arr].red = 0;
			
			if (g < 0)			
			display->fbuf[arr].green = 0;
			
			if (b < 0)			
			display->fbuf[arr].blue = 0;
			
			if(a<0)			
			display->fbuf[arr].alpha=0;			
			else
				display->fbuf[arr].alpha=a;

			if(z<0)			
			display->fbuf[arr].z=0;			
			else
				display->fbuf[arr].z=z;

			if (r > 4095)			
			display->fbuf[arr].red = 4095;			
			else
				display->fbuf[arr].red = r;

			if (g > 4095)
			display->fbuf[arr].green = 4095;			
			else
				display->fbuf[arr].green = g;

			if (b > 4095)			
			display->fbuf[arr].blue = 4095;			
			else
				display->fbuf[arr].blue = b;
						
			return GZ_SUCCESS;
}


int GzGetDisplay(GzDisplay *display, int i, int j, GzIntensity *r, GzIntensity *g, GzIntensity *b, GzIntensity *a, GzDepth *z)
{
	/* pass back pixel value in the display */
	if(display==NULL)
		return GZ_FAILURE;
	if(i<0 || j<0) return GZ_FAILURE;
			if(i>display->xres || j>display->yres) return GZ_FAILURE;

	int arr=0;
	arr=ARRAY(i,j);
	*r=display->fbuf[arr].red;
	*g=display->fbuf[arr].green;
	*b=display->fbuf[arr].blue;
	*a=display->fbuf[arr].alpha;
	*z=display->fbuf[arr].z;
	return GZ_SUCCESS;
}


int GzFlushDisplay2File(FILE* outfile, GzDisplay *display)
{

	/* write pixels to ppm file -- "P6 %d %d 255\r" */
if(display==NULL)
		return GZ_FAILURE;
	if(outfile==NULL)
		return GZ_FAILURE;
	int arr=0;
	fprintf(outfile,"P6 %d %d %d\r",display->xres,display->yres,255);//header writing
	for(int i=0;i<display->yres;i++)
		for(int j=0;j<display->xres;j++)
		{
			arr=ARRAY(j,i);//(j,i)
			//convert short to char (x=x>>4)
			fprintf(outfile,"%c%c%c",(display->fbuf[arr].red)>>4,(display->fbuf[arr].green)>>4,(display->fbuf[arr].blue)>>4);			
		}
	
	return GZ_SUCCESS;
}

int GzFlushDisplay2FrameBuffer(char* framebuffer, GzDisplay *display)
{

	/* write pixels to framebuffer: 
		- Put the pixels into the frame buffer
		- Caution: store the pixel to the frame buffer as the order of blue, green, and red 
		- Not red, green, and blue !!!
	*/

if(display==NULL)
		return GZ_FAILURE;
	int arr=0;
	//int id=0;
	for(int i=0;i<display->yres;i++)
	{		for(int j=0;j<display->xres;j++)
		{
			arr=ARRAY(j,i);//(j,i)
			framebuffer[arr*3]=(display->fbuf[arr].blue)>>4;
			framebuffer[arr*3+1*sizeof(char)]=(display->fbuf[arr].green)>>4;
			framebuffer[arr*3+2*sizeof(char)]=(display->fbuf[arr].red)>>4;
		}
	}
	return GZ_SUCCESS;
}