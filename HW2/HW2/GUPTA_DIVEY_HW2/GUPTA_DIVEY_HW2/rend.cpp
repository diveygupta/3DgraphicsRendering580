#include	"stdafx.h"
#include	"stdio.h"
#include	"math.h"
#include	"Gz.h"
#include	"rend.h"

int Rasterization( GzRender * , GzCoord *  );


int GzNewRender(GzRender **render, GzDisplay *display)
{
/* 
- malloc a renderer struct
- span interpolator needs pointer to display for pixel writes
*/
	*render=(GzRender *)malloc(sizeof(GzRender));
if(*render==NULL || display==NULL)
	return GZ_FAILURE;

(*render)->display=display;

	return GZ_SUCCESS;
}


int GzFreeRender(GzRender *render)
{
/* 
-free all renderer resources
*/
	free(render);
	return GZ_SUCCESS;
}


int GzBeginRender(GzRender	*render)
{
/* 
- set up for start of each frame - init frame buffer
*/

	
	for(int i=0;i<3;i++)
	render->flatcolor[i]=0;
	//render->flatcolor[RED]=0;
	//render->flatcolor[GREEN]=0;
	//render->flatcolor[BLUE]=0;
	GzInitDisplay(render->display);
	return GZ_SUCCESS;
}


int GzPutAttribute(GzRender	*render, int numAttributes, GzToken	*nameList, 
	GzPointer *valueList) /* void** valuelist */
{
/*
- set renderer attribute states (e.g.: GZ_RGB_COLOR default color)
- later set shaders, interpolaters, texture maps, and lights
*/
	for(int i=0;i<numAttributes;i++)
	{
		if(nameList[i]==GZ_RGB_COLOR)
		{
		GzColor *c =(GzColor *)(valueList[i]);		
		for(int i=0;i<3;i++)
		render->flatcolor[i]=(*c)[i];
	//		render->flatcolor[RED]=(*c)[RED];
		//render->flatcolor[GREEN]=(*c)[GREEN];
		//render->flatcolor[BLUE]=(*c)[BLUE];
		}
	
	}
	return GZ_SUCCESS;
}


int GzPutTriangle(GzRender *render, int	numParts, GzToken *nameList,
	GzPointer *valueList) 
/* numParts - how many names and values */
{
/* 
- pass in a triangle description with tokens and values corresponding to
      GZ_NULL_TOKEN:		do nothing - no values
      GZ_POSITION:		3 vert positions in model space
- Invoke the scan converter and return an error code
*/
		if(render==NULL)
		return GZ_FAILURE;
	for(int i=0;i<numParts;i++)
	{
	  if(nameList[i]==GZ_POSITION)
		{
			GzCoord *v= (GzCoord *)(valueList[i]);
			Rasterization(render,v);
		}
	}
	return GZ_SUCCESS;
}

/* NOT part of API - just for general assistance */

short	ctoi(float color)		/* convert float color to GzIntensity short */
{
  return(short)((int)(color * ((1 << 12) - 1)));
}

//using Linear expression evaluation(LEE)
int Rasterization( GzRender * render, GzCoord * vertex )
{
	float temp1=0.0,temp2=0.0,temp3=0.0,m=0.0,midPoint=0.0;
float edges[3][6]; //x1,y1,z1,x2,y2,z2
float xDiffE0,yDiffE0,zDiffE0,xDiffE1,yDiffE1,zDiffE1,xDiffE2,yDiffE2,zDiffE2;
float A,B,C,D;
float a0,b0,c0,a1,b1,c1,a2,b2,c2;
float lowerX=99999999.0,lowerY=99999999.0,upperX=0.0,upperY=0.0;
float v0,v1,v2,v;
GzIntensity r,g,b,a;
GzDepth z;
	
for(int i=0;i<2;i++)
{
	for(int j=0;j<2-i;j++)
	{
		if( (vertex[j+1][1]<vertex[j][1])|| (vertex[j+1][0]<vertex[j][0] && vertex[j][1]==vertex[j+1][1]))
		{
			temp1=vertex[j][0];//x
			temp2=vertex[j][1];//y
			temp3=vertex[j][2];//z
			
			vertex[j][0]=vertex[j+1][0];
			vertex[j][1]=vertex[j+1][1];
			vertex[j][2]=vertex[j+1][2];
			
			vertex[j+1][0]=temp1;
			vertex[j+1][1]=temp2;
			vertex[j+1][2]=temp3;
		
		}

	}
}

//get in anti clockwise
if(vertex[1][1]==vertex[2][1])
{
//edge 0-1
	m=0.0;
edges[0][0]=vertex[0][0];//x1
edges[0][3]=vertex[1][0];//x2
edges[0][1]=vertex[0][1];//y1
edges[0][4]=vertex[1][1];//y2
edges[0][2]=vertex[0][2];//z1
edges[0][5]=vertex[1][2];//z2

//edge 1-2
edges[1][0]=vertex[1][0];
edges[1][3]=vertex[2][0];
edges[1][1]=vertex[1][1];
edges[1][4]=vertex[2][1];
edges[1][2]=vertex[1][2];
edges[1][5]=vertex[2][2];

//edge 2-0
edges[2][0]=vertex[2][0];
edges[2][3]=vertex[0][0];
edges[2][1]=vertex[2][1];
edges[2][4]=vertex[0][1];
edges[2][2]=vertex[2][2];
edges[2][5]=vertex[0][2];
}

else if(vertex[1][1]==vertex[0][1])
{
	m=0.0;
//edge 1-0
edges[0][0]=vertex[1][0];
edges[0][3]=vertex[0][0];
edges[0][1]=vertex[1][1];
edges[0][4]=vertex[0][1];
edges[0][2]=vertex[1][2];
edges[0][5]=vertex[0][2];

//edge 0-2
edges[1][0]=vertex[0][0];
edges[1][3]=vertex[2][0];
edges[1][1]=vertex[0][1];
edges[1][4]=vertex[2][1];
edges[1][2]=vertex[0][2];
edges[1][5]=vertex[2][2];

//edge 2-1
edges[2][0]=vertex[2][0];
edges[2][3]=vertex[1][0];
edges[2][1]=vertex[2][1];
edges[2][4]=vertex[1][1];
edges[2][2]=vertex[2][2];
edges[2][5]=vertex[1][2];
}
else
{
	m=0.0;
float slope=0.0;
slope=(float)(vertex[2][1]-vertex[0][1])/(float)(vertex[2][0]-vertex[0][0]);
midPoint= vertex[0][0]+( (vertex[1][1]-vertex[0][1])/slope );
if(midPoint<vertex[1][0])
{
//edge 0-2
edges[0][0]=vertex[0][0];
edges[0][3]=vertex[2][0];
edges[0][1]=vertex[0][1];
edges[0][4]=vertex[2][1];
edges[0][2]=vertex[0][2];
edges[0][5]=vertex[2][2];

//edge 2-1
edges[1][0]=vertex[2][0];
edges[1][3]=vertex[1][0];
edges[1][1]=vertex[2][1];
edges[1][4]=vertex[1][1];
edges[1][2]=vertex[2][2];
edges[1][5]=vertex[1][2];

//edge 1-0
edges[2][0]=vertex[1][0];
edges[2][3]=vertex[0][0];
edges[2][1]=vertex[1][1];
edges[2][4]=vertex[0][1];
edges[2][2]=vertex[1][2];
edges[2][5]=vertex[0][2];
}

else if(midPoint>vertex[1][0])
{
	m=0.0;
//edge 0-1
edges[0][0]=vertex[0][0];
edges[0][3]=vertex[1][0];
edges[0][1]=vertex[0][1];
edges[0][4]=vertex[1][1];
edges[0][2]=vertex[0][2];
edges[0][5]=vertex[1][2];

//edge 1-2
edges[1][0]=vertex[1][0];
edges[1][3]=vertex[2][0];
edges[1][1]=vertex[1][1];
edges[1][4]=vertex[2][1];
edges[1][2]=vertex[1][2];
edges[1][5]=vertex[2][2];

//edge 2-0
edges[2][0]=vertex[2][0];
edges[2][3]=vertex[0][0];
edges[2][1]=vertex[2][1];
edges[2][4]=vertex[0][1];
edges[2][2]=vertex[2][2];
edges[2][5]=vertex[0][2];
}
}

xDiffE0=edges[0][3]-edges[0][0];
xDiffE1=edges[1][3]-edges[1][0];
xDiffE2=edges[2][3]-edges[2][0];

yDiffE0=edges[0][4]-edges[0][1];
yDiffE1=edges[1][4]-edges[1][1];
yDiffE2=edges[2][4]-edges[2][1];

zDiffE0=edges[0][5]-edges[0][2];
zDiffE1=edges[1][5]-edges[1][2];
zDiffE2=edges[2][5]-edges[2][2];

//get coefficients of line
a0=yDiffE0;
a1=yDiffE1;
a2=yDiffE2;

b0=-xDiffE0;
b1=-xDiffE1;
b2=-xDiffE2;

c0=xDiffE0*edges[0][1] - yDiffE0*edges[0][0];
c1=xDiffE1*edges[1][1] - yDiffE1*edges[1][0];
c2=xDiffE2*edges[2][1] - yDiffE2*edges[2][0];

float temp=0.0;

//get Ax+By+Cz+D
A=yDiffE0*zDiffE1-yDiffE1*zDiffE0;
B=xDiffE1*zDiffE0-xDiffE0*zDiffE1;
C=xDiffE0*yDiffE1-xDiffE1*yDiffE0;
temp=A*edges[1][0] + B*edges[1][1] + C*edges[1][2];
D=-temp;
	
//construct bounding box
for(int i=0;i<3;i++)
	{
		m=0.0;
		float temp=0.0;
		if(vertex[i][0]<lowerX)
			lowerX=vertex[i][0];
		if(vertex[i][1]<lowerY)
			lowerY=vertex[i][1];
		temp=lowerY-lowerX;
		if(vertex[i][0]>upperX)
			upperX=vertex[i][0];
		if(vertex[i][1]>upperY)
			upperY=vertex[i][1];
	}
	
for(int i=lowerY;i<=upperY;i++)
{
for(int j=lowerX;j<=upperX;j++)
{
			float temp=0.0;
			v =-(A*j+B*i+D)/C;
			v0=a0*j+b0*i+c0;
			v1=a1*j+b1*i+c1;
			
			temp=a2*j+b1*i;
			temp=-temp/C;
			v2=a2*j+b2*i+c2;
			
			if(v<0||v0<0||v1<0||v2<0)
			continue;

			int ret=GzGetDisplay(render->display,j,i,&r,&g,&b,&a,&z);
			float q1,q2,q3;
			q1=render->flatcolor[RED];
			q2=render->flatcolor[GREEN];
			q3=render->flatcolor[BLUE];
			if(z==0 || v < z)
			ret=GzPutDisplay(render->display,j,i,ctoi(q1),ctoi(q2),ctoi(q3),a,(GzDepth)v);
			
}
}
	
	
	
	
	return GZ_SUCCESS;
}