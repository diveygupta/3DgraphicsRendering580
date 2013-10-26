
#include	"stdafx.h"
#include	"stdio.h"
#include	"math.h"
#include	"Gz.h"
#include	"rend.h"

int Rasterization( GzRender * render, GzCoord * vertex );

const GzMatrix defaultMatrix={	{1,0,0,0},
								{0,1,0,0},
								{0,0,1,0},
								{0,0,0,1}};

float degreeToRadian(float degree)
{
	return (float)(degree / 180.0) * (3.1415);
}


void matrixMul(GzMatrix matrixA,GzMatrix matrixB,GzMatrix output)
{
GzMatrix temp;
for(int i=0;i<4;i++){
for(int j=0;j<4;j++){
        temp[i][j]=0;
        for(int k=0;k<4;k++){
		temp[i][j]=temp[i][j]+(matrixA[i][k] * matrixB[k][j]);
							}
	}
}
memcpy(output,temp,sizeof(GzMatrix));
}

int GzRotXMat(float degree, GzMatrix mat)
{
// Create rotate matrix : rotate along x axis
// Pass back the matrix using mat value
	memcpy(mat, defaultMatrix, sizeof(GzMatrix));
	mat[1][1]=mat[2][2]=cos(degreeToRadian(degree));
	mat[2][1]=sin(degreeToRadian(degree));
	mat[1][2]=-sin(degreeToRadian(degree));

	return GZ_SUCCESS;
}


int GzRotYMat(float degree, GzMatrix mat)
{
// Create rotate matrix : rotate along y axis
// Pass back the matrix using mat value
	memcpy(mat, defaultMatrix, sizeof(GzMatrix));
	mat[0][0]=mat[2][2]=cos(degreeToRadian(degree));
	mat[0][2]=sin(degreeToRadian(degree));
	mat[2][0]=-sin(degreeToRadian(degree));

	return GZ_SUCCESS;
}


int GzRotZMat(float degree, GzMatrix mat)
{
// Create rotate matrix : rotate along z axis
// Pass back the matrix using mat value
	memcpy(mat, defaultMatrix, sizeof(GzMatrix));
	mat[0][0]=mat[1][1]=cos(degreeToRadian(degree));
	mat[1][0]=sin(degreeToRadian(degree));
	mat[0][1]=-sin(degreeToRadian(degree));
	return GZ_SUCCESS;
}


int GzTrxMat(GzCoord translate, GzMatrix mat)
{
// Create translation matrix
// Pass back the matrix using mat value
	memcpy(mat, defaultMatrix, sizeof(GzMatrix));
	mat[0][3]=translate[0];
	mat[1][3]=translate[1];
	mat[2][3]=translate[2];


	return GZ_SUCCESS;
}


int GzScaleMat(GzCoord scale, GzMatrix mat)
{
// Create scaling matrix
// Pass back the matrix using mat value
	memcpy(mat, defaultMatrix, sizeof(GzMatrix));
	mat[0][0]=scale[0];
	mat[1][1]=scale[1];
	mat[2][2]=scale[2];
	return GZ_SUCCESS;
}


void setXsp(GzRender *render)
{
	memcpy(render->Xsp,defaultMatrix,sizeof(GzMatrix));
	float d = 1.0/tan(degreeToRadian(render->camera.FOV)/2.0);

	render->Xsp[0][0]=((float)(render->display->xres))/2.0;
	render->Xsp[0][3]=((float)(render->display->xres))/2.0;
	render->Xsp[1][1]=-((float)(render->display->yres))/2.0;
	render->Xsp[1][3]=((float)(render->display->yres))/2.0;
	render->Xsp[2][2]=INT_MAX/d;
}

void setXpi(GzRender *render)
{
	memcpy(render->camera.Xpi,defaultMatrix,sizeof(GzMatrix));
	float d = 1.0/tan(degreeToRadian(render->camera.FOV)/2.0);

	render->camera.Xpi[3][2]=1.0/d;
}

void setXiw(GzRender *render)
{
	GzCoord Xs,Ys,Zs,Xtemp,Ytemp,Ztemp;
	GzMatrix temp;
	memcpy(temp,defaultMatrix,sizeof(GzMatrix));
	//calc Zaxis
	Ztemp[0]=render->camera.lookat[0]-render->camera.position[0];
	Ztemp[1]=render->camera.lookat[1]-render->camera.position[1];
	Ztemp[2]=render->camera.lookat[2]-render->camera.position[2];

	float norm=sqrt(pow(Ztemp[0],2)+pow(Ztemp[1],2)+pow(Ztemp[2],2));

	Zs[0]=Ztemp[0]/norm;
	Zs[1]=Ztemp[1]/norm;
	Zs[2]=Ztemp[2]/norm;

	//calc Yaxis

	float upDotZs=(render->camera.worldup[0])*Zs[0]+(render->camera.worldup[1])*Zs[1]+(render->camera.worldup[2])*Zs[2];

	Ytemp[0]=render->camera.worldup[0]-upDotZs*Zs[0];
	Ytemp[1]=render->camera.worldup[1]-upDotZs*Zs[1];
	Ytemp[2]=render->camera.worldup[2]-upDotZs*Zs[2];

	norm=sqrt(pow(Ytemp[0],2)+pow(Ytemp[1],2)+pow(Ytemp[2],2));

	Ys[0]=Ytemp[0]/norm;
	Ys[1]=Ytemp[1]/norm;
	Ys[2]=Ytemp[2]/norm;

	//calc Xaxis
	Xs[0]= Ys[1]*Zs[2]-Ys[2]*Zs[1];
	Xs[1]= Ys[2]*Zs[0]-Ys[0]*Zs[2];
	Xs[2]= Ys[0]*Zs[1]-Ys[1]*Zs[0];

	temp[0][0]=Xs[0];
	temp[0][1]=Xs[1];
	temp[0][2]=Xs[2];
	temp[0][3]=-(render->camera.position[0]*Xs[0] + render->camera.position[1]*Xs[1] + render->camera.position[2]*Xs[2]);

	temp[1][0]=Ys[0];
	temp[1][1]=Ys[1];
	temp[1][2]=Ys[2];
	temp[1][3]=-(render->camera.position[0]*Ys[0] + render->camera.position[1]*Ys[1] + render->camera.position[2]*Ys[2]);

	temp[2][0]=Zs[0];
	temp[2][1]=Zs[1];
	temp[2][2]=Zs[2];
	temp[2][3]=-(render->camera.position[0]*Zs[0] + render->camera.position[1]*Zs[1] + render->camera.position[2]*Zs[2]);

	memcpy(render->camera.Xiw,temp,sizeof(GzMatrix));

}

int GzNewRender(GzRender **render, GzDisplay *display)
{
/*  
- malloc a renderer struct 
- setup Xsp and anything only done once 
- save the pointer to display 
- init default camera 
*/ 
*render=(GzRender *)malloc(sizeof(GzRender));
if(*render==NULL || display==NULL)
	return GZ_FAILURE;

(*render)->display=display;

//set camera
for(int i=0;i<4;i++){
	for(int j=0;j<4;j++){
		(*render)->camera.Xiw[i][j]=0;
		(*render)->camera.Xpi[i][j]=0;
	}
	}
(*render)->camera.FOV=DEFAULT_FOV;
for(int i=0;i<3;i++)
	{
		(*render)->camera.lookat[i]=0;
		//(*render)->camera.position[i]=0;
		//(*render)->camera.worldup[i]=0;
}


(*render)->camera.position[0]=DEFAULT_IM_X;
(*render)->camera.position[1]=DEFAULT_IM_Y;
(*render)->camera.position[2]=DEFAULT_IM_Z;

//set Xsp
for(int i=0;i<4;i++){
	for(int j=0;j<4;j++){
		(*render)->Xsp[i][j]=0;
	}
}

(*render)->camera.worldup[0] = 0;
(*render)->camera.worldup[1] = 1;
(*render)->camera.worldup[2] = 0;

(*render)->matlevel=0;

//set flat color
for(int i=0;i<3;i++)
	{
		(*render)->flatcolor[i]=0;
		(*render)->Ka[i]= (*render)->Kd[i]=(*render)->Ks[i]=0;
}

(*render)->interp_mode=0;
(*render)->numlights=0;
(*render)->spec=0;




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


int GzBeginRender(GzRender *render)
{
/*  
- set up for start of each frame - clear frame buffer 
- compute Xiw and projection xform Xpi from camera definition 
- init Ximage - put Xsp at base of stack, push on Xpi and Xiw 
- now stack contains Xsw and app can push model Xforms if it want to. 
*/ 
for(int i=0;i<3;i++)
	render->flatcolor[i]=0;
	
render->matlevel=-1;
GzInitDisplay(render->display);
	
	setXsp(render);
	setXpi(render);
	setXiw(render);
	
	GzPushMatrix(render,render->Xsp);
	GzPushMatrix(render,render->camera.Xpi);
	GzPushMatrix(render,render->camera.Xiw);
	
	return GZ_SUCCESS;
}

int GzPutCamera(GzRender *render, GzCamera *camera)
{
/*
- overwrite renderer camera structure with new camera definition
*/
	if(render==NULL || camera==NULL)
		return GZ_FAILURE;

	render->camera.FOV=camera->FOV;

	render->camera.position[0]=camera->position[0];
	render->camera.position[1]=camera->position[1];
	render->camera.position[2]=camera->position[2];

	render->camera.worldup[0]=camera->worldup[0];
	render->camera.worldup[1]=camera->worldup[1];
	render->camera.worldup[2]=camera->worldup[2];

	for(int i=0;i<4;i++){
	for(int j=0;j<4;j++){
		render->camera.Xiw[i][j]=camera->Xiw[i][j];
		render->camera.Xpi[i][j]=camera->Xpi[i][j];
	}
	}
	
	render->camera.lookat[0]=camera->lookat[0];
	render->camera.lookat[1]=camera->lookat[1];
	render->camera.lookat[2]=camera->lookat[2];

	return GZ_SUCCESS;	
}

int GzPushMatrix(GzRender *render, GzMatrix	matrix)
{
/*
- push a matrix onto the Ximage stack
- check for stack overflow
*/
	if(render->matlevel>=MATLEVELS)
		return GZ_FAILURE;
	if(render->matlevel==-1)
	{
		render->matlevel++;
		int i=render->matlevel++;
		memcpy(render->Ximage[render->matlevel],matrix,sizeof(GzMatrix));
		return GZ_SUCCESS;
	}
	else
	{
		int i=render->matlevel;
		matrixMul(render->Ximage[i],matrix,render->Ximage[render->matlevel+1]);
		render->matlevel++;
		return GZ_SUCCESS;
	}
	
}

int GzPopMatrix(GzRender *render)
{
/*
- pop a matrix off the Ximage stack
- check for stack underflow
*/
	if(render->matlevel<=0)
	return GZ_FAILURE;
	render->matlevel--;
	return GZ_SUCCESS;
}


int GzPutAttribute(GzRender	*render, int numAttributes, GzToken	*nameList, 
	GzPointer	*valueList) /* void** valuelist */
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
	
		}
	
	}
	return GZ_SUCCESS;
}

void transformation(GzMatrix mat,GzCoord c,GzCoord transfVert)
{
	float temp[4],x,y=0.0;
	memset(temp,0,sizeof(temp));

	for(int i=0;i<4;i++ )
	{
	for(int j=0; j<4;j++ )
	{
			x=0.0;
	if(j==3)
	{
				//for z
	temp[i]=temp[i]+mat[i][j];
	x=temp[i];
	}
	else
	{ 
	temp[i]=temp[i]+mat[i][j]*c[j];
	x=temp[i];
	}
	}
	}

	float t=temp[3];
	transfVert[0]=temp[0]/t;
	transfVert[1]=temp[1]/t;
	y=transfVert[1];
	transfVert[2]=temp[2]/t;

}


int GzPutTriangle(GzRender	*render, int numParts, GzToken *nameList, 
				  GzPointer	*valueList)
/* numParts : how many names and values */
{
/*  
- pass in a triangle description with tokens and values corresponding to 
      GZ_POSITION:3 vert positions in model space 
- Xform positions of verts  
- Clip - just discard any triangle with verts behind view plane 
       - test for triangles with all three verts off-screen 
- invoke triangle rasterizer  
*/ 

	//GzCoord *transfVert;
	if(render==NULL)
		return GZ_FAILURE;
	for(int i=0; i<numParts;i++ )
	{
	  if(nameList[i]==GZ_POSITION)
		{
		GzCoord	*transfVert=(GzCoord *)malloc(3*sizeof(GzCoord));
			GzCoord *coord=(GzCoord *)(valueList[i]);	
		
			for(int i=0;i<3;i++)
			transformation(render->Ximage[render->matlevel],coord[i],transfVert[i] );
				
			Rasterization( render,transfVert);
		}
	   else if (nameList[i]==GZ_NULL_TOKEN)
		{
			
		}
	}
	

	return GZ_SUCCESS;
}

/* NOT part of API - just for general assistance */

short	ctoi(float color)		/* convert float color to GzIntensity short */
{
  return(short)((int)(color * ((1 << 12) - 1)));
}


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
			q1=render->flatcolor[0];
			q2=render->flatcolor[1];
			q3=render->flatcolor[2];
			if(z==0 || v < z)
			ret=GzPutDisplay(render->display,j,i,ctoi(q1),ctoi(q2),ctoi(q3),a,(GzDepth)v);
			
}
}
	
	
	
	
	return GZ_SUCCESS;
}