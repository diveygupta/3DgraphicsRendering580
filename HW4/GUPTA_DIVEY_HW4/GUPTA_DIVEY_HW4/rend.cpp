
#include	"stdafx.h"
#include	"stdio.h"
#include	"math.h"
#include	"Gz.h"
#include	"rend.h"
const GzMatrix defaultMatrix={	{1,0,0,0},
								{0,1,0,0},
								{0,0,1,0},
								{0,0,0,1}};
typedef struct triangleEdges
{
	float x1,y1,z1,x2,y2,z2;
	int tl,hd;
} triEdge;

typedef struct planeCoefficient
{
float planeA,planeB,planeC,planeD;
}planeCoeffi;

planeCoeffi *pCof = (planeCoeffi*)malloc(sizeof(planeCoeffi)*3);



float degreeToRadian(float degree)
{
	return (float)(degree / 180.0) * (3.1415);
}

void doShading(GzRender	*render,  GzColor color,GzCoord cod);
void getPlaneCoeff(triEdge edge[3], float color[3][3]);
void Rasterization( GzRender *, GzCoord * ,GzCoord *);

void matrixMul( GzMatrix matrixA,GzMatrix matrixB,GzMatrix output)
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
GzColor defKa = DEFAULT_AMBIENT, defKd=DEFAULT_DIFFUSE, defKs=DEFAULT_SPECULAR;

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
		(*render)->Ka[i]=defKa[i];
		(*render)->Kd[i]=defKd[i];
		(*render)->Ks[i]=defKs[i];
		(*render)->ambientlight.color[i]=0;
}

(*render)->interp_mode=GZ_RGB_COLOR;
(*render)->numlights=0;
(*render)->spec=DEFAULT_SPEC;



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
		int i=render->matlevel + 1;
		memcpy(render->Ximage[render->matlevel],matrix,sizeof(GzMatrix));
		//return GZ_SUCCESS;
	}
	else
	{
		int i=render->matlevel;
		matrixMul(render->Ximage[i],matrix,render->Ximage[render->matlevel+1]);
		render->matlevel++;
	//	return GZ_SUCCESS;
	}
	
	if(render->matlevel==0 || render->matlevel==1)
		memcpy(render->Xnorm[render->matlevel],defaultMatrix,sizeof(GzMatrix));
	else
	{
	float scaleFactor=0.0;
	GzMatrix temp;
	memcpy(temp,matrix,sizeof(GzMatrix));
	temp[0][3]=0;
	temp[1][3]=0;
	temp[2][3]=0;
	temp[3][3]=1;

	float f=sqrt(temp[0][0]*temp[0][0] + temp[1][0]*temp[1][0] + temp[2][0]*temp[2][0]);
	scaleFactor=1/f;
	for (int i=0;i<3;i++)
	  {
	    for(int j=0;j<3;j++)
			temp[i][j] = temp[i][j]*scaleFactor;		
	  }
	f=0.0;
	matrixMul(render->Xnorm[render->matlevel-1],temp, render->Xnorm[render->matlevel]);
	}
	return GZ_SUCCESS;
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
		else if(nameList[i]==GZ_AMBIENT_COEFFICIENT)
		{
		GzColor *c =(GzColor *)(valueList[i]);		
		for(int i=0;i<3;i++)
		render->Ka[i]=(*c)[i];
	
		}
		else if(nameList[i]==GZ_DIFFUSE_COEFFICIENT)
		{
		GzColor *c =(GzColor *)(valueList[i]);		
		for(int i=0;i<3;i++)
		render->Kd[i]=(*c)[i];
	
		}
		else if(nameList[i]==GZ_SPECULAR_COEFFICIENT)
		{
		GzColor *c =(GzColor *)(valueList[i]);		
		for(int i=0;i<3;i++)
		render->Ks[i]=(*c)[i];
	
		}
		else if(nameList[i]==GZ_DIRECTIONAL_LIGHT)
		{
		GzLight *c =(GzLight *)(valueList[i]);		
		for(int i=0;i<3;i++)
		{
			render->lights[render->numlights].color[i]=c->color[i];
			render->lights[render->numlights].direction[i]=c->direction[i];
		}
		//normalize
		float s=c->direction[0]*c->direction[0]+c->direction[1]*c->direction[1]+c->direction[2]*c->direction[2];
		float n=sqrt(s);
		render->lights[render->numlights].direction[0]=(c->direction[0])/n;
		render->lights[render->numlights].direction[1]=(c->direction[1])/n;
		render->lights[render->numlights].direction[2]=(c->direction[2])/n;

		render->numlights++;
		}
		else if(nameList[i]==GZ_AMBIENT_LIGHT)
		{
		GzColor *c =(GzColor *)(valueList[i]);		
		for(int i=0;i<3;i++)
		render->ambientlight.color[i]=(*c)[i];
	
		}
		else if(nameList[i]==GZ_DISTRIBUTION_COEFFICIENT)
		{
		float *c =(float *)(valueList[i]);		
		
		render->spec=(*c);
	
		}
		else if(nameList[i]==GZ_INTERPOLATE)
		{
		int *c =(int *)(valueList[i]);		
		
		render->interp_mode=(*c);
	
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

	GzCoord *transfVert, *NormVert;
	if(render==NULL)
		return GZ_FAILURE;
	for(int i=0; i<numParts;i++ )
	{
		GzCoord *coord=(GzCoord *)(valueList[i]);	
	  if(nameList[i]==GZ_POSITION)
		{
		transfVert=(GzCoord *)malloc(3*sizeof(GzCoord));
			
		
			for(int i=0;i<3;i++)
			transformation(render->Ximage[render->matlevel],coord[i],transfVert[i] );
				
			//
			//Rasterization( render,transfVert);
		}
	  else if (nameList[i]==GZ_NORMAL)
                {
            NormVert=(GzCoord *)malloc(3*sizeof(GzCoord));
		//	GzCoord *coord=(GzCoord *)(valueList[i]);	
		
			for(int i=0;i<3;i++)
			transformation(render->Xnorm[render->matlevel],coord[i],NormVert[i] );
				
			//Rasterization( render,transfVert);   
                }
	   else if (nameList[i]==GZ_NULL_TOKEN)
		{
			
		}
	}
	
	Rasterization( render,transfVert,NormVert);
	return GZ_SUCCESS;
}

/* NOT part of API - just for general assistance */

void getNormal(GzCoord vert)
{
float s = vert[0]*vert[0] + vert[1]*vert[1] +vert[2]*vert[2];
float n=sqrt(s);

for(int i=0;i<3;i++)
vert[i]=vert[i]/n;


}

short	ctoi(float color)		/* convert float color to GzIntensity short */
{
  return(short)((int)(color * ((1 << 12) - 1)));
}


void sortTriangle(GzCoord *vertex,int *defVertex)
{
	float temp1=0.0,temp2=0.0,temp3=0.0,temp4=0.0;
		int i,j;
		
	for(i=0;i<3;i++)
	{
	  for(j=0;j<3;j++)
		{
			if(vertex[i][1]<vertex[j][1])
			{
				float temp;
				temp1= vertex[i][1];
				temp2= vertex[i][0];
				temp3= vertex[i][2];
				temp4= defVertex[i];

				vertex[i][1]= vertex[j][1];
				vertex[i][0]= vertex[j][0];
				vertex[i][2]= vertex[j][2];
				defVertex[i]= defVertex[j];

				vertex[j][1]= temp1;				
				vertex[j][0]= temp2;				
				vertex[j][2]= temp3;					
				defVertex[j]= temp4;	
			}
		}
	}

}

void createBoundingBox(GzCoord *vertex,float *lowerX, float *lowerY,float *upperX,float *upperY )
{
	for(int i=0;i<3;i++)
	{
		float m=0.0;
		float temp=0.0;
		if(vertex[i][0]<(*lowerX))
			*lowerX=vertex[i][0];
		if(vertex[i][1]<(*lowerY))
			*lowerY=vertex[i][1];
		temp=(*lowerY)-(*lowerX);
		if(vertex[i][0]>(*upperX))
			*upperX=vertex[i][0];
		if(vertex[i][1]>(*upperY))
			*upperY=vertex[i][1];
	}
}

void Rasterization( GzRender * render, GzCoord * vertex, GzCoord * normXY)
{
	float temp1=0.0,temp2=0.0,temp3=0.0,temp4=0.0,m=0.0,midPoint=0.0;
	triEdge tEdges[3];
	int flag =0;
	int defVertex[3]={0,1,2};
	
float xDiffE0,yDiffE0,zDiffE0,xDiffE1,yDiffE1,zDiffE1,xDiffE2,yDiffE2,zDiffE2;
float A,B,C,D;
float a0,b0,c0,a1,b1,c1,a2,b2,c2;
float lowerX=99999999.0,lowerY=99999999.0,upperX=0.0,upperY=0.0;
float v0,v1,v2,v;
GzIntensity r,g,b,a;
GzDepth z;
	//First sort the vertices by Y if Y is same sort it according to X

sortTriangle(vertex,&defVertex[0]);

	//Assign edge in CCW
	if(vertex[0][1]== vertex[1][1])
	{
		tEdges[0].x1= vertex[1][0];
		tEdges[0].x2= vertex[0][0];
		tEdges[0].y1= vertex[1][1];
		tEdges[0].y2= vertex[0][1];
		tEdges[0].z1= vertex[1][2];
		tEdges[0].z2= vertex[0][2];
		

		tEdges[1].x1= vertex[0][0];
		tEdges[1].x2= vertex[2][0];
		tEdges[1].y1= vertex[0][1];
		tEdges[1].y2= vertex[2][1];
		tEdges[1].z1= vertex[0][2];
		tEdges[1].z2= vertex[2][2];
		

		tEdges[2].x1= vertex[2][0];
		tEdges[2].x2= vertex[1][0];
		tEdges[2].y1= vertex[2][1];
		tEdges[2].y2= vertex[1][1];
		tEdges[2].z1= vertex[2][2];
		tEdges[2].z2= vertex[1][2];

		tEdges[0].tl= defVertex[1];		
		tEdges[0].hd= defVertex[0];
		tEdges[1].tl= defVertex[0];		
		tEdges[1].hd= defVertex[2];
		tEdges[2].tl= defVertex[2];		
		tEdges[2].hd= defVertex[1];

		
	}


	if(vertex[1][1]== vertex[2][1])
	{

		
		tEdges[0].x1= vertex[0][0];
		tEdges[0].x2= vertex[1][0];
		tEdges[0].y1= vertex[0][1];
		tEdges[0].y2= vertex[1][1];
		tEdges[0].z1= vertex[0][2];
		tEdges[0].z2= vertex[1][2];
		


		tEdges[1].x1= vertex[1][0];
		tEdges[1].x2= vertex[2][0];
		tEdges[1].y1= vertex[1][1];
		tEdges[1].y2= vertex[2][1];
		tEdges[1].z1= vertex[1][2];
		tEdges[1].z2= vertex[2][2];
		


		tEdges[2].x1= vertex[2][0];
		tEdges[2].x2= vertex[0][0];
		tEdges[2].y1= vertex[2][1];
		tEdges[2].y2= vertex[0][1];
		tEdges[2].z1= vertex[2][2];
		tEdges[2].z2= vertex[0][2];

		tEdges[0].tl= defVertex[0];	
		tEdges[0].hd= defVertex[1];
		tEdges[1].tl= defVertex[1];		
		tEdges[1].hd= defVertex[2];
		tEdges[2].tl= defVertex[2];	
		tEdges[2].hd= defVertex[0];

		
	}

	else
	{
		
	float slope=0.0;
	slope= ( vertex[2][1] - vertex[0][1] ) / ( vertex[2][0] - vertex[0][0] );
		midPoint=vertex[0][0]+( ( vertex[1][1] - vertex[0][1] ) / slope )  ;
		if( vertex[1][0]< midPoint )
		{
			tEdges[0].x1= vertex[0][0];
			tEdges[0].x2= vertex[1][0];
			tEdges[0].y1= vertex[0][1];
			tEdges[0].y2= vertex[1][1];
			tEdges[0].z1= vertex[0][2];
			tEdges[0].z2= vertex[1][2];
			


			tEdges[1].x1= vertex[1][0];
			tEdges[1].x2= vertex[2][0];
			tEdges[1].y1= vertex[1][1];
			tEdges[1].y2= vertex[2][1];
			tEdges[1].z1= vertex[1][2];
			tEdges[1].z2= vertex[2][2];
			


			tEdges[2].x1= vertex[2][0];
			tEdges[2].x2= vertex[0][0];
			tEdges[2].y1= vertex[2][1];
			tEdges[2].y2= vertex[0][1];
			tEdges[2].z1= vertex[2][2];
			tEdges[2].z2= vertex[0][2];

			tEdges[0].tl= defVertex[0];			
			tEdges[0].hd= defVertex[1];
			tEdges[1].tl= defVertex[1];			
			tEdges[1].hd= defVertex[2];
			tEdges[2].tl= defVertex[2];			
			tEdges[2].hd= defVertex[0];
		}

		else if( vertex[1][0]>midPoint  )
		{
			tEdges[0].x1= vertex[0][0];
			tEdges[0].x2= vertex[2][0];
			tEdges[0].y1= vertex[0][1];
			tEdges[0].y2= vertex[2][1];
			tEdges[0].z1= vertex[0][2];
			tEdges[0].z2= vertex[2][2];
			


			tEdges[1].x1= vertex[2][0];
			tEdges[1].x2= vertex[1][0];
			tEdges[1].y1= vertex[2][1];
			tEdges[1].y2= vertex[1][1];
			tEdges[1].z1= vertex[2][2];
			tEdges[1].z2= vertex[1][2];
			


			tEdges[2].x1= vertex[1][0];
			tEdges[2].x2= vertex[0][0];
			tEdges[2].y1= vertex[1][1];
			tEdges[2].y2= vertex[0][1];
			tEdges[2].z1= vertex[1][2];
			tEdges[2].z2= vertex[0][2];

			tEdges[0].tl= defVertex[0];			
			tEdges[0].hd= defVertex[2];
			tEdges[1].tl= defVertex[2];			
			tEdges[1].hd= defVertex[1];
			tEdges[2].tl= defVertex[1];			
			tEdges[2].hd= defVertex[0];

		}

		
	}

	
	xDiffE0=tEdges[0].x2 - tEdges[0].x1;
	xDiffE1= tEdges[1].x2 - tEdges[1].x1;
	xDiffE2=tEdges[2].x2 - tEdges[2].x1;

	yDiffE0= tEdges[0].y2 - tEdges[0].y1;
	yDiffE1=tEdges[1].y2 - tEdges[1].y1;
	yDiffE2=tEdges[2].y2 - tEdges[2].y1;

	zDiffE0= tEdges[0].z2 - tEdges[0].z1;
	zDiffE1= tEdges[1].z2 - tEdges[1].z1;
	zDiffE2=tEdges[2].z2-tEdges[1].z1;
	
	//Find ABC value for the line equation
	a0 = yDiffE0;
	a1 = yDiffE1;
	a2 =yDiffE2;

	 b0 = -xDiffE0;
	 b1 = -xDiffE1;
	 b2 = -xDiffE2;

	 c0 = xDiffE0 * tEdges[0].y1  - yDiffE0 * tEdges[0].x1 ;	
	 c1 = xDiffE1 * tEdges[1].y1 - yDiffE1 * tEdges[1].x1 ;	
	 c2 =  xDiffE2* tEdges[2].y1  - yDiffE2 * tEdges[2].x1 ;

	 //Find Ax+By+Cz+d=0
	 float temp=0.0;
	A = yDiffE0 * zDiffE1 - zDiffE0 * yDiffE1;
	B = - xDiffE0 * zDiffE1 + zDiffE0 * xDiffE1 ;
	C = xDiffE0 * yDiffE1 - yDiffE0 * xDiffE1;
	temp=A* vertex[0][0] + B* vertex[0][1] + C* vertex[0][2];
	D=-temp;

	//Find the bounding box for the triangle
	
	//construct bounding box
	createBoundingBox(vertex,&lowerX,&lowerY,&upperX,&upperY);
	

	 if(render->interp_mode== GZ_NORMALS)
	{
		getPlaneCoeff(tEdges,normXY);
	}
	else if(render->interp_mode== GZ_COLOR)
	{
		GzColor c[3];
		memset(c,0,sizeof(GzColor)*3);

		for( int p =0; p <3; p++ )
		doShading(render,c[p],normXY[p]);
		
		getPlaneCoeff(tEdges,c);

	}
	

	//Check for each pixel within the bounding box whether it lies in the triangle
	for(int i=lowerY;i<= upperY; i++ )
	{
		for(int j=lowerX;j<= upperX; j++ )
		{
			GzIntensity r, g, b, a;
			GzDepth z;

			v =-(A*j+B*i+D)/C;

			 v0 = a0 * j + b0 * i + c0;
			 v1 = a1 * j + b1 * i + c1;
			 v2 = a2 * j + b2 * i + c2;
			

			if(v<0||v0<0||v1<0||v2<0)
			continue;

			
			GzGetDisplay( render->display, j, i, &r, &g, &b, &a, &z );
			//Check with prev Z vaule
			GzColor q;

			
				
				 for(int k=0;k<3;k++)
				q[k] = -( pCof[k].planeA * j + pCof[k].planeB * i  + pCof[k].planeD ) / pCof[k].planeC;
				

				if(render->interp_mode == GZ_FLAT)
				{
					for(int k=0;k<3;k++)
					q[k] = render->flatcolor[k];
						if( v < z || z==0)
					GzPutDisplay(render->display,j,i,ctoi(q[0]),ctoi(q[1]),ctoi(q[2]),a,(GzDepth )v);
				}
				if(render->interp_mode == GZ_COLOR)
				{
						if( v < z || z==0)
					GzPutDisplay(render->display,j,i,ctoi(q[0]),ctoi(q[1]),ctoi(q[2]),a,(GzDepth )v);
				}
				else if(render->interp_mode == GZ_NORMALS)
				{
					GzColor c;
					memset(c,0,sizeof(GzColor));
					getNormal(q);
					doShading(render,c,q);
						if( v < z || z==0)
					GzPutDisplay(render->display,j,i,ctoi(c[0]),ctoi(c[1]),ctoi(c[2]),a,(GzDepth )v);
				}
				

			
		} 
	} 

}

//get the color value
void doShading(GzRender	*render,  GzColor color,GzCoord cod)
{ 
	
	float nL=0.0,nE=0.0,rE=0.0;
	GzCoord cam={0,0,-1}, ray;
		
	int i=0;
	for(i; i< render->numlights; i++)
	{
		nE= cod[0]*cam[0]+cod[1]*cam[1]+cod[2]*cam[2];

		getNormal(render->lights[i].direction);
		nL= cod[0]*render->lights[i].direction[0]+cod[1]*render->lights[i].direction[1]+cod[2]*render->lights[i].direction[2]; 

		if(nL<0 && nE<0)
		{
			nL=-nL;
			nE=-nE;	
			for(int j=0;j<3;j++)
				cod[j] = cod[j]*(-1);			
		}
		if(nL>0 && nE<0)
			continue;
		if(nL<0 && nE>0)
			continue;
		

	//cal R= 2(N.L)N - L
		for(int j=0;j<3;j++)
		ray[j]= 2*(nL)*cod[j] - render->lights[i].direction[j];

		getNormal(ray);

		rE= ray[0]*cam[0] + ray[1]*cam[1] + ray[2]*cam[2];
		
		if(rE<0)
		  rE= 0;
		
		for(int j=0;j<3;j++)
		color[j]= (color[j])+(render->Ks[j] * render->lights[i].color[j] * pow(rE,render->spec)) + (render->Kd[j] * render->lights[i].color[j]*nL);

	}

	for(int j=0;j<3;j++)
		color[j]= (color[j]) + render->Ka[j] * render->ambientlight.color[j];
}

//plane eq
void getPlaneCoeff(triEdge edge[3],float color[3][3])
{
	float vX1=0.0,vX2=0.0,vY1=0.0,vY2=0.0,vZ1=0.0,vZ2=0.0;
	 vX1= edge[0].x2 - edge[0].x1;
	 vX2= edge[1].x2 - edge[1].x1;

	 vY1= edge[0].y2 - edge[0].y1;
	 vY2= edge[1].y2 - edge[1].y1;
	 
	 for(int i=0;i<3;i++)
	 {
	 vZ1= color[edge[0].hd][i] - color[edge[0].tl][i];
	 vZ2= color[edge[1].hd][i] - color[edge[1].tl][i];

	pCof[i].planeA= (vY1 * vZ2) - (vZ1 * vY2);
	pCof[i].planeB= (vZ1 * vX2) - (vX1 * vZ2);
	pCof[i].planeC= (vX1 * vY2) - (vY1 * vX2);
	pCof[i].planeD= -(pCof[i].planeA * edge[0].x1 + pCof[i].planeB * edge[0].y1 + pCof[i].planeC * color[edge[0].tl][i] );

	 }
	
}