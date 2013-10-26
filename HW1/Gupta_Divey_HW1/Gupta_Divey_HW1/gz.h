/*
 * Gz.h - include file for rendering library
 * CSCI 580   USC 
*/

/*
 * universal constants
 */
#define GZ_SUCCESS      0
#define GZ_FAILURE      1


typedef void    *GzPointer;
typedef float   GzColor[3];
typedef short   GzIntensity;		/* 0-4095 in lower 12-bits for RGBA */
typedef int	GzDepth;		/* signed z for clipping */


#define RED     0               /* array indicies for color vector */
#define GREEN   1
#define BLUE    2

#define X       0               /* array indicies for position vector */
#define Y       1
#define Z       2

#define U       0               /* array indicies for texture coords */
#define V       1

 