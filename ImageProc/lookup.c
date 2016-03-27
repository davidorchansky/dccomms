#include<stdio.h>
#include<math.h>
#include<stdint.h>

#define RAD_2_DEG 57.29577951308232
static uint8_t getDireccion(float rad)
{

	float _deg = rad * RAD_2_DEG;

	float deg;
	if(_deg < 0)
		deg = 180 + _deg;
	else
		deg= _deg;


	float dif180 = fabsf(deg-180);
	float dif135 = fabsf(deg - 135);
	float dif90 = fabsf(deg - 90);
	float dif45 = fabsf(deg - 45);
	float dif0 = deg;

	uint8_t res = 0;
	float diff = dif180;

	if(dif135 < diff){ res = 135; diff = dif135;}
	if(dif90 < diff){ res = 90; diff = dif90;}
	if(dif45 < diff){ res = 45; diff = dif45;}
	if(dif0 < diff){ res =  0; diff = dif0;}

	//fprintf(stderr, "_deg: %f ; deg: %f ; res: %d\n", _deg, deg, res);
        return res;


}


int main(int argc, char ** argv)
{

	int a;
	int b;

	int vmin = atoi(argv[1]), vmax = atoi(argv[2]);
	
	int size = vmax*2+1;

	fprintf(stdout, "#define LOOKUPTABLE_DEG_VMAX %d\n", vmax);
	fprintf(stdout, "unsigned char LOOKUP_DEG[%d][%d] =\n", size, size);
	fprintf(stdout, "{\n");
	for(a=vmin; a <= vmax; a++)
	{
		fprintf(stdout, "{");
		for(b=vmin; b<vmax; b++)
		{
			float deg = atan(((float)a)/b);
			uint8_t dis = getDireccion(deg);
			fprintf(stdout, "%3d, ",dis);
		}

		float deg = atan(a/(float)vmax);
		uint8_t dis = getDireccion(deg);

		if(a <vmax)
		{
			fprintf(stdout, "%d},\n",dis);
		}
		else
		{
			fprintf(stdout, "%d}\n", dis);
		}
	}

	fprintf(stdout, "};\n\n");
/*
	fprintf(stdout, "#define LOOKUPTABLE_GRADM_VMAX %d\n", vmax);
	fprintf(stdout, "float LOOKUP_GRADM[%d][%d] =\n", size, size);
	fprintf(stdout, "{\n");
	for(a=vmin; a <= vmax; a++)
	{
		fprintf(stdout, "{");
		for(b=vmin; b<vmax; b++)
		{
			float mod = sqrt(a*a+b*b);
			fprintf(stdout, "%f, ",mod);
		}

		float mod = atan(a*a+vmax*vmax);

		if(a <vmax)
		{
			fprintf(stdout, "%f},\n",mod);
		}
		else
		{
			fprintf(stdout, "%f}\n", mod);
		}
	}
	fprintf(stdout, "};\n");

*/
	return 0;
}
