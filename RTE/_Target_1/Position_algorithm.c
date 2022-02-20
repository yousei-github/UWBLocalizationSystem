#include "Position_algorithm.h"
#include  <arm_math.h>

/****************************************************************************//**
 *
 *         APP global variables
 *
 *******************************************************************************/
Position_calcalation fundamental_one;
/****************************************************************************//**
 *
 *         MACRO
 *
 *******************************************************************************/
#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif
 /****************************************************************************//**
 *
 *         MACRO function
 *
 *******************************************************************************/

 /****************************************************************************//**
 *
 *         Private variables and function prototypes
 *
 *******************************************************************************/
inline static float safe_divide_f32(float _x, float _y);

 /****************************************************************************//**
  *
  *         Positioning function section
  *
  *******************************************************************************/
uint8 Positioning_initialization(void)
{
	uint16 i;
	/* initialize all variables */
	for (i = 0; i < THREED_TRILATERATION_LEAST_ANCHOR_NUM; i++)
	{
		fundamental_one.anchor[i].coord.x = 0;
		fundamental_one.anchor[i].coord.y = 0;
		fundamental_one.anchor[i].coord.z = 0;
		fundamental_one.anchor[i].distance = 0;
	}
	fundamental_one.position.x = 0;
	fundamental_one.position.y = 0;
	fundamental_one.position.z = 0;

	return TRUE;
}

uint8 Update_threeD_trilaterationpositioning(float _x, float _y, float _z, double _distance, uint16 _no)
{
	uint16 i;
	if ((All_anchors < _no ) && (_no != 0))
	{
		return FALSE;
	}

	i = _no - 1;

	fundamental_one.anchor[i].coord.x = _x;
	fundamental_one.anchor[i].coord.y = _y;
	fundamental_one.anchor[i].coord.z = _z;
	fundamental_one.anchor[i].distance = _distance;

	return TRUE;

}

uint8 ThreeD_trilaterationpositioning_run(void)
{
#define x1 (fundamental_one.anchor[Anchor_NO1].coord.x)
#define x2 (fundamental_one.anchor[Anchor_NO2].coord.x)
#define x3 (fundamental_one.anchor[Anchor_NO3].coord.x)
#define x4 (fundamental_one.anchor[Anchor_NO4].coord.x)
#define y1 (fundamental_one.anchor[Anchor_NO1].coord.y)
#define y2 (fundamental_one.anchor[Anchor_NO2].coord.y)
#define y3 (fundamental_one.anchor[Anchor_NO3].coord.y)
#define y4 (fundamental_one.anchor[Anchor_NO4].coord.y)
#define z1 (fundamental_one.anchor[Anchor_NO1].coord.z)
#define z2 (fundamental_one.anchor[Anchor_NO2].coord.z)
#define z3 (fundamental_one.anchor[Anchor_NO3].coord.z)
#define z4 (fundamental_one.anchor[Anchor_NO4].coord.z)
#define d1 (fundamental_one.anchor[Anchor_NO1].distance)
#define d2 (fundamental_one.anchor[Anchor_NO2].distance)
#define d3 (fundamental_one.anchor[Anchor_NO3].distance)
#define d4 (fundamental_one.anchor[Anchor_NO4].distance)
#define k1 (k[0])
#define k2 (k[1])
#define k3 (k[2])
#define k4 (k[3])
#define h1 (h[0])
#define h2 (h[1])
#define h3 (h[2])

	float zeta_i_1[THREED_TRILATERATION_LEAST_ANCHOR_NUM - 1];
	float x_i_1[THREED_TRILATERATION_LEAST_ANCHOR_NUM - 1];
	float y_i_1[THREED_TRILATERATION_LEAST_ANCHOR_NUM - 1];
	float z_i_1[THREED_TRILATERATION_LEAST_ANCHOR_NUM - 1];
	float f_1_i[THREED_TRILATERATION_LEAST_ANCHOR_NUM - 1];
	float k[4];
	float h[3];
	float temp[4];
	float est_x[2];
	float est_y[2];
	float est_z[2];
	float v[2];

	zeta_i_1[Anchor_NO2] = x2 * x2 + y2 * y2 + z2 * z2 - (x1 * x1 + y1 * y1 + z1 * z1);
	zeta_i_1[Anchor_NO3] = x3 * x3 + y3 * y3 + z3 * z3 - (x1 * x1 + y1 * y1 + z1 * z1);
	x_i_1[Anchor_NO2] = x2 - x1;
	x_i_1[Anchor_NO3] = x3 - x1;
	y_i_1[Anchor_NO2] = y2 - y1;
	y_i_1[Anchor_NO3] = y3 - y1;
	z_i_1[Anchor_NO2] = z2 - z1;
	z_i_1[Anchor_NO3] = z3 - z1;
	f_1_i[Anchor_NO2] = 0.5f * (d1 * d1 - d2 * d2 + zeta_i_1[Anchor_NO2]);
	f_1_i[Anchor_NO3] = 0.5f * (d1 * d1 - d3 * d3 + zeta_i_1[Anchor_NO3]);
	//k1 = (x_i_1[Anchor_NO2] * z_i_1[Anchor_NO3] - x_i_1[Anchor_NO3] * z_i_1[Anchor_NO2]) / (x_i_1[Anchor_NO3] * y_i_1[Anchor_NO2] - x_i_1[Anchor_NO2] * y_i_1[Anchor_NO3]);
	//k2 = (x_i_1[Anchor_NO3] * f_1_i[Anchor_NO2] - x_i_1[Anchor_NO2] * f_1_i[Anchor_NO3]) / (x_i_1[Anchor_NO3] * y_i_1[Anchor_NO2] - x_i_1[Anchor_NO2] * y_i_1[Anchor_NO3]);
	//k3 = (y_i_1[Anchor_NO3] * z_i_1[Anchor_NO2] - y_i_1[Anchor_NO2] * z_i_1[Anchor_NO3]) / (x_i_1[Anchor_NO3] * y_i_1[Anchor_NO2] - x_i_1[Anchor_NO2] * y_i_1[Anchor_NO3]);
	//k4 = (y_i_1[Anchor_NO2] * f_1_i[Anchor_NO3] - y_i_1[Anchor_NO3] * f_1_i[Anchor_NO2]) / (x_i_1[Anchor_NO3] * y_i_1[Anchor_NO2] - x_i_1[Anchor_NO2] * y_i_1[Anchor_NO3]);
	k1 = safe_divide_f32((x_i_1[Anchor_NO2] * z_i_1[Anchor_NO3] - x_i_1[Anchor_NO3] * z_i_1[Anchor_NO2]), (x_i_1[Anchor_NO3] * y_i_1[Anchor_NO2] - x_i_1[Anchor_NO2] * y_i_1[Anchor_NO3]));
	k2 = safe_divide_f32((x_i_1[Anchor_NO3] * f_1_i[Anchor_NO2] - x_i_1[Anchor_NO2] * f_1_i[Anchor_NO3]), (x_i_1[Anchor_NO3] * y_i_1[Anchor_NO2] - x_i_1[Anchor_NO2] * y_i_1[Anchor_NO3]));
	k3 = safe_divide_f32((y_i_1[Anchor_NO3] * z_i_1[Anchor_NO2] - y_i_1[Anchor_NO2] * z_i_1[Anchor_NO3]), (x_i_1[Anchor_NO3] * y_i_1[Anchor_NO2] - x_i_1[Anchor_NO2] * y_i_1[Anchor_NO3]));
	k4 = safe_divide_f32((y_i_1[Anchor_NO2] * f_1_i[Anchor_NO3] - y_i_1[Anchor_NO3] * f_1_i[Anchor_NO2]), (x_i_1[Anchor_NO3] * y_i_1[Anchor_NO2] - x_i_1[Anchor_NO2] * y_i_1[Anchor_NO3]));

	h1 = k1 * k1 + k3 * k3 + 1;
	h2 = k1 * (y1 - k2) + k3 * (x1 - k4) + z1;
	temp[0] = x1 - k4;
	temp[1] = y1 - k2;
	h3 = temp[0] * temp[0] + temp[1] * temp[1] + z1 * z1 - d1 * d1;

	//temp[0] = h2 / h1;
	temp[0] = safe_divide_f32(h2, h1);
	//temp[1] = temp[0] * temp[0] - h3 / h1;
	temp[1] = temp[0] * temp[0] - safe_divide_f32(h3, h1);
	arm_sqrt_f32(temp[1], &temp[1]);
	est_z[0] = temp[0] + temp[1];
	est_z[1] = temp[0] - temp[1];

	est_x[0] = k3 * est_z[0] + k4;
	est_x[1] = k3 * est_z[1] + k4;
	est_y[0] = k1 * est_z[0] + k2;
	est_y[1] = k1 * est_z[1] + k2;

	temp[0] = x4 - est_x[0];
	temp[1] = y4 - est_y[0];
	temp[2] = z4 - est_z[0];
	temp[3] = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	arm_sqrt_f32(temp[3], &temp[3]);
	v[0] = temp[3] - d4;
	//v[0] = v[0] * v[0];
	arm_abs_f32(&v[0], &v[0], 1);

	temp[0] = x4 - est_x[1];
	temp[1] = y4 - est_y[1];
	temp[2] = z4 - est_z[1];
	temp[3] = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	arm_sqrt_f32(temp[3], &temp[3]);
	v[1] = temp[3] - d4;
	//v[1] = v[1] * v[1];
	arm_abs_f32(&v[1], &v[1], 1);

	if (v[0] > v[1])
	{
		fundamental_one.position.x = est_x[1];
		fundamental_one.position.y = est_y[1];
		fundamental_one.position.z = est_z[1];
	}
	else
	{
		fundamental_one.position.x = est_x[0];
		fundamental_one.position.y = est_y[0];
		fundamental_one.position.z = est_z[0];
	}


	return TRUE;

#undef x1
#undef x2
#undef x3
#undef x4
#undef y1
#undef y2
#undef y3
#undef y4
#undef z1
#undef z2
#undef z3
#undef z4
#undef d1
#undef d2
#undef d3
#undef d4
#undef k1
#undef k2
#undef k3
#undef k4
#undef h1
#undef h2
#undef h3
}

/* x/y */
static float safe_divide_f32(float _x, float _y)
{
	if (_y == 0)
		return 0;

	return (_x / _y);
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * TEST algorithm
 * Positioning_initialization();
 * // (2, 2, 0)
 * Update_threeD_trilaterationpositioning(0, 0, 0, 2 * 1.414f, 1);
 * Update_threeD_trilaterationpositioning(4, 0, 0, 2 * 1.414f, 2);
 * Update_threeD_trilaterationpositioning(4, 4, 0, 2 * 1.414f, 3);
 * Update_threeD_trilaterationpositioning(0, 4, 2, 2 * 1.732f, 4);
 * ThreeD_trilaterationpositioning_run();
 * // (5, 0, 0)
 * Update_threeD_trilaterationpositioning(0, 0, 0, 5, 1);
 * Update_threeD_trilaterationpositioning(4, 0, 0, 1, 2);
 * Update_threeD_trilaterationpositioning(4, 4, 0, 4.123f, 3);
 * Update_threeD_trilaterationpositioning(0, 4, 2, 6.708f, 4);
 * ThreeD_trilaterationpositioning_run();
 * // (4, 5, 0)
 * Update_threeD_trilaterationpositioning(0, 0, 0, 6.403f, 1);
 * Update_threeD_trilaterationpositioning(4, 0, 0, 5, 2);
 * Update_threeD_trilaterationpositioning(4, 4, 0, 1, 3);
 * Update_threeD_trilaterationpositioning(0, 4, 2, 4.582f, 4);
 * ThreeD_trilaterationpositioning_run();
 * // (-1, 0, 0)
 * Update_threeD_trilaterationpositioning(0, 0, 0, 1, 1);
 * Update_threeD_trilaterationpositioning(4, 0, 0, 5, 2);
 * Update_threeD_trilaterationpositioning(4, 4, 0, 6.403f, 3);
 * Update_threeD_trilaterationpositioning(0, 4, 2, 4.582f, 4);
 * ThreeD_trilaterationpositioning_run();
 * // (0, -1, 0)
 * Update_threeD_trilaterationpositioning(0, 0, 0, 1, 1);
 * Update_threeD_trilaterationpositioning(4, 0, 0, 4.123f, 2);
 * Update_threeD_trilaterationpositioning(4, 4, 0, 6.403f, 3);
 * Update_threeD_trilaterationpositioning(0, 4, 2, 5.385f, 4);
 * ThreeD_trilaterationpositioning_run();
 * // (0, 4, 3)
 * Update_threeD_trilaterationpositioning(0, 0, 0, 5, 1);
 * Update_threeD_trilaterationpositioning(4, 0, 0, 6.403f, 2);
 * Update_threeD_trilaterationpositioning(4, 4, 0, 5, 3);
 * Update_threeD_trilaterationpositioning(0, 4, 2, 3, 4);
 * ThreeD_trilaterationpositioning_run();
 *
 *
 *
 *
 ****************************************************************************************************************************************************/




