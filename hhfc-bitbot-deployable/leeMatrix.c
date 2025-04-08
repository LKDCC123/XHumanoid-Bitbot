#include "leeMatrix.h"

#include <stdio.h>
#include <math.h>

MStruct m_mul(MStruct * m1, MStruct * m2)
{
	MStruct res;
	int i1, i2, i3;

	if (m1->dim[1] != m2->dim[0])
	{
		printf("Matrix Multiply Error!!!\n");
		return *m1;
	}
	for (i1 = 0; i1 < m1->dim[0]; i1++)
	{
		for (i2 = 0; i2 < m2->dim[1]; i2++)
		{
			res.m[i1][i2] = 0;
			for (i3 = 0; i3 < m1->dim[1]; i3++)
			{
				res.m[i1][i2] += m1->m[i1][i3] * m2->m[i3][i2];
			}
		}
	}

	res.dim[0] = m1->dim[0];
	res.dim[1] = m2->dim[1];

	return res;
}

MStruct init_matrix(int m, int n)
{
	MStruct res;
	int i1, i2;
	res.dim[0] = m;
	res.dim[1] = n;
	for (i1 = 0; i1 < m; i1++)
	{
		for (i2 = 0; i2 < n; i2++)
		{
			res.m[i1][i2] = 0.0;
			if (i2 == i1)
			{
				res.m[i1][i2] = 1.0;
			}
		}
	}
	return res;
}

MStruct init_matrix0(int m, int n)
{
	MStruct res;
	int i1, i2;
	res.dim[0] = m;
	res.dim[1] = n;
	for (i1 = 0; i1 < m; i1++)
	{
		for (i2 = 0; i2 < n; i2++)
		{
			res.m[i1][i2] = 0.0;
		}
	}
	return res;
}


void show_matrix(MStruct * m)
{
	int i1, i2;
	printf("\n-------------------------\n");
	for (i1 = 0; i1 < m->dim[0]; i1++)
	{
		
		for (i2 = 0; i2 < m->dim[1]; i2++)
		{
			printf("%.3f\t", m->m[i1][i2]);
		}
		printf("\n");
	}
	printf("-------------------------\n");
}

MStruct get_3lcross_m(double vec[3])
{
	MStruct lcm;
	double x, y, z;

	lcm = init_matrix(3, 3);
	x = vec[0];
	y = vec[1];
	z = vec[2];

	//matrix = [0 - z y; z 0 - x; -y x 0];
	lcm.m[0][0] = 0.0;	lcm.m[0][1] = -z;	lcm.m[0][2] = y;
	lcm.m[1][0] = z;	lcm.m[1][1] = 0.0;	lcm.m[1][2] = -x;
	lcm.m[2][0] = -y;	lcm.m[2][1] = x;	lcm.m[2][2] = 0.0;

	return lcm;
}

MStruct cross_3(double vec1[3], double vec2[3])
{
	MStruct m1, m2, m;

	m1 = get_3lcross_m(vec1);
	m2 = init_matrix(3, 1);
	m2.m[0][0] = vec2[0];
	m2.m[1][0] = vec2[1];
	m2.m[2][0] = vec2[2];
	m = m_mul(&m1, &m2);

	return m;
}


MStruct m_sum(MStruct * m1, MStruct * m2)
{
	int i, j;
	MStruct m_res;

	if (m1->dim[0] != m2->dim[0] || m1->dim[1] != m2->dim[1])
	{
		printf("Dimension is not the same!!\n");
		return *m1;
	}

	m_res = *m1;
	for (i = 0; i < m1->dim[0]; i++)
	{
		for (j = 0; j < m1->dim[1]; j++)
		{
			m_res.m[i][j] = m1->m[i][j] + m2->m[i][j];
		}
	}
	return m_res;
}

MStruct m_sub(MStruct * m1, MStruct * m2)
{
	int i, j;
	MStruct m_res;

	if (m1->dim[0] != m2->dim[0] || m1->dim[1] != m2->dim[1])
	{
		printf("Dimension is not the same!!\n");
		return *m1;
	}

	m_res = *m1;
	for (i = 0; i < m1->dim[0]; i++)
	{
		for (j = 0; j < m1->dim[1]; j++)
		{
			m_res.m[i][j] = m1->m[i][j] - m2->m[i][j];
		}
	}
	return m_res;
}

MStruct create_unit_matrix(int m)
{
	//int i, j;
	MStruct m_res;

	m_res = init_matrix(1, 1);

	if (m<1 || m>_MAX_MATRIX_DIM)
	{
		printf("Dimension is over the range!!\n");
		return m_res;
	}

	m_res.dim[0] = m;
	m_res.dim[1] = m;
	m_res = init_matrix(m, m);

	return m_res;
}

MStruct create_zero_matrix(int m)
{
	int i, j;
	MStruct m_res;
	m_res = init_matrix(1, 1);
	m_res.m[0][0] = 0.0;

	if (m<1 || m>_MAX_MATRIX_DIM)
	{
		printf("Dimension is over the range!!\n");
		return m_res;
	}

	m_res.dim[0] = m;
	m_res.dim[1] = m;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			m_res.m[i][j] = 0.0;
		}
	}

	return m_res;
}

MStruct comb_matrix_h(MStruct * m1, MStruct * m2)
{
	int i, j;
	MStruct m_res;
	m_res = *m1;

	if (m1->dim[0] != m2->dim[1])
	{
		printf("H-Dimension is not the same!!\n");
		return m_res;
	}

	m_res.dim[0] = m1->dim[0];
	m_res.dim[1] = m1->dim[1] + m2->dim[1];
	for (i = 0; i < m1->dim[0]; i++)
	{
		for (j = 0; j < (m1->dim[1] + m2->dim[1]); j++)
		{
			if (j < m1->dim[1])
			{
				m_res.m[i][j] = m1->m[i][j];
			}
			else
			{
				m_res.m[i][j] = m2->m[i][j];
			}
		}
	}
	return m_res;
}

MStruct comb_matrix_l(MStruct * m1, MStruct * m2)
{
	int i, j;
	MStruct m_res;
	m_res = *m1;

	if (m1->dim[0] != m2->dim[1])
	{
		printf("H-Dimension is not the same!!\n");
		return m_res;
	}

	m_res.dim[0] = m1->dim[0] + m2->dim[0];
	m_res.dim[1] = m1->dim[1];
	for (i = 0; i < (m1->dim[0] + m2->dim[0]); i++)
	{
		for (j = 0; j < (m1->dim[1]); j++)
		{
			if (i < m1->dim[0])
			{
				m_res.m[i][j] = m1->m[i][j];
			}
			else
			{
				m_res.m[i][j] = m2->m[i][j];
			}
		}
	}
	return m_res;
}

// MStruct m_pinv(MStruct * m)
// {
//
// 	return *m;
// }

// MStruct m_inv(MStruct *m)
// {
// 	MStruct m_res;
// 	double n;
// 	int i, j;
//
// 	n = m->dim[0];
// 	m_res = *m;
// 	if (m->dim[0] != m->dim[1])
// 	{
// 		printf("Matrix Inverse ERROR!! Matrix is not square\n");
// 		return *m;
// 	}
//
// 	// 求行列式 
//
// 	// 求伴随矩阵 
// 	for (i = 0; i < n; i++)
// 	{
// 		for (j = 0; j < n; j++)
// 		{
// 			// 求余子式 
//
// 			// 求代数余子式 
//
// 			// 填入伴随矩阵对应位置 
// 		}
// 	}
//
// 	// 计算逆矩阵 
//
// 	return m_res;
// }

MStruct m_T(MStruct *m)
{
	MStruct m_res;
	int i, j;

	m_res.dim[0] = m->dim[1];
	m_res.dim[1] = m->dim[0];

	for (i = 0; i < m->dim[0]; i++)
	{
		for (j = 0; j < m->dim[1]; j++)
		{
			m_res.m[j][i] = m->m[i][j];
		}
	}
	return m_res;
}

void set_matrix(MStruct *m, double *a)
{
	int i, j;
	for (i = 0; i < m->dim[0]; i++)
	{
		for (j = 0; j < m->dim[1]; j++)
		{
			m->m[i][j] = *(a + i*m->dim[1] + j);
		}
	}
}

double _lsign(double num)
{
	if (fabs(num) < 1e-10)
		return 0;
	else if (num > 0)
		return 1.0;
	else //if (num < 0)
		return -1.0;
}

MStruct m2_inv(MStruct *m)
{
	double a11, a12, a21, a22;
	int i, j;
	MStruct res = *m;

	if (m->dim[0] != m->dim[1])
	{
		printf("m2_inv error 1\n");
		return res;
	}
	if (m->dim[0] != 2)
	{
		printf("m2_inv error 2\n");
		return res;
	}
	//  m->m[0][0], m->m[0][1], m->m[1][0], m->m[1][1]
	//	[  a22/(a11*a22 - a12*a21), -a12/(a11*a22 - a12*a21)]
	//	[-a21 / (a11*a22 - a12*a21), a11 / (a11*a22 - a12*a21)]
	a11 = m->m[0][0];
	a12 = m->m[0][1];
	a21 = m->m[1][0];
	a22 = m->m[1][1];
	res.m[0][0] = a22 / (a11*a22 - a12*a21);
	res.m[0][1] = -a12 / (a11*a22 - a12*a21);
	res.m[1][0] = -a21 / (a11*a22 - a12*a21);
	res.m[1][1] = a11 / (a11*a22 - a12*a21);

	return res;
}

void offline_3D_interpolation(double * p, double * v, double * p0, double * v0, double * p1, double * v1, double t, double ST)
{
	MStruct P1, P2, P, Pt;
	MStruct Res;
	int	i;

	P1 = init_matrix(4, 4);
	P2 = init_matrix(4, 3);
	P = init_matrix(4, 3);
	Pt = init_matrix(2, 4);
	Res = init_matrix(2, 3);

	P1.m[0][0] = 2.0 / pow(ST, 3.0);	P1.m[0][1] = 1.0 / pow(ST, 2.0);	P1.m[0][2] = -2.0 / pow(ST, 3.0);	P1.m[0][3] = 1.0 / pow(ST, 2.0);
	P1.m[1][0] = -3.0 / pow(ST, 2.0);	P1.m[1][1] = -2.0 / pow(ST, 1.0);	P1.m[1][2] = 3.0 / pow(ST, 2.0);	P1.m[1][3] = -1.0 / pow(ST, 1.0);
	P1.m[2][0] = 0;						P1.m[2][1] = 1;						P1.m[2][2] = 0;						P1.m[2][3] = 0;
	P1.m[3][0] = 1;						P1.m[3][1] = 0;						P1.m[3][2] = 0;						P1.m[3][3] = 0;

	P2.m[0][0] = p0[0];		P2.m[0][1] = p0[1];		P2.m[0][2] = p0[2];
	P2.m[1][0] = v0[0];		P2.m[1][1] = v0[1];		P2.m[1][2] = v0[2];
	P2.m[2][0] = p1[0];		P2.m[2][1] = p1[1];		P2.m[2][2] = p1[2];
	P2.m[3][0] = v1[0];		P2.m[3][1] = v1[1];		P2.m[3][2] = v1[2];

	Pt.m[0][0] = pow(t, 3.0);			Pt.m[0][1] = pow(t, 2.0);			Pt.m[0][2] = pow(t, 1.0);			Pt.m[0][3] = 1;
	Pt.m[1][0] = 3.0*pow(t, 2.0);		Pt.m[1][1] = 2.0*pow(t, 1.0);		Pt.m[1][2] = 1.0;					Pt.m[1][3] = 0;

	P = m_mul(&P1, &P2);
	Res = m_mul(&Pt, &P);

	for (i = 0; i < 3; i++)
	{
		p[i] = Res.m[0][i];
		v[i] = Res.m[1][i];
	}

	if (t >= ST)
	{
		for (i = 0; i < 3; i++)
		{
			p[i] = p1[i];
			v[i] = v1[i];
		}
	}
}

void offline_1D_interpolation(double *p, double * v, double p0, double v0, double p1, double v1, double t, double ST)
{
	MStruct P1, P2, P, Pt;
	MStruct Res;

	P1 = init_matrix(4, 4);
	P2 = init_matrix(4, 1);
	P = init_matrix(4, 1);
	Pt = init_matrix(2, 4);
	Res = init_matrix(2, 1);

	P1.m[0][0] = 2.0 / pow(ST, 3.0);	P1.m[0][1] = 1.0 / pow(ST, 2.0);	P1.m[0][2] = -2.0 / pow(ST, 3.0);	P1.m[0][3] = 1.0 / pow(ST, 2.0);
	P1.m[1][0] = -3.0 / pow(ST, 2.0);	P1.m[1][1] = -2.0 / pow(ST, 1.0);	P1.m[1][2] = 3.0 / pow(ST, 2.0);	P1.m[1][3] = -1.0 / pow(ST, 1.0);
	P1.m[2][0] = 0;						P1.m[2][1] = 1;						P1.m[2][2] = 0;						P1.m[2][3] = 0;
	P1.m[3][0] = 1;						P1.m[3][1] = 0;						P1.m[3][2] = 0;						P1.m[3][3] = 0;

	P2.m[0][0] = p0;
	P2.m[1][0] = v0;
	P2.m[2][0] = p1;
	P2.m[3][0] = v1;

	Pt.m[0][0] = pow(t, 3.0);			Pt.m[0][1] = pow(t, 2.0);			Pt.m[0][2] = pow(t, 1.0);			Pt.m[0][3] = 1;
	Pt.m[1][0] = 3.0*pow(t, 2.0);		Pt.m[1][1] = 2.0*pow(t, 1.0);		Pt.m[1][2] = 1.0;					Pt.m[1][3] = 0;

	P = m_mul(&P1, &P2);
	Res = m_mul(&Pt, &P);

	*p = Res.m[0][0];
	*v = Res.m[1][0];

	if (t >= ST)
	{
		*p = p1;
		*v = v1;
	}
}

void realtime_1D_interpolation(double *p, double * v, double p1, double v1, double t, double ST, double CT)
{
	MStruct P1, P2, P, Pt;
	MStruct Res;
	double T = ST - t;

	if (T <= 0.1*CT)
	{
		*p = p1;
		*v = v1;
	}
	else
	{
		P1 = init_matrix(4, 4);
		P2 = init_matrix(4, 1);
		P = init_matrix(4, 1);
		Pt = init_matrix(2, 4);
		Res = init_matrix(2, 1);

		P1.m[0][0] = 2.0 / pow(T, 3.0);		P1.m[0][1] = 1.0 / pow(T, 2.0);		P1.m[0][2] = -2.0 / pow(T, 3.0);	P1.m[0][3] = 1.0 / pow(T, 2.0);
		P1.m[1][0] = -3.0 / pow(T, 2.0);	P1.m[1][1] = -2.0 / pow(T, 1.0);	P1.m[1][2] = 3.0 / pow(T, 2.0);		P1.m[1][3] = -1.0 / pow(T, 1.0);
		P1.m[2][0] = 0;						P1.m[2][1] = 1;						P1.m[2][2] = 0;						P1.m[2][3] = 0;
		P1.m[3][0] = 1;						P1.m[3][1] = 0;						P1.m[3][2] = 0;						P1.m[3][3] = 0;

		P2.m[0][0] = *p;
		P2.m[1][0] = *v;
		P2.m[2][0] = p1;
		P2.m[3][0] = v1;

		Pt.m[0][0] = pow(CT, 3.0);			Pt.m[0][1] = pow(CT, 2.0);			Pt.m[0][2] = pow(CT, 1.0);			Pt.m[0][3] = 1;
		Pt.m[1][0] = 3.0*pow(CT, 2.0);		Pt.m[1][1] = 2.0*pow(CT, 1.0);		Pt.m[1][2] = 1.0;					Pt.m[1][3] = 0;

		P = m_mul(&P1, &P2);
		Res = m_mul(&Pt, &P);

		*p = Res.m[0][0];
		*v = Res.m[1][0];
	}
}

void realtime_3D_interpolation(double *p, double * v, double *p1, double *v1, double t, double ST, double CT)
{
	MStruct P1, P2, P, Pt;
	MStruct Res;
	double T = ST - t;
	int	i;

	if (T <= 0.1*CT)
	{
		for (i = 0; i < 3; i++)
		{
			p[i] = p1[i];
			v[i] = v1[i];
		}
	}
	else
	{
		P1 = init_matrix(4, 4);
		P2 = init_matrix(4, 3);
		P = init_matrix(4, 3);
		Pt = init_matrix(2, 4);
		Res = init_matrix(2, 3);

		P1.m[0][0] = 2.0 / pow(T, 3.0);		P1.m[0][1] = 1.0 / pow(T, 2.0);		P1.m[0][2] = -2.0 / pow(T, 3.0);	P1.m[0][3] = 1.0 / pow(T, 2.0);
		P1.m[1][0] = -3.0 / pow(T, 2.0);	P1.m[1][1] = -2.0 / pow(T, 1.0);	P1.m[1][2] = 3.0 / pow(T, 2.0);		P1.m[1][3] = -1.0 / pow(T, 1.0);
		P1.m[2][0] = 0;						P1.m[2][1] = 1;						P1.m[2][2] = 0;						P1.m[2][3] = 0;
		P1.m[3][0] = 1;						P1.m[3][1] = 0;						P1.m[3][2] = 0;						P1.m[3][3] = 0;

		P2.m[0][0] = p[0];					P2.m[0][1] = p[1];					P2.m[0][2] = p[2];
		P2.m[1][0] = v[0];					P2.m[1][1] = v[1];					P2.m[1][2] = v[2];
		P2.m[2][0] = p1[0];					P2.m[2][1] = p1[1];					P2.m[2][2] = p1[2];
		P2.m[3][0] = v1[0];					P2.m[3][1] = v1[1];					P2.m[3][2] = v1[2];

		Pt.m[0][0] = pow(CT, 3.0);			Pt.m[0][1] = pow(CT, 2.0);			Pt.m[0][2] = pow(CT, 1.0);			Pt.m[0][3] = 1;
		Pt.m[1][0] = 3.0*pow(CT, 2.0);		Pt.m[1][1] = 2.0*pow(CT, 1.0);		Pt.m[1][2] = 1.0;					Pt.m[1][3] = 0;

		P = m_mul(&P1, &P2);
		Res = m_mul(&Pt, &P);

		for (i = 0; i < 3; i++)
		{
			p[i] = Res.m[0][i];
			v[i] = Res.m[1][i];
		}
	}
}

void realtime_1D_interpolation_5(double *p, double * v, double * a, double p1, double v1, double a1, double t, double ST, double CT)
{
	MStruct P1, P2, P, Pt;
	MStruct Res;
	double T = ST - t;
	double T2, T3, T4, T5;
	double CT2, CT3, CT4, CT5;
	T2 = pow(T, 2.0);
	T3 = pow(T, 3.0);
	T4 = pow(T, 4.0);
	T5 = pow(T, 5.0);
	CT2 = pow(CT, 2.0);
	CT3 = pow(CT, 3.0);
	CT4 = pow(CT, 4.0);
	CT5 = pow(CT, 5.0);
	if (T <= 0.1*CT)
	{
		*p = p1;
		*v = v1;
	}
	else
	{
		P1 = init_matrix(6, 6);
		P2 = init_matrix(6, 1);
		P = init_matrix(6, 1);
		Pt = init_matrix(3, 6);
		Res = init_matrix(3, 1);

		P1.m[0][0] = -6.0 / T5;		P1.m[0][1] = -3.0 / T4;		P1.m[0][2] = -1.0 / (2 *T3);	P1.m[0][3] = 6.0 / T5;		P1.m[0][4] = -3.0 / T4;		P1.m[0][5] = 1.0 / (2.0 * T3);
		P1.m[1][0] = 15.0 / T4;		P1.m[1][1] = 8.0 / T3;		P1.m[1][2] = 3.0 / (2 * T2);	P1.m[1][3] = -15.0 / T4;	P1.m[1][4] = 7.0 /T3;		P1.m[1][5] = -1.0 / T2;
		P1.m[2][0] = -10.0 / T3;	P1.m[2][1] = -6.0 / T2;		P1.m[2][2] = -3.0 / (2 * T);	P1.m[2][3] = 10.0 /T3;		P1.m[2][4] = -4.0 / T2;		P1.m[2][5] = 1.0 / (2.0 * T);
		P1.m[3][0] = 0.0;			P1.m[3][1] = 0.0;			P1.m[3][2] = 1.0 / 2.0;			P1.m[3][3] = 0;				P1.m[3][4] = 0;				P1.m[3][5] = 0;
		P1.m[4][0] = 0.0;			P1.m[4][1] = 1.0;			P1.m[4][2] = 0;				    P1.m[4][3] = 0;				P1.m[4][4] = 0;				P1.m[4][5] = 0;
		P1.m[5][0] = 1.0;			P1.m[5][1] = 0.0;			P1.m[5][2] = 0;					P1.m[5][3] = 0;				P1.m[5][4] = 0;				P1.m[5][5] = 0;

		P2.m[0][0] = *p;
		P2.m[1][0] = *v;
		P2.m[2][0] = *a;
		P2.m[3][0] = p1;
		P2.m[4][0] = v1;
		P2.m[5][0] = a1;

		Pt.m[0][0] = CT5;		Pt.m[0][1] = CT4;		Pt.m[0][2] = CT3;		Pt.m[0][3] = CT2;		Pt.m[0][4] = CT;	Pt.m[0][5] = 1.0;
		Pt.m[1][0] = 5.0*CT4;	Pt.m[1][1] = 4.0*CT3;	Pt.m[1][2] = 3.0*CT2;	Pt.m[1][3] = 2.0*CT;	Pt.m[1][4] = 1.0;	Pt.m[1][5] = 0.0;
		Pt.m[2][0] = 20.0*CT3;	Pt.m[2][1] = 12.0*CT2;	Pt.m[2][2] = 6.0*CT;	Pt.m[2][3] = 2.0;		Pt.m[2][4] = 0.0;	Pt.m[2][5] = 0.0;

		P = m_mul(&P1, &P2);
		Res = m_mul(&Pt, &P);

		*p = Res.m[0][0];
		*v = Res.m[1][0];
		*a = Res.m[2][0];
	}
}