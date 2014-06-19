
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <iostream>

#include "Exports.h"

#ifndef _MATRIXDEFS
#define _MATRIXDEFS


#if (USEFLOAT)
	typedef float NUMBER;
#else
	typedef double NUMBER;
#endif

namespace matrices
{
#if (USEFLOAT)
	typedef Eigen::Vector3f Vector3;
	typedef Eigen::Vector2f Vector2;
	typedef Eigen::VectorXf VectorX;
	typedef Eigen::MatrixXf MatrixX;
	typedef Eigen::Matrix4f Matrix4;
	typedef Eigen::Matrix3f Matrix3;
#else
	typedef Eigen::Vector3d Vector3;
	typedef Eigen::Vector2d Vector2;
	typedef Eigen::VectorXd VectorX;
	typedef Eigen::MatrixXd MatrixX;
	typedef Eigen::Matrix4d Matrix4;
	typedef Eigen::Matrix3d Matrix3;
#endif

	void Matrix3ToMatrix4(const Matrix3& from, Matrix4& to);

	void Matrix4ToMatrix3(const Matrix4& from, Matrix3& to);

	/* Rotates rotated around axis by angle theta, and returns it */
	Matrix4 Translate(NUMBER x, NUMBER y, NUMBER z);
	Matrix4 Translate(Vector3 vector);
	Matrix4 Rotx4(NUMBER theta);
	Matrix4 Roty4(NUMBER theta);
	Matrix4 Rotz4(NUMBER theta);

	Matrix3 Rotx3(NUMBER theta);
	Matrix3 Roty3(NUMBER theta);
	Matrix3 Rotz3(NUMBER theta);

	// project vector onto another and returns cosinnus angle
	double Project(const Vector3& from, const Vector3& to);

	Vector3 ProjectOnPlan(const Vector3& normalAxis, const Vector3& vector);

	bool NotZero(const Vector3& vect);

	/*Tu considères u,v deux vecteurs de norme L2 = 1 dans R^3
	Tu cherches la rotation R, telle que Ru=v.
	R = cos theta * I + (I x [a,a,a])^T * sin theta + (1 - cos theta) * a*a^T
	avec :
	cos theta = u . v
	sin theta = ||u x v||
	a=u x v / sin theta
	I étant l'identité, * le produit matriciel, x le cross product et ^T la transposée.
	http://fr.wikipedia.org/wiki/Rotation_vectorielle
	Dérivée de la formule de Rodriguez*/
	void GetRotationMatrix(const Vector3& from, const Vector3& to, Matrix3& result);

	// TODO forward dec
	Vector3& Rotate(const Vector3& axis, Vector3& rotated, NUMBER theta);


	void vect4ToVect3(const VectorX& from, Vector3& to);

	extern void vect3ToVect4(const Vector3& from, VectorX& to);

	Vector3 matrix4TimesVect3(const Matrix4& mat4, const Vector3& vect3);

	Vector3 matrix4TimesVect4(const Matrix4& mat4, const VectorX& vect4);

	void matrixToArray(float * tab, const Matrix4& mat4);
	void matrix3ToArray(float * tab, const Matrix3& mat3);
	void matrix3ToArrayD(double * tab, const Matrix3& mat3);

	void matrixID(float * tab);


	void matrixTo16Array(float  * tab, const Matrix4& mat4);
	void matrixTo16Array(double * tab, const Matrix4& mat4);

	void array16ToMatrix4(const float  * tab, Matrix4& mat4);
	void array16ToMatrix4(const double * tab, Matrix4& mat4);

	void vect4ToArray(float * tab, const VectorX& vect);
	void vect4ToArray(double * tab, const VectorX& vect);

	void vect3ToArray(float * tab, const Vector3& vect);
	void vect3ToArray(double * tab, const Vector3& vect);

	void arrayToVect3(const float  * tab, Vector3& vect);
	void arrayToVect3(const double * tab, Vector3& vect);
} //namespace matrices


/*template<typename _Matrix_Type_>
void PseudoInverse(_Matrix_Type_& from, _Matrix_Type_& to)
{
	Eigen::JacobiSVD<_Matrix_Type_> svd(from, Eigen::ComputeFullU | Eigen::ComputeFullU);
	svd.pinv(to);
}*/


//REF: boulic et al An inverse kinematics architecture enforcing an arbitrary number of strict priority levels
template<typename _Matrix_Type_>
void PseudoInverseDLS(_Matrix_Type_& pinvmat,  NUMBER lambda)
{
	Eigen::JacobiSVD<_Matrix_Type_> svd(pinvmat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	VectorX m_sigma = svd.singularValues();

// temp computation foireuse pour lambda
// REF: must apply numerical filtering for the operation of robotic manipulators through kinematically singular ...
	bool found = false; int i = m_sigma.rows() -1;
	NUMBER val = 0;
	while (!found && i >= 0)
	{
		val = m_sigma(i);
		found = m_sigma(i) > 0;
		if (found) lambda = val  / 1.f;
		i--;
	}
//end tmp

	NUMBER  pinvtoler= NUMBER(lambda != 0 ? 0 : 1.e-6); // choose your tolerance widely!
	NUMBER lambda2 = lambda * lambda;

	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (int i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=m_sigma(i)/(m_sigma(i) * m_sigma(i) + lambda2);
	}
	pinvmat = (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}

template<typename _Matrix_Type_>
void PseudoInverseSVDDLS(_Matrix_Type_& pinvmat, Eigen::JacobiSVD<_Matrix_Type_>& svd, _Matrix_Type_& dest, NUMBER lambda = 0.f)
{
	VectorX m_sigma = svd.singularValues();
		
	// temp computation foireuse pour lambda
	// REF: must apply numerical filtering for the operation of robotic manipulators through kinematically singular ...
	bool found = false; int i = m_sigma.rows() -1;
	NUMBER val = 0;
	while (!found && i >= 0)
	{
		val = m_sigma(i);
		found = m_sigma(i) > 0;
		if (found) lambda = val / 1.f;
		i--;
	}
	//end tmp
	NUMBER  pinvtoler = NUMBER(lambda != 0 ? 0 : 1.e-6); // choose your tolerance widely!
	NUMBER lambda2 = lambda * lambda;
		
	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (int i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=m_sigma(i)/(m_sigma(i) * m_sigma(i) + lambda2);
			//m_sigma_inv(i,i)=1.0/m_sigma(i);
	}
	dest= (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}

//REF: boulic et al An inverse kinematics architecture enforcing an arbitrary number of strict priority levels
template<typename _Matrix_Type_>
void PseudoInverse(_Matrix_Type_& pinvmat)
{
	Eigen::JacobiSVD<_Matrix_Type_> svd(pinvmat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	VectorX m_sigma = svd.singularValues();

	NUMBER  pinvtoler= 1.e-6; // choose your tolerance widely!

	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (long i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=1.0/m_sigma(i);
	}
	pinvmat = (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}


#endif //_MATRIXDEFS
