#include "Transform.h"

using namespace Manifest_Math;

//Ax,y,z,0 Bx,y,z,0	Cx,y,z,0 Tx,y,z,1
MFtransform::MFtransform(const MFfloat& f00, const MFfloat& f01, const MFfloat& f02,
	const MFfloat& f10, const MFfloat& f11, const MFfloat& f12,
	const MFfloat& f20, const MFfloat& f21, const MFfloat& f22,
	const MFfloat& p30, const MFfloat& p31, const MFfloat& p32) :	
	MFmat4{ f00, f01, f02, 0.0f, 
			f10, f11, f12, 0.0f, 
			f20, f21, f22, 0.0f, 
			p30, p31, p32, 1.0f }
{};
//vectors are typically column major, vector ctor takes row major
MFtransform::MFtransform(const MFvec3& a, const MFvec3& b, const MFvec3& c, const MFpoint3& p) :
	MFmat4{	a.x,a.y,a.z,0.0f,
			b.x,b.y,b.z,0.0f,
			c.x,c.y,c.z,0.0f,
			p.x,p.y,p.z,1.0f }
{};
//used for taking in 3 row vectors with implicit {0}1 4th row, translation is contained in V
MFtransform::MFtransform(const MFvec4& a, const MFvec4& b, const MFvec4& c) : MFmat4{ a,b,c,{{0.0f},1.0f} }
{};



void MFtransform::SetTranslation(const MFpoint3& translation)
{
	field[3][0] = translation.x;
	field[3][1] = translation.y;
	field[3][2] = translation.z;
}


//retruns the inverse of M4
//see hAtrix4.h for details
//implied homogenous vector {0},1 ; calculation optimizations can be used
MFtransform Manifest_Math::Inverse(const MFtransform& hA)

{
	//grab 3D C-Vectors of M4x3
	const MFvec3& a{ reinterpret_cast<const MFvec3&>(hA[0]) };
	const MFvec3& b{ reinterpret_cast<const MFvec3&>(hA[1]) };
	const MFvec3& c{ reinterpret_cast<const MFvec3&>(hA[2]) };
	const MFvec3& d{ reinterpret_cast<const MFvec3&>(hA[3]) };

	//create 0 vector once, when crossed in 3d all vectors are parallel to 0,0,0
	//Cross product of parallel vectors is a 0 vector
	MFvec3 s{ Cross(a, b) };//returns vector perpendicular to vectors a&b
	MFvec3 t{ Cross(c, d) };//returns vector perpendicular to vectors c&d

	const MFfloat iDet{ 1.0f / Dot(s, c) };
	//create inverse scalars
	s *= iDet;
	t *= iDet;
	const MFvec3 v = c * iDet;
	//using inverse scalars, calculate inverse row vectors
	const MFvec3 r0{ Cross(b, v) };
	const MFvec3 r1{ Cross(v, a) };
	
	return MFtransform
	{
		r0.x,r1.x,s.x,
		r0.y,r1.y,s.y,
		r0.z,r1.z,s.z,
		-Dot(b,t),Dot(a,t),-Dot(d,s)
	};
}


MFpoint3 Manifest_Math::operator*(const MFtransform& hA, const MFpoint3& p)
{
	return MFpoint3
	{
		hA(0,0) * p.x + hA(0,1) * p.y + hA(0,2) * p.z + hA(0,3),
		hA(1,0) * p.x + hA(1,1) * p.y + hA(1,2) * p.z + hA(1,3),
		hA(2,0) * p.x + hA(2,1) * p.y + hA(2,2) * p.z + hA(2,3)
	};
}

MFpoint3 Manifest_Math::operator*(const MFtransform& hA, const MFvec3& v)
{
	return MFpoint3
	{
		hA(0,0) * v.x + hA(0,1) * v.y + hA(0,2) * v.z,
		hA(1,0) * v.x + hA(1,1) * v.y + hA(1,2) * v.z,
		hA(2,0) * v.x + hA(2,1) * v.y + hA(2,2) * v.z
	};
}

MFvec3 Manifest_Math::operator*(const MFvec3& n, const MFtransform& hA)
{
	return MFvec3
	{
		n.x * hA(0,0) + n.y * hA(1,0) + n.z * hA(2,0),
		n.x* hA(0,1) + n.y * hA(1,1) + n.z * hA(2,1),
		n.x* hA(0,2) + n.y * hA(1,2) + n.z * hA(2,2)
	};
}

MFtransform Manifest_Math::Identity()
{
	return MFtransform
	{
		1.0f,0.0f,0.0f,
		0.0f,1.0f,0.0f,
		0.0f,0.0f,1.0f,
		0.0f,0.0f,0.0f
	};
}