/// autogenerated analytical inverse kinematics code from ikfast program part of OpenRAVE
/// \author Rosen Diankov
///
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///     http://www.apache.org/licenses/LICENSE-2.0
/// 
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.
///
/// ikfast version 0x1000004a generated on 2021-04-12 02:02:56.857991
/// To compile with gcc:
///     gcc -lstdc++ ik.cpp
/// To compile without any main function as a shared object (might need -llapack):
///     gcc -fPIC -lstdc++ -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -shared -Wl,-soname,libik.so -o libik.so ik.cpp
#define IKFAST_HAS_LIBRARY
#include "ikfast.h" // found inside share/openrave-X.Y/python/ikfast.h
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==0x1000004a);

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

#ifndef IKFAST_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define IKFAST_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }

#endif

#if defined(_MSC_VER)
#define IKFAST_ALIGNED16(x) __declspec(align(16)) x
#else
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))
#endif

#define IK2PI  ((IkReal)6.28318530717959)
#define IKPI  ((IkReal)3.14159265358979)
#define IKPI_2  ((IkReal)1.57079632679490)

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#ifndef isinf
#define isinf _isinf
#endif
//#ifndef isfinite
//#define isfinite _isfinite
//#endif
#endif // _MSC_VER

// lapack routines
extern "C" {
  void dgetrf_ (const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
  void zgetrf_ (const int* m, const int* n, std::complex<double>* a, const int* lda, int* ipiv, int* info);
  void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info);
  void dgesv_ (const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
  void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
  void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi,double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
}

using namespace std; // necessary to get std math routines

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKsqr(float f) { return f*f; }
inline double IKsqr(double f) { return f*f; }

inline float IKlog(float f) { return logf(f); }
inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)1e-7)
#endif

// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)1e-7)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

// there are checkpoints in ikfast that are evaluated to make sure they are 0. This threshold speicfies by how much they can deviate
#ifndef IKFAST_EVALCOND_THRESH
#define IKFAST_EVALCOND_THRESH ((IkReal)0.00001)
#endif


inline float IKasin(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(-IKPI_2);
else if( f >= 1 ) return float(IKPI_2);
return asinf(f);
}
inline double IKasin(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0) {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline double IKfmod(double x, double y)
{
    while(x < 0) {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(IKPI);
else if( f >= 1 ) return float(0);
return acosf(f);
}
inline double IKacos(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKtan(float f) { return tanf(f); }
inline double IKtan(double f) { return tan(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2Simple(float fy, float fx) {
    return atan2f(fy,fx);
}
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return float(IKPI_2);
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2Simple(double fy, double fx) {
    return atan2(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2(fy,fx);
}

template <typename T>
struct CheckValue
{
    T value;
    bool valid;
};

template <typename T>
inline CheckValue<T> IKatan2WithCheck(T fy, T fx, T epsilon)
{
    CheckValue<T> ret;
    ret.valid = false;
    ret.value = 0;
    if( !isnan(fy) && !isnan(fx) ) {
        if( IKabs(fy) >= IKFAST_ATAN2_MAGTHRESH || IKabs(fx) > IKFAST_ATAN2_MAGTHRESH ) {
            ret.value = IKatan2Simple(fy,fx);
            ret.valid = true;
        }
    }
    return ret;
}

inline float IKsign(float f) {
    if( f > 0 ) {
        return float(1);
    }
    else if( f < 0 ) {
        return float(-1);
    }
    return 0;
}

inline double IKsign(double f) {
    if( f > 0 ) {
        return 1.0;
    }
    else if( f < 0 ) {
        return -1.0;
    }
    return 0;
}

template <typename T>
inline CheckValue<T> IKPowWithIntegerCheck(T f, int n)
{
    CheckValue<T> ret;
    ret.valid = true;
    if( n == 0 ) {
        ret.value = 1.0;
        return ret;
    }
    else if( n == 1 )
    {
        ret.value = f;
        return ret;
    }
    else if( n < 0 )
    {
        if( f == 0 )
        {
            ret.valid = false;
            ret.value = (T)1.0e30;
            return ret;
        }
        if( n == -1 ) {
            ret.value = T(1.0)/f;
            return ret;
        }
    }

    int num = n > 0 ? n : -n;
    if( num == 2 ) {
        ret.value = f*f;
    }
    else if( num == 3 ) {
        ret.value = f*f*f;
    }
    else {
        ret.value = 1.0;
        while(num>0) {
            if( num & 1 ) {
                ret.value *= f;
            }
            num >>= 1;
            f *= f;
        }
    }
    
    if( n < 0 ) {
        ret.value = T(1.0)/ret.value;
    }
    return ret;
}

/// solves the forward kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot) {
IkReal x0,x1,x2,x3,x4;
x0=IKcos(j[0]);
x1=IKsin(j[1]);
x2=IKsin(j[0]);
x3=((0.1)*x2);
x4=((0.1)*x0);
eetrans[0]=(((x1*x3))+x4);
eetrans[1]=((0.127)+(((0.1)*(IKcos(j[1])))));
eetrans[2]=((((-1.0)*x3))+((x1*x4)));
}

IKFAST_API int GetNumFreeParameters() { return 0; }
IKFAST_API int* GetFreeParameters() { return NULL; }
IKFAST_API int GetNumJoints() { return 3; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

IKFAST_API int GetIkType() { return 0x33000003; }

class IKSolver {
public:
IkReal j3,cj3,sj3,htj3,j3mul,j4,cj4,sj4,htj4,j4mul,j5,cj5,sj5,htj5,j5mul,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
unsigned char _ij3[2], _nj3,_ij4[2], _nj4,_ij5[2], _nj5;

IkReal j100, cj100, sj100;
unsigned char _ij100[2], _nj100;
bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
j3=numeric_limits<IkReal>::quiet_NaN(); _ij3[0] = -1; _ij3[1] = -1; _nj3 = -1; j4=numeric_limits<IkReal>::quiet_NaN(); _ij4[0] = -1; _ij4[1] = -1; _nj4 = -1; j5=numeric_limits<IkReal>::quiet_NaN(); _ij5[0] = -1; _ij5[1] = -1; _nj5 = -1; 
for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {
    solutions.Clear();
px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];

new_px=px;
new_py=((-1.0)*pz);
new_pz=((-0.127)+py);
px = new_px; py = new_py; pz = new_pz;
pp=((px*px)+(py*py)+(pz*pz));
{
IkReal verifyeval[1];
verifyeval[0]=((1.0)+(((-50.0)*(py*py)))+(((-50.0)*(pz*pz)))+(((-50.0)*(px*px))));
if( IKabs(verifyeval[0]) < 0.0000010000000000  )
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
cj4array[0]=((10.0)*pz);
if( cj4array[0] >= -1-IKFAST_SINCOS_THRESH && cj4array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j4valid[0] = j4valid[1] = true;
    j4array[0] = IKacos(cj4array[0]);
    sj4array[0] = IKsin(j4array[0]);
    cj4array[1] = cj4array[0];
    j4array[1] = -j4array[0];
    sj4array[1] = -sj4array[0];
}
else if( isnan(cj4array[0]) )
{
    // probably any value will work
    j4valid[0] = true;
    cj4array[0] = 1; sj4array[0] = 0; j4array[0] = 0;
}
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
IkReal j3eval[3];
IkReal x5=px*px;
IkReal x6=py*py;
j3eval[0]=(x6+x5);
j3eval[1]=((IKabs((py+((px*sj4)))))+(IKabs((px+(((-1.0)*py*sj4))))));
j3eval[2]=IKsign(((((10.0)*x5))+(((10.0)*x6))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  || IKabs(j3eval[2]) < 0.0000010000000000  )
{
{
IkReal j3eval[3];
IkReal x7=px*px;
IkReal x8=py*py;
IkReal x9=((10.0)*pz*sj4);
IkReal x10=(cj4*x8);
IkReal x11=(cj4*x7);
j3eval[0]=(x11+x10);
j3eval[1]=((IKabs((((cj4*px))+(((-1.0)*py*x9)))))+(IKabs((((cj4*py))+((px*x9))))));
j3eval[2]=IKsign(((((10.0)*x10))+(((10.0)*x11))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  || IKabs(j3eval[2]) < 0.0000010000000000  )
{
{
IkReal j3eval[3];
IkReal x12=px*px;
IkReal x13=py*py;
IkReal x14=((10.0)*cj4*pz);
IkReal x15=(sj4*x13);
IkReal x16=(sj4*x12);
j3eval[0]=(x15+x16);
j3eval[1]=IKsign(((((10.0)*x16))+(((10.0)*x15))));
j3eval[2]=((IKabs((((py*x14))+(((-1.0)*py))+((px*sj4)))))+(IKabs((px+(((-1.0)*px*x14))+((py*sj4))))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  || IKabs(j3eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[2];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j4))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j3eval[3];
sj4=0;
cj4=1.0;
j4=0;
IkReal x17=px*px;
IkReal x18=py*py;
j3eval[0]=(x17+x18);
j3eval[1]=((IKabs(px))+(IKabs(py)));
j3eval[2]=IKsign(((((10.0)*x17))+(((10.0)*x18))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  || IKabs(j3eval[2]) < 0.0000010000000000  )
{
continue; // 1 cases reached

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
CheckValue<IkReal> x19 = IKatan2WithCheck(IkReal(py),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x19.valid){
continue;
}
CheckValue<IkReal> x20=IKPowWithIntegerCheck(IKsign(((((10.0)*(px*px)))+(((10.0)*(py*py))))),-1);
if(!x20.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(x19.value)+(((1.5707963267949)*(x20.value))));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[2];
IkReal x21=IKsin(j3);
IkReal x22=IKcos(j3);
evalcond[0]=((-0.1)+((py*x21))+((px*x22)));
evalcond[1]=(((px*x21))+(((-1.0)*py*x22)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
j5array[0]=0;
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(3);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j3;
vinfos[0].indices[0] = _ij3[0];
vinfos[0].indices[1] = _ij3[1];
vinfos[0].maxsolutions = _nj3;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j4;
vinfos[1].indices[0] = _ij4[0];
vinfos[1].indices[1] = _ij4[1];
vinfos[1].maxsolutions = _nj4;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j5;
vinfos[2].indices[0] = _ij5[0];
vinfos[2].indices[1] = _ij5[1];
vinfos[2].maxsolutions = _nj5;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j3eval[3];
sj4=0;
cj4=-1.0;
j4=3.14159265358979;
IkReal x23=px*px;
IkReal x24=py*py;
j3eval[0]=(x24+x23);
j3eval[1]=((IKabs(px))+(IKabs(py)));
j3eval[2]=IKsign(((((10.0)*x23))+(((10.0)*x24))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  || IKabs(j3eval[2]) < 0.0000010000000000  )
{
continue; // 1 cases reached

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
CheckValue<IkReal> x25 = IKatan2WithCheck(IkReal(py),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x25.valid){
continue;
}
CheckValue<IkReal> x26=IKPowWithIntegerCheck(IKsign(((((10.0)*(px*px)))+(((10.0)*(py*py))))),-1);
if(!x26.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(x25.value)+(((1.5707963267949)*(x26.value))));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[2];
IkReal x27=IKsin(j3);
IkReal x28=IKcos(j3);
evalcond[0]=((-0.1)+((py*x27))+((px*x28)));
evalcond[1]=(((px*x27))+(((-1.0)*py*x28)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
j5array[0]=0;
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(3);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j3;
vinfos[0].indices[0] = _ij3[0];
vinfos[0].indices[1] = _ij3[1];
vinfos[0].maxsolutions = _nj3;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j4;
vinfos[1].indices[0] = _ij4[0];
vinfos[1].indices[1] = _ij4[1];
vinfos[1].maxsolutions = _nj4;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j5;
vinfos[2].indices[0] = _ij5[0];
vinfos[2].indices[1] = _ij5[1];
vinfos[2].maxsolutions = _nj5;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j3eval[2];
sj4=1.0;
cj4=0;
j4=1.5707963267949;
IkReal x29=px*px;
IkReal x30=py*py;
j3eval[0]=(x30+x29);
j3eval[1]=IKsign(((((10.0)*x30))+(((10.0)*x29))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  )
{
continue; // 1 cases reached

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
CheckValue<IkReal> x31 = IKatan2WithCheck(IkReal((py+px)),IkReal(((((-1.0)*py))+px)),IKFAST_ATAN2_MAGTHRESH);
if(!x31.valid){
continue;
}
CheckValue<IkReal> x32=IKPowWithIntegerCheck(IKsign(((((10.0)*(px*px)))+(((10.0)*(py*py))))),-1);
if(!x32.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(x31.value)+(((1.5707963267949)*(x32.value))));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[2];
IkReal x33=IKsin(j3);
IkReal x34=IKcos(j3);
evalcond[0]=((-0.1)+((py*x33))+((px*x34)));
evalcond[1]=((-0.1)+(((-1.0)*py*x34))+((px*x33)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
j5array[0]=0;
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(3);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j3;
vinfos[0].indices[0] = _ij3[0];
vinfos[0].indices[1] = _ij3[1];
vinfos[0].maxsolutions = _nj3;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j4;
vinfos[1].indices[0] = _ij4[0];
vinfos[1].indices[1] = _ij4[1];
vinfos[1].maxsolutions = _nj4;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j5;
vinfos[2].indices[0] = _ij5[0];
vinfos[2].indices[1] = _ij5[1];
vinfos[2].maxsolutions = _nj5;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j4)))), 6.28318530717959)));
evalcond[1]=pz;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j3eval[2];
sj4=-1.0;
cj4=0;
j4=-1.5707963267949;
IkReal x35=px*px;
IkReal x36=py*py;
j3eval[0]=(x36+x35);
j3eval[1]=IKsign(((((10.0)*x36))+(((10.0)*x35))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  )
{
continue; // 1 cases reached

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
CheckValue<IkReal> x37 = IKatan2WithCheck(IkReal(((((-1.0)*px))+py)),IkReal((py+px)),IKFAST_ATAN2_MAGTHRESH);
if(!x37.valid){
continue;
}
CheckValue<IkReal> x38=IKPowWithIntegerCheck(IKsign(((((10.0)*(px*px)))+(((10.0)*(py*py))))),-1);
if(!x38.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(x37.value)+(((1.5707963267949)*(x38.value))));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[2];
IkReal x39=IKsin(j3);
IkReal x40=IKcos(j3);
evalcond[0]=((-0.1)+((py*x39))+((px*x40)));
evalcond[1]=((0.1)+(((-1.0)*py*x40))+((px*x39)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
j5array[0]=0;
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(3);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j3;
vinfos[0].indices[0] = _ij3[0];
vinfos[0].indices[1] = _ij3[1];
vinfos[0].maxsolutions = _nj3;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j4;
vinfos[1].indices[0] = _ij4[0];
vinfos[1].indices[1] = _ij4[1];
vinfos[1].maxsolutions = _nj4;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j5;
vinfos[2].indices[0] = _ij5[0];
vinfos[2].indices[1] = _ij5[1];
vinfos[2].maxsolutions = _nj5;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j3, j5]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
IkReal x41=((10.0)*sj4);
IkReal x42=((10.0)*cj4*pz);
CheckValue<IkReal> x43 = IKatan2WithCheck(IkReal((px+((py*sj4))+(((-1.0)*px*x42)))),IkReal((((py*x42))+(((-1.0)*py))+((px*sj4)))),IKFAST_ATAN2_MAGTHRESH);
if(!x43.valid){
continue;
}
CheckValue<IkReal> x44=IKPowWithIntegerCheck(IKsign((((x41*(px*px)))+((x41*(py*py))))),-1);
if(!x44.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(x43.value)+(((1.5707963267949)*(x44.value))));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[4];
IkReal x45=IKsin(j3);
IkReal x46=IKcos(j3);
IkReal x47=((1.0)*pz);
IkReal x48=(py*x46);
IkReal x49=(px*x45);
evalcond[0]=((-0.1)+((py*x45))+((px*x46)));
evalcond[1]=((((-1.0)*x48))+x49+(((-0.1)*sj4)));
evalcond[2]=((((-1.0)*cj4*x48))+((cj4*x49))+(((-1.0)*sj4*x47)));
evalcond[3]=((0.1)+((sj4*x48))+(((-1.0)*sj4*x49))+(((-1.0)*cj4*x47)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
j5array[0]=0;
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(3);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j3;
vinfos[0].indices[0] = _ij3[0];
vinfos[0].indices[1] = _ij3[1];
vinfos[0].maxsolutions = _nj3;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j4;
vinfos[1].indices[0] = _ij4[0];
vinfos[1].indices[1] = _ij4[1];
vinfos[1].maxsolutions = _nj4;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j5;
vinfos[2].indices[0] = _ij5[0];
vinfos[2].indices[1] = _ij5[1];
vinfos[2].maxsolutions = _nj5;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}

}

}

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
IkReal x50=((10.0)*cj4);
IkReal x51=((10.0)*pz*sj4);
CheckValue<IkReal> x52=IKPowWithIntegerCheck(IKsign((((x50*(px*px)))+((x50*(py*py))))),-1);
if(!x52.valid){
continue;
}
CheckValue<IkReal> x53 = IKatan2WithCheck(IkReal((((cj4*py))+((px*x51)))),IkReal((((cj4*px))+(((-1.0)*py*x51)))),IKFAST_ATAN2_MAGTHRESH);
if(!x53.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(((1.5707963267949)*(x52.value)))+(x53.value));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[4];
IkReal x54=IKsin(j3);
IkReal x55=IKcos(j3);
IkReal x56=((1.0)*pz);
IkReal x57=(py*x55);
IkReal x58=(px*x54);
evalcond[0]=((-0.1)+((px*x55))+((py*x54)));
evalcond[1]=(x58+(((-0.1)*sj4))+(((-1.0)*x57)));
evalcond[2]=(((cj4*x58))+(((-1.0)*sj4*x56))+(((-1.0)*cj4*x57)));
evalcond[3]=((0.1)+(((-1.0)*sj4*x58))+(((-1.0)*cj4*x56))+((sj4*x57)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
j5array[0]=0;
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(3);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j3;
vinfos[0].indices[0] = _ij3[0];
vinfos[0].indices[1] = _ij3[1];
vinfos[0].maxsolutions = _nj3;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j4;
vinfos[1].indices[0] = _ij4[0];
vinfos[1].indices[1] = _ij4[1];
vinfos[1].maxsolutions = _nj4;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j5;
vinfos[2].indices[0] = _ij5[0];
vinfos[2].indices[1] = _ij5[1];
vinfos[2].maxsolutions = _nj5;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}

}

}

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
CheckValue<IkReal> x59=IKPowWithIntegerCheck(IKsign(((((10.0)*(px*px)))+(((10.0)*(py*py))))),-1);
if(!x59.valid){
continue;
}
CheckValue<IkReal> x60 = IKatan2WithCheck(IkReal((py+((px*sj4)))),IkReal((px+(((-1.0)*py*sj4)))),IKFAST_ATAN2_MAGTHRESH);
if(!x60.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(((1.5707963267949)*(x59.value)))+(x60.value));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[4];
IkReal x61=IKsin(j3);
IkReal x62=IKcos(j3);
IkReal x63=((1.0)*pz);
IkReal x64=(py*x62);
IkReal x65=(px*x61);
evalcond[0]=((-0.1)+((px*x62))+((py*x61)));
evalcond[1]=(x65+(((-0.1)*sj4))+(((-1.0)*x64)));
evalcond[2]=((((-1.0)*sj4*x63))+((cj4*x65))+(((-1.0)*cj4*x64)));
evalcond[3]=((0.1)+(((-1.0)*sj4*x65))+(((-1.0)*cj4*x63))+((sj4*x64)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
j5array[0]=0;
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(3);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j3;
vinfos[0].indices[0] = _ij3[0];
vinfos[0].indices[1] = _ij3[1];
vinfos[0].maxsolutions = _nj3;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j4;
vinfos[1].indices[0] = _ij4[0];
vinfos[1].indices[1] = _ij4[1];
vinfos[1].maxsolutions = _nj4;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j5;
vinfos[2].indices[0] = _ij5[0];
vinfos[2].indices[1] = _ij5[1];
vinfos[2].maxsolutions = _nj5;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}

}

}
}
}

} else
{
continue; // verifyAllEquations

}

}
}
return solutions.GetNumSolutions()>0;
}
};


/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API bool ComputeIk2(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions, void* pOpenRAVEManip) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API const char* GetKinematicsHash() { return "<robot:GenericRobot - ur5 (66dd9d12af076bc9c36b7f61851b85ae)>"; }

IKFAST_API const char* GetIkFastVersion() { return "0x1000004a"; }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif

#ifndef IKFAST_NO_MAIN
#include <stdio.h>
#include <stdlib.h>
#ifdef IKFAST_NAMESPACE
using namespace IKFAST_NAMESPACE;
#endif
int main(int argc, char** argv)
{
    if( argc != 12+GetNumFreeParameters()+1 ) {
        printf("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
               "Returns the ik solutions given the transformation of the end effector specified by\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
               "There are %d free parameters that have to be specified.\n\n",GetNumFreeParameters());
        return 1;
    }

    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    IkReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\n");
        return -1;
    }

    printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");
    }
    return 0;
}

#endif
