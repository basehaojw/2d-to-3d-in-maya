/* 
////////////////////////////////////////////////////////////////////////////////////
// 
//  Prototypes and definitions for the Levenberg - Marquardt minimization algorithm
//  Copyright (C) 2004  Manolis Lourakis (lourakis at ics forth gr)
//  Institute of Computer Science, Foundation for Research & Technology - Hellas
//  Heraklion, Crete, Greece.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
////////////////////////////////////////////////////////////////////////////////////
*/

#ifndef _LEVMAR_H_
#define _LEVMAR_H_

#ifndef _LEVMAR_LIB_INTERNAL_
	#ifdef _M_AMD64
		#ifdef _DEBUG
			#pragma comment(lib, "levmarDebugx64.lib")
		#else
			#pragma comment(lib, "levmarReleasex64.lib")
		#endif
	#else
		#ifdef _DEBUG
			#pragma comment(lib, "levmarDebugWin32.lib")
		#else
			#pragma comment(lib, "levmarReleaseWin32.lib")
		#endif
	#endif
#endif

/************************************* Start of configuration options *************************************/

/* specify whether to use LAPACK or not. The first option is strongly recommended */
#define HAVE_LAPACK /* use LAPACK */
/* #undef HAVE_LAPACK */  /* uncomment this to force not using LAPACK */

/* determine the precision variants to be build. Default settings build
 * both the single and double precision routines
 */
#define LM_DBL_PREC  /* comment this if you don't want the double precision routines to be compiled */
#define LM_SNGL_PREC /* comment this if you don't want the single precision routines to be compiled */

/****************** End of configuration options, no changes necessary beyond this point ******************/

#ifdef HAVE_LAPACK
	#define sorgqr  SORGQR
	#define strtri  STRTRI
	#define sgeqp3  SGEQP3
	#define dorgqr  DORGQR
	#define dtrtri  DTRTRI
	#define dgeqp3  DGEQP3
	#define sgemm   SGEMM 
	#define sgesvd  SGESVD
	#define spotf2  SPOTF2
	#define dgemm   DGEMM 
	#define dgesvd  DGESVD
	#define dpotf2  DPOTF2
	#define dtrtrs  DTRTRS
	#define dgeqrf  DGEQRF
	#define dpotrs  DPOTRS
	#define dpotrf  DPOTRF
	#define dgetrs  DGETRS
	#define dgetrf  DGETRF
	#define dsytrs  DSYTRS
	#define dsytrf  DSYTRF
	#define strtrs  STRTRS
	#define sgeqrf  SGEQRF
	#define spotrs  SPOTRS
	#define spotrf  SPOTRF
	#define sgetrs  SGETRS
	#define sgetrf  SGETRF
	#define ssytrs  SSYTRS
	#define ssytrf  SSYTRF
#endif


#ifdef __cplusplus
extern "C" {
#endif


#define FABS(x) (((x)>=0.0)? (x) : -(x))

/* work arrays size for ?levmar_der and ?levmar_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_DER_WORKSZ(npar, nmeas) (2*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))
#define LM_DIF_WORKSZ(npar, nmeas) (4*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))

/* work arrays size for ?levmar_bc_der and ?levmar_bc_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BC_DER_WORKSZ(npar, nmeas) (2*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))
#define LM_BC_DIF_WORKSZ(npar, nmeas) LM_BC_DER_WORKSZ((npar), (nmeas)) /* LEVMAR_BC_DIF currently implemented using LEVMAR_BC_DER()! */

/* work arrays size for ?levmar_lec_der and ?levmar_lec_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_LEC_DER_WORKSZ(npar, nmeas, nconstr) LM_DER_WORKSZ((npar)-(nconstr), (nmeas))
#define LM_LEC_DIF_WORKSZ(npar, nmeas, nconstr) LM_DIF_WORKSZ((npar)-(nconstr), (nmeas))

/* work arrays size for ?levmar_blec_der and ?levmar_blec_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BLEC_DER_WORKSZ(npar, nmeas, nconstr) LM_LEC_DER_WORKSZ((npar), (nmeas)+(npar), (nconstr))
#define LM_BLEC_DIF_WORKSZ(npar, nmeas, nconstr) LM_LEC_DIF_WORKSZ((npar), (nmeas)+(npar), (nconstr))

/* work arrays size for ?levmar_bleic_der and ?levmar_bleic_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BLEIC_DER_WORKSZ(npar, nmeas, nconstr1, nconstr2) LM_BLEC_DER_WORKSZ((npar)+(nconstr2), (nmeas)+(nconstr2), (nconstr1)+(nconstr2))
#define LM_BLEIC_DIF_WORKSZ(npar, nmeas, nconstr1, nconstr2) LM_BLEC_DIF_WORKSZ((npar)+(nconstr2), (nmeas)+(nconstr2), (nconstr1)+(nconstr2))

#define LM_OPTS_SZ    	 5 /* max(4, 5) */
#define LM_INFO_SZ    	 10
#define LM_ERROR         -1
#define LM_INIT_MU    	 1E-03
#define LM_STOP_THRESH	 1E-17
#define LM_DIFF_DELTA    1E-06
#define LM_VERSION       "2.5 (December 2009)"

#ifdef LM_DBL_PREC
/* double precision LM, with & without Jacobian */
/* unconstrained minimization */
	extern int dlevmar_der(
		void(*func)(double *p, double *hx, int m, int n, void *adata),
		void(*jacf)(double *p, double *j, int m, int n, void *adata),
		double *p, double *x, int m, int n, int itmax, double *opts,
		double *info, double *work, double *covar, void *adata);

extern int dlevmar_der_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, int itmax, double *opts,
	double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

extern int dlevmar_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, int itmax, double *opts,
      double *info, double *work, double *covar, void *adata);

extern int dlevmar_dif_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, int itmax, double *opts,
	double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

/* box-constrained minimization */
extern int dlevmar_bc_der(
       void (*func)(double *p, double *hx, int m, int n, void *adata),
       void (*jacf)(double *p, double *j, int m, int n, void *adata),  
       double *p, double *x, int m, int n, double *lb, double *ub,
	   int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_bc_der_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

extern int dlevmar_bc_dif(
       void (*func)(double *p, double *hx, int m, int n, void *adata),
       double *p, double *x, int m, int n, double *lb, double *ub,
	   int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_bc_dif_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

#ifdef HAVE_LAPACK
/* linear equation constrained minimization */
extern int dlevmar_lec_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *A, double *b, int k,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_lec_der_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *A, double *b, int k,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

extern int dlevmar_lec_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *A, double *b, int k,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_lec_dif_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *A, double *b, int k,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

/* box & linear equation constrained minimization */
extern int dlevmar_blec_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, double *A, double *b, int k, double *wghts,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_blec_der_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub, double *A, double *b, int k, double *wghts,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

extern int dlevmar_blec_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, double *A, double *b, int k, double *wghts,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_blec_dif_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub, double *A, double *b, int k, double *wghts,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

/* box, linear equations & inequalities constrained minimization */
extern int dlevmar_bleic_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub,
      double *A, double *b, int k1, double *C, double *d, int k2,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_bleic_der_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub,
	double *A, double *b, int k1, double *C, double *d, int k2,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

extern int dlevmar_bleic_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, 
      double *A, double *b, int k1, double *C, double *d, int k2,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_bleic_dif_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub,
	double *A, double *b, int k1, double *C, double *d, int k2,
	int itmax, double *opts, double *info, double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

/* box & linear inequality constraints */
extern int dlevmar_blic_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, double *C, double *d, int k2,
      int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_blic_der_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub, double *C, double *d, int k2,
	int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

extern int dlevmar_blic_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, double *C, double *d, int k2,
      int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_blic_dif_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *lb, double *ub, double *C, double *d, int k2,
	int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

/* linear equation & inequality constraints */
extern int dlevmar_leic_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *A, double *b, int k1, double *C, double *d, int k2,
      int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_leic_der_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *A, double *b, int k1, double *C, double *d, int k2,
	int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

extern int dlevmar_leic_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *A, double *b, int k1, double *C, double *d, int k2,
      int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_leic_dif_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *A, double *b, int k1, double *C, double *d, int k2,
	int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

/* linear inequality constraints */
extern int dlevmar_lic_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *C, double *d, int k2,
      int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_lic_der_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	void(*jacf)(double *p, double *j, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *C, double *d, int k2,
	int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);

extern int dlevmar_lic_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *C, double *d, int k2,
      int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_lic_dif_linbuf(
	void(*func)(double *p, double *hx, int m, int n, void *adata),
	double *p, double *x, int m, int n, double *C, double *d, int k2,
	int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata,
	double **linbuf, int *linbufsz);
#endif /* HAVE_LAPACK */

#endif /* LM_DBL_PREC */


#ifdef LM_SNGL_PREC
/* single precision LM, with & without Jacobian */
/* unconstrained minimization */
extern int slevmar_der(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      void (*jacf)(float *p, float *j, int m, int n, void *adata),
      float *p, float *x, int m, int n, int itmax, float *opts,
      float *info, float *work, float *covar, void *adata);

extern int slevmar_der_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, int itmax, float *opts,
	float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

extern int slevmar_dif(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, int itmax, float *opts,
	float *info, float *work, float *covar, void *adata);

extern int slevmar_dif_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, int itmax, float *opts,
	float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

/* box-constrained minimization */
extern int slevmar_bc_der(
       void (*func)(float *p, float *hx, int m, int n, void *adata),
       void (*jacf)(float *p, float *j, int m, int n, void *adata),  
       float *p, float *x, int m, int n, float *lb, float *ub,
       int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_bc_der_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

extern int slevmar_bc_dif(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_bc_dif_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

#ifdef HAVE_LAPACK
/* linear equation constrained minimization */
extern int slevmar_lec_der(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *A, float *b, int k,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_lec_der_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *A, float *b, int k,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

extern int slevmar_lec_dif(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *A, float *b, int k,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_lec_dif_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *A, float *b, int k,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

/* box & linear equation constrained minimization */
extern int slevmar_blec_der(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub, float *A, float *b, int k, float *wghts,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_blec_der_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub, float *A, float *b, int k, float *wghts,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

extern int slevmar_blec_dif(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub, float *A, float *b, int k, float *wghts,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_blec_dif_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub, float *A, float *b, int k, float *wghts,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

/* box, linear equations & inequalities constrained minimization */
extern int slevmar_bleic_der(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub,
	float *A, float *b, int k1, float *C, float *d, int k2,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_bleic_der_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub,
	float *A, float *b, int k1, float *C, float *d, int k2,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

extern int slevmar_bleic_dif(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub,
	float *A, float *b, int k1, float *C, float *d, int k2,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_bleic_dif_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub,
	float *A, float *b, int k1, float *C, float *d, int k2,
	int itmax, float *opts, float *info, float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

/* box & linear inequality constraints */
extern int slevmar_blic_der(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub, float *C, float *d, int k2,
	int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_blic_der_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub, float *C, float *d, int k2,
	int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

extern int slevmar_blic_dif(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub, float *C, float *d, int k2,
	int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_blic_dif_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *lb, float *ub, float *C, float *d, int k2,
	int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

/* linear equality & inequality constraints */
extern int slevmar_leic_der(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *A, float *b, int k1, float *C, float *d, int k2,
	int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_leic_der_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *A, float *b, int k1, float *C, float *d, int k2,
	int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

extern int slevmar_leic_dif(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *A, float *b, int k1, float *C, float *d, int k2,
	int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_leic_dif_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *A, float *b, int k1, float *C, float *d, int k2,
	int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

/* linear inequality constraints */
extern int slevmar_lic_der(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *C, float *d, int k2,
	int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_lic_der_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	void(*jacf)(float *p, float *j, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *C, float *d, int k2,
	int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);

extern int slevmar_lic_dif(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *C, float *d, int k2,
	int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_lic_dif_linbuf(
	void(*func)(float *p, float *hx, int m, int n, void *adata),
	float *p, float *x, int m, int n, float *C, float *d, int k2,
	int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata,
	float **linbuf, int *linbufsz);
#endif /* HAVE_LAPACK */

#endif /* LM_SNGL_PREC */

/* linear system solvers */
#ifdef HAVE_LAPACK

#ifdef LM_DBL_PREC
extern int dAx_eq_b_QR(double *A, double *B, double *x, int m, double **buf, int *buf_sz);
extern int dAx_eq_b_QRLS(double *A, double *B, double *x, int m, int n, double **buf, int *buf_sz);
extern int dAx_eq_b_Chol(double *A, double *B, double *x, int m, double **buf, int *buf_sz);
extern int dAx_eq_b_LU(double *A, double *B, double *x, int m, double **buf, int *buf_sz);
extern int dAx_eq_b_SVD(double *A, double *B, double *x, int m, double **buf, int *buf_sz);
extern int dAx_eq_b_BK(double *A, double *B, double *x, int m, double **buf, int *buf_sz);
#endif /* LM_DBL_PREC */

#ifdef LM_SNGL_PREC
extern int sAx_eq_b_QR(float *A, float *B, float *x, int m, float **buf, int *buf_sz);
extern int sAx_eq_b_QRLS(float *A, float *B, float *x, int m, int n, float **buf, int *buf_sz);
extern int sAx_eq_b_Chol(float *A, float *B, float *x, int m, float **buf, int *buf_sz);
extern int sAx_eq_b_LU(float *A, float *B, float *x, int m, float **buf, int *buf_sz);
extern int sAx_eq_b_SVD(float *A, float *B, float *x, int m, float **buf, int *buf_sz);
extern int sAx_eq_b_BK(float *A, float *B, float *x, int m, float **buf, int *buf_sz);
#endif /* LM_SNGL_PREC */

#else /* no LAPACK */

#ifdef LM_DBL_PREC
extern int dAx_eq_b_LU_noLapack(double *A, double *B, double *x, int n);
#endif /* LM_DBL_PREC */

#ifdef LM_SNGL_PREC
extern int sAx_eq_b_LU_noLapack(float *A, float *B, float *x, int n);
#endif /* LM_SNGL_PREC */

#endif /* HAVE_LAPACK */

/* Jacobian verification, double & single precision */
#ifdef LM_DBL_PREC
extern void dlevmar_chkjac(
    void (*func)(double *p, double *hx, int m, int n, void *adata),
    void (*jacf)(double *p, double *j, int m, int n, void *adata),
    double *p, int m, int n, void *adata, double *err);
#endif /* LM_DBL_PREC */

#ifdef LM_SNGL_PREC
extern void slevmar_chkjac(
    void (*func)(float *p, float *hx, int m, int n, void *adata),
    void (*jacf)(float *p, float *j, int m, int n, void *adata),
    float *p, int m, int n, void *adata, float *err);
#endif /* LM_SNGL_PREC */

/* standard deviation, coefficient of determination (R2) & Pearson's correlation coefficient for best-fit parameters */
#ifdef LM_DBL_PREC
extern double dlevmar_stddev( double *covar, int m, int i);
extern double dlevmar_corcoef(double *covar, int m, int i, int j);
extern double dlevmar_R2(void (*func)(double *p, double *hx, int m, int n, void *adata), double *p, double *x, int m, int n, void *adata);

#endif /* LM_DBL_PREC */

#ifdef LM_SNGL_PREC
extern float slevmar_stddev( float *covar, int m, int i);
extern float slevmar_corcoef(float *covar, int m, int i, int j);
extern float slevmar_R2(void (*func)(float *p, float *hx, int m, int n, void *adata), float *p, float *x, int m, int n, void *adata);
#endif /* LM_SNGL_PREC */

#ifdef __cplusplus
}
#endif

#endif /* _LEVMAR_H_ */
