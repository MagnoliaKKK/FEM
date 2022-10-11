#include "blaswrap.h"
#include "f2c.h"

/* Subroutine */ int sorgrq_(integer *m, integer *n, integer *k, real *a, 
	integer *lda, real *tau, real *work, integer *lwork, integer *info)
{
/*  -- LAPACK routine (version 3.1) --   
       Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd..   
       November 2006   


    Purpose   
    =======   

    SORGRQ generates an M-by-N real matrix Q with orthonormal rows,   
    which is defined as the last M rows of a product of K elementary   
    reflectors of order N   

          Q  =  H(1) H(2) . . . H(k)   

    as returned by SGERQF.   

    Arguments   
    =========   

    M       (input) INTEGER   
            The number of rows of the matrix Q. M >= 0.   

    N       (input) INTEGER   
            The number of columns of the matrix Q. N >= M.   

    K       (input) INTEGER   
            The number of elementary reflectors whose product defines the   
            matrix Q. M >= K >= 0.   

    A       (input/output) REAL array, dimension (LDA,N)   
            On entry, the (m-k+i)-th row must contain the vector which   
            defines the elementary reflector H(i), for i = 1,2,...,k, as   
            returned by SGERQF in the last k rows of its array argument   
            A.   
            On exit, the M-by-N matrix Q.   

    LDA     (input) INTEGER   
            The first dimension of the array A. LDA >= max(1,M).   

    TAU     (input) REAL array, dimension (K)   
            TAU(i) must contain the scalar factor of the elementary   
            reflector H(i), as returned by SGERQF.   

    WORK    (workspace/output) REAL array, dimension (MAX(1,LWORK))   
            On exit, if INFO = 0, WORK(1) returns the optimal LWORK.   

    LWORK   (input) INTEGER   
            The dimension of the array WORK. LWORK >= max(1,M).   
            For optimum performance LWORK >= M*NB, where NB is the   
            optimal blocksize.   

            If LWORK = -1, then a workspace query is assumed; the routine   
            only calculates the optimal size of the WORK array, returns   
            this value as the first entry of the WORK array, and no error   
            message related to LWORK is issued by XERBLA.   

    INFO    (output) INTEGER   
            = 0:  successful exit   
            < 0:  if INFO = -i, the i-th argument has an illegal value   

    =====================================================================   


       Test the input arguments   

       Parameter adjustments */
    /* Table of constant values */
    static integer c__1 = 1;
    static integer c_n1 = -1;
    static integer c__3 = 3;
    static integer c__2 = 2;
    
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2, i__3, i__4;
    /* Local variables */
    static integer i__, j, l, ib, nb, ii, kk, nx, iws, nbmin, iinfo;
    extern /* Subroutine */ int sorgr2_(integer *, integer *, integer *, real 
	    *, integer *, real *, real *, integer *), slarfb_(char *, char *, 
	    char *, char *, integer *, integer *, integer *, real *, integer *
	    , real *, integer *, real *, integer *, real *, integer *),
            xerbla_(char *, integer *);
    extern integer ilaenv_(integer *, char *, char *, integer *, integer *, 
	    integer *, integer *, ftnlen, ftnlen);
    extern /* Subroutine */ int slarft_(char *, char *, integer *, integer *, 
	    real *, integer *, real *, real *, integer *);
    static integer ldwork, lwkopt;
    static logical lquery;


    a_dim1 = *lda;
    a_offset = 1 + a_dim1;
    a -= a_offset;
    --tau;
    --work;

    /* Function Body */
    *info = 0;
    lquery = *lwork == -1;
    if (*m < 0) {
	*info = -1;
    } else if (*n < *m) {
	*info = -2;
    } else if (*k < 0 || *k > *m) {
	*info = -3;
    } else if (*lda < max(1,*m)) {
	*info = -5;
    }

    if (*info == 0) {
	if (*m <= 0) {
	    lwkopt = 1;
	} else {
	    nb = ilaenv_(&c__1, "SORGRQ", " ", m, n, k, &c_n1, (ftnlen)6, (
		    ftnlen)1);
	    lwkopt = *m * nb;
	}
	work[1] = (real) lwkopt;

	if (*lwork < max(1,*m) && ! lquery) {
	    *info = -8;
	}
    }

    if (*info != 0) {
	i__1 = -(*info);
	xerbla_("SORGRQ", &i__1);
	return 0;
    } else if (lquery) {
	return 0;
    }

/*     Quick return if possible */

    if (*m <= 0) {
	return 0;
    }

    nbmin = 2;
    nx = 0;
    iws = *m;
    if (nb > 1 && nb < *k) {

/*        Determine when to cross over from blocked to unblocked code.   

   Computing MAX */
	i__1 = 0, i__2 = ilaenv_(&c__3, "SORGRQ", " ", m, n, k, &c_n1, (
		ftnlen)6, (ftnlen)1);
	nx = max(i__1,i__2);
	if (nx < *k) {

/*           Determine if workspace is large enough for blocked code. */

	    ldwork = *m;
	    iws = ldwork * nb;
	    if (*lwork < iws) {

/*              Not enough workspace to use optimal NB:  reduce NB and   
                determine the minimum value of NB. */

		nb = *lwork / ldwork;
/* Computing MAX */
		i__1 = 2, i__2 = ilaenv_(&c__2, "SORGRQ", " ", m, n, k, &c_n1,
			 (ftnlen)6, (ftnlen)1);
		nbmin = max(i__1,i__2);
	    }
	}
    }

    if (nb >= nbmin && nb < *k && nx < *k) {

/*        Use blocked code after the first block.   
          The last kk rows are handled by the block method.   

   Computing MIN */
	i__1 = *k, i__2 = (*k - nx + nb - 1) / nb * nb;
	kk = min(i__1,i__2);

/*        Set A(1:m-kk,n-kk+1:n) to zero. */

	i__1 = *n;
	for (j = *n - kk + 1; j <= i__1; ++j) {
	    i__2 = *m - kk;
	    for (i__ = 1; i__ <= i__2; ++i__) {
		a[i__ + j * a_dim1] = 0.f;
/* L10: */
	    }
/* L20: */
	}
    } else {
	kk = 0;
    }

/*     Use unblocked code for the first or only block. */

    i__1 = *m - kk;
    i__2 = *n - kk;
    i__3 = *k - kk;
    sorgr2_(&i__1, &i__2, &i__3, &a[a_offset], lda, &tau[1], &work[1], &iinfo)
	    ;

    if (kk > 0) {

/*        Use blocked code */

	i__1 = *k;
	i__2 = nb;
	for (i__ = *k - kk + 1; i__2 < 0 ? i__ >= i__1 : i__ <= i__1; i__ += 
		i__2) {
/* Computing MIN */
	    i__3 = nb, i__4 = *k - i__ + 1;
	    ib = min(i__3,i__4);
	    ii = *m - *k + i__;
	    if (ii > 1) {

/*              Form the triangular factor of the block reflector   
                H = H(i+ib-1) . . . H(i+1) H(i) */

		i__3 = *n - *k + i__ + ib - 1;
		slarft_("Backward", "Rowwise", &i__3, &ib, &a[ii + a_dim1], 
			lda, &tau[i__], &work[1], &ldwork);

/*              Apply H' to A(1:m-k+i-1,1:n-k+i+ib-1) from the right */

		i__3 = ii - 1;
		i__4 = *n - *k + i__ + ib - 1;
		slarfb_("Right", "Transpose", "Backward", "Rowwise", &i__3, &
			i__4, &ib, &a[ii + a_dim1], lda, &work[1], &ldwork, &
			a[a_offset], lda, &work[ib + 1], &ldwork);
	    }

/*           Apply H' to columns 1:n-k+i+ib-1 of current block */

	    i__3 = *n - *k + i__ + ib - 1;
	    sorgr2_(&ib, &i__3, &ib, &a[ii + a_dim1], lda, &tau[i__], &work[1]
		    , &iinfo);

/*           Set columns n-k+i+ib:n of current block to zero */

	    i__3 = *n;
	    for (l = *n - *k + i__ + ib; l <= i__3; ++l) {
		i__4 = ii + ib - 1;
		for (j = ii; j <= i__4; ++j) {
		    a[j + l * a_dim1] = 0.f;
/* L30: */
		}
/* L40: */
	    }
/* L50: */
	}
    }

    work[1] = (real) iws;
    return 0;

/*     End of SORGRQ */

} /* sorgrq_ */