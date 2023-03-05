#ifndef UT_CLAPACK_H
#define UT_CLAPACK_H

//#include <Springhead.h>


#define BOOST_NUMERIC_BINDINGS_USE_CLAPACK
#pragma warning(push)
#pragma warning(disable:4267)
#pragma warning(disable:4005)
#include <boost/numeric/ublas/fwd.hpp>
#include <boost/numeric/bindings/lapack/driver/sygv.hpp>
#include <boost/numeric/bindings/lapack/driver/sygvx.hpp>
#include <boost/numeric/bindings/lapack/driver/gesv.hpp>
#include <boost/numeric/bindings/lapack/driver/gels.hpp>
#include <boost/numeric/bindings/lapack/driver/gelsd.hpp>
#include <boost/numeric/bindings/lapack/driver/gesdd.hpp>
#include <boost/numeric/bindings/noop.hpp>
#include <boost/numeric/bindings/ublas/banded.hpp>
#include <boost/numeric/bindings/ublas/matrix.hpp>
#include <boost/numeric/bindings/ublas/matrix_proxy.hpp>
#include <boost/numeric/bindings/ublas/symmetric.hpp>
#include <boost/numeric/bindings/ublas/vector.hpp>
#include <boost/numeric/bindings/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>
#pragma warning(pop)


#define ublas boost::numeric::ublas
#define bindings boost::numeric::bindings
#define lapack bindings::lapack


#endif
#pragma once
