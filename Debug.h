#ifndef _MYDEBUG
#define _MYDEBUG

#define _DEBUGMODE
#include "Base.h"
#define _CREATE_GROUP

#pragma warning(disable : 4996)

static void PrintV(const Eigen::VectorXf& v){
	std::cout << v << std::endl << std::endl;
}
static void PrintM(const Eigen::MatrixXf& m){
	std::cout << m << std::endl << std::endl;
}

template<typename T>
static void P(T t){
	std::cout << t << std::endl;
}
#endif