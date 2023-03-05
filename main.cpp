#include "Base.h"

#include "UseBlockObjectDouble.h"

#include <stdio.h>

#include <stdlib.h>

#include <fenv.h>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>



std::vector<ParticleD*> Create_ParticlesD(Eigen::Vector3d origin, ObjectSize size_data);
std::vector<ParticleD*> Create_Particles_OneTetraD(Eigen::Vector3d origin, ObjectSize size_data);
std::vector<ParticleD*> Create_Particles_TwoTetraD(Eigen::Vector3d origin, ObjectSize size_data);
std::vector<ParticleD*> Create_Particles_Tri_prismD(Eigen::Vector3d origin, ObjectSize size_data);
void Draw_Mesh();
void Draw_Rotation(ObjectD* obj, float SinParam, float CosParam, float CameraVAngle, float CameraHAngle, double cameraZoom);
void Draw_Group_Grid(ObjectD* obj, float SinParam, float CosParam, float CameraVAngle, float CameraHAngle, double cameraZoom);
Eigen::Vector3d Calc_Draw_Grid(Eigen::Vector3d a, float SinParam, float CosParam, float CameraVAngle, float CameraHAngle, double cameraZoom);

// (x,y)の点を(mx,my)を中心にang角回転する
void rotate(float* x, float* y, const float ang, const float mx, const float my) {
	const float ox = *x - mx, oy = *y - my;
	*x = ox * cos(ang) + oy * sin(ang);
	*y = -ox * sin(ang) + oy * cos(ang);
	*x += mx;
	*y += my;
}
double MOUSE_RANGE;
double TIME_STEP;  //秒, second(リアルタイム時間なら0.03ぐらい)
double THRESHHOLD; // 1.0e+8;
int PBD_LOOP;      //制約条件における更新式(int)
double config;	  //修正値和による閾値の設定

float Gravity;  //重力加速度(m/s2)
bool useCRS;    //CRS(1)か昔の実装(0)か
bool mdiag;    //質量行列が対角(1)か否か(0)
int mSys; //質量行列がモデル全体の平均か(0)グループ別か(1)グループ和か(2)
bool loopite;    //更新数を決め打ち(1)か否か(0)
bool rollcamera;   //カメラが回転する(1)か否か(0)
bool fixedion;   //節点を固定する(1)か否か(0)
bool whichmethodused;//差分で反復する(1)かしないか(0)
bool useGMRES;//反復計算を非定常反復(GMRESなど)で解くか(1)かLUか(0)
int innerGMRES;//GMRESの内部反復の数(restart)
int outerGMRES;//GMRESの外部反復の数
bool usePreIte;//GMRESなどの反復法で前処理をするか(1)しないか(0)
bool useSparse; //行列をSparseで計算するか(1)しないか(0)
bool useUpRotate;//回転行列を更新するか(1)しないか(0)
bool useAPD;//回転行列をAPDで更新するか(1)しないか(0)
bool useUpBind;//拘束力を更新するか(1)しないか(0)
bool UseIniForce;//最初に力をかけるか(1)いなか(0)
int howmanytetras; //物体の形、箱型(3以上)、四面体一つ(1)、四面体二つ(2)
double M_damping;	  //バネダンパ係数(M,alpha)
double K_damping;	  //バネダンパ係数(K,Beta)
int dividedbyn;//x方向をn分割
int dividedbym;//y方向をm分割
int dividedbyl;//z方向をl分割　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　

double F_bind_coeff;//拘束力の係数
double F_bind_damping;//拘束力のダンパー係数

int Tri_prismnum;//三角柱の長さ
int GroupDividedby;//ブロックごとに切り分ける　

unsigned int xsize;	  //x方向のサイズ
unsigned int ysize;	  //y方向のサイズ
unsigned int zsize;	  //z方向のサイズ
double sidelength;	//一辺の長さ
double cipher;//スケーリング用（描画）
// カメラの回転速度
#define CAMERA_ANGLE_SPEED		2.0
 // カメラの注視点の高さ
#define CAMERA_LOOK_AT_HEIGHT	400.0f
 // カメラと注視点の距離
#define CAMERA_LOOK_AT_DISTANCE	2150.0f
 // ラインを描く範囲
#define LINE_AREA_SIZE	10000.0f

// ラインの数
#define LINE_NUM 500

// Dataから数値を読む(Read numbers from Data.txt)
void BasicInformation() {

	std::ifstream ifs("Data.txt");
	std::string line;

	if (ifs.fail()) {
		std::cerr << "File Open Error" << std::endl;
	}
	while (getline(ifs, line)) {
		if (line[0] == '#')
			continue;
		if (line.find('=') == std::string::npos)
			continue;
		std::stringstream ss(line);
		std::string name;
		ss >> name;
		ss.ignore(line.size(), '=');
		std::cout << name << " = ";
		float temp;
		ss >> temp;
		std::cout << temp << ",";
		if (name == "MOUSE_RANGE")
			MOUSE_RANGE = temp;
		else if (name == "TIME_STEP")
			TIME_STEP = temp;
		else if (name == "THRESHHOLD")
			THRESHHOLD = temp;
		else if (name == "PBD_LOOP")
			PBD_LOOP = int(temp);
		else if (name == "config")
			config = temp;
		else if (name == "Gravity")
			Gravity = temp;
		else if (name == "useCRS")
			if (temp)  useCRS = TRUE;
			else  useCRS = FALSE;
		else if (name == "mdiag")
			if (temp)  mdiag = TRUE;
			else  mdiag = FALSE;
		else if (name == "mSys")
			mSys = int(temp);
		else if (name == "loopite")
			if (temp)  loopite = TRUE;
			else  loopite = FALSE;
		else if (name == "rollcamera")
			if (temp)  rollcamera = TRUE;
			else  rollcamera = FALSE;
		else if (name == "fixedion")
			if (temp)  fixedion = TRUE;
			else  fixedion = FALSE;
		else if (name == "howmanytetras")
			howmanytetras = int(temp);
		else if (name == "M_damping")
			M_damping = temp;
		else if (name == "K_damping")
			K_damping = temp;
		else if (name == "F_bind_coeff")
			F_bind_coeff = temp;
		else if (name == "F_bind_damping")
			F_bind_damping = temp;
		else if (name == "whichmethodused")
			whichmethodused = temp;
		else if (name == "useGMRES")
			useGMRES = temp;
		else if (name == "innerGMRES")
			innerGMRES = int(temp);
		else if (name == "outerGMRES")
			outerGMRES = int(temp);
		else if (name == "usePreIte")
			usePreIte = temp;
		else if (name == "useSparse")
			useSparse = temp;
		else if (name == "useUpRotate")
			useUpRotate = temp;
		else if (name == "useAPD")
			useAPD = temp;
		else if (name == "useUpBind")
			useUpBind = temp;
		else if (name == "UseIniForce")
			UseIniForce = temp;
		else if (name == "xsize")
			xsize = int(temp);
		else if (name == "ysize")
			ysize = int(temp);
		else if (name == "zsize")
			zsize = int(temp);
		else if (name == "sidelength")
			sidelength = double(temp);
		else if (name == "dividedbyn")
			dividedbyn = int(temp);
		else if (name == "dividedbym")
			dividedbym = int(temp);
		else if (name == "dividedbyl")
			dividedbyl = int(temp);
		else if (name == "Tri_prismnum")
			Tri_prismnum = int(temp);
		else if (name == "GroupDividedby")
			GroupDividedby = int(temp);
	}
	std::cout << std::endl;
}
//===========================================================================//
//===========================================================================//
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	AllocConsole();
	freopen("CONOUT$", "w", stdout);


	// Dataから数値を読む
	BasicInformation();

	//	X+が右、Y+が下、Z+が手前
	std::vector<ObjectD*> obj;									//シミュレーションで生成するオブジェクト群
	ObjectSize size_data = { xsize, ysize, zsize ,sidelength };	//	モデルの大きさ(x,y,z方向), 1辺の長さ
	ObjectData almi = { 2.7e+03, 6.9e+10, 0.3, size_data };	    //	アルミの（密度、ヤング率、ポアソン比）
	ObjectData almin = { 2.7e+03, 1.0e+7, 0.49, size_data };	//	ゴムの（密度、ヤング率、ポアソン比）
	ObjectData gum = { 0.91e+03, 1.0e+06, 0.49, size_data };	//	ゴムの（密度、ヤング率、ポアソン比）
	ObjectData gum2 = { 0.91e+03, 1.0e+07, 0.49, size_data };	//	堅いゴムの（密度、ヤング率、ポアソン比）
	ObjectData gum3 = { 0.91e+03, 1.0e+08, 0.49, size_data };	//	堅いゴムの（密度、ヤング率、ポアソン比）
	ObjectData orihar = { 2.7e-03, 6.9e+12, 0.3, size_data };	//	幻想物体オリハルコンの（密度、ヤング率、ポアソン比）

	if (sidelength >= 40.0) {
		cipher = 40.0 / sidelength;
	}
	else if (sidelength > 1) {
		cipher = 40.0 / sidelength;
	}
	else {
		cipher = 40.0 / sidelength;
	}
	// オブジェクトの頂点群
	std::vector<ParticleD*> particles;
	//オブジェクトのインスタンス
	ObjectD* o;

	particles = Create_ParticlesD(Eigen::Vector3d(0.0, 0.0, 0.0), size_data);
	//Vector3dが原点で、右下奥に直方体ができる。
	//オブジェクトのインスタンスを生成する
	//o = new UseBlockObjectDouble(particles, gum);  // モデルを従来のFEMでシミュレーションする
	o = new UseBlockObjectDouble(particles, gum3);

	obj.push_back(o);// 生成したオブジェクトをシミュレーションで使うオブジェクト群にpushする


	//事前計算の終了
	//End of precomputation
	std::cout << "Let's start!! " << std::endl;

	//===========================================================================//
	//描画の設定（Drawing Settings）

	//===========================================================================//
	//実行時処理(run-time processing)
	//===========================================================================//

	while (1) {

		//===========================================================================//
		// 実行時計算(run-time calculation)
		//===========================================================================//
		for (unsigned int i = 0; i < obj.size(); ++i) {	//現在,オブジェクトの数は一つ

			obj[i]->Timestep_Init(); //1ステップの外力をリセット

			if (countup > 50 && countup < 100000) {
				obj[i]->Update();			//オブジェクトの位置更新
			}

			countup++;						 // 経過したステップ数を記録する
			if (countup < 50) {				 // 50ステップのときの総計算時間を測定
				TIMERof50 += mtUpdate.getDt();// 1ステップ中の位置更新の計算時間を取得し加算
			}

			//実験用の変数
			int countup2 = 0;
			if (countup > 50) {
				countup2 = countup;
			}
			if (countup > 100000) {
				countup2 = 100000;
			}
			//実験用の変数（終了）


		}

	}
	return 0;
}

//===========================================================================//
//size_dataから頂点の集合を作成(Create a set of vertices from size_data)
//===========================================================================//

//Nodes for a box model
std::vector<ParticleD*> Create_ParticlesD(Eigen::Vector3d origin, ObjectSize size_data) {
	std::vector<ParticleD*> particles;//オブジェクトの頂点群
	unsigned int num = size_data.x_vertex_num * size_data.y_vertex_num * size_data.z_vertex_num; //頂点の数は各軸の要素の積 xyz
	std::vector<ParticleD*> p(num);//頂点の集合
	std::vector<Eigen::Vector3d> phys;//頂点の各座標
	for (unsigned int xi = 0; xi < size_data.x_vertex_num; ++xi) {
		for (unsigned int yi = 0; yi < size_data.y_vertex_num; ++yi) {
			for (unsigned int zi = 0; zi < size_data.z_vertex_num; ++zi) {
				phys.push_back(Eigen::Vector3d(origin.x() + xi * size_data.size, origin.y() + yi * size_data.size, origin.z() + zi * size_data.size));
			}
		}
	}
	//各頂点にidと座標を入れる
	for (unsigned int i = 0; i < num; i++) {
		p[i] = new ParticleD(phys[i]);
		p[i]->p_id = i;
		particles.push_back(p[i]);
		if (i < size_data.y_vertex_num * size_data.z_vertex_num) {
			p[i]->Set_Fixed(fixedion);// 一番端の頂点を固定する
		}
	}
	std::cout << "success create particle" << ":" << num << std::endl;//頂点の作成に成功したことを出力
	return particles;
}
//Nodes for one Tetra model
std::vector<ParticleD*> Create_Particles_OneTetraD(Eigen::Vector3d origin, ObjectSize size_data) {
	std::vector<ParticleD*> particles;//オブジェクトの頂点群
	unsigned int num = 4; //頂点の数は4で決め打ち
	std::vector<ParticleD*> p(num);//頂点の集合
	std::vector<Eigen::Vector3d> phys;//頂点の各座標
	phys.push_back(Eigen::Vector3d(origin.x(), origin.y(), origin.z()));
	phys.push_back(Eigen::Vector3d(origin.x() + size_data.size, origin.y(), origin.z()));
	phys.push_back(Eigen::Vector3d(origin.x() + size_data.size * 0.5, origin.y() + size_data.size * sqrt(3.0) * 0.5, origin.z()));
	phys.push_back(Eigen::Vector3d(origin.x() + size_data.size * 0.5, origin.y() + size_data.size * sqrt(3.0) * 0.5 / 3.0, origin.z() + size_data.size * sqrt(6.0) / 3.0));
	//各頂点にidと座標を入れる
	for (unsigned int i = 0; i < num; i++) {
		p[i] = new ParticleD(phys[i]);
		p[i]->p_id = i;
		particles.push_back(p[i]);
		if (i != 3) {
			p[i]->Set_Fixed(fixedion);// 一番端の頂点を固定する
		}
		std::cout << i << "particle is " << std::endl;
		std::cout << p[i]->Get_Grid() << std::endl;
	}
	//初期速度代入
	//particles[3]->Update_Velocity(Eigen::Vector3d(0.0, 1.0, 0.0));	
	std::cout << "success create particle" << ":" << num << std::endl;//頂点の作成に成功したことを出力
	return particles;
}
//Nodes for two Tetra model
std::vector<ParticleD*> Create_Particles_TwoTetraD(Eigen::Vector3d origin, ObjectSize size_data) {
	std::vector<ParticleD*> particles;//オブジェクトの頂点群
	unsigned int num = 5; //頂点の数は5で決め打ち
	std::vector<ParticleD*> p(num);//頂点の集合
	std::vector<Eigen::Vector3d> phys;//頂点の各座標
	phys.push_back(Eigen::Vector3d(origin.x(), origin.y(), origin.z()));
	phys.push_back(Eigen::Vector3d(origin.x() + size_data.size, origin.y(), origin.z()));
	phys.push_back(Eigen::Vector3d(origin.x() + size_data.size * 0.5, origin.y() + size_data.size * sqrt(3.0) * 0.5, origin.z()));
	phys.push_back(Eigen::Vector3d(origin.x() + size_data.size * 0.5, origin.y() + size_data.size * sqrt(3.0) * 0.5 / 3.0, origin.z() + size_data.size * sqrt(6.0) / 3.0));
	phys.push_back(Eigen::Vector3d(origin.x() + size_data.size * 0.5, origin.y() - size_data.size * sqrt(3.0) * 0.5 / 3.0, origin.z() + size_data.size * sqrt(6.0) / 3.0));
	//各頂点にidと座標を入れる
	for (unsigned int i = 0; i < num; i++) {
		p[i] = new ParticleD(phys[i]);
		p[i]->p_id = i;
		particles.push_back(p[i]);
		if (i <= 2) {
			p[i]->Set_Fixed(fixedion);// 一番端の頂点を固定する
		}
		//if (i==4) {
			//p[i]->Set_Fixed(fixedion);
		//}
		std::cout << i << "particle is " << std::endl;
		std::cout << p[i]->Get_Grid() << std::endl;
	}
	//初期速度代入
	//particles[3]->Update_Velocity(Eigen::Vector3d(0.0, 1.0, 0.0));	
	std::cout << "success create particle of Two Tetra" << ":" << num << std::endl;//頂点の作成に成功したことを出力
	return particles;
}
//Nodes for Triangular columns model
std::vector<ParticleD*> Create_Particles_Tri_prismD(Eigen::Vector3d origin, ObjectSize size_data) {
	std::vector<ParticleD*> particles;//オブジェクトの頂点群
	unsigned int num = 3 * (Tri_prismnum + 1); //頂点の数
	//std::cout << "Tri_particle = " << num << std::endl;
	std::vector<ParticleD*> p(num);//頂点の集合
	std::vector<Eigen::Vector3d> phys;//頂点の各座標
	for (int i = 0; i < Tri_prismnum + 1; i++) {
		phys.push_back(Eigen::Vector3d(origin.x(), origin.y(), origin.z() + size_data.size * i));
		phys.push_back(Eigen::Vector3d(origin.x() + size_data.size * sqrt(3.0) * 0.5, origin.y() + size_data.size * 0.5, origin.z() + size_data.size * i));
		phys.push_back(Eigen::Vector3d(origin.x(), origin.y() + size_data.size, origin.z() + size_data.size * i));
	}
	//各頂点にidと座標を入れる
	for (unsigned int i = 0; i < num; i++) {
		p[i] = new ParticleD(phys[i]);
		p[i]->p_id = i;
		particles.push_back(p[i]);
		if (i <= 2) {
			p[i]->Set_Fixed(fixedion);// 一番端の頂点3つを固定する
		}
		std::cout << i << "particle is " << std::endl;
		std::cout << p[i]->Get_Grid() << std::endl;
	}
	//初期速度代入
	//particles[14]->Update_Velocity(Eigen::Vector3d(0.0, 100.0, 0.0));	
	std::cout << "success create particle of Tri prism" << ":" << num << std::endl;//頂点の作成に成功したことを出力
	return particles;
}


//===========================================================================//
//節点の描画用の関数(Functions for drawing nodes.)
//===========================================================================//

//Draw_Mesh

