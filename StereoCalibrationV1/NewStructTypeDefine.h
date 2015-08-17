#ifndef NEW_STRUCT_TYPE_DEFINE
#define NEW_STRUCT_TYPE_DEFINE
#include <fstream>
#include <string>

#define SIZE_S 2
#define SQRT_SIZE_K 3
#define SIZE_K SQRT_SIZE_K * SQRT_SIZE_K
#define SIZE_D 5
#define SQRT_SIZE_R 3
#define SIZE_R SQRT_SIZE_R * SQRT_SIZE_R
#define SIZE_T 3
#define SIZE_P_ROWS 3
#define SIZE_P_COLS 4
#define SIZE_P SIZE_P_ROWS * SIZE_P_COLS
#define SIZE_TIME 64
#define SIZE_ROI 4

//struct for calibration result
struct Calib_Data_Type{
	std::string calib_time;

	double corner_dist;
	double S_00[SIZE_S];//raw image size; S_00: 1.392000e+03 5.120000e+02
	double K_00[SIZE_K];//raw camera intrinsic parameters : 9.842439e+02 0.000000e+00 6.900000e+02 0.000000e+00 9.808141e+02 2.331966e+02 0.000000e+00 0.000000e+00 1.000000e+00
	double D_00[SIZE_D];//distortion coefficients : -3.728755e-01 2.037299e-01 2.219027e-03 1.383707e-03 - 7.233722e-02
	double R_00[SIZE_R];// : 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00
	double T_00[SIZE_T];//translation with respect to the first camera : 2.573699e-16 - 1.059758e-16 1.614870e-16
	int ROI_00[SIZE_ROI];
	double S_rect_00[SIZE_S];// : 1.242000e+03 3.750000e+02
	double R_rect_00[SIZE_R];// : 9.999239e-01 9.837760e-03 - 7.445048e-03 - 9.869795e-03 9.999421e-01 - 4.278459e-03 7.402527e-03 4.351614e-03 9.999631e-01
	double P_rect_00[SIZE_P];//useful : 7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00

	double S_01[SIZE_S];//raw image size; S_00: 1.392000e+03 5.120000e+02
	double K_01[SIZE_K];//raw camera intrinsic parameters : 9.842439e+02 0.000000e+00 6.900000e+02 0.000000e+00 9.808141e+02 2.331966e+02 0.000000e+00 0.000000e+00 1.000000e+00
	double D_01[SIZE_D];//distortion coefficients : -3.728755e-01 2.037299e-01 2.219027e-03 1.383707e-03 - 7.233722e-02
	double R_01[SIZE_R];// : 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00
	double T_01[SIZE_T];// : -5.370000e-01 4.822061e-03 - 1.252488e-02
	int ROI_01[SIZE_ROI];
	double S_rect_01[SIZE_S];// : 1.242000e+03 3.750000e+02
	double R_rect_01[SIZE_R];// : 9.996878e-01 - 8.976826e-03 2.331651e-02 8.876121e-03 9.999508e-01 4.418952e-03 - 2.335503e-02 - 4.210612e-03 9.997184e-01
	double P_rect_01[SIZE_P];// : 7.215377e+02 0.000000e+00 6.095593e+02 - 3.875744e+02 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
	
public:
	friend std::ifstream& operator >> (std::ifstream &in, Calib_Data_Type &calib_data);
	friend std::ofstream& operator << (std::ofstream &of, const Calib_Data_Type &calib_data);
};

#endif
