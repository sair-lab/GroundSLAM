#include <math.h>
#include <opencv2/imgproc.hpp>
#include "correlation_flow.h"
#include "read_configs.h"
#include "circ_shift.h"
#include "optimization_2d/pose_graph_2d_error_term.h"
#include <iostream>
#include <fstream>
using namespace std;

#include "unistd.h"

#include <chrono>
using namespace std::chrono;

// void PrintArrayToFile (const Eigen::ArrayXXf target, int rows, int cols) {
void PrintArrayToFile (const auto& target, int rows, int cols) {

  ofstream myfile ("example.txt");
  if (myfile.is_open())
  {
    // myfile << "This is a line.\n";
    // myfile << "This is another line.\n";
    for(int count_row = 0; count_row < rows; count_row ++){
        for(int count_col = 0; count_col < cols; count_col ++){
            myfile << target(count_row, count_col) << " " ;
            if(count_col == cols - 1){
                myfile << "\n";
            }
        }
        
    }
    myfile.close();
  }
  else std::cout << "Unable to open file";

}
CorrelationFlow::CorrelationFlow(CFConfig& cf_config, double &image_height, double &image_width):cfg(cf_config)
{

    cfg.height = int(image_height);
    cfg.width = int(image_width);
    target_fft = GetTargetFFT(cfg.height, cfg.width);
    // cout << "cfg.height:" << cfg.height << endl;
    // cout << "cfg.width:" << cfg.width << endl;
    // cout << "target_fft:" << target_fft.size() << endl;
    // auto start = high_resolution_clock::now();

    target_rotation_fft = GetTargetFFT(cfg.rotation_divisor, cfg.rotation_channel);
    // PrintArrayToFile(target_rotation_fft, target_rotation_fft.rows(), target_rotation_fft.cols());
    // auto stop = high_resolution_clock::now();
    // auto duration = duration_cast<microseconds>(stop - start);
    // cout << "fft processing time is " << duration.count() << "microseconds" << endl;
    // sleep(0.8);
    // cout << "target_rotation_fft:" << target_rotation_fft.size() << endl;
}

Eigen::ArrayXXcf CorrelationFlow::GetTargetFFT(int rows, int cols)
{
    Eigen::ArrayXXf target = Eigen::ArrayXXf::Zero(rows, cols);
    target(rows/2, cols/2) = 1;
    // PrintArrayToFile(target, target.rows(), target.cols());
    // PrintArrayToFile(target, rows, cols);
    // std::cout << "have been written" <<std::endl;
    // sleep(1000);
    return FFT(target);
}

Eigen::ArrayXXcf CorrelationFlow::FFT(const Eigen::ArrayXXf& x)
{
    // here x is undistort and normalized image matrix
    // cout << x.rows() << endl; // 448
    // cout << x.cols() << endl; // 448
    // sleep(1000);
    Eigen::ArrayXXcf xf = Eigen::ArrayXXcf(x.rows()/2+1, x.cols()); // xf [225, 448]
    // cout << "xf:" << xf.rows() << endl;
    // cout << "xf:" << xf.cols() << endl;
    // sleep(1000);
    // cout << "fft_plan:" << typeid((float(*))(xf.data())).name() << endl;
    // cout << "fft_plan:" << typeid((float(*)[2])(xf.data())).name() << endl;
    // sleep(1000);
    fftwf_plan fft_plan = fftwf_plan_dft_r2c_2d(x.cols(), x.rows(), (float(*))(x.data()),
        (float(*)[2])(xf.data()), FFTW_ESTIMATE);
        // (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major
    // int i, j;
    // for(i=0;i<100;++i){
    //     for(j=0;j<100;++j){
    //         cout << "xf" << xf(i, j) << endl;
    //     }
    // }
    // cout << "xf" << xf(100,100) << endl;


    fftwf_execute(fft_plan);
    // cout << "xf" << xf(100,100) << endl;
    // sleep(1000);
    fftwf_destroy_plan(fft_plan);
    fftw_cleanup();
    return xf;
}

Eigen::ArrayXXf CorrelationFlow::IFFT(const Eigen::ArrayXXcf& xf)
{
    Eigen::ArrayXXf x = Eigen::ArrayXXf((xf.rows()-1)*2, xf.cols());
    Eigen::ArrayXXcf cxf = xf;
    fftwf_plan fft_plan = fftwf_plan_dft_c2r_2d(xf.cols(), (xf.rows()-1)*2, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    
    fftwf_execute(fft_plan);
    fftwf_destroy_plan(fft_plan);
    fftw_cleanup();
    return x/x.size();
}

inline Eigen::ArrayXXf CorrelationFlow::RemoveZeroComponent(const Eigen::ArrayXXf& x)
{
    Eigen::ArrayXXf y(x);
    unsigned int cols = x.cols();
    unsigned int rows = x.rows();
    y.block(0, 0, 1, cols) = (x.block(1, 0, 1, cols) + x.block(rows-1, 0, 1, cols))/2.0;
    y.block(0, 0, rows, 1) = (x.block(0, 1, rows, 1) + x.block(0, cols-1, rows, 1))/2.0;
    return y;
}

void CorrelationFlow::ComputeIntermedium(const Eigen::ArrayXXf& image, Eigen::ArrayXXcf& fft_result, Eigen::ArrayXXcf& fft_polar)
{
    fft_result = FFT(image); // undistorted image process FFT transform
    // ShowArray(image,"image", 1);
    // cout << "image:" << fft_result.size() << endl;
    // sleep(1000);
    Eigen::ArrayXXf power = IFFT(fft_result.abs());
    auto high_power = RemoveZeroComponent(power);
    fft_polar = FFT(polar(fftshift(high_power)));
}

Eigen::Vector3d CorrelationFlow::ComputePose(const Eigen::ArrayXXcf& last_fft_result, const Eigen::ArrayXXf& image,
                                   const Eigen::ArrayXXcf& last_fft_polar, const Eigen::ArrayXXcf& fft_polar,
                                   Eigen::Vector3d& pose)
{

    Eigen::Vector2d trans, trans_orig, trans_veri, rots; Eigen::Vector3d info; float info_trans;
    auto info_rots = EstimateTrans(last_fft_polar, fft_polar, target_rotation_fft, cfg.rotation_divisor, cfg.rotation_channel, rots);

    float degree = rots[0]*(2.0/cfg.rotation_divisor)*180;

    auto fft_rot_orig = FFT(RotateArray(image, -degree));
    // ShowArray(RotateArray(image, -degree),"image", 1);
    auto fft_rot_veri = FFT(RotateArray(image, -degree+180));
    // cout << "height" << cfg.height << endl;

    float info_trans_orig = EstimateTrans(last_fft_result, fft_rot_orig, target_fft, cfg.height, cfg.width, trans_orig); //error
    // cout << "aaaaaaaaaaaaaaaaa: " << cfg.width << endl;
    float info_trans_veri = EstimateTrans(last_fft_result, fft_rot_veri, target_fft, cfg.height, cfg.width, trans_veri);
    // auto rectify_orig = WarpArray(IFFT(last_fft_result),-trans_orig[1],-trans_orig[0], degree);
    // auto rectify_veri = WarpArray(IFFT(last_fft_result),-trans_veri[1],-trans_veri[0], degree+180);
    if (info_trans_orig > info_trans_veri)
        {
            info_trans = info_trans_orig;
            trans = trans_orig;
            degree = degree;
        }
    else{
            info_trans = info_trans_veri;
            trans = trans_veri;
            degree = degree + 180;
        }

    (degree>180)? degree=degree-360 : degree=degree;
    float theta = degree/180*M_PI;
    info[0] = info_trans; pose[0] = trans[1];
    info[1] = info_trans; pose[1] = trans[0];
    info[2] = info_rots;  pose[2] = theta;
    // cout << "trans_veri" << trans_veri << endl;
    // sleep(100);
    std::cout<<"X, Y, \u0398: "<<pose.transpose()<<" Rad = "<< degree <<"Degree"<<std::endl;
    std::cout<<"Info: "<<info.transpose()<<std::endl;
    auto rectify = WarpArray(IFFT(last_fft_result),-pose[0],-pose[1], degree);
    ShowArray(image,"image", 1);
    // ShowArray(IFFT(last_fft_result),"last", 1);
    // ShowArray(rectify,"rectify", 100);
    // ShowArray(rectify_orig,"rectify_orig", 100);
    // ShowArray(rectify_veri,"rectify_veri", 0);
    return info;
}

float CorrelationFlow::EstimateTrans(const Eigen::ArrayXXcf& last_fft_result, const Eigen::ArrayXXcf& fft_result,
                                     const Eigen::ArrayXXcf& output_fft, int height, int width, Eigen::Vector2d& trans)
{
    /// @brief can esrimate translation, roatation, and scale
    /// @param last_fft_result fft result of key frame
    /// @param fft_result current frame's fft result
    /// @param output_fft target_rotation_fft generate from GetTargetFFT(720, 480)
    /// @param height cfg.rotation_divisor
    /// @param width cfg.rotation_channel
    /// @param trans the result, ie. rotation degree, notice that the trans length is two, trans[0] is actually the degree we need
 
    Eigen::ArrayXXcf Kzz, Kxz;
    switch(cfg.kernel) {
    case 0:
        
        Kzz = polynomial_kernel(last_fft_result, height, width);
        Kxz = polynomial_kernel(fft_result, last_fft_result, height, width);
        break;
    case 1:
        // cout << "hello world" << endl;
        Kzz = gaussian_kernel(last_fft_result, height, width);
        Kxz = gaussian_kernel(fft_result, last_fft_result, height, width);
        break;
    default:
        throw std::invalid_argument( "Received invalid kernel type" );
    }
    

    // cout << "output_fft" << output_fft.size() << endl;
    // cout << "Kzz" << Kzz.size() << endl;
    // cout << "cfg.lambda" << cfg.lambda << endl;
    auto H = output_fft/(Kzz + cfg.lambda); 
    // PrintArrayToFile(output_fft, output_fft.rows(), output_fft.cols());
    // sleep(1000);

    Eigen::ArrayXXcf G = H*Kxz;
    Eigen::ArrayXXf g = IFFT(G);
    Eigen::ArrayXXf::Index row, col;
    float response = g.maxCoeff(&(row), &(col));
    // cout << "response:" << response << endl; // is the maximum of g
    // sleep(1000);
    trans[0] = -(row-height/2);
    trans[1] = -(col-width/2);
    // cout << "row: " << row << endl; cout << "height: " << height << endl;
    return GetInfo(g, response);
}

inline Eigen::ArrayXXcf CorrelationFlow::gaussian_kernel(const Eigen::ArrayXXcf& xf, const Eigen::ArrayXXcf& zf, int height, int width)
{
    unsigned int N = height * width;
    auto xx = xf.square().abs().sum()/N; // Parseval's Theorem
    auto zz = zf.square().abs().sum()/N;
    auto zfc = zf.conjugate();
    Eigen::ArrayXXcf xzf = xf * zfc;
    auto xz = IFFT(xzf);
    auto xxzz = (xx+zz-2*xz)/N;
    Eigen::ArrayXXf kernel = (-1/(cfg.sigma*cfg.sigma)*xxzz).exp();
    kernel = kernel/kernel.abs().maxCoeff();
    return FFT(kernel);
}

inline Eigen::ArrayXXcf CorrelationFlow::gaussian_kernel(const Eigen::ArrayXXcf& xf, int height, int width)
{
    unsigned int N = height * width;
    auto xx = xf.square().abs().sum()/N; // Parseval's Theorem
    auto zfc = xf.conjugate();
    Eigen::ArrayXXcf xzf = xf * zfc;
    auto xz = IFFT(xzf);
    auto xxzz = (xx+xx-2*xz)/N;
    Eigen::ArrayXXf kernel = (-1/(cfg.sigma*cfg.sigma)*xxzz).exp();
    kernel = kernel/kernel.abs().maxCoeff();
    return FFT(kernel);
}

inline Eigen::ArrayXXcf CorrelationFlow::polynomial_kernel(const Eigen::ArrayXXcf& xf, const Eigen::ArrayXXcf& zf, int height, int width)
{
    auto zfc = zf.conjugate();
    Eigen::ArrayXXcf xzf = xf * zfc;
    auto xz = IFFT(xzf);
    Eigen::ArrayXXf kernel = (xz+cfg.offset).pow(cfg.power);
    kernel = kernel/kernel.abs().maxCoeff();
    return FFT(kernel);
}

inline Eigen::ArrayXXcf CorrelationFlow::polynomial_kernel(const Eigen::ArrayXXcf& xf, int height, int width)
{
    auto zfc = xf.conjugate();
    // cout << "xf:" << xf.rows() << " " << xf.cols() << endl;
    // cout << "zfc:" << zfc.rows() << " " << zfc.cols() << endl;

    // sleep(1000);
    Eigen::ArrayXXcf xzf = xf * zfc;
    auto xz = IFFT(xzf);
    Eigen::ArrayXXf kernel = (xz+cfg.offset).pow(cfg.power);
    kernel = kernel/kernel.abs().maxCoeff();
    return FFT(kernel);
}

inline Eigen::ArrayXXf CorrelationFlow::polar(const Eigen::ArrayXXf& array)
{
    cv::Mat polar_img, img=ConvertArrayToMat(array);
    cv::Point2f center((float)img.cols/2, (float)img.rows/2);
    double radius = (double)std::min(img.rows/2, img.cols/2);
    cv::Size dsize = cv::Size(cfg.rotation_channel, cfg.rotation_divisor);
    cv::warpPolar(img, polar_img, dsize, center, radius, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
    return ConvertMatToArray(polar_img);
}

inline float CorrelationFlow::GetInfo(const Eigen::ArrayXXf& output, float response)
{
    float side_lobe_mean = (output.sum()-response)/(output.size()-1);
    float std  = sqrt((output-side_lobe_mean).square().mean());
    return (response - side_lobe_mean)/(std+1e-7);
}
