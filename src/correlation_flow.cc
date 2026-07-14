#include <math.h>
#include <opencv2/imgproc.hpp>
#include "correlation_flow.h"
#include "read_configs.h"
#include "circ_shift.h"
#include "optimization_2d/pose_graph_2d_error_term.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdexcept>
using namespace std;

#include "unistd.h"

#include <chrono>
using namespace std::chrono;

// // void PrintArrayToFile (const Eigen::ArrayXXf target, int rows, int cols) {
// void PrintArrayToFile (const auto& target, int rows, int cols) {

//   ofstream myfile ("example.txt");
//   if (myfile.is_open())
//   {
//     for(int count_row = 0; count_row < rows; count_row ++){
//         for(int count_col = 0; count_col < cols; count_col ++){
//             myfile << target(count_row, count_col) << " " ;
//             if(count_col == cols - 1){
//                 myfile << "\n";
//             }
//         }
        
//     }
//     myfile.close();
//   }
//   else std::cout << "Unable to open file";

// }


struct CorrelationFlow::FFTPlan {
    FFTPlan(int rows, int cols)
        : rows(rows), cols(cols), real_size(rows * cols),
          complex_size((rows / 2 + 1) * cols),
          real_buffer(static_cast<float*>(fftwf_malloc(sizeof(float) * real_size))),
          complex_buffer(static_cast<fftwf_complex*>(
              fftwf_malloc(sizeof(fftwf_complex) * complex_size))),
          forward(nullptr), inverse(nullptr) {
        if(!real_buffer || !complex_buffer){
            if(complex_buffer) fftwf_free(complex_buffer);
            if(real_buffer) fftwf_free(real_buffer);
            throw std::bad_alloc();
        }

        // Eigen arrays are column-major, so the FFTW dimensions are reversed.
        forward = fftwf_plan_dft_r2c_2d(
            cols, rows, real_buffer, complex_buffer, FFTW_ESTIMATE);
        inverse = fftwf_plan_dft_c2r_2d(
            cols, rows, complex_buffer, real_buffer, FFTW_ESTIMATE);
        if(!forward || !inverse){
            if(forward) fftwf_destroy_plan(forward);
            if(inverse) fftwf_destroy_plan(inverse);
            fftwf_free(complex_buffer);
            fftwf_free(real_buffer);
            throw std::runtime_error("Failed to create FFTW plan");
        }
    }

    ~FFTPlan(){
        if(forward) fftwf_destroy_plan(forward);
        if(inverse) fftwf_destroy_plan(inverse);
        fftwf_free(complex_buffer);
        fftwf_free(real_buffer);
    }

    int rows;
    int cols;
    size_t real_size;
    size_t complex_size;
    float* real_buffer;
    fftwf_complex* complex_buffer;
    fftwf_plan forward;
    fftwf_plan inverse;
};

CorrelationFlow::CorrelationFlow(CFConfig& cf_config, double &image_height, double &image_width):cfg(cf_config){
    if(!fftwf_init_threads()){
        throw std::runtime_error("Failed to initialize FFTW threads");
    }
    fftwf_plan_with_nthreads(cfg.fftw_threads);
    cfg.height = int(image_height);
    cfg.width = int(image_width);
    target_fft = GetTargetFFT(cfg.height, cfg.width);
    target_rotation_fft = GetTargetFFT(cfg.rotation_divisor, cfg.rotation_channel);
}

CorrelationFlow::~CorrelationFlow(){
    // Plans must be destroyed before the global FFTW thread resources.
    fft_plans.clear();
    fftwf_cleanup_threads();
}

CorrelationFlow::FFTPlan& CorrelationFlow::GetFFTPlan(int rows, int cols){
    const auto key = std::make_pair(rows, cols);
    auto plan = fft_plans.find(key);
    if(plan == fft_plans.end()){
        plan = fft_plans.emplace(key, std::unique_ptr<FFTPlan>(new FFTPlan(rows, cols))).first;
    }
    return *plan->second;
}

Eigen::ArrayXXcf CorrelationFlow::GetTargetFFT(int rows, int cols){
    Eigen::ArrayXXf target = Eigen::ArrayXXf::Zero(rows, cols);
    target(rows/2, cols/2) = 1;
    return FFT(target);
}

Eigen::ArrayXXcf CorrelationFlow::FFT(const Eigen::ArrayXXf& x){
    std::lock_guard<std::mutex> lock(fft_mutex);
    FFTPlan& plan = GetFFTPlan(x.rows(), x.cols());
    Eigen::ArrayXXcf xf = Eigen::ArrayXXcf(x.rows()/2+1, x.cols()); // xf [225, 448]
    std::copy(x.data(), x.data() + plan.real_size, plan.real_buffer);
    fftwf_execute(plan.forward);
    for(size_t i = 0; i < plan.complex_size; ++i){
        xf.data()[i] = std::complex<float>(
            plan.complex_buffer[i][0], plan.complex_buffer[i][1]);
    }
    return xf;
}

Eigen::ArrayXXf CorrelationFlow::IFFT(const Eigen::ArrayXXcf& xf){
    std::lock_guard<std::mutex> lock(fft_mutex);
    const int rows = (xf.rows()-1)*2;
    FFTPlan& plan = GetFFTPlan(rows, xf.cols());
    Eigen::ArrayXXf x = Eigen::ArrayXXf(rows, xf.cols());
    for(size_t i = 0; i < plan.complex_size; ++i){
        plan.complex_buffer[i][0] = xf.data()[i].real();
        plan.complex_buffer[i][1] = xf.data()[i].imag();
    }
    fftwf_execute(plan.inverse);
    std::copy(plan.real_buffer, plan.real_buffer + plan.real_size, x.data());
    return x/x.size();
}

inline Eigen::ArrayXXf CorrelationFlow::RemoveZeroComponent(const Eigen::ArrayXXf& x){
    Eigen::ArrayXXf y(x);
    unsigned int cols = x.cols();
    unsigned int rows = x.rows();
    y.block(0, 0, 1, cols) = (x.block(1, 0, 1, cols) + x.block(rows-1, 0, 1, cols))/2.0;
    y.block(0, 0, rows, 1) = (x.block(0, 1, rows, 1) + x.block(0, cols-1, rows, 1))/2.0;
    return y;
}

void CorrelationFlow::ComputeIntermedium(const Eigen::ArrayXXf& image, Eigen::ArrayXXcf& fft_result, Eigen::ArrayXXcf& fft_polar){
    fft_result = FFT(image); 
    Eigen::ArrayXXf power = IFFT(fft_result.abs());
    auto high_power = RemoveZeroComponent(power);
    fft_polar = FFT(polar(fftshift(high_power)));
}

Eigen::Vector3d CorrelationFlow::ComputePose(const Eigen::ArrayXXcf& last_fft_result, const Eigen::ArrayXXf& image,
                                   const Eigen::ArrayXXcf& last_fft_polar, const Eigen::ArrayXXcf& fft_polar,
                                   Eigen::Vector3d& pose, bool not_large_rotation){
    Eigen::Vector2d trans, trans_orig, trans_veri, rots; Eigen::Vector3d info; float info_trans;
    auto info_rots = EstimateTrans(last_fft_polar, fft_polar, target_rotation_fft, cfg.rotation_divisor, cfg.rotation_channel, rots);

    float degree = rots[0]*(2.0/cfg.rotation_divisor)*180;
    degree = NormalizeDegree(degree);
    if(not_large_rotation){
        degree = std::abs(degree) > 90 ? degree - 180 : degree;
        auto fft_rot_orig = FFT(RotateArray(image, -degree));     
        float info_trans_orig = EstimateTrans(last_fft_result, fft_rot_orig, target_fft, cfg.height, cfg.width, trans_orig);   

        info_trans = info_trans_orig;
        trans = trans_orig;
        degree = degree;
    }else{
        auto fft_rot_orig = FFT(RotateArray(image, -degree));
        auto fft_rot_veri = FFT(RotateArray(image, -degree+180));

        float info_trans_orig = EstimateTrans(last_fft_result, fft_rot_orig, target_fft, cfg.height, cfg.width, trans_orig); //error
        float info_trans_veri = EstimateTrans(last_fft_result, fft_rot_veri, target_fft, cfg.height, cfg.width, trans_veri);
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
    }

    (degree>180)? degree=degree-360 : degree=degree;
    float theta = degree/180*M_PI;
    info[0] = info_trans; pose[0] = trans[1];
    info[1] = info_trans; pose[1] = trans[0];
    info[2] = info_rots;  pose[2] = theta;
    std::cout<<"X, Y, \u0398: "<<pose.transpose()<<" Rad = "<< degree <<"Degree"<<std::endl;
    std::cout<<"Info: "<<info.transpose()<<std::endl;
    auto rectify = WarpArray(IFFT(last_fft_result),-pose[0],-pose[1], degree);
    return info;
}

float CorrelationFlow::EstimateTrans(const Eigen::ArrayXXcf& last_fft_result, const Eigen::ArrayXXcf& fft_result,
                                     const Eigen::ArrayXXcf& output_fft, int height, int width, Eigen::Vector2d& trans){
    /// @brief can esrimate translation, rotation, and scale
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
        Kzz = gaussian_kernel(last_fft_result, height, width);
        Kxz = gaussian_kernel(fft_result, last_fft_result, height, width);
        break;
    default:
        throw std::invalid_argument( "Received invalid kernel type" );
    }

    auto H = output_fft/(Kzz + cfg.lambda); 
    Eigen::ArrayXXcf G = H*Kxz;
    Eigen::ArrayXXf g = IFFT(G);
    Eigen::ArrayXXf::Index row, col;
    float response = g.maxCoeff(&(row), &(col));
    trans[0] = -(row-height/2);
    trans[1] = -(col-width/2);
    return GetInfo(g, response);
}

inline Eigen::ArrayXXcf CorrelationFlow::gaussian_kernel(const Eigen::ArrayXXcf& xf, const Eigen::ArrayXXcf& zf, int height, int width){
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

inline Eigen::ArrayXXcf CorrelationFlow::gaussian_kernel(const Eigen::ArrayXXcf& xf, int height, int width){
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

inline Eigen::ArrayXXcf CorrelationFlow::polynomial_kernel(const Eigen::ArrayXXcf& xf, const Eigen::ArrayXXcf& zf, int height, int width){
    auto zfc = zf.conjugate();
    Eigen::ArrayXXcf xzf = xf * zfc;
    auto xz = IFFT(xzf);
    Eigen::ArrayXXf kernel = (xz+cfg.offset).pow(cfg.power);
    kernel = kernel/kernel.abs().maxCoeff();
    return FFT(kernel);
}

inline Eigen::ArrayXXcf CorrelationFlow::polynomial_kernel(const Eigen::ArrayXXcf& xf, int height, int width){
    auto zfc = xf.conjugate();
    Eigen::ArrayXXcf xzf = xf * zfc;
    auto xz = IFFT(xzf);
    Eigen::ArrayXXf kernel = (xz+cfg.offset).pow(cfg.power);
    kernel = kernel/kernel.abs().maxCoeff();
    return FFT(kernel);
}

inline Eigen::ArrayXXf CorrelationFlow::polar(const Eigen::ArrayXXf& array){
    cv::Mat polar_img, img=ConvertArrayToMat(array);
    cv::Point2f center((float)img.cols/2, (float)img.rows/2);
    double radius = (double)std::min(img.rows/2, img.cols/2);
    cv::Size dsize = cv::Size(cfg.rotation_channel, cfg.rotation_divisor);
    cv::warpPolar(img, polar_img, dsize, center, radius, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
    return ConvertMatToArray(polar_img);
}

inline float CorrelationFlow::GetInfo(const Eigen::ArrayXXf& output, float response){
    float side_lobe_mean = (output.sum()-response)/(output.size()-1);
    float std  = sqrt((output-side_lobe_mean).square().mean());
    return (response - side_lobe_mean)/(std+1e-7);
}
