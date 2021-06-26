#include <math.h>
#include "correlation_flow.h"
#include "read_configs.h"
#include "circ_shift.h"

CorrelationFlow::CorrelationFlow(CFConfig& cf_config):cfg(cf_config)
{
    target_fft = GetTargetFFT(cfg.height, cfg.width);
    target_rotation_fft = GetTargetFFT(cfg.rotation_divisor, cfg.width);
}

Eigen::ArrayXXcf CorrelationFlow::GetTargetFFT(int rows, int cols)
{
    Eigen::ArrayXXf target = Eigen::ArrayXXf::Zero(rows, cols);
    target(rows/2, cols/2) = 1;
    return FFT(target);
}

Eigen::ArrayXXcf CorrelationFlow::FFT(const Eigen::ArrayXXf& x)
{
    Eigen::ArrayXXcf xf = Eigen::ArrayXXcf(x.rows()/2+1, x.cols());
    fftwf_plan fft_plan = fftwf_plan_dft_r2c_2d(x.cols(), x.rows(), (float(*))(x.data()),
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major
    fftwf_execute(fft_plan);
    return xf;
}

Eigen::ArrayXXf CorrelationFlow::IFFT(const Eigen::ArrayXXcf& xf)
{
    Eigen::ArrayXXf x = Eigen::ArrayXXf((xf.rows()-1)*2, xf.cols());
    Eigen::ArrayXXcf cxf = xf;
    fftwf_plan fft_plan = fftwf_plan_dft_c2r_2d(xf.cols(), (xf.rows()-1)*2, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    fftwf_execute(fft_plan);
    return x/x.size();
}

inline Eigen::ArrayXXf CorrelationFlow::RemoveZeroFrequency(const Eigen::ArrayXXf& x)
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
    fft_result = FFT(image);
    Eigen::ArrayXXf power = IFFT(fft_result.abs());
    auto high_power = RemoveZeroFrequency(power);
    fft_polar = FFT(polar(fftshift(high_power)));
}

Eigen::Vector3d CorrelationFlow::ComputePose(const Eigen::ArrayXXcf& last_fft_result, const Eigen::ArrayXXcf& fft_result,
                                   const Eigen::ArrayXXcf& last_fft_polar, const Eigen::ArrayXXcf& fft_polar,
                                   Eigen::Vector3d& pose)
{
    Eigen::Vector2d trans, trans_orig, trans_veri, rots; Eigen::Vector3d info; float info_trans;
    auto info_rots = EstimateTrans(last_fft_polar, fft_polar, target_rotation_fft, cfg.rotation_divisor, cfg.width, rots);
    float degree = rots[0]*(2.0/cfg.rotation_divisor)*180;

    auto fft_rot_orig = FFT(RotateArray(IFFT(fft_result), -degree));
    auto fft_rot_veri = FFT(RotateArray(IFFT(fft_result), -degree+180));

    float info_trans_orig = EstimateTrans(last_fft_result, fft_rot_orig, target_fft, cfg.height, cfg.width, trans_orig);
    float info_trans_veri = EstimateTrans(last_fft_result, fft_rot_veri, target_fft, cfg.height, cfg.width, trans_veri);

    ShowArray(IFFT(last_fft_result), "last");
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

    info[0] = info_trans; pose[0] = trans[0];
    info[1] = info_trans; pose[1] = trans[1];
    info[2] = info_rots;  pose[2] = degree/180*M_PI;
    std::cout<<"X, Y, \u0398: "<<pose.transpose()<<"="<<degree<<std::endl;
    std::cout<<"Info: "<<info.transpose()<<std::endl;
    return info;
}

float CorrelationFlow::EstimateTrans(const Eigen::ArrayXXcf& last_fft_result, const Eigen::ArrayXXcf& fft_result,
                                     const Eigen::ArrayXXcf& output_fft, int height, int width, Eigen::Vector2d& trans)
{
    auto Kzz = gaussian_kernel(last_fft_result, height, width);
    auto Kxz = gaussian_kernel(fft_result, last_fft_result, height, width);
    auto H = output_fft/(Kzz + cfg.lambda);
    Eigen::ArrayXXcf G = H*Kxz;
    Eigen::ArrayXXf g = IFFT(G);
    Eigen::ArrayXXf::Index row, col;
    float response = g.maxCoeff(&(row), &(col));
    trans[0] = -(row-height/2);
    trans[1] = -(col-width/2);
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
    return FFT(kernel);
}

inline Eigen::ArrayXXf CorrelationFlow::polar(const Eigen::ArrayXXf& array)
{
    cv::Mat polar_img, img=ConvertArrayToMat(array);
    cv::Point2f center((float)img.cols/2, (float)img.rows/2);
    double radius = (double)std::min(img.rows/2, img.cols/2);
    cv::Size dsize = cv::Size(cfg.width, cfg.rotation_divisor);
    cv::warpPolar(img, polar_img, dsize, center, radius, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
    return ConvertMatToArray(polar_img);
}

inline float CorrelationFlow::GetInfo(const Eigen::ArrayXXf& output, float response)
{
    float side_lobe_mean = (output.sum()-response)/(output.size()-1);
    float std  = sqrt((output-side_lobe_mean).square().mean());
    return (response - side_lobe_mean)/(std+1e-7);
}

// Eigen::ArrayXXcf CorrelationFlow::FFT(const Eigen::ArrayXXf& x)
// {
//     Eigen::ArrayXXcf output = Eigen::ArrayXXcf(x.rows(), x.cols());
//     Eigen::ArrayXXcf input = Eigen::ArrayXXcf(x);
//     fftwf_plan fft_plan = fftwf_plan_dft_2d(input.cols(), input.rows(),
//                 (float(*)[2])(input.data()), (float(*)[2])(output.data()),
//                 FFTW_FORWARD, FFTW_ESTIMATE);  // reverse order for column major
//     fftwf_execute(fft_plan);
//     return output;
// }

// Eigen::ArrayXXf CorrelationFlow::IFFT(const Eigen::ArrayXXcf& input)
// {
//     Eigen::ArrayXXcf output = Eigen::ArrayXXf(input.rows(), input.cols());
//     fftwf_plan fft_plan = fftwf_plan_dft_2d(input.cols(), input.rows(), 
//                 (float(*)[2])(input.data()), (float(*)[2])(output.data()), 
//                 FFTW_BACKWARD, FFTW_ESTIMATE);
//     fftwf_execute(fft_plan);
//     return output.abs()/output.size();
// }

// Eigen::ArrayXXf removeRow(const Eigen::ArrayXXf& x, unsigned int i)
// {
//     Eigen::ArrayXXf y(x.rows()-1, x.cols());
//     y.block(0, 0, i, y.cols()) = x.block(0, 0, i, y.cols());
//     y.block(i, 0, y.rows()-i, x.cols()) = x.block(i+1, 0, y.rows()-i, y.cols());
//     return y;
// }

// Eigen::ArrayXXf removeColumn(const Eigen::ArrayXXf& x, unsigned int j)
// {
//     Eigen::ArrayXXf y(x.rows(), x.cols()-1);
//     y.block(0, 0, y.rows(), j) = x.block(0, 0, y.rows(), j);
//     y.block(0, j, y.rows(), y.cols()-j) = x.block(0, j+1, y.rows(), y.cols()-j);
//     return y;
// }

// Eigen::ArrayXXf removeRowCol(const Eigen::ArrayXXf& x, unsigned int i, unsigned int j)
// {
//     return removeColumn(removeRow(x, i), j);
// }