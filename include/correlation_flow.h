#ifndef CORRELATION_FLOW_H
#define CORRELATION_FLOW_H

#include <fftw3.h>
#include "utils.h"
#include "read_configs.h"

class CorrelationFlow{

public:
    CorrelationFlow(CFConfig& cf_config);
    void ComputeIntermedium(const Eigen::ArrayXXf&, Eigen::ArrayXXcf&, Eigen::ArrayXXcf&);
    Eigen::Vector3d ComputePose(const Eigen::ArrayXXcf&, const Eigen::ArrayXXcf&, const Eigen::ArrayXXcf&, const Eigen::ArrayXXcf&, Eigen::Vector3d&);

private:
    CFConfig cfg;
    Eigen::ArrayXXcf target_fft;
    Eigen::ArrayXXcf target_rotation_fft;
    Eigen::ArrayXXcf FFT(const Eigen::ArrayXXf&);
    Eigen::ArrayXXf IFFT(const Eigen::ArrayXXcf&);
    Eigen::ArrayXXcf GetTargetFFT(int, int);
    inline Eigen::ArrayXXf RemoveZeroFrequency(const Eigen::ArrayXXf&);
    inline Eigen::ArrayXXcf gaussian_kernel(const Eigen::ArrayXXcf&, int, int);
    inline Eigen::ArrayXXcf gaussian_kernel(const Eigen::ArrayXXcf&, const Eigen::ArrayXXcf&, int, int);
    float EstimateTrans(const Eigen::ArrayXXcf&, const Eigen::ArrayXXcf&, const Eigen::ArrayXXcf&, int, int, Eigen::Vector2d&);
    inline Eigen::ArrayXXf polar(const Eigen::ArrayXXf&);
    inline float GetInfo(const Eigen::ArrayXXf&, float);
};

typedef std::shared_ptr<CorrelationFlow> CorrelationFlowPtr;

#endif  // CORRELATION_FLOW_H