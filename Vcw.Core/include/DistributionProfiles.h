
#include <ctime>
#include <vector>
#include <math.h>
#include <cmath>
#include <limits>

#pragma once

typedef unsigned long long int Uint64;
typedef unsigned long Uint32;

namespace Vcw
{
    // Uniform Distribution
    class UniformDistribution
    {
    public:

            UniformDistribution(const Uint32 Seed = std::time(NULL));

            virtual int GenerateInt();
            virtual Uint64 GenerateUint64();
            virtual double GenerateDouble();

            std::vector<Uint64> GenerateArrayUint64(size_t N);

            std::vector<int> GenerateArrayInt(size_t N);

            std::vector<double> GenerateArrayDouble(size_t N);

    protected:

            inline Uint64 GenerateUint64_();

            inline double GenerateDouble_();

    private:

            Uint64 u, v, w;
    };

    // Normal Distribution
    class NormalDistribution : public UniformDistribution
    {
    public:

        NormalDistribution(const double Mean, const double Sigma);
        NormalDistribution(const double Mean, const double Sigma, const Uint32 Seed);

        double GenerateDouble();

        const double Mean, Sigma;

    private:

        double s0 = 0.0;
    };

    // Poisson Distribution
    class PoissonDistribution : public UniformDistribution
    {
    public:

        PoissonDistribution(const double Lambda);
        PoissonDistribution(const double Lambda, const Uint32 Seed);

        int GenerateInt();

        double Lambda;

    private:

        double sqlam, loglam, lamexp, lambold;
        std::vector<double> logfact;
        int swch;
    };

    static inline double LnGamma(const double xx)
    {
        if(xx <= 0.0) return std::numeric_limits<double>::infinity();

        int j;
        double x, t, y, ser;

        static const double cof[14] = {57.1562356658629235,-59.5979603554754912,
                                       14.1360979747417471,-0.491913816097620199,.339946499848118887e-4,
                                       .465236289270485756e-4,-.983744753048795646e-4,.158088703224912494e-3,
                                       -.210264441724104883e-3,.217439618115212643e-3,-.164318106536763890e-3,
                                       .844182239838527433e-4,-.261908384015814087e-4,.368991826595316234e-5};

        y=x=xx;
        t = x+5.24218750000000000;
        t = (x+0.5)*log(t)-t;
        ser = 0.999999999999997092;
        for (j=0;j<14;j++) ser += cof[j]/++y;

        return t+log(2.5066282746310005*ser/x);
    }
}
