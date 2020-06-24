
#include <ctime>
#include <vector>
#include <math.h>

#pragma once

typedef unsigned long long int Uint64;
typedef unsigned long Uint32;

namespace Vcw
{
    class UniformDistribution
    {
    public:

            UniformDistribution(const Uint32 Seed = std::time(NULL));


            virtual Uint64 GenerateUint64();
            virtual double GenerateDouble();

            virtual std::vector<Uint64> GenerateUint64(size_t N);

            virtual std::vector<double> GenerateDouble(size_t N);

    protected:

            inline Uint64 GenerateUint64_();

            inline double GenerateDouble_();

    private:

            Uint64 u, v, w;
    };

    class NormalDistribution : public UniformDistribution
    {
    public:

        NormalDistribution(const double Mean, const double Sigma);
        NormalDistribution(const double Mean, const double Sigma, const Uint32 Seed);

        double GenerateDouble();

        std::vector<double> GenerateDouble(size_t N);

        const double Mean, Sigma;

    private:

        double s0 = 0.0;
    };
}
