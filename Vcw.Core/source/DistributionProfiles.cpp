
#include "DistributionProfiles.h"

namespace Vcw
{
    // Uniform Distribution
    UniformDistribution::UniformDistribution(const Uint32 Seed) : v(4101842887655102017LL), w(1)
    {
        u = Seed ^ v; GenerateUint64_();
        v = u; GenerateUint64_();
        w = v; GenerateUint64_();
    }

    Uint64 UniformDistribution::GenerateUint64() {return GenerateUint64_(); }

    double UniformDistribution::GenerateDouble() {return GenerateDouble_(); }

    std::vector<Uint64> UniformDistribution::GenerateArrayUint64(size_t N)
    {
        std::vector<Uint64> V;
        for(int k=0; k< N;k++) V.push_back(GenerateUint64());

        return V;
    }

    std::vector<double> UniformDistribution::GenerateArrayDouble(size_t N)
    {
        std::vector<double> V;
        for(int k=0; k< N;k++) V.push_back(GenerateDouble());

        return V;
    }

    Uint64 UniformDistribution::GenerateUint64_()
    {
        u = u * 2862933555777941757LL + 7046029254386353087LL;
        v ^= v >> 17; v ^= v << 31; v ^= v >> 8;
        w = 4294957665U*(w & 0xffffffff) + (w >> 32);
        Uint64 x = u ^ (u << 21); x ^= x >> 35; x ^= x << 4;

        return (x + v) ^ w;
    }

    double UniformDistribution::GenerateDouble_() { return 5.42101086242752217E-20 * GenerateUint64_(); }

    // Normal Distribution
    NormalDistribution::NormalDistribution(const double Mean, const double Sigma) : UniformDistribution(), Mean(Mean), Sigma(Sigma) {}
    NormalDistribution::NormalDistribution(const double Mean, const double Sigma, const Uint32 Seed) : UniformDistribution(Seed), Mean(Mean), Sigma(Sigma) {}

    double NormalDistribution::GenerateDouble()
    {
        double fac;
        if(s0 == 0.0)
        {
            double v1, v2, rsq;

            do
            {
                v1 = 2.0 * GenerateDouble_() - 1.0;
                v2 = 2.0 * GenerateDouble_() - 1.0;

                rsq = v1 * v1 + v2 * v2;
            }
            while(rsq >= 1.0 || rsq == 0.0);

            fac = sqrt(-2.0 * log(rsq)/rsq);
            s0 = v1 * fac;

            return Mean + Sigma * v2 * fac;
        }
        else
        {
            fac = s0;
            s0 = 0.0;
            return Mean + Sigma * fac;
        }
    }
}
