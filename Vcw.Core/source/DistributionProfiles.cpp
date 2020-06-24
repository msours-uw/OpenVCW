
#include "DistributionProfiles.h"
#include <iostream>

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

    int UniformDistribution::GenerateInt() {return (int)GenerateUint64_(); }

    double UniformDistribution::GenerateDouble() {return GenerateDouble_(); }

    std::vector<Uint64> UniformDistribution::GenerateArrayUint64(size_t N)
    {
        std::vector<Uint64> V;
        for(int k=0; k< N;k++) V.push_back(GenerateUint64());

        return V;
    }
    std::vector<int> UniformDistribution::GenerateArrayInt(size_t N)
    {
        std::vector<int> V;
        for(int k=0; k< N;k++) V.push_back(GenerateInt());

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

    // Poisson Distribution
    PoissonDistribution::PoissonDistribution(const double Lambda) : UniformDistribution(), Lambda(Lambda), logfact(std::vector<double>(1024, -1.0)), lambold(-1.0) {}
    PoissonDistribution::PoissonDistribution(const double Lambda, const Uint32 Seed) : UniformDistribution(Seed), Lambda(Lambda), logfact(std::vector<double>(1024, -1.0)), lambold(-1.0) {}

    int PoissonDistribution::GenerateInt()
    {
        double u, u2, v, v2, p, t, lfac;
        int K;

        if(Lambda < 5.0)
        {
            if (Lambda != lambold) lamexp = exp(-Lambda);

            K = -1;
            t = 1.0;

            do
            {
                ++K;
                t *= GenerateDouble_();
            }
            while(t > lamexp);
        }
        else
        {
            if(Lambda != lambold)
            {
                sqlam = sqrt(Lambda);
                loglam = log(Lambda);
            }
            for(;;)
            {
                u = 0.64 * GenerateDouble_();
                v = -0.68 + 1.28 * GenerateDouble_();

                if(Lambda > 13.5)
                {
                    v2 = v * v;
                    if (v >= 0.0) {if (v2 > 6.5 * u * (0.64-u) * (u + 0.2)) continue;}
                    else {if (v2 > 9.6 * u * (0.66 - u) * (u + 0.07)) continue;}
                }

                K = (int)(std::floor(sqlam*(v/u)+Lambda+0.5));

                if(K < 0) continue;

                u2 = u * u;

                if (Lambda > 13.5)
                {
                    if (v >= 0.0) {if (v2 < 15.2 * u2 * (0.61-u) * (0.8-u)) break;}
                    else {if (v2 < 6.76 * u2 * (0.62-u)*(1.4-u)) break; }
                }

                if (K < 1024)
                {
                    if (logfact[K] < 0.0) logfact[K] = LnGamma(K + 1.0);
                    lfac = logfact[K];
                }
                else lfac = LnGamma(K + 1.0);

                p = sqlam * exp(-Lambda + K * loglam - lfac);

                if (u2 < p) break;

            }
        }

        lambold = Lambda;
        return K;
    }
}




















