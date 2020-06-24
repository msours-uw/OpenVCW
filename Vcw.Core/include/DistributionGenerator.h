
#include <ctime>
#include <vector>

#pragma once

typedef unsigned long long int Uint64;
typedef unsigned long Uint32;

namespace Vcw
{
    struct UniformDistribution
    {
            UniformDistribution(const Uint32 Seed = std::time(NULL))
            {
                v = 4101842887655102017LL;
                w = 1;

                u = Seed ^ v; GenerateUint64_();
                v = u; GenerateUint64_();
                w = v; GenerateUint64_();
            }

            virtual Uint64 GenerateUint64() {return GenerateUint64_(); }
            virtual double GenerateDouble() {return GenerateUint64_(); }

            virtual std::vector<Uint64> GenerateUint64(size_t N)
            {
                std::vector<Uint64> V;
                for(int k=0; k< N;k++) V.push_back(GenerateUint64_());

                return V;
            }

            virtual std::vector<double> GenerateDouble(size_t N)
            {
                std::vector<double> V;
                for(int k=0; k< N;k++) V.push_back(GenerateDouble_());

                return V;
            }

    private:

            Uint64 u, v, w;

            inline Uint64 GenerateUint64_()
            {
                u = u * 2862933555777941757LL + 7046029254386353087LL;
                v ^= v >> 17; v ^= v << 31; v ^= v >> 8;
                w = 4294957665U*(w & 0xffffffff) + (w >> 32);
                Uint64 x = u ^ (u << 21); x ^= x >> 35; x ^= x << 4;

                return (x + v) ^ w;
            }

            inline double GenerateDouble_() { return 5.42101086242752217E-20 * GenerateUint64_(); }
    };
}
