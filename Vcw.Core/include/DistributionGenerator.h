#include <ctime>

#pragma once

typedef unsigned long long int Uint64;
typedef unsigned long Uint32;

namespace Vcw
{
    struct UniformDistribution
    {
            Uint64 u, v, w;

            UniformDistribution(const Uint32 Seed = std::time(NULL))
            {
                v = 4101842887655102017LL;
                w = 1;

                u = Seed ^ v; GenerateUint64();
                v = u; GenerateUint64();
                w = v; GenerateUint64();
            }

            inline Uint64 GenerateUint64()
            {
                u = u * 2862933555777941757LL + 7046029254386353087LL;
		v ^= v >> 17; v ^= v << 31; v ^= v >> 8;
		w = 4294957665U*(w & 0xffffffff) + (w >> 32);
                Uint64 x = u ^ (u << 21); x ^= x >> 35; x ^= x << 4;

                return (x + v) ^ w;
            }

            inline double GenerateDouble() { return 5.42101086242752217E-20 * GenerateUint64(); }
    };
}
