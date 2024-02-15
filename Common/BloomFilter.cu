#include "BloomFilter.cuh"

#include "math.h"

namespace BITFS {
    
    #define MAX_SOLUTION_COUNT 100000 // Unused, included to explain how MAX_K is computed
    #define MAX_K 20 // Computed manually from MAX_SOLUTION_COUNT

    bool bloomfilterC[MAX_K];
    __device__ bool bloomfilterG[MAX_K];

    // black magic bitshift stuff to basically wrap a 32-bit unsigned integer around a wheel, rotate it, and unwrap it to
    // scramble the 32-bit integer.
    __host__ __device__ uint32_t roll(uint32_t n, int m) {

        return (n << m) | (n >> 32 - m);
    }


    // black magic hash function from wikipedia that was advised on StackOverflow. Don't ask me where this shit came from.
    __host__ __device__ uint32_t murmur3(float d1, float d2, float d3, uint32_t seed) {

        uint32_t h = seed;
        uint32_t data[3];

        // what the below thing should hopefully be doing is turning the floats (large) into the nearest int
        // by dropping the fractional part (and the floats are below ~2 billion which is the int limit)
        // and then turning the signed 32-bit int into a 32-bit unsigned int.

        data[0] = uint32_t(int(d1));
        data[1] = uint32_t(int(d2));
        data[2] = uint32_t(int(d3));

        // and below, if you haven't seen ^ before, it's the XOR function.

        for (int i = 0; i < 3; i++) {
            uint32_t k = data[i];
            k *= 0xcc9e2d51;
            k = roll(k, 15);
            k *= 0x1b873593;
            h ^= k;
            h = roll(h, 13);
            h = (h * 5) + 0xe6546b64;
        }

        h ^= 12;
        h ^= h >> 16;
        h *= 0x85ebca6b;
        h ^= h >> 13;
        h *= 0xc2b2ae35;
        h ^= h >> 16;

        return h;
    }


    //takes three floats and shoves them into the murmur3 hash function with four different seeds to get four
    // locations in the bloom filter table. k is log2 of the bloom filter size.
    __host__ __device__ void hash(float d1, float d2, float d3, int* locations, int k) {

        // the four seeds are 0, 1 billion and 1, 2 billion and 2, and 3 billion and 3, which are all valid
        // unsigned 32-bit numbers. Then modulo 2^k gets a position between 0 and 2^k - 1 ie a position in the bloom filter.
        // converting this unsigned int to an int should work as long as the position is small enough, ie, k<=31.

        for (uint32_t i = 0; i < 4; i++) {
            locations[i] = int((murmur3(d1, d2, d3, i * 1000000001)) % (1 << k));
        }

        return;
    }


    // the main bloom filter function. n is the size of our three float arrays, unique is an empty boolean array of size n
    // and what this does is it populates unique with a 0 if the corresponding input is a maybe-duplicate and 1 if
    // definitely unique.
    __host__ __device__ void bloom_filter(int n, float* data1, float* data2, float* data3, bool* unique) {

        bool* bloomfilter;
        #if !defined(__CUDA_ARCH__)
            bloomfilter = bloomfilterC;
        #else
            bloomfilter = bloomfilterG;
        #endif // !defined(__CUDA_ARCH)

        int k = (int)(min((log2f(5.77f * (float)n) + 1.0f), (float) MAX_K));
        // 2^k is the number of entries in our bloom filter table. The way this computation works is that the optimal
        // number of bloom filter table entries is n * log2(1/epsilon) / ln(2), where epsilon is a desired upper bound on the
        // error rate and n is an upper bound on how many novel items will be inserted.
        // A 1-in-16 error rate (true value will be less) is acceptable, so log2(1/epsilon) is 4. 4 over ln(2) is about 5.77.
        // So 5.77 n is approximately the size of the bloom filter table. However, we also want the filter size to be a power
        // of two because that means we can just strip off an appropriate number of bits from the output of a hash function
        // to specify a position in the bloom filter. The smallest power of 2 greater than 5.77 n is 2^(ceil(log2(5.77 * n))
        // And the way we round log2 up to the nearest integer is by just adding one and then using that float-to-int conversions
        // round down closer to 0.

        for(int j = 0; j < 1 << k; j++) {
            bloomfilter[j] = false;
        }

        for(int i = 0; i < n; i++) {
            int locations[4];

            // The hashing fills the empty locations array with 4 positions in the bloom filter to check.

            hash(data1[i], data2[i], data3[i], locations, k);

            unique[i] = false;
            for(int m = 0; m < 4; m++) {
                if (!bloomfilter[locations[m]]) {
                    unique[i] = true;
                    bloomfilter[locations[m]] = true;
                }
            }
        }
        return;
    }


}