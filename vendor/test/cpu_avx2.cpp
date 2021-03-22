#include <immintrin.h>

int main() {
    int data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    __m256i a = _mm256_loadu_si256((const __m256i *)data);
    __m256i b = _mm256_bslli_epi128(a, 1);
    return 0;
}
