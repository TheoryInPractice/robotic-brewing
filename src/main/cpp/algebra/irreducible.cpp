#include "algebra/irreducible.hpp"

namespace algebra {
/**
 * @brief A table of low-weight irreducible polynomials over the finite field Z_2.
 *
 * [Representation] An irreducible polynomial of degree n is:
 *   x^n + x^j + 1             if table[n] = {j}, and
 *   x^n + x^i + x^j + x^k + 1 if table[n] = {i,j,k}.
 *
 * @see https://www.hpl.hp.com/techreports/98/HPL-98-135.pdf
 */
std::vector<std::vector<int>> const BINARY_IRREDUCIBLE_POLYNOMIALS = {
    {},         // 0
    {},         // 1
    {1},        // 2
    {1},        // 3
    {1},        // 4
    {2},        // 5
    {1},        // 6
    {1},        // 7
    {4, 3, 1},  // 8
    {1},        // 9
    {3},        // 10
    {2},        // 11
    {3},        // 12
    {4, 3, 1},  // 13
    {5},        // 14
    {1},        // 15
    {5, 3, 1},  // 16
    {3},        // 17
    {3},        // 18
    {5, 2, 1},  // 19
    {3},        // 20
    {2},        // 21
    {1},        // 22
    {5},        // 23
    {4, 3, 1},  // 24
    {3},        // 25
    {4, 3, 1},  // 26
    {5, 2, 1},  // 27
    {1},        // 28
    {2},        // 29
    {1},        // 30
    {3},        // 31
    {7, 3, 2},  // 32
    {10},       // 33
    {7},        // 34
    {2},        // 35
    {9},        // 36
    {6, 4, 1},  // 37
    {6, 5, 1},  // 38
    {4},        // 39
    {5, 4, 3},  // 40
    {3},        // 41
    {7},        // 42
    {6, 4, 3},  // 43
    {5},        // 44
    {4, 3, 1},  // 45
    {1},        // 46
    {5},        // 47
    {5, 3, 2},  // 48
    {9},        // 49
    {4, 3, 2},  // 50
    {6, 3, 1},  // 51
    {3},        // 52
    {6, 2, 1},  // 53
    {9},        // 54
    {7},        // 55
    {7, 4, 2},  // 56
    {4},        // 57
    {19},       // 58
    {7, 4, 2},  // 59
    {1},        // 60
    {5, 2, 1},  // 61
    {29},       // 62
    {1},        // 63
    {4, 3, 1},  // 64
};
}  // namespace algebra
