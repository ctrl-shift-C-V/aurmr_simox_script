#pragma once

#include <type_traits>

#include <Eigen/Dense>

// ////////////////////////////////////////////////////////////////////////// //
//base
namespace simox::meta
{
    template<class Derived, int M, int N, class T2 = void>
    using enable_if_matMN = std::enable_if_t <
                            Derived::RowsAtCompileTime == M &&
                            Derived::ColsAtCompileTime == N,
                            T2 >;

    template<class D1, int K, int L, class D2, int M, int N, class T2 = void>
    using enable_if_matKL_matMN = std::enable_if_t <
                                  D1::RowsAtCompileTime == K &&
                                  D1::ColsAtCompileTime == L &&
                                  D2::RowsAtCompileTime == M &&
                                  D2::ColsAtCompileTime == N,
                                  T2 >;

    template<class D1, int K, int L, class D2, int M, int N, class D3, int O, int P, class T2 = void>
    using enable_if_matKL_matMN_matOP = std::enable_if_t <
                                        D1::RowsAtCompileTime == K &&
                                        D1::ColsAtCompileTime == L &&
                                        D2::RowsAtCompileTime == M &&
                                        D2::ColsAtCompileTime == N &&
                                        D3::RowsAtCompileTime == O &&
                                        D3::ColsAtCompileTime == P,
                                        T2 >;
}

// ////////////////////////////////////////////////////////////////////////// //
//one v3 / m3 / m4
namespace simox::meta
{
    template<class Derived, class T2 = void>
    using enable_if_mat3 = enable_if_matMN<Derived, 3, 3, T2 >;

    template<class Derived, class T2 = void>
    using enable_if_mat4 = enable_if_matMN<Derived, 4, 4, T2 >;

    template<class Derived, class T2 = void>
    using enable_if_vec3 = enable_if_matMN<Derived, 3, 1, T2 >;
}

// ////////////////////////////////////////////////////////////////////////// //
//two
namespace simox::meta
{
#define make_two_combo(n1, r1, c1, n2, r2, c2)                                 \
    template<class D1, class D2, class T2 = void>                              \
    using enable_if_##n1##_##n2 = enable_if_matKL_matMN<                       \
                                  D1, r1, c1, D2, r2, c2, T2>

    make_two_combo(vec3, 3, 1, vec3, 3, 1);
    make_two_combo(vec3, 3, 1, mat3, 3, 3);
    make_two_combo(vec3, 3, 1, mat4, 4, 4);

    make_two_combo(mat3, 3, 3, vec3, 3, 1);
    make_two_combo(mat3, 3, 3, mat3, 3, 3);
    make_two_combo(mat3, 3, 3, mat4, 4, 4);

    make_two_combo(mat4, 4, 4, vec3, 3, 1);
    make_two_combo(mat4, 4, 4, mat3, 3, 3);
    make_two_combo(mat4, 4, 4, mat4, 4, 4);
#undef make_two_combo
}
// ////////////////////////////////////////////////////////////////////////// //
//three
namespace simox::meta
{
#define make_three_combo(n1, r1, c1, n2, r2, c2, n3, r3, c3)                   \
    template<class D1, class D2, class D3, class T2 = void>                    \
    using enable_if_##n1##_##n2##_##n3 = enable_if_matKL_matMN_matOP<          \
                                         D1, r1, c1, D2, r2, c2, D3, r3, c3, T2>

    make_three_combo(vec3, 3, 1, vec3, 3, 1, vec3, 3, 1);
    make_three_combo(vec3, 3, 1, vec3, 3, 1, mat3, 3, 3);
    make_three_combo(vec3, 3, 1, vec3, 3, 1, mat4, 4, 4);
    make_three_combo(vec3, 3, 1, mat3, 3, 3, vec3, 3, 1);
    make_three_combo(vec3, 3, 1, mat3, 3, 3, mat3, 3, 3);
    make_three_combo(vec3, 3, 1, mat3, 3, 3, mat4, 4, 4);
    make_three_combo(vec3, 3, 1, mat4, 4, 4, vec3, 3, 1);
    make_three_combo(vec3, 3, 1, mat4, 4, 4, mat3, 3, 3);
    make_three_combo(vec3, 3, 1, mat4, 4, 4, mat4, 4, 4);

    make_three_combo(mat3, 3, 3, vec3, 3, 1, vec3, 3, 1);
    make_three_combo(mat3, 3, 3, vec3, 3, 1, mat3, 3, 3);
    make_three_combo(mat3, 3, 3, vec3, 3, 1, mat4, 4, 4);
    make_three_combo(mat3, 3, 3, mat3, 3, 3, vec3, 3, 1);
    make_three_combo(mat3, 3, 3, mat3, 3, 3, mat3, 3, 3);
    make_three_combo(mat3, 3, 3, mat3, 3, 3, mat4, 4, 4);
    make_three_combo(mat3, 3, 3, mat4, 4, 4, vec3, 3, 1);
    make_three_combo(mat3, 3, 3, mat4, 4, 4, mat3, 3, 3);
    make_three_combo(mat3, 3, 3, mat4, 4, 4, mat4, 4, 4);

    make_three_combo(mat4, 4, 4, vec3, 3, 1, vec3, 3, 1);
    make_three_combo(mat4, 4, 4, vec3, 3, 1, mat3, 3, 3);
    make_three_combo(mat4, 4, 4, vec3, 3, 1, mat4, 4, 4);
    make_three_combo(mat4, 4, 4, mat3, 3, 3, vec3, 3, 1);
    make_three_combo(mat4, 4, 4, mat3, 3, 3, mat3, 3, 3);
    make_three_combo(mat4, 4, 4, mat3, 3, 3, mat4, 4, 4);
    make_three_combo(mat4, 4, 4, mat4, 4, 4, vec3, 3, 1);
    make_three_combo(mat4, 4, 4, mat4, 4, 4, mat3, 3, 3);
    make_three_combo(mat4, 4, 4, mat4, 4, 4, mat4, 4, 4);
#undef make_three_combo
}
