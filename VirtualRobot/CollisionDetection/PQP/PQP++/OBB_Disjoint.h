/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Gottschalk
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#pragma once

#include <cmath>

#include "MatVec.h"
#include "PQP_Compile.h"

namespace PQP
{


    // int
    // obb_disjoint(PQP_REAL B[3][3], PQP_REAL T[3], PQP_REAL a[3], PQP_REAL b[3]);
    //
    // This is a test between two boxes, box A and box B.  It is assumed that
    // the coordinate system is aligned and centered on box A.  The 3x3
    // matrix B specifies box B's orientation with respect to box A.
    // Specifically, the columns of B are the basis vectors (axis vectors) of
    // box B.  The center of box B is located at the vector T.  The
    // dimensions of box B are given in the array b.  The orientation and
    // placement of box A, in this coordinate system, are the identity matrix
    // and zero vector, respectively, so they need not be specified.  The
    // dimensions of box A are given in array a.
    class OBB_Processor
    {
    public:
        static
        inline
        int
        obb_disjoint(PQP_REAL B[3][3], PQP_REAL T[3], PQP_REAL a[3], PQP_REAL b[3])
        {
            PQP_REAL s;
            PQP_REAL Bf[3][3];

            {
                static constexpr PQP_REAL reps = (PQP_REAL)1e-6;

                // Bf = fabs(B)
                Bf[0][0] = std::abs(B[0][0]) + reps;
                Bf[0][1] = std::abs(B[0][1]) + reps;
                Bf[0][2] = std::abs(B[0][2]) + reps;

                Bf[1][0] = std::abs(B[1][0]) + reps;
                Bf[1][1] = std::abs(B[1][1]) + reps;
                Bf[1][2] = std::abs(B[1][2]) + reps;

                Bf[2][0] = std::abs(B[2][0]) + reps;
                Bf[2][1] = std::abs(B[2][1]) + reps;
                Bf[2][2] = std::abs(B[2][2]) + reps;
            }

            // if any of these tests are one-sided, then the polyhedra are disjoint

            // A1 x A2 = A0
            if (std::abs(T[0]) > (a[0] + b[0] * Bf[0][0] + b[1] * Bf[0][1] + b[2] * Bf[0][2]))
            {
                return 1;
            }

            // A2 x A0 = A1
            if (std::abs(T[1]) > (a[1] + b[0] * Bf[1][0] + b[1] * Bf[1][1] + b[2] * Bf[1][2]))
            {
                return 3;
            }

            // A0 x A1 = A2
            if (std::abs(T[2]) > (a[2] + b[0] * Bf[2][0] + b[1] * Bf[2][1] + b[2] * Bf[2][2]))
            {
                return 4;
            }

            // B1 x B2 = B0
            s = T[0] * B[0][0] + T[1] * B[1][0] + T[2] * B[2][0];
            if (std::abs(s) > (b[0] + a[0] * Bf[0][0] + a[1] * Bf[1][0] + a[2] * Bf[2][0]))
            {
                return 2;
            }

            // B2 x B0 = B1
            s = T[0] * B[0][1] + T[1] * B[1][1] + T[2] * B[2][1];
            if (std::abs(s) > (b[1] + a[0] * Bf[0][1] + a[1] * Bf[1][1] + a[2] * Bf[2][1]))
            {
                return 5;
            }

            // B0 x B1 = B2
            s = T[0] * B[0][2] + T[1] * B[1][2] + T[2] * B[2][2];
            if (std::abs(s) > (b[2] + a[0] * Bf[0][2] + a[1] * Bf[1][2] + a[2] * Bf[2][2]))
            {
                return 6;
            }

            // A0 x B0
            s = T[2] * B[1][0] - T[1] * B[2][0];
            if (std::abs(s) > (a[1] * Bf[2][0] + a[2] * Bf[1][0] + b[1] * Bf[0][2] + b[2] * Bf[0][1]))
            {
                return 7;
            }

            // A0 x B1
            s = T[2] * B[1][1] - T[1] * B[2][1];
            if (std::abs(s) > (a[1] * Bf[2][1] + a[2] * Bf[1][1] + b[0] * Bf[0][2] + b[2] * Bf[0][0]))
            {
                return 8;
            }

            // A0 x B2
            s = T[2] * B[1][2] - T[1] * B[2][2];
            if (std::abs(s) > (a[1] * Bf[2][2] + a[2] * Bf[1][2] + b[0] * Bf[0][1] + b[1] * Bf[0][0]))
            {
                return 9;
            }

            // A1 x B0
            s = T[0] * B[2][0] - T[2] * B[0][0];
            if (std::abs(s) > (a[0] * Bf[2][0] + a[2] * Bf[0][0] + b[1] * Bf[1][2] + b[2] * Bf[1][1]))
            {
                return 10;
            }

            // A1 x B1
            s = T[0] * B[2][1] - T[2] * B[0][1];
            if (std::abs(s) > (a[0] * Bf[2][1] + a[2] * Bf[0][1] + b[0] * Bf[1][2] + b[2] * Bf[1][0]))
            {
                return 11;
            }

            // A1 x B2
            s = T[0] * B[2][2] - T[2] * B[0][2];
            if (std::abs(s) > (a[0] * Bf[2][2] + a[2] * Bf[0][2] + b[0] * Bf[1][1] + b[1] * Bf[1][0]))
            {
                return 12;
            }

            // A2 x B0
            s = T[1] * B[0][0] - T[0] * B[1][0];
            if (std::abs(s) > (a[0] * Bf[1][0] + a[1] * Bf[0][0] + b[1] * Bf[2][2] + b[2] * Bf[2][1]))
            {
                return 13;
            }

            // A2 x B1
            s = T[1] * B[0][1] - T[0] * B[1][1];
            if (std::abs(s) > (a[0] * Bf[1][1] + a[1] * Bf[0][1] + b[0] * Bf[2][2] + b[2] * Bf[2][0]))
            {
                return 14;
            }

            // A2 x B2
            s = T[1] * B[0][2] - T[0] * B[1][2];
            if (std::abs(s) > (a[0] * Bf[1][2] + a[1] * Bf[0][2] + b[0] * Bf[2][1] + b[1] * Bf[2][0]))
            {
                return 15;
            }

            return 0;  // should equal 0
        }

        //used for unittesting (failed attempt to leverage loop optimization)
        static inline int
        obb_disjoint_with_loops(PQP_REAL B[3][3], PQP_REAL T[3], PQP_REAL a[3], PQP_REAL b[3])
        {
            PQP_REAL s;
            PQP_REAL Bf[3][3];

            {
                // Bf = fabs(B)
                for (std::size_t idxa = 0; idxa < 3; ++idxa)
                {
                    for (std::size_t idxb = 0; idxb < 3; ++idxb)
                    {
                        Bf[idxa][idxb] = std::abs(B[idxa][idxb]) + static_cast<PQP_REAL>(1e-6);
                    }
                }
            }

            // if any of these tests are one-sided, then the polyhedra are disjoint
            // 1  T[0]                                                 std::abs(s) > (a[0] + b[0] * Bf[0][0] + b[1] * Bf[0][1] + b[2] * Bf[0][2])
            // 2  T[0] * B[0][0] + T[1] * B[1][0] + T[2] * B[2][0];    std::abs(s) > (b[0] + a[0] * Bf[0][0] + a[1] * Bf[1][0] + a[2] * Bf[2][0])
            // 3  T[1]                                                 std::abs(s) > (a[1] + b[0] * Bf[1][0] + b[1] * Bf[1][1] + b[2] * Bf[1][2])
            // 4  T[2]                                                 std::abs(s) > (a[2] + b[0] * Bf[2][0] + b[1] * Bf[2][1] + b[2] * Bf[2][2])
            // 5  T[0] * B[0][1] + T[1] * B[1][1] + T[2] * B[2][1];    std::abs(s) > (b[1] + a[0] * Bf[0][1] + a[1] * Bf[1][1] + a[2] * Bf[2][1])
            // 6  T[0] * B[0][2] + T[1] * B[1][2] + T[2] * B[2][2];    std::abs(s) > (b[2] + a[0] * Bf[0][2] + a[1] * Bf[1][2] + a[2] * Bf[2][2])
            // A1 x A2 = A0
            if (std::abs(T[0]) > (a[0] + b[0] * Bf[0][0] + b[1] * Bf[0][1] + b[2] * Bf[0][2]))
            {
                return 1;
            }

            // A2 x A0 = A1
            if (std::abs(T[1]) > (a[1] + b[0] * Bf[1][0] + b[1] * Bf[1][1] + b[2] * Bf[1][2]))
            {
                return 3;
            }

            // A0 x A1 = A2
            if (std::abs(T[2]) > (a[2] + b[0] * Bf[2][0] + b[1] * Bf[2][1] + b[2] * Bf[2][2]))
            {
                return 4;
            }

            // B1 x B2 = B0
            s = T[0] * B[0][0] + T[1] * B[1][0] + T[2] * B[2][0];
            if (std::abs(s) > (b[0] + a[0] * Bf[0][0] + a[1] * Bf[1][0] + a[2] * Bf[2][0]))
            {
                return 2;
            }

            // B2 x B0 = B1
            s = T[0] * B[0][1] + T[1] * B[1][1] + T[2] * B[2][1];
            if (std::abs(s) > (b[1] + a[0] * Bf[0][1] + a[1] * Bf[1][1] + a[2] * Bf[2][1]))
            {
                return 5;
            }

            // B0 x B1 = B2
            s = T[0] * B[0][2] + T[1] * B[1][2] + T[2] * B[2][2];
            if (std::abs(s) > (b[2] + a[0] * Bf[0][2] + a[1] * Bf[1][2] + a[2] * Bf[2][2]))
            {
                return 6;
            }

            {
                //     201    120       120    201                                         100     221       221     100       100         221   221        100
                //     (a[o100[idxa]] * Bf[o221[idxa]][idxb] + a[o221[idxa]] * Bf[o100[idxa]][idxb] + b[o100[idxb]] * Bf[idxa][o221[idxb]] + b[o221[idxb]] * Bf[idxa][o100[idxb]])
                //      c?     c? v=     c?     c? v=                                       c?      c? v=     c?      c? v=     ?=      cv ?=     ?=      cv ?=
                // 7  T[2] * B[1][0] - T[1] * B[2][0];                     std::abs(s) > (a[1] * Bf[2][0] + a[2] * Bf[1][0] + b[1] * Bf[0][2] + b[2] * Bf[0][1])      7+3*0+0 = 7
                // 8  T[2] * B[1][1] - T[1] * B[2][1];                     std::abs(s) > (a[1] * Bf[2][1] + a[2] * Bf[1][1] + b[0] * Bf[0][2] + b[2] * Bf[0][0])      7+3*0+1 = 8
                // 9  T[2] * B[1][2] - T[1] * B[2][2];                     std::abs(s) > (a[1] * Bf[2][2] + a[2] * Bf[1][2] + b[0] * Bf[0][1] + b[1] * Bf[0][0])      7+3*0+2 = 9

                //10  T[0] * B[2][0] - T[2] * B[0][0];                     std::abs(s) > (a[0] * Bf[2][0] + a[2] * Bf[0][0] + b[1] * Bf[1][2] + b[2] * Bf[1][1])      7+3*1+0 = 10
                //11  T[0] * B[2][1] - T[2] * B[0][1];                     std::abs(s) > (a[0] * Bf[2][1] + a[2] * Bf[0][1] + b[0] * Bf[1][2] + b[2] * Bf[1][0])
                //12  T[0] * B[2][2] - T[2] * B[0][2];                     std::abs(s) > (a[0] * Bf[2][2] + a[2] * Bf[0][2] + b[0] * Bf[1][1] + b[1] * Bf[1][0])

                //13  T[1] * B[0][0] - T[0] * B[1][0];                     std::abs(s) > (a[0] * Bf[1][0] + a[1] * Bf[0][0] + b[1] * Bf[2][2] + b[2] * Bf[2][1])
                //14  T[1] * B[0][1] - T[0] * B[1][1];                     std::abs(s) > (a[0] * Bf[1][1] + a[1] * Bf[0][1] + b[0] * Bf[2][2] + b[2] * Bf[2][0])
                //15  T[1] * B[0][2] - T[0] * B[1][2];                     std::abs(s) > (a[0] * Bf[1][2] + a[1] * Bf[0][2] + b[0] * Bf[2][1] + b[1] * Bf[2][0])

                static constexpr int o201[3] = {2, 0, 1};
                static constexpr int o120[3] = {1, 2, 0};
                static constexpr int o100[3] = {1, 0, 0};
                static constexpr int o221[3] = {2, 2, 1};
                for (int idxa = 0; idxa < 3; ++idxa)
                {
                    for (int idxb = 0; idxb < 3; ++idxb)
                    {
                        // Aa x Bb
                        if (
                            std::abs(T[o201[idxa]] * B[o120[idxa]][idxb] - T[o120[idxa]] * B[o201[idxa]][idxb]) >
                            (a[o100[idxa]] * Bf[o221[idxa]][idxb] + a[o221[idxa]] * Bf[o100[idxa]][idxb] + b[o100[idxb]] * Bf[idxa][o221[idxb]] + b[o221[idxb]] * Bf[idxa][o100[idxb]])
                        )
                        {
                            return 7 + 3 * idxa + idxb;
                        }
                    }
                }
            }
            return 0;
        }
    };

} // namespace




