////////////////////////////////////////////////////////////////////////////////
//BV.cpp
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

  US Mail:             E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#include <algorithm>

#include <cstdlib>
#include <cmath>

#include "BV.h"
#include "MatVec.h"
#include "RectDist.h"
#include "OBB_Disjoint.h"


namespace PQP
{
    void
    BV::FitToTris(PQP_REAL O[3][3], Tri* tris, int num_tris)
    {
        // store orientation

        pqp_math.McM(R, O);

        // project points of tris to R coordinates

        int num_points = 3 * num_tris;
        PQP_REAL(*P)[3] = new PQP_REAL[num_points][3];
        int point = 0;
        int i;

        for (i = 0; i < num_tris; i++)
        {
            pqp_math.MTxV(P[point], R, tris[i].p1);
            point++;

            pqp_math.MTxV(P[point], R, tris[i].p2);
            point++;

            pqp_math.MTxV(P[point], R, tris[i].p3);
            point++;
        }


#if PQP_BV_TYPE & OBB_TYPE
        {
            PQP_REAL minx, maxx, miny, maxy, minz, maxz, c[3];
            minx = maxx = P[0][0];
            miny = maxy = P[0][1];
            minz = maxz = P[0][2];

            for (i = 1; i < num_points; i++)
            {
                if (P[i][0] < minx)
                {
                    minx = P[i][0];
                }
                else if (P[i][0] > maxx)
                {
                    maxx = P[i][0];
                }

                if (P[i][1] < miny)
                {
                    miny = P[i][1];
                }
                else if (P[i][1] > maxy)
                {
                    maxy = P[i][1];
                }

                if (P[i][2] < minz)
                {
                    minz = P[i][2];
                }
                else if (P[i][2] > maxz)
                {
                    maxz = P[i][2];
                }
            }

            c[0] = (PQP_REAL)0.5 * (maxx + minx);
            c[1] = (PQP_REAL)0.5 * (maxy + miny);
            c[2] = (PQP_REAL)0.5 * (maxz + minz);
            pqp_math.MxV(To, R, c);

            d[0] = (PQP_REAL)0.5 * (maxx - minx);
            d[1] = (PQP_REAL)0.5 * (maxy - miny);
            d[2] = (PQP_REAL)0.5 * (maxz - minz);
        }
#endif

#if PQP_BV_TYPE & RSS_TYPE
        {
            PQP_REAL minx, maxx, miny, maxy, c[3];
            PQP_REAL cz, radsqr;

            // compute thickness, which determines radius, and z of rectangle corner
            //AND
            // compute an initial length of rectangle along x / y direction
            // find minx and maxx / miny and maxy as starting points

            {
                int x_minindex = 0;
                int x_maxindex = 0;
                int y_minindex = 0;
                int y_maxindex = 0;

                {
                    PQP_REAL minz = P[0][2];
                    PQP_REAL maxz = P[0][2];

                    for (i = 1; i < num_points; i++)
                    {
                        if (P[i][0] < P[x_minindex][0])
                        {
                            x_minindex = i;
                        }
                        else if (P[i][0] > P[x_maxindex][0])
                        {
                            x_maxindex = i;
                        }

                        if (P[i][1] < P[y_minindex][1])
                        {
                            y_minindex = i;
                        }
                        else if (P[i][1] > P[y_maxindex][1])
                        {
                            y_maxindex = i;
                        }

                        if (P[i][2] < minz)
                        {
                            minz = P[i][2];
                        }
                        else if (P[i][2] > maxz)
                        {
                            maxz = P[i][2];
                        }
                    }
                    r = (PQP_REAL)0.5 * (maxz - minz);
                    radsqr = r * r;
                    cz = (PQP_REAL)0.5 * (maxz + minz);
                }

                PQP_REAL dz;
                dz = P[x_minindex][2] - cz;
                minx = P[x_minindex][0] + sqrt(std::max(radsqr - dz * dz, 0.f));
                dz = P[x_maxindex][2] - cz;
                maxx = P[x_maxindex][0] - sqrt(std::max(radsqr - dz * dz, 0.f));

                dz = P[y_minindex][2] - cz;
                miny = P[y_minindex][1] + sqrt(std::max(radsqr - dz * dz, 0.f));
                dz = P[y_maxindex][2] - cz;
                maxy = P[y_maxindex][1] - sqrt(std::max(radsqr - dz * dz, 0.f));
            }

            // grow minx / maxx / miny / maxy
            {
                for (i = 0; i < num_points; i++)
                {
                    // grow minx
                    if (P[i][0] < minx)
                    {
                        const PQP_REAL dz = P[i][2] - cz;
                        const PQP_REAL x = P[i][0] + sqrt(std::max(radsqr - dz * dz, 0.f));

                        if (x < minx)
                        {
                            minx = x;
                        }
                    }

                    // grow maxx
                    if (P[i][0] > maxx)
                    {
                        const PQP_REAL dz = P[i][2] - cz;
                        const PQP_REAL x = P[i][0] - sqrt(std::max(radsqr - dz * dz, 0.f));

                        if (x > maxx)
                        {
                            maxx = x;
                        }
                    }

                    // grow miny
                    if (P[i][1] < miny)
                    {
                        const PQP_REAL dz = P[i][2] - cz;
                        const PQP_REAL y = P[i][1] + sqrt(std::max(radsqr - dz * dz, 0.f));

                        if (y < miny)
                        {
                            miny = y;
                        }
                    }

                    // grow maxy
                    if (P[i][1] > maxy)
                    {
                        const PQP_REAL dz = P[i][2] - cz;
                        const PQP_REAL y = P[i][1] - sqrt(std::max(radsqr - dz * dz, 0.f));

                        if (y > maxy)
                        {
                            maxy = y;
                        }
                    }
                }
            }

            // corners may have some points which are not covered - grow lengths if
            // necessary

            PQP_REAL a = sqrt((PQP_REAL)0.5);

            for (i = 0; i < num_points; i++)
            {
                if (P[i][0] > maxx)
                {
                    if (P[i][1] > maxy)
                    {
                        const PQP_REAL dx = P[i][0] - maxx;
                        const PQP_REAL dy = P[i][1] - maxy;
                        PQP_REAL u = dx * a + dy * a;
                        const PQP_REAL t = (a * u - dx) * (a * u - dx) +
                                           (a * u - dy) * (a * u - dy) +
                                           (cz - P[i][2]) * (cz - P[i][2]);
                        u = u - sqrt(std::max(radsqr - t, 0.f));

                        if (u > 0)
                        {
                            maxx += u * a;
                            maxy += u * a;
                        }
                    }
                    else if (P[i][1] < miny)
                    {
                        const PQP_REAL dx = P[i][0] - maxx;
                        const PQP_REAL dy = P[i][1] - miny;
                        PQP_REAL u = dx * a - dy * a;
                        const PQP_REAL t = (a * u - dx) * (a * u - dx) +
                                           (-a * u - dy) * (-a * u - dy) +
                                           (cz - P[i][2]) * (cz - P[i][2]);
                        u = u - sqrt(std::max(radsqr - t, 0.f));

                        if (u > 0)
                        {
                            maxx += u * a;
                            miny -= u * a;
                        }
                    }
                }
                else if (P[i][0] < minx)
                {
                    if (P[i][1] > maxy)
                    {
                        const PQP_REAL dx = P[i][0] - minx;
                        const PQP_REAL dy = P[i][1] - maxy;
                        PQP_REAL u = dy * a - dx * a;
                        const PQP_REAL t = (-a * u - dx) * (-a * u - dx) +
                                           (a * u - dy) * (a * u - dy) +
                                           (cz - P[i][2]) * (cz - P[i][2]);
                        u = u - sqrt(std::max(radsqr - t, 0.f));

                        if (u > 0)
                        {
                            minx -= u * a;
                            maxy += u * a;
                        }
                    }
                    else if (P[i][1] < miny)
                    {
                        const PQP_REAL dx = P[i][0] - minx;
                        const PQP_REAL dy = P[i][1] - miny;
                        PQP_REAL u = -dx * a - dy * a;
                        const PQP_REAL t = (-a * u - dx) * (-a * u - dx) +
                                           (-a * u - dy) * (-a * u - dy) +
                                           (cz - P[i][2]) * (cz - P[i][2]);
                        u = u - sqrt(std::max(radsqr - t, 0.f));

                        if (u > 0)
                        {
                            minx -= u * a;
                            miny -= u * a;
                        }
                    }
                }
            }

            c[0] = minx;
            c[1] = miny;
            c[2] = cz;
            pqp_math.MxV(Tr, R, c);

            l[0] = maxx - minx;

            if (l[0] < 0)
            {
                l[0] = 0;
            }

            l[1] = maxy - miny;

            if (l[1] < 0)
            {
                l[1] = 0;
            }
        }

#endif

        delete [] P;
    }

    int
    BV_Processor::BV_Overlap(PQP_REAL R[3][3], PQP_REAL T[3], BV* b1, BV* b2)
    {
#if PQP_BV_TYPE & OBB_TYPE
        return (OBB_Processor::obb_disjoint(R, T, b1->d, b2->d) == 0);
#else
        PQP_REAL dist = p.RectDist(R, T, b1->l, b2->l);

        if (dist <= (b1->r + b2->r))
        {
            return 1;
        }

        return 0;
#endif
    }

#if PQP_BV_TYPE & RSS_TYPE
    PQP_REAL
    BV_Processor::BV_Distance(PQP_REAL R[3][3], PQP_REAL T[3], BV* b1, BV* b2)
    {
        PQP_REAL dist = p.RectDist(R, T, b1->l, b2->l);
        dist -= (b1->r + b2->r);
        return (dist < (PQP_REAL)0.0) ? (PQP_REAL)0.0 : dist;
    }
#endif

} // namespace


////////////////////////////////////////////////////////////////////////////////
//Build.cpp
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

  US Mail:             S. Gottschalk, E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "PQP.h"
#include "Build.h"
#include "MatVec.h"

// If this is set, build routines will use covariance matrix
// and mean finding code from RAPID 2.

#define RAPID2_FIT 0


namespace PQP
{

#if RAPID2_FIT

    struct moment
    {
        PQP_REAL A;
        PQP_REAL m[3];
        PQP_REAL s[3][3];
    };

    struct accum
    {
        PQP_REAL A;
        PQP_REAL m[3];
        PQP_REAL s[3][3];
    };

    inline
    void
    clear_accum(accum& a)
    {
        a.m[0] = a.m[1] = a.m[2] = 0.0;
        a.s[0][0] = a.s[0][1] = a.s[0][2] = 0.0;
        a.s[1][0] = a.s[1][1] = a.s[1][2] = 0.0;
        a.s[2][0] = a.s[2][1] = a.s[2][2] = 0.0;
        a.A = 0.0;
    }

    inline
    void
    accum_moment(accum& a, moment& b)
    {
        a.m[0] += b.m[0] * b.A;
        a.m[1] += b.m[1] * b.A;
        a.m[2] += b.m[2] * b.A;

        a.s[0][0] += b.s[0][0];
        a.s[0][1] += b.s[0][1];
        a.s[0][2] += b.s[0][2];
        a.s[1][0] += b.s[1][0];
        a.s[1][1] += b.s[1][1];
        a.s[1][2] += b.s[1][2];
        a.s[2][0] += b.s[2][0];
        a.s[2][1] += b.s[2][1];
        a.s[2][2] += b.s[2][2];

        a.A += b.A;
    }

    inline
    void
    mean_from_moment(PQP_REAL M[3], moment& m)
    {
        M[0] = m.m[0];
        M[1] = m.m[1];
        M[2] = m.m[2];
    }

    inline
    void
    mean_from_accum(PQP_REAL M[3], accum& a)
    {
        M[0] = a.m[0] / a.A;
        M[1] = a.m[1] / a.A;
        M[2] = a.m[2] / a.A;
    }

    inline
    void
    covariance_from_accum(PQP_REAL C[3][3], accum& a)
    {
        int i, j;

        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
            {
                C[i][j] = a.s[i][j] - a.m[i] * a.m[j] / a.A;
            }
    }

    inline
    void
    compute_moment(moment& M, PQP_REAL p[3], PQP_REAL q[3], PQP_REAL r[3])
    {
        PQP_REAL u[3], v[3], w[3];

        // compute the area of the triangle
        VmV(u, q, p);
        VmV(v, r, p);
        VcrossV(w, u, v);
        M.A = 0.5 * Vlength(w);

        if (M.A == 0.0)
        {
            // This triangle has zero area.  The second order components
            // would be eliminated with the usual formula, so, for the
            // sake of robustness we use an alternative form.  These are the
            // centroid and second-order components of the triangle's vertices.

            // centroid
            M.m[0] = (p[0] + q[0] + r[0]) / 3;
            M.m[1] = (p[1] + q[1] + r[1]) / 3;
            M.m[2] = (p[2] + q[2] + r[2]) / 3;

            // second-order components
            M.s[0][0] = (p[0] * p[0] + q[0] * q[0] + r[0] * r[0]);
            M.s[0][1] = (p[0] * p[1] + q[0] * q[1] + r[0] * r[1]);
            M.s[0][2] = (p[0] * p[2] + q[0] * q[2] + r[0] * r[2]);
            M.s[1][1] = (p[1] * p[1] + q[1] * q[1] + r[1] * r[1]);
            M.s[1][2] = (p[1] * p[2] + q[1] * q[2] + r[1] * r[2]);
            M.s[2][2] = (p[2] * p[2] + q[2] * q[2] + r[2] * r[2]);
            M.s[2][1] = M.s[1][2];
            M.s[1][0] = M.s[0][1];
            M.s[2][0] = M.s[0][2];

            return;
        }

        // get the centroid
        M.m[0] = (p[0] + q[0] + r[0]) / 3;
        M.m[1] = (p[1] + q[1] + r[1]) / 3;
        M.m[2] = (p[2] + q[2] + r[2]) / 3;

        // get the second order components -- note the weighting by the area
        M.s[0][0] = M.A * (9 * M.m[0] * M.m[0] + p[0] * p[0] + q[0] * q[0] + r[0] * r[0]) / 12;
        M.s[0][1] = M.A * (9 * M.m[0] * M.m[1] + p[0] * p[1] + q[0] * q[1] + r[0] * r[1]) / 12;
        M.s[1][1] = M.A * (9 * M.m[1] * M.m[1] + p[1] * p[1] + q[1] * q[1] + r[1] * r[1]) / 12;
        M.s[0][2] = M.A * (9 * M.m[0] * M.m[2] + p[0] * p[2] + q[0] * q[2] + r[0] * r[2]) / 12;
        M.s[1][2] = M.A * (9 * M.m[1] * M.m[2] + p[1] * p[2] + q[1] * q[2] + r[1] * r[2]) / 12;
        M.s[2][2] = M.A * (9 * M.m[2] * M.m[2] + p[2] * p[2] + q[2] * q[2] + r[2] * r[2]) / 12;
        M.s[2][1] = M.s[1][2];
        M.s[1][0] = M.s[0][1];
        M.s[2][0] = M.s[0][2];
    }

    inline
    void
    compute_moments(moment* M, Tri* tris, int num_tris)
    {
        int i;

        // first collect all the moments, and obtain the area of the
        // smallest nonzero area triangle.

        PQP_REAL Amin = 0.0;
        int zero = 0;
        int nonzero = 0;

        for (i = 0; i < num_tris; i++)
        {
            compute_moment(M[i],
                           tris[i].p1,
                           tris[i].p2,
                           tris[i].p3);

            if (M[i].A == 0.0)
            {
                zero = 1;
            }
            else
            {
                nonzero = 1;

                if (Amin == 0.0)
                {
                    Amin = M[i].A;
                }
                else if (M[i].A < Amin)
                {
                    Amin = M[i].A;
                }
            }
        }

        if (zero)
        {
            fprintf(stderr, "----\n");
            fprintf(stderr, "Warning!  Some triangles have zero area!\n");
            fprintf(stderr, "----\n");

            // if there are any zero area triangles, go back and set their area

            // if ALL the triangles have zero area, then set the area thingy
            // to some arbitrary value.
            if (Amin == 0.0)
            {
                Amin = 1.0;
            }

            for (i = 0; i < num_tris; i++)
            {
                if (M[i].A == 0.0)
                {
                    M[i].A = Amin;
                }
            }
        }
    }

#else

    PQP_REAL Builder::max(PQP_REAL a, PQP_REAL b, PQP_REAL c, PQP_REAL d)
    {
        PQP_REAL t = a;

        if (b > t)
        {
            t = b;
        }

        if (c > t)
        {
            t = c;
        }

        if (d > t)
        {
            t = d;
        }

        return t;
    }

    PQP_REAL Builder::min(PQP_REAL a, PQP_REAL b, PQP_REAL c, PQP_REAL d)
    {
        PQP_REAL t = a;

        if (b < t)
        {
            t = b;
        }

        if (c < t)
        {
            t = c;
        }

        if (d < t)
        {
            t = d;
        }

        return t;
    }

    void
    Builder::get_centroid_triverts(PQP_REAL c[3], Tri* tris, int num_tris)
    {
        int i;

        c[0] = c[1] = c[2] = 0.0;

        // get center of mass
        for (i = 0; i < num_tris; i++)
        {
            PQP_REAL* p1 = tris[i].p1;
            PQP_REAL* p2 = tris[i].p2;
            PQP_REAL* p3 = tris[i].p3;

            c[0] += p1[0] + p2[0] + p3[0];
            c[1] += p1[1] + p2[1] + p3[1];
            c[2] += p1[2] + p2[2] + p3[2];
        }

        PQP_REAL n = (PQP_REAL)(3 * num_tris);

        c[0] /= n;
        c[1] /= n;
        c[2] /= n;
    }

    void
    Builder::get_covariance_triverts(PQP_REAL M[3][3], Tri* tris, int num_tris)
    {
        int i;
        PQP_REAL S1[3];
        PQP_REAL S2[3][3];

        S1[0] = S1[1] = S1[2] = 0.0;
        S2[0][0] = S2[1][0] = S2[2][0] = 0.0;
        S2[0][1] = S2[1][1] = S2[2][1] = 0.0;
        S2[0][2] = S2[1][2] = S2[2][2] = 0.0;

        // get center of mass
        for (i = 0; i < num_tris; i++)
        {
            PQP_REAL* p1 = tris[i].p1;
            PQP_REAL* p2 = tris[i].p2;
            PQP_REAL* p3 = tris[i].p3;

            S1[0] += p1[0] + p2[0] + p3[0];
            S1[1] += p1[1] + p2[1] + p3[1];
            S1[2] += p1[2] + p2[2] + p3[2];

            S2[0][0] += (p1[0] * p1[0] +
                         p2[0] * p2[0] +
                         p3[0] * p3[0]);
            S2[1][1] += (p1[1] * p1[1] +
                         p2[1] * p2[1] +
                         p3[1] * p3[1]);
            S2[2][2] += (p1[2] * p1[2] +
                         p2[2] * p2[2] +
                         p3[2] * p3[2]);
            S2[0][1] += (p1[0] * p1[1] +
                         p2[0] * p2[1] +
                         p3[0] * p3[1]);
            S2[0][2] += (p1[0] * p1[2] +
                         p2[0] * p2[2] +
                         p3[0] * p3[2]);
            S2[1][2] += (p1[1] * p1[2] +
                         p2[1] * p2[2] +
                         p3[1] * p3[2]);
        }

        PQP_REAL n = (PQP_REAL)(3 * num_tris);

        // now get covariances

        M[0][0] = S2[0][0] - S1[0] * S1[0] / n;
        M[1][1] = S2[1][1] - S1[1] * S1[1] / n;
        M[2][2] = S2[2][2] - S1[2] * S1[2] / n;
        M[0][1] = S2[0][1] - S1[0] * S1[1] / n;
        M[1][2] = S2[1][2] - S1[1] * S1[2] / n;
        M[0][2] = S2[0][2] - S1[0] * S1[2] / n;
        M[1][0] = M[0][1];
        M[2][0] = M[0][2];
        M[2][1] = M[1][2];
    }

#endif

    // given a list of triangles, a splitting axis, and a coordinate on
    // that axis, partition the triangles into two groups according to
    // where their centroids fall on the axis (under axial projection).
    // Returns the number of tris in the first half

    int
    Builder::split_tris(Tri* tris, int num_tris, PQP_REAL a[3], PQP_REAL c)
    {
        int i;
        int c1 = 0;
        PQP_REAL p[3];
        PQP_REAL x;
        Tri temp;

        for (i = 0; i < num_tris; i++)
        {
            // loop invariant: up to (but not including) index c1 in group 1,
            // then up to (but not including) index i in group 2
            //
            //  [1] [1] [1] [1] [2] [2] [2] [x] [x] ... [x]
            //                   c1          i
            //
            pqp_math.VcV(p, tris[i].p1);
            pqp_math.VpV(p, p, tris[i].p2);
            pqp_math.VpV(p, p, tris[i].p3);
            x = pqp_math.VdotV(p, a);
            x /= 3.0;

            if (x <= c)
            {
                // group 1
                temp = tris[i];
                tris[i] = tris[c1];
                tris[c1] = temp;
                c1++;
            }
            else
            {
                // group 2 -- do nothing
            }
        }

        // split arbitrarily if one group empty

        if ((c1 == 0) || (c1 == num_tris))
        {
            c1 = num_tris / 2;
        }

        return c1;
    }

    // Fits m->child(bn) to the num_tris triangles starting at first_tri
    // Then, if num_tris is greater than one, partitions the tris into two
    // sets, and recursively builds two children of m->child(bn)

    int
    Builder::build_recurse(PQP_Model* m, int bn, int first_tri, int num_tris)
    {
        BV* b = m->child(bn);

        // compute a rotation matrix

        PQP_REAL C[3][3], E[3][3], R[3][3], s[3], axis[3], mean[3], coord;
        s[0] = 0;
        s[1] = 0;
        s[2] = 0;

#if RAPID2_FIT
        moment* tri_moment = new moment[num_tris];
        compute_moments(tri_moment, &(m->tris[first_tri]), num_tris);
        accum acc;
        clear_accum(acc);

        for (int i = 0; i < num_tris; i++)
        {
            accum_moment(acc, tri_moment[i]);
        }

        delete [] tri_moment;
        covariance_from_accum(C, acc);
#else
        get_covariance_triverts(C, &m->tris[first_tri], num_tris);
#endif

        pqp_math.Meigen(E, s, C);

        // place axes of E in order of increasing s

        int min, mid, max;

        if (s[0] > s[1])
        {
            max = 0;
            min = 1;
        }
        else
        {
            min = 0;
            max = 1;
        }

        if (s[2] < s[min])
        {
            mid = min;
            min = 2;
        }
        else if (s[2] > s[max])
        {
            mid = max;
            max = 2;
        }
        else
        {
            mid = 2;
        }

        pqp_math.McolcMcol(R, 0, E, max);
        pqp_math.McolcMcol(R, 1, E, mid);
        R[0][2] = E[1][max] * E[2][mid] - E[1][mid] * E[2][max];
        R[1][2] = E[0][mid] * E[2][max] - E[0][max] * E[2][mid];
        R[2][2] = E[0][max] * E[1][mid] - E[0][mid] * E[1][max];

        // fit the BV

        b->FitToTris(R, &m->tris[first_tri], num_tris);

        if (num_tris == 1)
        {
            // BV is a leaf BV - first_child will index a triangle

            b->first_child = -(first_tri + 1);
        }
        else if (num_tris > 1)
        {
            // BV not a leaf - first_child will index a BV

            b->first_child = m->num_bvs;
            m->num_bvs += 2;

            // choose splitting axis and splitting coord

            pqp_math.McolcV(axis, R, 0);

#if RAPID2_FIT
            mean_from_accum(mean, acc);
#else
            get_centroid_triverts(mean, &m->tris[first_tri], num_tris);
#endif
            coord = pqp_math.VdotV(axis, mean);

            // now split

            int num_first_half = split_tris(&m->tris[first_tri], num_tris,
                                            axis, coord);

            // recursively build the children

            build_recurse(m, m->child(bn)->first_child, first_tri, num_first_half);
            build_recurse(m, m->child(bn)->first_child + 1,
                          first_tri + num_first_half, num_tris - num_first_half);
        }

        return PQP_OK;
    }

    // this descends the hierarchy, converting world-relative
    // transforms to parent-relative transforms

    void
    Builder::make_parent_relative(PQP_Model* m, int bn,
                                  const PQP_REAL parentR[3][3]
#if PQP_BV_TYPE & RSS_TYPE
                                  , const PQP_REAL parentTr[3]
#endif
#if PQP_BV_TYPE & OBB_TYPE
                                  , const PQP_REAL parentTo[3]
#endif
                                 )
    {
        PQP_REAL Rpc[3][3], Tpc[3];

        if (!m->child(bn)->Leaf())
        {
            // make children parent-relative

            make_parent_relative(m, m->child(bn)->first_child,
                                 m->child(bn)->R
#if PQP_BV_TYPE & RSS_TYPE
                                 , m->child(bn)->Tr
#endif
#if PQP_BV_TYPE & OBB_TYPE
                                 , m->child(bn)->To
#endif
                                );
            make_parent_relative(m, m->child(bn)->first_child + 1,
                                 m->child(bn)->R
#if PQP_BV_TYPE & RSS_TYPE
                                 , m->child(bn)->Tr
#endif
#if PQP_BV_TYPE & OBB_TYPE
                                 , m->child(bn)->To
#endif
                                );
        }

        // make self parent relative

        pqp_math.MTxM(Rpc, parentR, m->child(bn)->R);
        pqp_math.McM(m->child(bn)->R, Rpc);
#if PQP_BV_TYPE & RSS_TYPE
        pqp_math.VmV(Tpc, m->child(bn)->Tr, parentTr);
        pqp_math.MTxV(m->child(bn)->Tr, parentR, Tpc);
#endif
#if PQP_BV_TYPE & OBB_TYPE
        pqp_math.VmV(Tpc, m->child(bn)->To, parentTo);
        pqp_math.MTxV(m->child(bn)->To, parentR, Tpc);
#endif

    }

    int
    Builder::build_model(PQP_Model* m)
    {
        // set num_bvs to 1, the first index for a child bv

        m->num_bvs = 1;

        // build recursively

        build_recurse(m, 0, 0, m->num_tris);

        // change BV orientations from world-relative to parent-relative

        PQP_REAL R[3][3], T[3];
        pqp_math.Midentity(R);
        pqp_math.Videntity(T);

        make_parent_relative(m, 0, R
#if PQP_BV_TYPE & RSS_TYPE
                             , T
#endif
#if PQP_BV_TYPE & OBB_TYPE
                             , T
#endif
                            );

        return PQP_OK;
    }

} // namespace
////////////////////////////////////////////////////////////////////////////////
//PQP.cpp
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

  US Mail:             S. Gottschalk, E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#include <cstdio>
#include <cstring>
#include "PQP.h"
#include "BVTQ.h"
#include "Build.h"
#include "MatVec.h"
#include "GetTime.h"
#include "TriDist.h"

namespace PQP
{

    enum BUILD_STATE
    {
        PQP_BUILD_STATE_EMPTY,     // empty state, immediately after constructor
        PQP_BUILD_STATE_BEGUN,     // after BeginModel(), state for adding triangles
        PQP_BUILD_STATE_PROCESSED  // after tree has been built, ready to use
    };



    PQP_Model::PQP_Model()
    {
        // no bounding volume tree yet

        b = nullptr;
        num_bvs_alloced = 0;
        num_bvs = 0;

        // no tri list yet

        tris = nullptr;
        num_tris = 0;
        num_tris_alloced = 0;

        last_tri = nullptr;

        build_state = PQP_BUILD_STATE_EMPTY;
    }

    PQP_Model::~PQP_Model()
    {
        if (b != nullptr)
        {
            delete [] b;
        }

        if (tris != nullptr)
        {
            delete [] tris;
        }
    }

    int
    PQP_Model::BeginModel(int n)
    {
        // reset to initial state if necessary

        if (build_state != PQP_BUILD_STATE_EMPTY)
        {
            delete [] b;
            delete [] tris;

            num_tris = num_bvs = num_tris_alloced = num_bvs_alloced = 0;
        }

        // prepare model for addition of triangles

        if (n <= 0)
        {
            n = 8;
        }

        num_tris_alloced = n;
        tris = new Tri[n];

        if (!tris)
        {
            fprintf(stderr, "PQP Error!  Out of memory for tri array on "
                    "BeginModel() call!\n");
            return PQP_ERR_MODEL_OUT_OF_MEMORY;
        }

        // give a warning if called out of sequence

        if (build_state != PQP_BUILD_STATE_EMPTY)
        {
            fprintf(stderr,
                    "PQP Warning! Called BeginModel() on a PQP_Model that \n"
                    "was not empty. This model was cleared and previous\n"
                    "triangle additions were lost.\n");
            build_state = PQP_BUILD_STATE_BEGUN;
            return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
        }

        build_state = PQP_BUILD_STATE_BEGUN;
        return PQP_OK;
    }

    int
    PQP_Model::AddTri(const PQP_REAL* p1,
                      const PQP_REAL* p2,
                      const PQP_REAL* p3,
                      int id)
    {
        if (build_state == PQP_BUILD_STATE_EMPTY)
        {
            BeginModel();
        }
        else if (build_state == PQP_BUILD_STATE_PROCESSED)
        {
            fprintf(stderr, "PQP Warning! Called AddTri() on PQP_Model \n"
                    "object that was already ended. AddTri() was\n"
                    "ignored.  Must do a BeginModel() to clear the\n"
                    "model for addition of new triangles\n");
            return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
        }

        // allocate for new triangles

        if (num_tris >= num_tris_alloced)
        {
            Tri* temp;
            temp = new Tri[num_tris_alloced * 2];

            if (!temp)
            {
                fprintf(stderr, "PQP Error!  Out of memory for tri array on"
                        " AddTri() call!\n");
                return PQP_ERR_MODEL_OUT_OF_MEMORY;
            }

            memcpy(temp, tris, sizeof(Tri)*num_tris);
            delete [] tris;
            tris = temp;
            num_tris_alloced = num_tris_alloced * 2;
        }

        // initialize the new triangle

        tris[num_tris].p1[0] = p1[0];
        tris[num_tris].p1[1] = p1[1];
        tris[num_tris].p1[2] = p1[2];

        tris[num_tris].p2[0] = p2[0];
        tris[num_tris].p2[1] = p2[1];
        tris[num_tris].p2[2] = p2[2];

        tris[num_tris].p3[0] = p3[0];
        tris[num_tris].p3[1] = p3[1];
        tris[num_tris].p3[2] = p3[2];

        tris[num_tris].id = id;

        num_tris += 1;

        return PQP_OK;
    }

    int
    PQP_Model::EndModel()
    {
        if (build_state == PQP_BUILD_STATE_PROCESSED)
        {
            fprintf(stderr, "PQP Warning! Called EndModel() on PQP_Model \n"
                    "object that was already ended. EndModel() was\n"
                    "ignored.  Must do a BeginModel() to clear the\n"
                    "model for addition of new triangles\n");
            return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
        }

        // report error is no tris

        if (num_tris == 0)
        {
            fprintf(stderr, "PQP Error! EndModel() called on model with"
                    " no triangles\n");
            return PQP_ERR_BUILD_EMPTY_MODEL;
        }

        // shrink fit tris array

        if (num_tris_alloced > num_tris)
        {
            Tri* new_tris = new Tri[num_tris];

            if (!new_tris)
            {
                fprintf(stderr, "PQP Error!  Out of memory for tri array "
                        "in EndModel() call!\n");
                return PQP_ERR_MODEL_OUT_OF_MEMORY;
            }

            memcpy(new_tris, tris, sizeof(Tri)*num_tris);
            delete [] tris;
            tris = new_tris;
            num_tris_alloced = num_tris;
        }

        // create an array of BVs for the model

        b = new BV[2 * num_tris - 1];

        if (!b)
        {
            fprintf(stderr, "PQP Error! out of memory for BV array "
                    "in EndModel()\n");
            return PQP_ERR_MODEL_OUT_OF_MEMORY;
        }

        num_bvs_alloced = 2 * num_tris - 1;
        num_bvs = 0;

        // we should build the model now.
        Builder b;
        b.build_model(this);
        build_state = PQP_BUILD_STATE_PROCESSED;

        last_tri = tris;

        return PQP_OK;
    }

    int
    PQP_Model::MemUsage(int msg)
    {
        int mem_bv_list = sizeof(BV) * num_bvs;
        int mem_tri_list = sizeof(Tri) * num_tris;

        int total_mem = mem_bv_list + mem_tri_list + sizeof(PQP_Model);

        if (msg)
        {
            //fprintf(stderr,"Total for model %x: %d bytes\n", (unsigned int)this, total_mem);
            fprintf(stderr, "BVs: %d alloced, take %zu bytes each\n",
                    num_bvs, sizeof(BV));
            fprintf(stderr, "Tris: %d alloced, take %zu bytes each\n",
                    num_tris, sizeof(Tri));
        }

        return total_mem;
    }

    //  COLLIDE STUFF
    //
    //--------------------------------------------------------------------------

    PQP_CollideResult::PQP_CollideResult()
    {
        pairs = nullptr;
        num_pairs = num_pairs_alloced = 0;
        num_bv_tests = 0;
        num_tri_tests = 0;
    }

    PQP_CollideResult::~PQP_CollideResult()
    {
        delete [] pairs;
    }

    void
    PQP_CollideResult::FreePairsList()
    {
        num_pairs = num_pairs_alloced = 0;
        delete [] pairs;
        pairs = nullptr;
    }

    // may increase OR reduce mem usage
    void
    PQP_CollideResult::SizeTo(int n)
    {
        CollisionPair* temp;

        if (n < num_pairs)
        {
            fprintf(stderr, "PQP Error: Internal error in "
                    "'PQP_CollideResult::SizeTo(int n)'\n");
            fprintf(stderr, "       n = %d, but num_pairs = %d\n", n, num_pairs);
            return;
        }

        temp = new CollisionPair[n];
        memcpy(temp, pairs, num_pairs * sizeof(CollisionPair));
        delete [] pairs;
        pairs = temp;
        num_pairs_alloced = n;
        return;
    }

    void
    PQP_CollideResult::Add(int a, int b)
    {
        if (num_pairs >= num_pairs_alloced)
        {
            // allocate more

            SizeTo(num_pairs_alloced * 2 + 8);
        }

        // now proceed as usual

        pairs[num_pairs].id1 = a;
        pairs[num_pairs].id2 = b;
        num_pairs++;
    }

    // TRIANGLE OVERLAP TEST

    inline
    PQP_REAL
    PQP_Checker::pqp_max(PQP_REAL a, PQP_REAL b, PQP_REAL c)
    {
        PQP_REAL t = a;

        if (b > t)
        {
            t = b;
        }

        if (c > t)
        {
            t = c;
        }

        return t;
    }

    inline
    PQP_REAL
    PQP_Checker::pqp_min(PQP_REAL a, PQP_REAL b, PQP_REAL c)
    {
        PQP_REAL t = a;

        if (b < t)
        {
            t = b;
        }

        if (c < t)
        {
            t = c;
        }

        return t;
    }

    int
    PQP_Checker::project6(PQP_REAL* ax,
                          PQP_REAL* p1, PQP_REAL* p2, PQP_REAL* p3,
                          PQP_REAL* q1, PQP_REAL* q2, PQP_REAL* q3)
    {
        PQP_REAL P1 = pqp_math.VdotV(ax, p1);
        PQP_REAL P2 = pqp_math.VdotV(ax, p2);
        PQP_REAL P3 = pqp_math.VdotV(ax, p3);
        PQP_REAL Q1 = pqp_math.VdotV(ax, q1);
        PQP_REAL Q2 = pqp_math.VdotV(ax, q2);
        PQP_REAL Q3 = pqp_math.VdotV(ax, q3);

        PQP_REAL mx1 = pqp_max(P1, P2, P3);
        PQP_REAL mn1 = pqp_min(P1, P2, P3);
        PQP_REAL mx2 = pqp_max(Q1, Q2, Q3);
        PQP_REAL mn2 = pqp_min(Q1, Q2, Q3);

        if (mn1 > mx2)
        {
            return 0;
        }

        if (mn2 > mx1)
        {
            return 0;
        }

        return 1;
    }

    // very robust triangle intersection test
    // uses no divisions
    // works on coplanar triangles
    int
    PQP_Checker::TriContact(PQP_REAL* P1, PQP_REAL* P2, PQP_REAL* P3,
                            PQP_REAL* Q1, PQP_REAL* Q2, PQP_REAL* Q3)
    {

        // One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
        // Edges are (e1,e2,e3) and (f1,f2,f3).
        // Normals are n1 and m1
        // Outwards are (g1,g2,g3) and (h1,h2,h3).
        //
        // We assume that the triangle vertices are in the same coordinate system.
        //
        // First thing we do is establish a new c.s. so that p1 is at (0,0,0).

        PQP_REAL p1[3], p2[3], p3[3];
        PQP_REAL q1[3], q2[3], q3[3];
        PQP_REAL e1[3], e2[3], e3[3];
        PQP_REAL f1[3], f2[3], f3[3];
        PQP_REAL g1[3], g2[3], g3[3];
        PQP_REAL h1[3], h2[3], h3[3];
        PQP_REAL n1[3], m1[3];

        PQP_REAL ef11[3], ef12[3], ef13[3];
        PQP_REAL ef21[3], ef22[3], ef23[3];
        PQP_REAL ef31[3], ef32[3], ef33[3];

        p1[0] = P1[0] - P1[0];
        p1[1] = P1[1] - P1[1];
        p1[2] = P1[2] - P1[2];
        p2[0] = P2[0] - P1[0];
        p2[1] = P2[1] - P1[1];
        p2[2] = P2[2] - P1[2];
        p3[0] = P3[0] - P1[0];
        p3[1] = P3[1] - P1[1];
        p3[2] = P3[2] - P1[2];

        q1[0] = Q1[0] - P1[0];
        q1[1] = Q1[1] - P1[1];
        q1[2] = Q1[2] - P1[2];
        q2[0] = Q2[0] - P1[0];
        q2[1] = Q2[1] - P1[1];
        q2[2] = Q2[2] - P1[2];
        q3[0] = Q3[0] - P1[0];
        q3[1] = Q3[1] - P1[1];
        q3[2] = Q3[2] - P1[2];

        e1[0] = p2[0] - p1[0];
        e1[1] = p2[1] - p1[1];
        e1[2] = p2[2] - p1[2];
        e2[0] = p3[0] - p2[0];
        e2[1] = p3[1] - p2[1];
        e2[2] = p3[2] - p2[2];
        e3[0] = p1[0] - p3[0];
        e3[1] = p1[1] - p3[1];
        e3[2] = p1[2] - p3[2];

        f1[0] = q2[0] - q1[0];
        f1[1] = q2[1] - q1[1];
        f1[2] = q2[2] - q1[2];
        f2[0] = q3[0] - q2[0];
        f2[1] = q3[1] - q2[1];
        f2[2] = q3[2] - q2[2];
        f3[0] = q1[0] - q3[0];
        f3[1] = q1[1] - q3[1];
        f3[2] = q1[2] - q3[2];

        pqp_math.VcrossV(n1, e1, e2);
        pqp_math.VcrossV(m1, f1, f2);

        pqp_math.VcrossV(g1, e1, n1);
        pqp_math.VcrossV(g2, e2, n1);
        pqp_math.VcrossV(g3, e3, n1);
        pqp_math.VcrossV(h1, f1, m1);
        pqp_math.VcrossV(h2, f2, m1);
        pqp_math.VcrossV(h3, f3, m1);

        pqp_math.VcrossV(ef11, e1, f1);
        pqp_math.VcrossV(ef12, e1, f2);
        pqp_math.VcrossV(ef13, e1, f3);
        pqp_math.VcrossV(ef21, e2, f1);
        pqp_math.VcrossV(ef22, e2, f2);
        pqp_math.VcrossV(ef23, e2, f3);
        pqp_math.VcrossV(ef31, e3, f1);
        pqp_math.VcrossV(ef32, e3, f2);
        pqp_math.VcrossV(ef33, e3, f3);

        // now begin the series of tests

        if (!project6(n1, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(m1, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef11, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef12, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef13, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef21, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef22, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef23, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef31, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef32, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(ef33, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(g1, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(g2, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(g3, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(h1, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(h2, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        if (!project6(h3, p1, p2, p3, q1, q2, q3))
        {
            return 0;
        }

        return 1;
    }

    inline
    PQP_REAL
    PQP_Checker::TriDistance(PQP_REAL R[3][3], PQP_REAL T[3], Tri* t1, Tri* t2,
                             PQP_REAL p[3], PQP_REAL q[3])
    {
        // transform tri 2 into same space as tri 1

        PQP_REAL tri1[3][3], tri2[3][3];

        pqp_math.VcV(tri1[0], t1->p1);
        pqp_math.VcV(tri1[1], t1->p2);
        pqp_math.VcV(tri1[2], t1->p3);
        pqp_math.MxVpV(tri2[0], R, t2->p1, T);
        pqp_math.MxVpV(tri2[1], R, t2->p2, T);
        pqp_math.MxVpV(tri2[2], R, t2->p3, T);

        return triProcessor.TriDist(p, q, tri1, tri2);
    }


    void
    PQP_Checker::CollideRecurse(PQP_CollideResult* res,
                                PQP_REAL R[3][3], PQP_REAL T[3], // b2 relative to b1
                                PQP_Model* o1, int b1,
                                PQP_Model* o2, int b2, int flag)
    {
        // first thing, see if we're overlapping

        res->num_bv_tests++;

        auto o1cb1 = o1->child(b1);
        auto o2cb2 = o2->child(b2);

#if 0 & PQP_BV_TYPE & OBB_TYPE
        //manually inlined version
        //inlined fn
        //BV_Processor::BV_Overlap(PQP_REAL R[3][3], PQP_REAL T[3], BV* b1, BV* b2)
        //{
        //    return (OBB_Processor::obb_disjoint(R, T, b1->d, b2->d) == 0);
        //}
        if (OBB_Processor::obb_disjoint(R, T, o1cb1->d, o2cb2->d) != 0)
        {
            return;
        }
#else
        //original version
        BV_Processor p;
        if (!p.BV_Overlap(R, T, o1cb1, o2cb2))
        {
            return;
        }
#endif

        // if we are, see if we test triangles next

        int l1 = o1cb1->Leaf();
        int l2 = o2cb2->Leaf();

        if (l1 && l2)
        {
            res->num_tri_tests++;

#if 1
            // transform the points in b2 into space of b1, then compare

            Tri* t1 = &o1->tris[-o1cb1->first_child - 1];
            Tri* t2 = &o2->tris[-o2cb2->first_child - 1];
            PQP_REAL q1[3], q2[3], q3[3];
            PQP_REAL* p1 = t1->p1;
            PQP_REAL* p2 = t1->p2;
            PQP_REAL* p3 = t1->p3;
            pqp_math.MxVpV(q1, res->R, t2->p1, res->T);
            pqp_math.MxVpV(q2, res->R, t2->p2, res->T);
            pqp_math.MxVpV(q3, res->R, t2->p3, res->T);

            if (TriContact(p1, p2, p3, q1, q2, q3))
            {
                // add this to result

                res->Add(t1->id, t2->id);
            }

#else
            PQP_REAL p[3], q[3];

            Tri* t1 = &o1->tris[-o1cb1->first_child - 1];
            Tri* t2 = &o2->tris[-o2cb2->first_child - 1];

            if (TriDistance(res->R, res->T, t1, t2, p, q) == 0.0)
            {
                // add this to result

                res->Add(t1->id, t2->id);
            }

#endif

            return;
        }

        // we dont, so decide whose children to visit next

        PQP_REAL sz1 = o1cb1->GetSize();
        PQP_REAL sz2 = o2cb2->GetSize();

        PQP_REAL Rc[3][3], Tc[3], Ttemp[3];

        if (l2 || (!l1 && (sz1 > sz2)))
        {
            int c1 = o1cb1->first_child;
            int c2 = c1 + 1;

            auto o1cc1 = o1->child(c1);

            pqp_math.MTxM(Rc, o1cc1->R, R);
#if PQP_BV_TYPE & OBB_TYPE
            pqp_math.VmV(Ttemp, T, o1cc1->To);
#else
            pqp_math.VmV(Ttemp, T, o1cc1->Tr);
#endif
            pqp_math.MTxV(Tc, o1cc1->R, Ttemp);
            CollideRecurse(res, Rc, Tc, o1, c1, o2, b2, flag);

            if ((flag == PQP_FIRST_CONTACT) && (res->num_pairs > 0))
            {
                return;
            }

            auto o1cc2 = o1->child(c2);
            pqp_math.MTxM(Rc, o1cc2->R, R);
#if PQP_BV_TYPE & OBB_TYPE
            pqp_math.VmV(Ttemp, T, o1cc2->To);
#else
            pqp_math.VmV(Ttemp, T, o1cc2->Tr);
#endif
            pqp_math.MTxV(Tc, o1cc2->R, Ttemp);
            CollideRecurse(res, Rc, Tc, o1, c2, o2, b2, flag);
        }
        else
        {
            int c1 = o2cb2->first_child;
            int c2 = c1 + 1;

            auto o2cc1 = o2->child(c1);

            pqp_math.MxM(Rc, R, o2cc1->R);
#if PQP_BV_TYPE & OBB_TYPE
            pqp_math.MxVpV(Tc, R, o2cc1->To, T);
#else
            pqp_math.MxVpV(Tc, R, o2cc1->Tr, T);
#endif
            CollideRecurse(res, Rc, Tc, o1, b1, o2, c1, flag);

            if ((flag == PQP_FIRST_CONTACT) && (res->num_pairs > 0))
            {
                return;
            }

            auto o2cc2 = o2->child(c2);
            pqp_math.MxM(Rc, R, o2cc2->R);
#if PQP_BV_TYPE & OBB_TYPE
            pqp_math.MxVpV(Tc, R, o2cc2->To, T);
#else
            pqp_math.MxVpV(Tc, R, o2cc2->Tr, T);
#endif
            CollideRecurse(res, Rc, Tc, o1, b1, o2, c2, flag);
        }
    }

    int
    PQP_Checker::PQP_Collide(PQP_CollideResult* res,
                             PQP_REAL R1[3][3], PQP_REAL T1[3], PQP_Model* o1,
                             PQP_REAL R2[3][3], PQP_REAL T2[3], PQP_Model* o2,
                             int flag)
    {
        // make sure that the models are built

        if (o1->build_state != PQP_BUILD_STATE_PROCESSED || o2->build_state != PQP_BUILD_STATE_PROCESSED)
        {
            return PQP_ERR_UNPROCESSED_MODEL;
        }

        // clear the stats

        res->num_bv_tests = 0;
        res->num_tri_tests = 0;

        // don't release the memory, but reset the num_pairs counter

        res->num_pairs = 0;

        // Okay, compute what transform [R,T] that takes us from cs1 to cs2.
        // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
        // First compute the rotation part, then translation part

        pqp_math.MTxM(res->R, R1, R2);
        PQP_REAL Ttemp[3];
        pqp_math.VmV(Ttemp, T2, T1);
        pqp_math.MTxV(res->T, R1, Ttemp);

        // compute the transform from o1->child(0) to o2->child(0)

        PQP_REAL Rtemp[3][3], R[3][3], T[3];

        auto o1c0 = o1->child(0);
        auto o2c0 = o2->child(0);

        pqp_math.MxM(Rtemp, res->R, o2c0->R);
        pqp_math.MTxM(R, o1c0->R, Rtemp);

#if PQP_BV_TYPE & OBB_TYPE
        pqp_math.MxVpV(Ttemp, res->R, o2c0->To, res->T);
        pqp_math.VmV(Ttemp, Ttemp, o1c0->To);
#else
        pqp_math.MxVpV(Ttemp, res->R, o2c0->Tr, res->T);
        pqp_math.VmV(Ttemp, Ttemp, o1c0->Tr);
#endif

        pqp_math.MTxV(T, o1c0->R, Ttemp);

        // now start with both top level BVs

        CollideRecurse(res, R, T, o1, 0, o2, 0, flag);

        return PQP_OK;
    }

#if PQP_BV_TYPE & RSS_TYPE // distance/tolerance only available with RSS
    // unless an OBB distance test is supplied in
    // BV.cpp

    // DISTANCE STUFF
    //
    //--------------------------------------------------------------------------

    void
    PQP_Checker::DistanceRecurse(PQP_DistanceResult* res,
                                 PQP_REAL R[3][3], PQP_REAL T[3], // b2 relative to b1
                                 PQP_Model* o1, int b1,
                                 PQP_Model* o2, int b2)
    {
        PQP_REAL sz1 = o1->child(b1)->GetSize();
        PQP_REAL sz2 = o2->child(b2)->GetSize();
        int l1 = o1->child(b1)->Leaf();
        int l2 = o2->child(b2)->Leaf();

        if (l1 && l2)
        {
            // both leaves.  Test the triangles beneath them.

            res->num_tri_tests++;

            PQP_REAL p[3], q[3];

            Tri* t1 = &o1->tris[-o1->child(b1)->first_child - 1];
            Tri* t2 = &o2->tris[-o2->child(b2)->first_child - 1];

            PQP_REAL d = TriDistance(res->R, res->T, t1, t2, p, q);

            if (d < res->distance)
            {
                res->distance = d;
                // ADDED FOR SIMOX: store IDs
                res->p1ID = t1->id;
                res->p2ID = t2->id;
                /////////////////////////////

                pqp_math.VcV(res->p1, p);         // p already in c.s. 1
                pqp_math.VcV(res->p2, q);         // q must be transformed
                // into c.s. 2 later
                o1->last_tri = t1;
                o2->last_tri = t2;
            }

            return;
        }

        // First, perform distance tests on the children. Then traverse
        // them recursively, but test the closer pair first, the further
        // pair second.

        int a1, a2, c1, c2; // new bv tests 'a' and 'c'
        PQP_REAL R1[3][3], T1[3], R2[3][3], T2[3], Ttemp[3];

        if (l2 || (!l1 && (sz1 > sz2)))
        {
            // visit the children of b1

            a1 = o1->child(b1)->first_child;
            a2 = b2;
            c1 = o1->child(b1)->first_child + 1;
            c2 = b2;

            pqp_math.MTxM(R1, o1->child(a1)->R, R);
#if PQP_BV_TYPE & RSS_TYPE
            pqp_math.VmV(Ttemp, T, o1->child(a1)->Tr);
#else
            pqp_math.VmV(Ttemp, T, o1->child(a1)->To);
#endif
            pqp_math.MTxV(T1, o1->child(a1)->R, Ttemp);

            pqp_math.MTxM(R2, o1->child(c1)->R, R);
#if PQP_BV_TYPE & RSS_TYPE
            pqp_math.VmV(Ttemp, T, o1->child(c1)->Tr);
#else
            pqp_math.VmV(Ttemp, T, o1->child(c1)->To);
#endif
            pqp_math.MTxV(T2, o1->child(c1)->R, Ttemp);
        }
        else
        {
            // visit the children of b2

            a1 = b1;
            a2 = o2->child(b2)->first_child;
            c1 = b1;
            c2 = o2->child(b2)->first_child + 1;

            pqp_math.MxM(R1, R, o2->child(a2)->R);
#if PQP_BV_TYPE & RSS_TYPE
            pqp_math.MxVpV(T1, R, o2->child(a2)->Tr, T);
#else
            pqp_math.MxVpV(T1, R, o2->child(a2)->To, T);
#endif

            pqp_math.MxM(R2, R, o2->child(c2)->R);
#if PQP_BV_TYPE & RSS_TYPE
            pqp_math.MxVpV(T2, R, o2->child(c2)->Tr, T);
#else
            pqp_math.MxVpV(T2, R, o2->child(c2)->To, T);
#endif
        }

        res->num_bv_tests += 2;

        PQP_REAL d1 = bvProcessor.BV_Distance(R1, T1, o1->child(a1), o2->child(a2));
        PQP_REAL d2 = bvProcessor.BV_Distance(R2, T2, o1->child(c1), o2->child(c2));

        if (d2 < d1)
        {
            if ((d2 < (res->distance - res->abs_err)) ||
                (d2 * (1 + res->rel_err) < res->distance))
            {
                DistanceRecurse(res, R2, T2, o1, c1, o2, c2);
            }

            if ((d1 < (res->distance - res->abs_err)) ||
                (d1 * (1 + res->rel_err) < res->distance))
            {
                DistanceRecurse(res, R1, T1, o1, a1, o2, a2);
            }
        }
        else
        {
            if ((d1 < (res->distance - res->abs_err)) ||
                (d1 * (1 + res->rel_err) < res->distance))
            {
                DistanceRecurse(res, R1, T1, o1, a1, o2, a2);
            }

            if ((d2 < (res->distance - res->abs_err)) ||
                (d2 * (1 + res->rel_err) < res->distance))
            {
                DistanceRecurse(res, R2, T2, o1, c1, o2, c2);
            }
        }
    }

    void
    PQP_Checker::DistanceQueueRecurse(PQP_DistanceResult* res,
                                      PQP_REAL R[3][3], PQP_REAL T[3],
                                      PQP_Model* o1, int b1,
                                      PQP_Model* o2, int b2)
    {
        BVTQ bvtq(res->qsize);

        BVT min_test;
        min_test.b1 = b1;
        min_test.b2 = b2;
        pqp_math.McM(min_test.R, R);
        pqp_math.VcV(min_test.T, T);

        while (true)
        {
            int l1 = o1->child(min_test.b1)->Leaf();
            int l2 = o2->child(min_test.b2)->Leaf();

            if (l1 && l2)
            {
                // both leaves.  Test the triangles beneath them.

                res->num_tri_tests++;

                PQP_REAL p[3], q[3];

                Tri* t1 = &o1->tris[-o1->child(min_test.b1)->first_child - 1];
                Tri* t2 = &o2->tris[-o2->child(min_test.b2)->first_child - 1];

                PQP_REAL d = TriDistance(res->R, res->T, t1, t2, p, q);

                if (d < res->distance)
                {
                    res->distance = d;
                    // ADDED FOR SIMOX: store IDs
                    res->p1ID = t1->id;
                    res->p2ID = t2->id;
                    //////////////////////////////

                    pqp_math.VcV(res->p1, p);         // p already in c.s. 1
                    pqp_math.VcV(res->p2, q);         // q must be transformed
                    // into c.s. 2 later
                    o1->last_tri = t1;
                    o2->last_tri = t2;
                }
            }
            else if (bvtq.GetNumTests() == bvtq.GetSize() - 1)
            {
                // queue can't get two more tests, recur

                DistanceQueueRecurse(res, min_test.R, min_test.T,
                                     o1, min_test.b1, o2, min_test.b2);
            }
            else
            {
                // decide how to descend to children

                PQP_REAL sz1 = o1->child(min_test.b1)->GetSize();
                PQP_REAL sz2 = o2->child(min_test.b2)->GetSize();

                res->num_bv_tests += 2;

                BVT bvt1, bvt2;
                PQP_REAL Ttemp[3];

                if (l2 || (!l1 && (sz1 > sz2)))
                {
                    // put new tests on queue consisting of min_test.b2
                    // with children of min_test.b1

                    int c1 = o1->child(min_test.b1)->first_child;
                    int c2 = c1 + 1;

                    // init bv test 1

                    bvt1.b1 = c1;
                    bvt1.b2 = min_test.b2;
                    pqp_math.MTxM(bvt1.R, o1->child(c1)->R, min_test.R);
#if PQP_BV_TYPE & RSS_TYPE
                    pqp_math.VmV(Ttemp, min_test.T, o1->child(c1)->Tr);
#else
                    pqp_math.VmV(Ttemp, min_test.T, o1->child(c1)->To);
#endif
                    pqp_math.MTxV(bvt1.T, o1->child(c1)->R, Ttemp);
                    bvt1.d = bvProcessor.BV_Distance(bvt1.R, bvt1.T,
                                                     o1->child(bvt1.b1), o2->child(bvt1.b2));

                    // init bv test 2

                    bvt2.b1 = c2;
                    bvt2.b2 = min_test.b2;
                    pqp_math.MTxM(bvt2.R, o1->child(c2)->R, min_test.R);
#if PQP_BV_TYPE & RSS_TYPE
                    pqp_math.VmV(Ttemp, min_test.T, o1->child(c2)->Tr);
#else
                    pqp_math.VmV(Ttemp, min_test.T, o1->child(c2)->To);
#endif
                    pqp_math.MTxV(bvt2.T, o1->child(c2)->R, Ttemp);
                    bvt2.d = bvProcessor.BV_Distance(bvt2.R, bvt2.T,
                                                     o1->child(bvt2.b1), o2->child(bvt2.b2));
                }
                else
                {
                    // put new tests on queue consisting of min_test.b1
                    // with children of min_test.b2

                    int c1 = o2->child(min_test.b2)->first_child;
                    int c2 = c1 + 1;

                    // init bv test 1

                    bvt1.b1 = min_test.b1;
                    bvt1.b2 = c1;
                    pqp_math.MxM(bvt1.R, min_test.R, o2->child(c1)->R);
#if PQP_BV_TYPE & RSS_TYPE
                    pqp_math.MxVpV(bvt1.T, min_test.R, o2->child(c1)->Tr, min_test.T);
#else
                    pqp_math.MxVpV(bvt1.T, min_test.R, o2->child(c1)->To, min_test.T);
#endif
                    bvt1.d = bvProcessor.BV_Distance(bvt1.R, bvt1.T,
                                                     o1->child(bvt1.b1), o2->child(bvt1.b2));

                    // init bv test 2

                    bvt2.b1 = min_test.b1;
                    bvt2.b2 = c2;
                    pqp_math.MxM(bvt2.R, min_test.R, o2->child(c2)->R);
#if PQP_BV_TYPE & RSS_TYPE
                    pqp_math.MxVpV(bvt2.T, min_test.R, o2->child(c2)->Tr, min_test.T);
#else
                    pqp_math.MxVpV(bvt2.T, min_test.R, o2->child(c2)->To, min_test.T);
#endif
                    bvt2.d = bvProcessor.BV_Distance(bvt2.R, bvt2.T,
                                                     o1->child(bvt2.b1), o2->child(bvt2.b2));
                }

                bvtq.AddTest(bvt1);
                bvtq.AddTest(bvt2);
            }

            if (bvtq.Empty())
            {
                break;
            }
            else
            {
                min_test = bvtq.ExtractMinTest();

                if ((min_test.d + res->abs_err >= res->distance) &&
                    ((min_test.d * (1 + res->rel_err)) >= res->distance))
                {
                    break;
                }
            }
        }
    }

    int
    PQP_Checker::PQP_Distance(PQP_DistanceResult* res,
                              PQP_REAL R1[3][3], PQP_REAL T1[3], PQP_Model* o1,
                              PQP_REAL R2[3][3], PQP_REAL T2[3], PQP_Model* o2,
                              PQP_REAL rel_err, PQP_REAL abs_err,
                              int qsize)
    {
        //        Timer ti;
        //        double time1 = ti.GetTime();

        // make sure that the models are built

        if (o1->build_state != PQP_BUILD_STATE_PROCESSED)
        {
            return PQP_ERR_UNPROCESSED_MODEL;
        }

        if (o2->build_state != PQP_BUILD_STATE_PROCESSED)
        {
            return PQP_ERR_UNPROCESSED_MODEL;
        }

        // Okay, compute what transform [R,T] that takes us from cs2 to cs1.
        // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
        // First compute the rotation part, then translation part

        pqp_math.MTxM(res->R, R1, R2);
        PQP_REAL Ttemp[3];
        pqp_math.VmV(Ttemp, T2, T1);
        pqp_math.MTxV(res->T, R1, Ttemp);

        // establish initial upper bound using last triangles which
        // provided the minimum distance

        PQP_REAL p[3], q[3];
        res->distance = TriDistance(res->R, res->T, o1->last_tri, o2->last_tri, p, q);

        // ADDED FOR SIMOX: store IDs
        if (o1->last_tri)
        {
            res->p1ID = o1->last_tri->id;
        }
        else
        {
            res->p1ID = -1;
        }

        if (o2->last_tri)
        {
            res->p2ID = o2->last_tri->id;
        }
        else
        {
            res->p2ID = -1;
        }

        /////////////////////////////////

        pqp_math.VcV(res->p1, p);
        pqp_math.VcV(res->p2, q);

        // initialize error bounds

        res->abs_err = abs_err;
        res->rel_err = rel_err;

        // clear the stats

        res->num_bv_tests = 0;
        res->num_tri_tests = 0;

        // compute the transform from o1->child(0) to o2->child(0)

        PQP_REAL Rtemp[3][3], R[3][3], T[3];

        pqp_math.MxM(Rtemp, res->R, o2->child(0)->R);
        pqp_math.MTxM(R, o1->child(0)->R, Rtemp);

#if PQP_BV_TYPE & RSS_TYPE
        pqp_math.MxVpV(Ttemp, res->R, o2->child(0)->Tr, res->T);
        pqp_math.VmV(Ttemp, Ttemp, o1->child(0)->Tr);
#else
        pqp_math.MxVpV(Ttemp, res->R, o2->child(0)->To, res->T);
        pqp_math.VmV(Ttemp, Ttemp, o1->child(0)->To);
#endif
        pqp_math.MTxV(T, o1->child(0)->R, Ttemp);

        // choose routine according to queue size

        if (qsize <= 2)
        {
            DistanceRecurse(res, R, T, o1, 0, o2, 0);
        }
        else
        {
            res->qsize = qsize;

            DistanceQueueRecurse(res, R, T, o1, 0, o2, 0);
        }

        // res->p2 is in cs 1 ; transform it to cs 2
        PQP_REAL u[3];

        // skip this transformation, instead T1,R1 are used for transforming pos2 to the global coord system
        //pqp_math.VmV(u, res->p2, res->T);
        //pqp_math.MTxV(res->p2, res->R, u);

        /*printf ("cs1 cs2:\n");
        Vprint(res->p1);
        Vprint(res->p2);*/

        // transform to correct (global) coord system

        pqp_math.VcV(u, res->p1);
        pqp_math.MxVpV(res->p1, R1, u, T1);// same pointer for target and source will result in an invalid calculation!
        pqp_math.VcV(u, res->p2);
        pqp_math.MxVpV(res->p2, R1, u, T1);// same pointer for target and source will result in an invalid calculation!

        /*printf ("global:\n");
        Vprint(res->p1);
        Vprint(res->p2);*/

        //        double time2 = ti.GetTime();
        //        res->query_time_secs = time2 - time1;

        return PQP_OK;
    }

    // Tolerance Stuff
    //
    //---------------------------------------------------------------------------
    void
    PQP_Checker::ToleranceRecurse(PQP_ToleranceResult* res,
                                  PQP_REAL R[3][3], PQP_REAL T[3],
                                  PQP_Model* o1, int b1, PQP_Model* o2, int b2)
    {
        PQP_REAL sz1 = o1->child(b1)->GetSize();
        PQP_REAL sz2 = o2->child(b2)->GetSize();
        int l1 = o1->child(b1)->Leaf();
        int l2 = o2->child(b2)->Leaf();

        if (l1 && l2)
        {
            // both leaves - find if tri pair within tolerance

            res->num_tri_tests++;

            PQP_REAL p[3], q[3];

            Tri* t1 = &o1->tris[-o1->child(b1)->first_child - 1];
            Tri* t2 = &o2->tris[-o2->child(b2)->first_child - 1];

            PQP_REAL d = TriDistance(res->R, res->T, t1, t2, p, q);

            if (d <= res->tolerance)
            {
                // triangle pair distance less than tolerance

                res->closer_than_tolerance = 1;
                res->distance = d;
                pqp_math.VcV(res->p1, p);         // p already in c.s. 1
                pqp_math.VcV(res->p2, q);         // q must be transformed
                // into c.s. 2 later
            }

            return;
        }

        int a1, a2, c1, c2; // new bv tests 'a' and 'c'
        PQP_REAL R1[3][3], T1[3], R2[3][3], T2[3], Ttemp[3];

        if (l2 || (!l1 && (sz1 > sz2)))
        {
            // visit the children of b1

            a1 = o1->child(b1)->first_child;
            a2 = b2;
            c1 = o1->child(b1)->first_child + 1;
            c2 = b2;

            pqp_math.MTxM(R1, o1->child(a1)->R, R);
#if PQP_BV_TYPE & RSS_TYPE
            pqp_math.VmV(Ttemp, T, o1->child(a1)->Tr);
#else
            pqp_math.VmV(Ttemp, T, o1->child(a1)->To);
#endif
            pqp_math.MTxV(T1, o1->child(a1)->R, Ttemp);

            pqp_math.MTxM(R2, o1->child(c1)->R, R);
#if PQP_BV_TYPE & RSS_TYPE
            pqp_math.VmV(Ttemp, T, o1->child(c1)->Tr);
#else
            pqp_math.VmV(Ttemp, T, o1->child(c1)->To);
#endif
            pqp_math.MTxV(T2, o1->child(c1)->R, Ttemp);
        }
        else
        {
            // visit the children of b2

            a1 = b1;
            a2 = o2->child(b2)->first_child;
            c1 = b1;
            c2 = o2->child(b2)->first_child + 1;

            pqp_math.MxM(R1, R, o2->child(a2)->R);
#if PQP_BV_TYPE & RSS_TYPE
            pqp_math.MxVpV(T1, R, o2->child(a2)->Tr, T);
#else
            pqp_math.MxVpV(T1, R, o2->child(a2)->To, T);
#endif
            pqp_math.MxM(R2, R, o2->child(c2)->R);
#if PQP_BV_TYPE & RSS_TYPE
            pqp_math.MxVpV(T2, R, o2->child(c2)->Tr, T);
#else
            pqp_math.MxVpV(T2, R, o2->child(c2)->To, T);
#endif
        }

        res->num_bv_tests += 2;

        PQP_REAL d1 = bvProcessor.BV_Distance(R1, T1, o1->child(a1), o2->child(a2));
        PQP_REAL d2 = bvProcessor.BV_Distance(R2, T2, o1->child(c1), o2->child(c2));

        if (d2 < d1)
        {
            if (d2 <= res->tolerance)
            {
                ToleranceRecurse(res, R2, T2, o1, c1, o2, c2);
            }

            if (res->closer_than_tolerance)
            {
                return;
            }

            if (d1 <= res->tolerance)
            {
                ToleranceRecurse(res, R1, T1, o1, a1, o2, a2);
            }
        }
        else
        {
            if (d1 <= res->tolerance)
            {
                ToleranceRecurse(res, R1, T1, o1, a1, o2, a2);
            }

            if (res->closer_than_tolerance)
            {
                return;
            }

            if (d2 <= res->tolerance)
            {
                ToleranceRecurse(res, R2, T2, o1, c1, o2, c2);
            }
        }
    }

    void
    PQP_Checker::ToleranceQueueRecurse(PQP_ToleranceResult* res,
                                       PQP_REAL R[3][3], PQP_REAL T[3],
                                       PQP_Model* o1, int b1,
                                       PQP_Model* o2, int b2)
    {
        BVTQ bvtq(res->qsize);
        BVT min_test;
        min_test.b1 = b1;
        min_test.b2 = b2;
        pqp_math.McM(min_test.R, R);
        pqp_math.VcV(min_test.T, T);

        while (true)
        {
            int l1 = o1->child(min_test.b1)->Leaf();
            int l2 = o2->child(min_test.b2)->Leaf();

            if (l1 && l2)
            {
                // both leaves - find if tri pair within tolerance

                res->num_tri_tests++;

                PQP_REAL p[3], q[3];

                Tri* t1 = &o1->tris[-o1->child(min_test.b1)->first_child - 1];
                Tri* t2 = &o2->tris[-o2->child(min_test.b2)->first_child - 1];

                PQP_REAL d = TriDistance(res->R, res->T, t1, t2, p, q);

                if (d <= res->tolerance)
                {
                    // triangle pair distance less than tolerance

                    res->closer_than_tolerance = 1;
                    res->distance = d;
                    pqp_math.VcV(res->p1, p);         // p already in c.s. 1
                    pqp_math.VcV(res->p2, q);         // q must be transformed
                    // into c.s. 2 later
                    return;
                }
            }
            else if (bvtq.GetNumTests() == bvtq.GetSize() - 1)
            {
                // queue can't get two more tests, recur

                ToleranceQueueRecurse(res, min_test.R, min_test.T,
                                      o1, min_test.b1, o2, min_test.b2);

                if (res->closer_than_tolerance == 1)
                {
                    return;
                }
            }
            else
            {
                // decide how to descend to children

                PQP_REAL sz1 = o1->child(min_test.b1)->GetSize();
                PQP_REAL sz2 = o2->child(min_test.b2)->GetSize();

                res->num_bv_tests += 2;

                BVT bvt1, bvt2;
                PQP_REAL Ttemp[3];

                if (l2 || (!l1 && (sz1 > sz2)))
                {
                    // add two new tests to queue, consisting of min_test.b2
                    // with the children of min_test.b1

                    int c1 = o1->child(min_test.b1)->first_child;
                    int c2 = c1 + 1;

                    // init bv test 1

                    bvt1.b1 = c1;
                    bvt1.b2 = min_test.b2;
                    pqp_math.MTxM(bvt1.R, o1->child(c1)->R, min_test.R);
#if PQP_BV_TYPE & RSS_TYPE
                    pqp_math.VmV(Ttemp, min_test.T, o1->child(c1)->Tr);
#else
                    pqp_math.VmV(Ttemp, min_test.T, o1->child(c1)->To);
#endif
                    pqp_math.MTxV(bvt1.T, o1->child(c1)->R, Ttemp);
                    bvt1.d = bvProcessor.BV_Distance(bvt1.R, bvt1.T,
                                                     o1->child(bvt1.b1), o2->child(bvt1.b2));

                    // init bv test 2

                    bvt2.b1 = c2;
                    bvt2.b2 = min_test.b2;
                    pqp_math.MTxM(bvt2.R, o1->child(c2)->R, min_test.R);
#if PQP_BV_TYPE & RSS_TYPE
                    pqp_math.VmV(Ttemp, min_test.T, o1->child(c2)->Tr);
#else
                    pqp_math.VmV(Ttemp, min_test.T, o1->child(c2)->To);
#endif
                    pqp_math.MTxV(bvt2.T, o1->child(c2)->R, Ttemp);
                    bvt2.d = bvProcessor.BV_Distance(bvt2.R, bvt2.T,
                                                     o1->child(bvt2.b1), o2->child(bvt2.b2));
                }
                else
                {
                    // add two new tests to queue, consisting of min_test.b1
                    // with the children of min_test.b2

                    int c1 = o2->child(min_test.b2)->first_child;
                    int c2 = c1 + 1;

                    // init bv test 1

                    bvt1.b1 = min_test.b1;
                    bvt1.b2 = c1;
                    pqp_math.MxM(bvt1.R, min_test.R, o2->child(c1)->R);
#if PQP_BV_TYPE & RSS_TYPE
                    pqp_math.MxVpV(bvt1.T, min_test.R, o2->child(c1)->Tr, min_test.T);
#else
                    pqp_math.MxVpV(bvt1.T, min_test.R, o2->child(c1)->To, min_test.T);
#endif
                    bvt1.d = bvProcessor.BV_Distance(bvt1.R, bvt1.T,
                                                     o1->child(bvt1.b1), o2->child(bvt1.b2));

                    // init bv test 2

                    bvt2.b1 = min_test.b1;
                    bvt2.b2 = c2;
                    pqp_math.MxM(bvt2.R, min_test.R, o2->child(c2)->R);
#if PQP_BV_TYPE & RSS_TYPE
                    pqp_math.MxVpV(bvt2.T, min_test.R, o2->child(c2)->Tr, min_test.T);
#else
                    pqp_math.MxVpV(bvt2.T, min_test.R, o2->child(c2)->To, min_test.T);
#endif
                    bvt2.d = bvProcessor.BV_Distance(bvt2.R, bvt2.T,
                                                     o1->child(bvt2.b1), o2->child(bvt2.b2));
                }

                // put children tests in queue

                if (bvt1.d <= res->tolerance)
                {
                    bvtq.AddTest(bvt1);
                }

                if (bvt2.d <= res->tolerance)
                {
                    bvtq.AddTest(bvt2);
                }
            }

            if (bvtq.Empty() || (bvtq.MinTest() > res->tolerance))
            {
                res->closer_than_tolerance = 0;
                return;
            }
            else
            {
                min_test = bvtq.ExtractMinTest();
            }
        }
    }

    int
    PQP_Checker::PQP_Tolerance(PQP_ToleranceResult* res,
                               PQP_REAL R1[3][3], PQP_REAL T1[3], PQP_Model* o1,
                               PQP_REAL R2[3][3], PQP_REAL T2[3], PQP_Model* o2,
                               PQP_REAL tolerance,
                               int qsize)
    {
        //                Timer ti;
        //        double time1 = ti.GetTime();

        // make sure that the models are built

        if (o1->build_state != PQP_BUILD_STATE_PROCESSED)
        {
            return PQP_ERR_UNPROCESSED_MODEL;
        }

        if (o2->build_state != PQP_BUILD_STATE_PROCESSED)
        {
            return PQP_ERR_UNPROCESSED_MODEL;
        }

        // Compute the transform [R,T] that takes us from cs2 to cs1.
        // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]

        pqp_math.MTxM(res->R, R1, R2);
        PQP_REAL Ttemp[3];
        pqp_math.VmV(Ttemp, T2, T1);
        pqp_math.MTxV(res->T, R1, Ttemp);

        // set tolerance, used to prune the search

        if (tolerance < 0.0)
        {
            tolerance = 0.0;
        }

        res->tolerance = tolerance;

        // clear the stats

        res->num_bv_tests = 0;
        res->num_tri_tests = 0;

        // initially assume not closer than tolerance

        res->closer_than_tolerance = 0;

        // compute the transform from o1->child(0) to o2->child(0)

        PQP_REAL Rtemp[3][3], R[3][3], T[3];

        pqp_math.MxM(Rtemp, res->R, o2->child(0)->R);
        pqp_math.MTxM(R, o1->child(0)->R, Rtemp);
#if PQP_BV_TYPE & RSS_TYPE
        pqp_math.MxVpV(Ttemp, res->R, o2->child(0)->Tr, res->T);
        pqp_math.VmV(Ttemp, Ttemp, o1->child(0)->Tr);
#else
        pqp_math.MxVpV(Ttemp, res->R, o2->child(0)->To, res->T);
        pqp_math.VmV(Ttemp, Ttemp, o1->child(0)->To);
#endif
        pqp_math.MTxV(T, o1->child(0)->R, Ttemp);

        // find a distance lower bound for trivial reject

        PQP_REAL d = bvProcessor.BV_Distance(R, T, o1->child(0), o2->child(0));

        if (d <= res->tolerance)
        {
            // more work needed - choose routine according to queue size

            if (qsize <= 2)
            {
                ToleranceRecurse(res, R, T, o1, 0, o2, 0);
            }
            else
            {
                res->qsize = qsize;
                ToleranceQueueRecurse(res, R, T, o1, 0, o2, 0);
            }
        }

        // res->p2 is in cs 1 ; transform it to cs 2

        PQP_REAL u[3];
        pqp_math.VmV(u, res->p2, res->T);
        pqp_math.MTxV(res->p2, res->R, u);

        //        double time2 = ti.GetTime();
        //        res->query_time_secs = time2 - time1;

        return PQP_OK;
    }
#endif

} // namespace

////////////////////////////////////////////////////////////////////////////////
//TriDist.cpp
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

  US Mail:             E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

//--------------------------------------------------------------------------
// File:   TriDist.cpp
// Author: Eric Larsen
// Description:
// contains SegPoints() for finding closest points on a pair of line
// segments and TriDist() for finding closest points on a pair of triangles
//--------------------------------------------------------------------------

#include "MatVec.h"
#include "TriDist.h"
#ifdef _WIN32
#include <float.h>
#define isnan _isnan
#endif

namespace PQP
{

    //--------------------------------------------------------------------------
    // SegPoints()
    //
    // Returns closest points between an segment pair.
    // Implemented from an algorithm described in
    //
    // Vladimir J. Lumelsky,
    // On fast computation of distance between line segments.
    // In Information Processing Letters, no. 21, pages 55-61, 1985.
    //--------------------------------------------------------------------------

    void
    Tri_Processor::SegPoints(PQP_REAL VEC[3],
                             PQP_REAL X[3], PQP_REAL Y[3],             // closest points
                             const PQP_REAL P[3], const PQP_REAL A[3], // seg 1 origin, vector
                             const PQP_REAL Q[3], const PQP_REAL B[3]) // seg 2 origin, vector
    {
        PQP_REAL T[3], A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
        PQP_REAL TMP[3];

        pqp_math.VmV(T, Q, P);
        A_dot_A = pqp_math.VdotV(A, A);
        B_dot_B = pqp_math.VdotV(B, B);
        A_dot_B = pqp_math.VdotV(A, B);
        A_dot_T = pqp_math.VdotV(A, T);
        B_dot_T = pqp_math.VdotV(B, T);

        // t parameterizes ray P,A
        // u parameterizes ray Q,B

        PQP_REAL t, u;

        // compute t for the closest point on ray P,A to
        // ray Q,B

        PQP_REAL denom = A_dot_A * B_dot_B - A_dot_B * A_dot_B;

        t = (A_dot_T * B_dot_B - B_dot_T * A_dot_B) / denom;

        // clamp result so t is on the segment P,A

        if ((t < 0) || isnan(t))
        {
            t = 0;
        }
        else if (t > 1)
        {
            t = 1;
        }

        // find u for point on ray Q,B closest to point at t

        u = (t * A_dot_B - B_dot_T) / B_dot_B;

        // if u is on segment Q,B, t and u correspond to
        // closest points, otherwise, clamp u, recompute and
        // clamp t

        if ((u <= 0) || isnan(u))
        {

            pqp_math.VcV(Y, Q);

            t = A_dot_T / A_dot_A;

            if ((t <= 0) || isnan(t))
            {
                pqp_math.VcV(X, P);
                pqp_math.VmV(VEC, Q, P);
            }
            else if (t >= 1)
            {
                pqp_math.VpV(X, P, A);
                pqp_math.VmV(VEC, Q, X);
            }
            else
            {
                pqp_math.VpVxS(X, P, A, t);
                pqp_math.VcrossV(TMP, T, A);
                pqp_math.VcrossV(VEC, A, TMP);
            }
        }
        else if (u >= 1)
        {

            pqp_math.VpV(Y, Q, B);

            t = (A_dot_B + A_dot_T) / A_dot_A;

            if ((t <= 0) || isnan(t))
            {
                pqp_math.VcV(X, P);
                pqp_math.VmV(VEC, Y, P);
            }
            else if (t >= 1)
            {
                pqp_math.VpV(X, P, A);
                pqp_math.VmV(VEC, Y, X);
            }
            else
            {
                pqp_math.VpVxS(X, P, A, t);
                pqp_math.VmV(T, Y, P);
                pqp_math.VcrossV(TMP, T, A);
                pqp_math.VcrossV(VEC, A, TMP);
            }
        }
        else
        {

            pqp_math.VpVxS(Y, Q, B, u);

            if ((t <= 0) || isnan(t))
            {
                pqp_math.VcV(X, P);
                pqp_math.VcrossV(TMP, T, B);
                pqp_math.VcrossV(VEC, B, TMP);
            }
            else if (t >= 1)
            {
                pqp_math.VpV(X, P, A);
                pqp_math.VmV(T, Q, X);
                pqp_math.VcrossV(TMP, T, B);
                pqp_math.VcrossV(VEC, B, TMP);
            }
            else
            {
                pqp_math.VpVxS(X, P, A, t);
                pqp_math.VcrossV(VEC, A, B);

                if (pqp_math.VdotV(VEC, T) < 0)
                {
                    pqp_math.VxS(VEC, VEC, -1);
                }
            }
        }
    }

    //--------------------------------------------------------------------------
    // TriDist()
    //
    // Computes the closest points on two triangles, and returns the
    // distance between them.
    //
    // S and T are the triangles, stored tri[point][dimension].
    //
    // If the triangles are disjoint, P and Q give the closest points of
    // S and T respectively. However, if the triangles overlap, P and Q
    // are basically a random pair of points from the triangles, not
    // coincident points on the intersection of the triangles, as might
    // be expected.
    //--------------------------------------------------------------------------

    PQP_REAL
    Tri_Processor::TriDist(PQP_REAL P[3], PQP_REAL Q[3],
                           const PQP_REAL S[3][3], const PQP_REAL T[3][3])
    {
        // Compute vectors along the 6 sides

        PQP_REAL Sv[3][3], Tv[3][3];
        PQP_REAL VEC[3];

        pqp_math.VmV(Sv[0], S[1], S[0]);
        pqp_math.VmV(Sv[1], S[2], S[1]);
        pqp_math.VmV(Sv[2], S[0], S[2]);

        pqp_math.VmV(Tv[0], T[1], T[0]);
        pqp_math.VmV(Tv[1], T[2], T[1]);
        pqp_math.VmV(Tv[2], T[0], T[2]);

        // For each edge pair, the vector connecting the closest points
        // of the edges defines a slab (parallel planes at head and tail
        // enclose the slab). If we can show that the off-edge vertex of
        // each triangle is outside of the slab, then the closest points
        // of the edges are the closest points for the triangles.
        // Even if these tests fail, it may be helpful to know the closest
        // points found, and whether the triangles were shown disjoint

        PQP_REAL V[3] = {0, 0, 0};
        PQP_REAL Z[3] = {0, 0, 0};
        PQP_REAL minP[3] = {0, 0, 0};
        PQP_REAL minQ[3] = {0, 0, 0};
        PQP_REAL mindd = 0;
        int shown_disjoint = 0;

        mindd = pqp_math.VdistV2(S[0], T[0]) + 1; // Set first minimum safely high

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // Find closest points on edges i & j, plus the
                // vector (and distance squared) between these points

                SegPoints(VEC, P, Q, S[i], Sv[i], T[j], Tv[j]);

                pqp_math.VmV(V, Q, P);
                PQP_REAL dd = pqp_math.VdotV(V, V);

                // Verify this closest point pair only if the distance
                // squared is less than the minimum found thus far.

                if (dd <= mindd)
                {
                    pqp_math.VcV(minP, P);
                    pqp_math.VcV(minQ, Q);
                    mindd = dd;

                    pqp_math.VmV(Z, S[(i + 2) % 3], P);
                    PQP_REAL a = pqp_math.VdotV(Z, VEC);
                    pqp_math.VmV(Z, T[(j + 2) % 3], Q);
                    PQP_REAL b = pqp_math.VdotV(Z, VEC);

                    if ((a <= 0) && (b >= 0))
                    {
                        return sqrt(dd);
                    }

                    PQP_REAL p = pqp_math.VdotV(V, VEC);

                    if (a < 0)
                    {
                        a = 0;
                    }

                    if (b > 0)
                    {
                        b = 0;
                    }

                    if ((p - a + b) > 0)
                    {
                        shown_disjoint = 1;
                    }
                }
            }
        }

        // No edge pairs contained the closest points.
        // either:
        // 1. one of the closest points is a vertex, and the
        //    other point is interior to a face.
        // 2. the triangles are overlapping.
        // 3. an edge of one triangle is parallel to the other's face. If
        //    cases 1 and 2 are not true, then the closest points from the 9
        //    edge pairs checks above can be taken as closest points for the
        //    triangles.
        // 4. possibly, the triangles were degenerate.  When the
        //    triangle points are nearly colinear or coincident, one
        //    of above tests might fail even though the edges tested
        //    contain the closest points.

        // First check for case 1

        PQP_REAL Sn[3], Snl;
        pqp_math.VcrossV(Sn, Sv[0], Sv[1]); // Compute normal to S triangle
        Snl = pqp_math.VdotV(Sn, Sn);     // Compute square of length of normal

        // If cross product is long enough,

        if (Snl > 1e-15)
        {
            // Get projection lengths of T points

            PQP_REAL Tp[3];

            pqp_math.VmV(V, S[0], T[0]);
            Tp[0] = pqp_math.VdotV(V, Sn);

            pqp_math.VmV(V, S[0], T[1]);
            Tp[1] = pqp_math.VdotV(V, Sn);

            pqp_math.VmV(V, S[0], T[2]);
            Tp[2] = pqp_math.VdotV(V, Sn);

            // If Sn is a separating direction,
            // find point with smallest projection

            int point = -1;

            if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
            {
                if (Tp[0] < Tp[1])
                {
                    point = 0;
                }
                else
                {
                    point = 1;
                }

                if (Tp[2] < Tp[point])
                {
                    point = 2;
                }
            }
            else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
            {
                if (Tp[0] > Tp[1])
                {
                    point = 0;
                }
                else
                {
                    point = 1;
                }

                if (Tp[2] > Tp[point])
                {
                    point = 2;
                }
            }

            // If Sn is a separating direction,

            if (point >= 0)
            {
                shown_disjoint = 1;

                // Test whether the point found, when projected onto the
                // other triangle, lies within the face.

                pqp_math.VmV(V, T[point], S[0]);
                pqp_math.VcrossV(Z, Sn, Sv[0]);

                if (pqp_math.VdotV(V, Z) > 0)
                {
                    pqp_math.VmV(V, T[point], S[1]);
                    pqp_math.VcrossV(Z, Sn, Sv[1]);

                    if (pqp_math.VdotV(V, Z) > 0)
                    {
                        pqp_math.VmV(V, T[point], S[2]);
                        pqp_math.VcrossV(Z, Sn, Sv[2]);

                        if (pqp_math.VdotV(V, Z) > 0)
                        {
                            // T[point] passed the test - it's a closest point for
                            // the T triangle; the other point is on the face of S

                            pqp_math.VpVxS(P, T[point], Sn, Tp[point] / Snl);
                            pqp_math.VcV(Q, T[point]);
                            return sqrt(pqp_math.VdistV2(P, Q));
                        }
                    }
                }
            }
        }

        PQP_REAL Tn[3], Tnl;
        pqp_math.VcrossV(Tn, Tv[0], Tv[1]);
        Tnl = pqp_math.VdotV(Tn, Tn);

        if (Tnl > 1e-15)
        {
            PQP_REAL Sp[3];

            pqp_math.VmV(V, T[0], S[0]);
            Sp[0] = pqp_math.VdotV(V, Tn);

            pqp_math.VmV(V, T[0], S[1]);
            Sp[1] = pqp_math.VdotV(V, Tn);

            pqp_math.VmV(V, T[0], S[2]);
            Sp[2] = pqp_math.VdotV(V, Tn);

            int point = -1;

            if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
            {
                if (Sp[0] < Sp[1])
                {
                    point = 0;
                }
                else
                {
                    point = 1;
                }

                if (Sp[2] < Sp[point])
                {
                    point = 2;
                }
            }
            else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
            {
                if (Sp[0] > Sp[1])
                {
                    point = 0;
                }
                else
                {
                    point = 1;
                }

                if (Sp[2] > Sp[point])
                {
                    point = 2;
                }
            }

            if (point >= 0)
            {
                shown_disjoint = 1;

                pqp_math.VmV(V, S[point], T[0]);
                pqp_math.VcrossV(Z, Tn, Tv[0]);

                if (pqp_math.VdotV(V, Z) > 0)
                {
                    pqp_math.VmV(V, S[point], T[1]);
                    pqp_math.VcrossV(Z, Tn, Tv[1]);

                    if (pqp_math.VdotV(V, Z) > 0)
                    {
                        pqp_math.VmV(V, S[point], T[2]);
                        pqp_math.VcrossV(Z, Tn, Tv[2]);

                        if (pqp_math.VdotV(V, Z) > 0)
                        {
                            pqp_math.VcV(P, S[point]);
                            pqp_math.VpVxS(Q, S[point], Tn, Sp[point] / Tnl);
                            return sqrt(pqp_math.VdistV2(P, Q));
                        }
                    }
                }
            }
        }

        // Case 1 can't be shown.
        // If one of these tests showed the triangles disjoint,
        // we assume case 3 or 4, otherwise we conclude case 2,
        // that the triangles overlap.

        if (shown_disjoint)
        {
            pqp_math.VcV(P, minP);
            pqp_math.VcV(Q, minQ);
            return sqrt(mindd);
        }
        else
        {
            return 0;
        }
    }

} // namespace


