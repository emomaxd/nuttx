/****************************************************************************
 * crypto/des_locl.h
 *
 * SPDX-License-Identifier: SSLeay-standalone
 * SPDX-FileCopyrightText: Copyright (C) 1995 Eric Young (eay@mincom.oz.au)
 * SPDX-FileCopyrightText: Eric Young (eay@mincom.oz.au).
 *
 * This file is part of an SSL implementation written
 * by Eric Young (eay@mincom.oz.au).
 * The implementation was written so as to conform with Netscapes SSL
 * specification.  This library and applications are
 * FREE FOR COMMERCIAL AND NON-COMMERCIAL USE
 * as long as the following conditions are aheared to.
 *
 * Copyright remains Eric Young's, and as such any Copyright notices in
 * the code are not to be removed.  If this code is used in a product,
 * Eric Young should be given attribution as the author of the parts used.
 * This can be in the form of a textual message at program startup or
 * in documentation (online or textual) provided with the package.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by
 *    Eric Young (eay@mincom.oz.au)
 *
 * THIS SOFTWARE IS PROVIDED BY ERIC YOUNG ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * The licence and distribution terms for any publically available version or
 * derivative of this code cannot be changed.
 * i.e. this code cannot simply be
 * copied and put under another distribution licence
 * [including the GNU Public Licence.]
 ****************************************************************************/

#ifndef __CRYPTO_DES_LOCL_H
#define __CRYPTO_DES_LOCL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

typedef unsigned char des_cblock[8];
typedef struct des_ks_struct
{
  union
    {
      des_cblock cblock;
      /* make sure things are correct size on machines with
       * 8 byte longs
       */

      int32_t pad[2];
    } ks;
}
des_key_schedule[16];

#define DES_KEY_SZ (sizeof(des_cblock))
#define DES_SCHEDULE_SZ (sizeof(des_key_schedule))

void des_encrypt2(FAR uint32_t *data, caddr_t ks, int enc);

#define ITERATIONS 16
#define HALF_ITERATIONS 8

#define c2l(c, l)                          \
  do                                       \
    {                                      \
      l = ((uint32_t)(*((c)++)));          \
      l |= ((uint32_t)(*((c)++))) << 8l;   \
      l |= ((uint32_t)(*((c)++))) << 16l;  \
      l |= ((uint32_t)(*((c)++))) << 24l;  \
    }                                      \
  while (0)

#define l2c(l,c)                                          \
  do                                                      \
    {                                                     \
      *((c)++) = (unsigned char)(((l)) & 0xff);           \
      *((c)++) = (unsigned char)(((l) >> 8L) & 0xff);     \
      *((c)++) = (unsigned char)(((l) >> 16L) & 0xff);    \
      *((c)++) = (unsigned char)(((l) >> 24L) & 0xff);    \
    }                                                     \
  while (0)

#define D_ENCRYPT(Q, R, S)              \
  do                                    \
    {                                   \
      u = (R ^ s[S]);                   \
      t = R ^ s[S + 1];                 \
      t = ((t >> 4L) + (t << 28L));     \
      Q ^= des_sptrans[1][(t) & 0x3f]|  \
      des_sptrans[3][(t >> 8l) & 0x3f]| \
      des_sptrans[5][(t >>16l) & 0x3f]| \
      des_sptrans[7][(t >>24l) & 0x3f]| \
      des_sptrans[0][(u) & 0x3f]|       \
      des_sptrans[2][(u >> 8l) & 0x3f]| \
      des_sptrans[4][(u >>16l) & 0x3f]| \
      des_sptrans[6][(u >>24l) & 0x3f]; \
    }                                   \
  while (0)

  /* IP and FP
   * The problem is more of a geometric problem that random bit fiddling.
   *    0  1  2  3  4  5  6  7      62 54 46 38 30 22 14  6
   *    8  9 10 11 12 13 14 15      60 52 44 36 28 20 12  4
   *  16 17 18 19 20 21 22 23      58 50 42 34 26 18 10  2
   *  24 25 26 27 28 29 30 31  to  56 48 40 32 24 16  8  0
   *
   *  32 33 34 35 36 37 38 39      63 55 47 39 31 23 15  7
   *  40 41 42 43 44 45 46 47      61 53 45 37 29 21 13  5
   *  48 49 50 51 52 53 54 55      59 51 43 35 27 19 11  3
   *  56 57 58 59 60 61 62 63      57 49 41 33 25 17  9  1
   *
   * The output has been subject to swaps of the form
   * 0 1 -> 3 1 but the odd and even bits have been put into
   * 2 3    2 0
   * different words.  The main trick is to remember that
   * t=((l>>size)^r)&(mask);
   * r^=t;
   * l^=(t<<size);
   * can be used to swap and move bits between words.
   *
   * So l =  0  1  2  3  r = 16 17 18 19
   *      4  5  6  7      20 21 22 23
   *       8  9 10 11      24 25 26 27
   *      12 13 14 15      28 29 30 31
   * becomes (for size == 2 and mask == 0x3333)
   *   t =   2^16  3^17 -- --   l =  0  1 16 17  r =  2  3 18 19
   *   6^20  7^21 -- --        4  5 20 21       6  7 22 23
   *  10^24 11^25 -- --        8  9 24 25      10 11 24 25
   *  14^28 15^29 -- --       12 13 28 29      14 15 28 29
   *
   * Thanks for hints from Richard Outerbridge - he told me IP&FP
   * could be done in 15 xor, 10 shifts and 5 ands.
   * When I finally started to think of the problem in 2D
   * I first got ~42 operations without xors.  When I remembered
   * how to use xors :-) I got it to its final state.
   */

#define PERM_OP(a, b, t, n, m) ((t) = ((((a) >> (n)) ^ (b)) & (m)),\
  (b) ^= (t),\
  (a) ^= ((t) << (n)))

#define IP(l, r)                        \
  do                                    \
    {                                   \
    register uint32_t tt;               \
    PERM_OP(r, l, tt, 4, 0x0f0f0f0fl);  \
    PERM_OP(l, r, tt, 16, 0x0000ffffl); \
    PERM_OP(r, l, tt, 2, 0x33333333l);  \
    PERM_OP(l, r, tt, 8, 0x00ff00ffl);  \
    PERM_OP(r, l, tt, 1, 0x55555555l);  \
    }                                   \
  while (0)

#define FP(l, r) \
  do                                    \
    {                                   \
    register uint32_t tt;               \
    PERM_OP(l, r, tt, 1, 0x55555555L);  \
    PERM_OP(r, l, tt, 8, 0x00ff00ffL);  \
    PERM_OP(l, r, tt, 2, 0x33333333L);  \
    PERM_OP(r, l, tt, 16, 0x0000ffffL); \
    PERM_OP(l, r, tt, 4, 0x0f0f0f0fL);  \
    }                                   \
  while (0)
#endif /* __CRYPTO_DES_LOCL_H */
