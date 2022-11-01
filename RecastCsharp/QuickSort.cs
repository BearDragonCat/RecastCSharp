// MIT License - Copyright (C) ryancheung and the FelCore team
// This file is subject to the terms and conditions defined in
// file 'LICENSE', which is part of this source code package.

using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

#pragma warning disable SA1121 // We use our own aliases since they differ per platform
#if TARGET_WINDOWS
using NativeType = System.Int32;
#else
using NativeType = System.IntPtr;
#endif

namespace RecastSharp
{
    [CLSCompliant(false)]
    //[Intrinsic]
    public readonly struct CLong : IEquatable<CLong>
    {
        private readonly NativeType _value;

        /// <summary>
        /// Constructs an instance from a 32-bit integer.
        /// </summary>
        /// <param name="value">The integer vaule.</param>
        public CLong(int value)
        {
            _value = (NativeType)value;
        }

        /// <summary>
        /// Constructs an instance from a native sized integer.
        /// </summary>
        /// <param name="value">The integer vaule.</param>
        /// <exception cref="OverflowException"><paramref name="value"/> is outside the range of the underlying storage type.</exception>
        public CLong(nint value)
        {
            _value = checked((NativeType)value);
        }

        /// <summary>
        /// The underlying integer value of this instance.
        /// </summary>
        public nint Value => _value;

        /// <summary>
        /// Returns a value indicating whether this instance is equal to a specified object.
        /// </summary>
        /// <param name="o">An object to compare with this instance.</param>
        /// <returns><c>true</c> if <paramref name="o"/> is an instance of <see cref="CLong"/> and equals the value of this instance; otherwise, <c>false</c>.</returns>
        public override bool Equals([NotNullWhen(true)] object? o) => o is CLong other && Equals(other);

        /// <summary>
        /// Returns a value indicating whether this instance is equal to a specified <see cref="CLong"/> value.
        /// </summary>
        /// <param name="other">A <see cref="CLong"/> value to compare to this instance.</param>
        /// <returns><c>true</c> if <paramref name="other"/> has the same value as this instance; otherwise, <c>false</c>.</returns>
        public bool Equals(CLong other) => _value == other._value;

        /// <summary>
        /// Returns the hash code for this instance.
        /// </summary>
        /// <returns>A 32-bit signed integer hash code.</returns>
        public override int GetHashCode() => _value.GetHashCode();

        /// <summary>
        /// Converts the numeric value of this instance to its equivalent string representation.
        /// </summary>
        /// <returns>The string representation of the value of this instance, consisting of a negative sign if the value is negative, and a sequence of digits ranging from 0 to 9 with no leading zeroes.</returns>
        public override string ToString() => _value.ToString();
    }
    
    public static unsafe class QuickSort
    {
        static void swapcode<T>(byte* parmi, byte* parmj, int n) where T : unmanaged
        {
            var i = (n) / sizeof(T);
            T* pi = (T*)parmi;
            T* pj = (T*)parmj;

            do
            {
                T t = *pi;
                *pi++ = *pj;
                *pj++ = t;
            } while (--i > 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void SWAPINIT(byte* a, int es, out int swaptype)
        {
            swaptype = ((a - (byte*)0) % sizeof(CLong) != 0) || (es % sizeof(CLong) != 0) ? 2 : (es == sizeof(CLong) ? 0 : 1);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void swapfunc(byte* a, byte* b, int n, int swaptype)
        {
            if (swaptype <= 1)
                swapcode<CLong>(a, b, n);
            else
                swapcode<byte>(a, b, n);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void swap(byte* a, byte* b, int es, int swaptype)
        {
            if (swaptype == 0)
            {
                var t = *(CLong*)a;
                *(CLong*)a = *(CLong*)b;
                *(CLong*)b = t;
            }
            else
                swapfunc(a, b, es, swaptype);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void vecswap(byte* a, byte* b, int n, int swaptype)
        {
            if (n > 0)
                swapfunc(a, b, n, swaptype);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static byte* med3(byte* a, byte* b, byte* c, delegate* managed<void*, void*, int> cmp)
        {
            return cmp(a, b) < 0 ?
                (cmp(b, c) < 0 ? b : (cmp(a, c) < 0 ? c : a))
                    : (cmp(b, c) > 0 ? b : (cmp(a, c) < 0 ? a : c));
        }

        internal static void qsort(void* aa, int n, int es, delegate* managed<void*, void*, int> cmp)
        {
            byte* pa; byte* pb; byte* pc; byte* pd; byte* pl; byte* pm; byte* pn;
            int d, r, swaptype, swap_cnt;
            byte* a = (byte*)aa;

        loop:
            SWAPINIT(a, es, out swaptype);

            swap_cnt = 0;
            if (n < 7)
            {
                for (pm = a + es; pm < a + n * es; pm += es)
                    for (pl = pm; pl > a && cmp(pl - es, pl) > 0; pl -= es)
                        swap(pl, pl - es, es, swaptype);
                return;
            }
            pm = a + (n / 2) * es;
            if (n > 7)
            {
                pl = a;
                pn = a + (n - 1) * es;
                if (n > 40)
                {
                    d = (n / 8) * es;
                    pl = med3(pl, pl + d, pl + 2 * d, cmp);
                    pm = med3(pm - d, pm, pm + d, cmp);
                    pn = med3(pn - 2 * d, pn - d, pn, cmp);
                }
                pm = med3(pl, pm, pn, cmp);
            }
            swap(a, pm, es, swaptype);
            pa = pb = a + es;

            pc = pd = a + (n - 1) * es;
            for (; ; )
            {
                while (pb <= pc && (r = cmp(pb, a)) <= 0)
                {
                    if (r == 0)
                    {
                        swap_cnt = 1;
                        swap(pa, pb, es, swaptype);
                        pa += es;
                    }
                    pb += es;
                }
                while (pb <= pc && (r = cmp(pc, a)) >= 0)
                {
                    if (r == 0)
                    {
                        swap_cnt = 1;
                        swap(pc, pd, es, swaptype);
                        pd -= es;
                    }
                    pc -= es;
                }
                if (pb > pc)
                    break;
                swap(pb, pc, es, swaptype);
                swap_cnt = 1;
                pb += es;
                pc -= es;
            }
            if (swap_cnt == 0)
            {  /* Switch to insertion sort */
                for (pm = a + es; pm < a + n * es; pm += es)
                    for (pl = pm; pl > a && cmp(pl - es, pl) > 0; pl -= es)
                        swap(pl, pl - es, es, swaptype);
                return;
            }
            pn = a + n * es;
            r = (int)Math.Min(pa - a, pb - pa);
            vecswap(a, pb - r, r, swaptype);
            r = (int)Math.Min(pd - pc, pn - pd - es);
            vecswap(pb, pn - r, r, swaptype);
            r = (int)(pb - pa);
            if (r > es)
                qsort(a, r / es, es, cmp);
            r = (int)(pd - pc);
            if (r > es)
            {
                /* Iterate rather than recurse to save stack space */
                a = pn - r;
                n = r / es;
                goto loop;
            }
        }
    }
}
