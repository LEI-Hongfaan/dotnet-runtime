// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace System.Numerics
{
    internal static class NumericsHelpers
    {
        private const int kcbitUint = 32;

        public static void GetDoubleParts(double dbl, out int sign, out int exp, out ulong man, out bool fFinite)
        {
            ulong bits = BitConverter.DoubleToUInt64Bits(dbl);

            sign = 1 - ((int)(bits >> 62) & 2);
            man = bits & 0x000FFFFFFFFFFFFF;
            exp = (int)(bits >> 52) & 0x7FF;
            if (exp == 0)
            {
                // Denormalized number.
                fFinite = true;
                if (man != 0)
                    exp = -1074;
            }
            else if (exp == 0x7FF)
            {
                // NaN or Infinite.
                fFinite = false;
                exp = int.MaxValue;
            }
            else
            {
                fFinite = true;
                man |= 0x0010000000000000;
                exp -= 1075;
            }
        }

        public static double GetDoubleFromParts(int sign, int exp, ulong man)
        {
            ulong bits;

            if (man == 0)
            {
                bits = 0;
            }
            else
            {
                // Normalize so that 0x0010 0000 0000 0000 is the highest bit set.
                int cbitShift = BitOperations.LeadingZeroCount(man) - 11;
                if (cbitShift < 0)
                    man >>= -cbitShift;
                else
                    man <<= cbitShift;
                exp -= cbitShift;
                Debug.Assert((man & 0xFFF0000000000000) == 0x0010000000000000);

                // Move the point to just behind the leading 1: 0x001.0 0000 0000 0000
                // (52 bits) and skew the exponent (by 0x3FF == 1023).
                exp += 1075;

                if (exp >= 0x7FF)
                {
                    // Infinity.
                    bits = 0x7FF0000000000000;
                }
                else if (exp <= 0)
                {
                    // Denormalized.
                    exp--;
                    if (exp < -52)
                    {
                        // Underflow to zero.
                        bits = 0;
                    }
                    else
                    {
                        bits = man >> -exp;
                        Debug.Assert(bits != 0);
                    }
                }
                else
                {
                    // Mask off the implicit high bit.
                    bits = (man & 0x000FFFFFFFFFFFFF) | ((ulong)exp << 52);
                }
            }

            if (sign < 0)
                bits |= 0x8000000000000000;

            return BitConverter.UInt64BitsToDouble(bits);
        }

        // Do an in-place two's complement. "Dangerous" because it causes
        // a mutation and needs to be used with care for immutable types.
        public static void DangerousMakeTwosComplement(Span<uint> d)
        {
            // Given a number:
            //     XXXXXXXXXXXY00000
            // where Y is non-zero,
            // The result of two's complement is
            //     AAAAAAAAAAAB00000
            // where A = ~X and B = -Y

            // Trim trailing 0s (at the first in little endian array)
            d = d.TrimStart(0u);

            // Make the first non-zero element to be two's complement
            if (d.Length > 0)
            {
                d[0] = (uint)(-(int)d[0]);
                d = d.Slice(1);
            }

            if (d.IsEmpty)
            {
                return;
            }

            // Make one's complement for other elements
            int offset = 0;

            ref uint start = ref MemoryMarshal.GetReference(d);

            while (Vector512.IsHardwareAccelerated && d.Length - offset >= Vector512<uint>.Count)
            {
                Vector512<uint> complement = ~Vector512.LoadUnsafe(ref start, (nuint)offset);
                Vector512.StoreUnsafe(complement, ref start, (nuint)offset);
                offset += Vector512<uint>.Count;
            }

            while (Vector256.IsHardwareAccelerated && d.Length - offset >= Vector256<uint>.Count)
            {
                Vector256<uint> complement = ~Vector256.LoadUnsafe(ref start, (nuint)offset);
                Vector256.StoreUnsafe(complement, ref start, (nuint)offset);
                offset += Vector256<uint>.Count;
            }

            while (Vector128.IsHardwareAccelerated && d.Length - offset >= Vector128<uint>.Count)
            {
                Vector128<uint> complement = ~Vector128.LoadUnsafe(ref start, (nuint)offset);
                Vector128.StoreUnsafe(complement, ref start, (nuint)offset);
                offset += Vector128<uint>.Count;
            }

            for (; offset < d.Length; offset++)
            {
                d[offset] = ~d[offset];
            }
        }

        public static void BitwiseAnd(ReadOnlySpan<uint> x, ReadOnlySpan<uint> y, Span<uint> destination)
        {
            DoBitwiseOperation<BitwiseAndOperator>(x, y, destination);
        }

        public static void AndNot(ReadOnlySpan<uint> x, ReadOnlySpan<uint> y, Span<uint> destination)
        {
            DoBitwiseOperation<AndNotOperator>(x, y, destination);
        }

        public static void BitwiseOr(ReadOnlySpan<uint> x, ReadOnlySpan<uint> y, Span<uint> destination)
        {
            DoBitwiseOperation<BitwiseOrOperator>(x, y, destination);
        }

        public static void Xor(ReadOnlySpan<uint> x, ReadOnlySpan<uint> y, Span<uint> destination)
        {
            DoBitwiseOperation<XorOperator>(x, y, destination);
        }

        private readonly struct BitwiseAndOperator : IBinaryOperator<uint>
        {
            public static uint Invoke(uint x, uint y) => x & y;

            public static Vector128<uint> Invoke(Vector128<uint> x, Vector128<uint> y) => Vector128.BitwiseAnd(x, y);
        }

        private readonly struct AndNotOperator : IBinaryOperator<uint>
        {
            public static uint Invoke(uint x, uint y) => x & ~y;

            public static Vector128<uint> Invoke(Vector128<uint> x, Vector128<uint> y) => Vector128.AndNot(x, y);
        }

        private readonly struct BitwiseOrOperator : IBinaryOperator<uint>
        {
            public static uint Invoke(uint x, uint y) => x & ~y;

            public static Vector128<uint> Invoke(Vector128<uint> x, Vector128<uint> y) => Vector128.BitwiseOr(x, y);
        }

        private readonly struct XorOperator : IBinaryOperator<uint>
        {
            public static uint Invoke(uint x, uint y) => x & ~y;

            public static Vector128<uint> Invoke(Vector128<uint> x, Vector128<uint> y) => Vector128.Xor(x, y);
        }

        public static void DoBitwiseOperation<TBinaryOperator>(ReadOnlySpan<uint> x, ReadOnlySpan<uint> y, Span<uint> destination)
            where TBinaryOperator : struct, IBinaryOperator<uint>
        {
            Debug.Assert(x.Length == y.Length);
            Debug.Assert(x.Length <= destination.Length);

            Debug.Assert(!destination.Overlaps(x));
            Debug.Assert(!destination.Overlaps(y));

            // This is a simplified version of the corresponding routine in TensorPrimitives.
            // It's suitable for usages in System.Runtime.Numerics.
            // Since the data may be unaligned, I'm not going to vectorize it too much.

            ref uint xRef = ref MemoryMarshal.GetReference(x);
            ref uint yRef = ref MemoryMarshal.GetReference(y);
            ref uint dRef = ref MemoryMarshal.GetReference(destination);

            nuint remainder = (uint)x.Length;

            if (Vector128.IsHardwareAccelerated && Vector128<uint>.IsSupported)
            {
                if (remainder >= (uint)Vector128<uint>.Count)
                {
                    Vectorized128(ref xRef, ref yRef, ref dRef, remainder);
                    return;
                }
            }

            // This is the software fallback when no acceleration is available
            // It requires no branches to hit

            for (nuint i = 0; i < remainder; i++)
            {
                Unsafe.Add(ref dRef, i) = TBinaryOperator.Invoke(Unsafe.Add(ref xRef, i),
                                                                 Unsafe.Add(ref yRef, i));
            }

            unsafe static void Vectorized128(ref uint xRef, ref uint yRef, ref uint dRef, nuint remainder)
            {
                ref uint dRefBeg = ref dRef;

                // Preload the beginning and end so that overlapping accesses don't negatively impact the data

                Vector128<uint> beg = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef),
                                                             Vector128.LoadUnsafe(ref yRef));
                Vector128<uint> end = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef, remainder - (uint)Vector128<uint>.Count),
                                                             Vector128.LoadUnsafe(ref yRef, remainder - (uint)Vector128<uint>.Count));

                if (remainder > (uint)(Vector128<uint>.Count * 8))
                {
                    // Pinning is cheap and will be short lived for small inputs and unlikely to be impactful
                    // for large inputs (> 85KB) which are on the LOH and unlikely to be compacted.

                    fixed (uint* px = &xRef)
                    fixed (uint* py = &yRef)
                    fixed (uint* pd = &dRef)
                    {
                        uint* xPtr = px;
                        uint* yPtr = py;
                        uint* dPtr = pd;

                        // We need to the ensure the underlying data can be aligned and only align
                        // it if it can. It is possible we have an unaligned ref, in which case we
                        // can never achieve the required SIMD alignment.

                        bool canAlign = ((nuint)dPtr % (nuint)sizeof(uint)) == 0;

                        if (canAlign)
                        {
                            // Compute by how many elements we're misaligned and adjust the pointers accordingly
                            //
                            // Noting that we are only actually aligning dPtr. This is because unaligned stores
                            // are more expensive than unaligned loads and aligning both is significantly more
                            // complex.

                            nuint misalignment = ((uint)sizeof(Vector128<uint>) - ((nuint)dPtr % (uint)sizeof(Vector128<uint>))) / (uint)sizeof(uint);

                            xPtr += misalignment;
                            yPtr += misalignment;
                            dPtr += misalignment;

                            Debug.Assert(((nuint)dPtr % (uint)sizeof(Vector128<uint>)) == 0);

                            remainder -= misalignment;
                        }

                        Vector128<uint> vector1;
                        Vector128<uint> vector2;
                        Vector128<uint> vector3;
                        Vector128<uint> vector4;

                        if ((remainder > (NonTemporalByteThreshold / (nuint)sizeof(uint))) && canAlign)
                        {
                            // This loop stores the data non-temporally, which benefits us when there
                            // is a large amount of data involved as it avoids polluting the cache.

                            while (remainder >= (uint)(Vector128<uint>.Count * 8))
                            {
                                // We load, process, and store the first four vectors

                                vector1 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 0)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 0)));
                                vector2 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 1)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 1)));
                                vector3 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 2)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 2)));
                                vector4 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 3)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 3)));

                                vector1.StoreAlignedNonTemporal(dPtr + (uint)(Vector128<uint>.Count * 0));
                                vector2.StoreAlignedNonTemporal(dPtr + (uint)(Vector128<uint>.Count * 1));
                                vector3.StoreAlignedNonTemporal(dPtr + (uint)(Vector128<uint>.Count * 2));
                                vector4.StoreAlignedNonTemporal(dPtr + (uint)(Vector128<uint>.Count * 3));

                                // We load, process, and store the next four vectors

                                vector1 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 4)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 4)));
                                vector2 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 5)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 5)));
                                vector3 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 6)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 6)));
                                vector4 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 7)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 7)));

                                vector1.StoreAlignedNonTemporal(dPtr + (uint)(Vector128<uint>.Count * 4));
                                vector2.StoreAlignedNonTemporal(dPtr + (uint)(Vector128<uint>.Count * 5));
                                vector3.StoreAlignedNonTemporal(dPtr + (uint)(Vector128<uint>.Count * 6));
                                vector4.StoreAlignedNonTemporal(dPtr + (uint)(Vector128<uint>.Count * 7));

                                // We adjust the source and destination references, then update
                                // the count of remaining elements to process.

                                xPtr += (uint)(Vector128<uint>.Count * 8);
                                yPtr += (uint)(Vector128<uint>.Count * 8);
                                dPtr += (uint)(Vector128<uint>.Count * 8);

                                remainder -= (uint)(Vector128<uint>.Count * 8);
                            }
                        }
                        else
                        {
                            while (remainder >= (uint)(Vector128<uint>.Count * 8))
                            {
                                // We load, process, and store the first four vectors

                                vector1 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 0)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 0)));
                                vector2 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 1)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 1)));
                                vector3 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 2)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 2)));
                                vector4 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 3)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 3)));

                                vector1.Store(dPtr + (uint)(Vector128<uint>.Count * 0));
                                vector2.Store(dPtr + (uint)(Vector128<uint>.Count * 1));
                                vector3.Store(dPtr + (uint)(Vector128<uint>.Count * 2));
                                vector4.Store(dPtr + (uint)(Vector128<uint>.Count * 3));

                                // We load, process, and store the next four vectors

                                vector1 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 4)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 4)));
                                vector2 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 5)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 5)));
                                vector3 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 6)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 6)));
                                vector4 = TBinaryOperator.Invoke(Vector128.Load(xPtr + (uint)(Vector128<uint>.Count * 7)),
                                                                 Vector128.Load(yPtr + (uint)(Vector128<uint>.Count * 7)));

                                vector1.Store(dPtr + (uint)(Vector128<uint>.Count * 4));
                                vector2.Store(dPtr + (uint)(Vector128<uint>.Count * 5));
                                vector3.Store(dPtr + (uint)(Vector128<uint>.Count * 6));
                                vector4.Store(dPtr + (uint)(Vector128<uint>.Count * 7));

                                // We adjust the source and destination references, then update
                                // the count of remaining elements to process.

                                xPtr += (uint)(Vector128<uint>.Count * 8);
                                yPtr += (uint)(Vector128<uint>.Count * 8);
                                dPtr += (uint)(Vector128<uint>.Count * 8);

                                remainder -= (uint)(Vector128<uint>.Count * 8);
                            }
                        }

                        // Adjusting the refs here allows us to avoid pinning for very small inputs

                        xRef = ref *xPtr;
                        yRef = ref *yPtr;
                        dRef = ref *dPtr;
                    }
                }

                // Process the remaining [Count, Count * 8] elements via a jump table
                //
                // Unless the original length was an exact multiple of Count, then we'll
                // end up reprocessing a couple elements in case 1 for end. We'll also
                // potentially reprocess a few elements in case 0 for beg, to handle any
                // data before the first aligned address.

                nuint endIndex = remainder;
                remainder = (remainder + (uint)(Vector128<uint>.Count - 1)) & (nuint)(-Vector128<uint>.Count);

                switch (remainder / (uint)Vector128<uint>.Count)
                {
                    case 8:
                    {
                        Vector128<uint> vector = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef, remainder - (uint)(Vector128<uint>.Count * 8)),
                                                                        Vector128.LoadUnsafe(ref yRef, remainder - (uint)(Vector128<uint>.Count * 8)));
                        vector.StoreUnsafe(ref dRef, remainder - (uint)(Vector128<uint>.Count * 8));
                        goto case 7;
                    }

                    case 7:
                    {
                        Vector128<uint> vector = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef, remainder - (uint)(Vector128<uint>.Count * 7)),
                                                                        Vector128.LoadUnsafe(ref yRef, remainder - (uint)(Vector128<uint>.Count * 7)));
                        vector.StoreUnsafe(ref dRef, remainder - (uint)(Vector128<uint>.Count * 7));
                        goto case 6;
                    }

                    case 6:
                    {
                        Vector128<uint> vector = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef, remainder - (uint)(Vector128<uint>.Count * 6)),
                                                                        Vector128.LoadUnsafe(ref yRef, remainder - (uint)(Vector128<uint>.Count * 6)));
                        vector.StoreUnsafe(ref dRef, remainder - (uint)(Vector128<uint>.Count * 6));
                        goto case 5;
                    }

                    case 5:
                    {
                        Vector128<uint> vector = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef, remainder - (uint)(Vector128<uint>.Count * 5)),
                                                                     Vector128.LoadUnsafe(ref yRef, remainder - (uint)(Vector128<uint>.Count * 5)));
                        vector.StoreUnsafe(ref dRef, remainder - (uint)(Vector128<uint>.Count * 5));
                        goto case 4;
                    }

                    case 4:
                    {
                        Vector128<uint> vector = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef, remainder - (uint)(Vector128<uint>.Count * 4)),
                                                                        Vector128.LoadUnsafe(ref yRef, remainder - (uint)(Vector128<uint>.Count * 4)));
                        vector.StoreUnsafe(ref dRef, remainder - (uint)(Vector128<uint>.Count * 4));
                        goto case 3;
                    }

                    case 3:
                    {
                        Vector128<uint> vector = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef, remainder - (uint)(Vector128<uint>.Count * 3)),
                                                                        Vector128.LoadUnsafe(ref yRef, remainder - (uint)(Vector128<uint>.Count * 3)));
                        vector.StoreUnsafe(ref dRef, remainder - (uint)(Vector128<uint>.Count * 3));
                        goto case 2;
                    }

                    case 2:
                    {
                        Vector128<uint> vector = TBinaryOperator.Invoke(Vector128.LoadUnsafe(ref xRef, remainder - (uint)(Vector128<uint>.Count * 2)),
                                                                        Vector128.LoadUnsafe(ref yRef, remainder - (uint)(Vector128<uint>.Count * 2)));
                        vector.StoreUnsafe(ref dRef, remainder - (uint)(Vector128<uint>.Count * 2));
                        goto case 1;
                    }

                    case 1:
                    {
                        // Store the last block, which includes any elements that wouldn't fill a full vector
                        end.StoreUnsafe(ref dRef, endIndex - (uint)Vector128<uint>.Count);
                        goto case 0;
                    }

                    case 0:
                    {
                        // Store the first block, which includes any elements preceding the first aligned block
                        beg.StoreUnsafe(ref dRefBeg);
                        break;
                    }
                }
            }

        }

        public interface IBinaryOperator<T>
        {
            static abstract T Invoke(T x, T y);

            static abstract Vector128<T> Invoke(Vector128<T> x, Vector128<T> y);
        }

        private const nuint NonTemporalByteThreshold = 256 * 1024;

        public static ulong MakeUInt64(uint uHi, uint uLo)
        {
            return ((ulong)uHi << kcbitUint) | uLo;
        }

        public static uint Abs(int a)
        {
            unchecked
            {
                uint mask = (uint)(a >> 31);
                return ((uint)a ^ mask) - mask;
            }
        }
    }
}
