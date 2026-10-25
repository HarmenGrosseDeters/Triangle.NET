// -----------------------------------------------------------------------
// <copyright file="InlineBuffers.cs">
// Triangle.NET Copyright (c) 2012-2022 Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

#if NET8_0_OR_GREATER && !FIELD_LAYOUT && !ARRAY_LAYOUT
namespace TriangleNet.Topology
{
    using System.Runtime.CompilerServices;

    // Fixed size arrays stored inline in the triangle and subsegment objects (no separate
    // objects on the heap). Indexed like arrays.

    [InlineArray(2)]
    internal struct Buffer2<T>
    {
        private T element;
    }

    [InlineArray(3)]
    internal struct Buffer3<T>
    {
        private T element;
    }

    [InlineArray(4)]
    internal struct Buffer4<T>
    {
        private T element;
    }
}
#endif
