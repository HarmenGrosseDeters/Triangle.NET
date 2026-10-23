namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Test category for red tests documenting known bugs. Tests in this category are
    /// expected to fail until the corresponding bug is fixed. Once a test turns green,
    /// remove the category so it becomes a regular regression test.
    /// </summary>
    /// <remarks>
    /// Run only the known bugs:   dotnet test --filter TestCategory=KnownBug
    /// Run everything else:       dotnet test --filter TestCategory!=KnownBug
    /// </remarks>
    static class KnownBug
    {
        public const string Category = "KnownBug";
    }
}
