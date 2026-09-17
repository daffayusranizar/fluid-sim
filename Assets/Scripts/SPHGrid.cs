using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

/// <summary>
/// Neighbours in compressed sparse row form.
///
/// Particle i's fluid neighbours are fluid[fluidStart[i] .. fluidStart[i + 1]],
/// and its boundary neighbours are boundary[boundaryStart[i] .. boundaryStart[i + 1]].
/// Both arrays of starts have fluidCount + 1 entries so the last particle's range
/// needs no special case.
///
/// This replaces List&lt;int&gt;[] because Burst cannot touch managed types. The layout
/// also happens to be exactly what a parallel job wants at T-025: each particle
/// reads only its own contiguous range of a read-only array, so no writes to
/// shared state and no race conditions.
/// </summary>
public struct NeighborLists
{
    public NativeArray<int> fluidStart;
    public NativeArray<int> fluid;
    public NativeArray<int> boundaryStart;
    public NativeArray<int> boundary;
}

/// <summary>Grid geometry. Kept blittable so Burst can take it directly.</summary>
public struct GridGeometry
{
    public float2 origin;
    public float cellSize;
    public int cols;
    public int rows;
    public int cellCount;
}

/// <summary>
/// Uniform grid over the simulation domain, rebuilt every step.
///
/// Owns its native storage and orchestrates the build. The loops themselves live
/// in <see cref="SPHGridOps"/> so they can be Burst compiled: a managed class
/// cannot be, but a static method on a dedicated static class can.
///
/// Cell size is exactly the smoothing length. That is not just a convention --
/// the 3x3 query is only guaranteed to find every neighbour if cell size is
/// greater than or equal to h. Cells slightly smaller than h would let a
/// neighbour at distance just under h land two cells away and be missed.
/// </summary>
public struct SPHGrid : IDisposable
{
    private const Allocator Alloc = Allocator.Persistent;

    // --- Cell structure ---
    private NativeArray<int> packedCell;    // packed index -> cell
    private NativeArray<int> cellStart;     // cell -> first slot of its run
    private NativeArray<int> cellCursor;    // scratch while scattering
    private NativeArray<int> sortedByCell;  // packed indices grouped by cell

    // --- Neighbour output (CSR) ---
    private NativeArray<int> fluidStart;
    private NativeArray<int> fluidList;
    private NativeArray<int> boundaryStart;
    private NativeArray<int> boundaryList;

    // --- Capacities, so growth is detectable without touching Length on null arrays ---
    private int packedCapacity;
    private int cellCapacity;
    private int fluidListCapacity;
    private int boundaryListCapacity;
    private int neighborStartCapacity;

    private GridGeometry geometry;
    private int fluidCount;
    private int boundaryCount;
    private int totalCount;
    private float radiusSq;

    /// <summary>
    /// Rebuilds the grid and the neighbour lists, and returns the lists for the
    /// solver to read. The returned struct must be re-read after every rebuild,
    /// because growth can replace the underlying arrays.
    /// </summary>
    public NeighborLists Rebuild(
        NativeArray<Particle2D> fluid,
        NativeArray<float2> boundary,
        float2 domainOrigin,
        float2 domainSize,
        float smoothingLength)
    {
        fluidCount = fluid.Length;
        boundaryCount = boundary.IsCreated ? boundary.Length : 0;
        totalCount = fluidCount + boundaryCount;

        float cellSize = math.max(0.0001f, smoothingLength);
        radiusSq = smoothingLength * smoothingLength;

        geometry = new GridGeometry
        {
            origin = domainOrigin,
            cellSize = cellSize,
            cols = math.max(1, (int)math.ceil(domainSize.x / cellSize)),
            rows = math.max(1, (int)math.ceil(domainSize.y / cellSize)),
        };
        geometry.cellCount = geometry.cols * geometry.rows;

        EnsureCellStorage();
        EnsureNeighborStartStorage();

        SPHGridOps.BuildCells(
            fluid, boundary,
            packedCell, cellStart, cellCursor, sortedByCell,
            geometry, fluidCount, boundaryCount, totalCount);

        // Count neighbours, then turn the counts into start offsets.
        // CountNeighbors writes counts into start[i + 1], leaving start[0] as 0.
        SPHGridOps.CountNeighbors(
            fluid, boundary,
            sortedByCell, cellStart,
            fluidStart, boundaryStart,
            geometry, fluidCount, totalCount, radiusSq);

        PrefixSum(fluidStart, fluidCount);
        PrefixSum(boundaryStart, fluidCount);

        EnsureNeighborListStorage(fluidStart[fluidCount], boundaryStart[fluidCount]);

        SPHGridOps.FillNeighbors(
            fluid, boundary,
            sortedByCell, cellStart,
            fluidStart, fluidList,
            boundaryStart, boundaryList,
            geometry, fluidCount, radiusSq);

        return new NeighborLists
        {
            fluidStart = fluidStart,
            fluid = fluidList,
            boundaryStart = boundaryStart,
            boundary = boundaryList,
        };
    }

    /// <summary>
    /// Turns per-particle counts into CSR start offsets, in place.
    /// On entry start[i + 1] holds particle i's count; on exit start[i] holds the
    /// offset of particle i's run and start[count] holds the total.
    /// </summary>
    private static void PrefixSum(NativeArray<int> start, int count)
    {
        start[0] = 0;

        for (int i = 1; i <= count; i++)
        {
            start[i] += start[i - 1];
        }
    }

    private void EnsureCellStorage()
    {
        if (!packedCell.IsCreated || packedCapacity < totalCount)
        {
            DisposeIfCreated(packedCell);
            DisposeIfCreated(sortedByCell);

            packedCapacity = math.max(1, totalCount);
            packedCell = new NativeArray<int>(packedCapacity, Alloc);
            sortedByCell = new NativeArray<int>(packedCapacity, Alloc);
        }

        if (!cellStart.IsCreated || cellCapacity < geometry.cellCount)
        {
            DisposeIfCreated(cellStart);
            DisposeIfCreated(cellCursor);

            cellCapacity = geometry.cellCount;
            cellStart = new NativeArray<int>(cellCapacity, Alloc);
            cellCursor = new NativeArray<int>(cellCapacity, Alloc);
        }
    }

    private void EnsureNeighborStartStorage()
    {
        int needed = fluidCount + 1;

        if (!fluidStart.IsCreated || neighborStartCapacity < needed)
        {
            DisposeIfCreated(fluidStart);
            DisposeIfCreated(boundaryStart);

            neighborStartCapacity = needed;
            fluidStart = new NativeArray<int>(needed, Alloc);
            boundaryStart = new NativeArray<int>(needed, Alloc);
        }
        else
        {
            // Counts are written into [1..count]; clear so stale counts cannot leak
            for (int i = 0; i <= fluidCount; i++)
            {
                fluidStart[i] = 0;
                boundaryStart[i] = 0;
            }
        }
    }

    private void EnsureNeighborListStorage(int fluidNeeded, int boundaryNeeded)
    {
        if (!fluidList.IsCreated || fluidListCapacity < fluidNeeded)
        {
            DisposeIfCreated(fluidList);
            fluidListCapacity = math.max(1, fluidNeeded);
            fluidList = new NativeArray<int>(fluidListCapacity, Alloc);
        }

        if (!boundaryList.IsCreated || boundaryListCapacity < boundaryNeeded)
        {
            DisposeIfCreated(boundaryList);
            boundaryListCapacity = math.max(1, boundaryNeeded);
            boundaryList = new NativeArray<int>(boundaryListCapacity, Alloc);
        }
    }

    private static void DisposeIfCreated(NativeArray<int> array)
    {
        if (array.IsCreated) array.Dispose();
    }

    public void Dispose()
    {
        DisposeIfCreated(packedCell);
        DisposeIfCreated(cellStart);
        DisposeIfCreated(cellCursor);
        DisposeIfCreated(sortedByCell);
        DisposeIfCreated(fluidStart);
        DisposeIfCreated(fluidList);
        DisposeIfCreated(boundaryStart);
        DisposeIfCreated(boundaryList);
    }
}

/// <summary>
/// The grid's hot loops, Burst compiled.
///
/// Kept as a dedicated static class rather than as methods on SPHGrid, because
/// that is the shape Burst documents for direct calls from managed code:
/// a [BurstCompile] static class holding [BurstCompile] static methods.
///
/// All state travels through NativeArray parameters. NativeArray is a struct
/// wrapping a pointer, so copying it at the call boundary still shares the same
/// underlying memory -- writes propagate even though the parameter is by value.
///
/// CountNeighbors and FillNeighbors both walk the 3x3 cell neighbourhood and
/// must stay in step: the first decides how large each particle's run is, the
/// second writes into it. Any change to the traversal or the radius test has to
/// be made in both.
/// </summary>
[BurstCompile]
public static class SPHGridOps
{
    [BurstCompile]
    public static void BuildCells(
        in NativeArray<Particle2D> fluid,
        in NativeArray<float2> boundary,
        NativeArray<int> packedCell,
        NativeArray<int> cellStart,
        NativeArray<int> cellCursor,
        NativeArray<int> sortedByCell,
        in GridGeometry g,
        int fluidCount,
        int boundaryCount,
        int totalCount)
    {
        // 1. Cell index per packed particle
        for (int i = 0; i < fluidCount; i++)
        {
            packedCell[i] = CellOf(fluid[i].position, g);
        }

        for (int b = 0; b < boundaryCount; b++)
        {
            packedCell[fluidCount + b] = CellOf(boundary[b], g);
        }

        // 2. Count per cell (cellStart doubles as the counter here)
        for (int c = 0; c < g.cellCount; c++)
        {
            cellStart[c] = 0;
        }

        for (int p = 0; p < totalCount; p++)
        {
            cellStart[packedCell[p]]++;
        }

        // 3. Exclusive prefix sum. cellStart becomes the first slot of each
        //    cell's run; cellCursor starts at the same value and is consumed
        //    slot by slot during the scatter.
        int running = 0;
        for (int c = 0; c < g.cellCount; c++)
        {
            int count = cellStart[c];
            cellStart[c] = running;
            cellCursor[c] = running;
            running += count;
        }

        // 4. Scatter. Runs stay in cell order, so cell c ends exactly where
        //    cell c + 1 begins.
        for (int p = 0; p < totalCount; p++)
        {
            int c = packedCell[p];
            sortedByCell[cellCursor[c]++] = p;
        }
    }

    [BurstCompile]
    public static void CountNeighbors(
        in NativeArray<Particle2D> fluid,
        in NativeArray<float2> boundary,
        in NativeArray<int> sortedByCell,
        in NativeArray<int> cellStart,
        NativeArray<int> fluidStart,
        NativeArray<int> boundaryStart,
        in GridGeometry g,
        int fluidCount,
        int totalCount,
        float radiusSq)
    {
        for (int i = 0; i < fluidCount; i++)
        {
            float2 posI = fluid[i].position;
            int cx = ColumnOf(posI.x, g);
            int cy = RowOf(posI.y, g);

            int fluidFound = 0;
            int boundaryFound = 0;

            for (int gy = cy - 1; gy <= cy + 1; gy++)
            {
                // Out-of-grid cells are skipped, not clamped. Clamping would
                // pull distant edge cells into range for corner particles.
                if (gy < 0 || gy >= g.rows) continue;

                for (int gx = cx - 1; gx <= cx + 1; gx++)
                {
                    if (gx < 0 || gx >= g.cols) continue;

                    int c = gy * g.cols + gx;
                    int end = CellEnd(c, cellStart, g.cellCount, totalCount);

                    for (int k = cellStart[c]; k < end; k++)
                    {
                        int packed = sortedByCell[k];

                        if (packed < fluidCount)
                        {
                            if (packed == i) continue;
                            if (math.distancesq(fluid[packed].position, posI) < radiusSq) fluidFound++;
                        }
                        else
                        {
                            int b = packed - fluidCount;
                            if (math.distancesq(boundary[b], posI) < radiusSq) boundaryFound++;
                        }
                    }
                }
            }

            fluidStart[i + 1] = fluidFound;
            boundaryStart[i + 1] = boundaryFound;
        }
    }

    [BurstCompile]
    public static void FillNeighbors(
        in NativeArray<Particle2D> fluid,
        in NativeArray<float2> boundary,
        in NativeArray<int> sortedByCell,
        in NativeArray<int> cellStart,
        in NativeArray<int> fluidStart,
        NativeArray<int> fluidList,
        in NativeArray<int> boundaryStart,
        NativeArray<int> boundaryList,
        in GridGeometry g,
        int fluidCount,
        float radiusSq)
    {
        int totalCount = fluidCount + boundary.Length;

        for (int i = 0; i < fluidCount; i++)
        {
            float2 posI = fluid[i].position;
            int cx = ColumnOf(posI.x, g);
            int cy = RowOf(posI.y, g);

            // Each particle writes only its own run, so a local cursor is enough
            int fluidWrite = fluidStart[i];
            int boundaryWrite = boundaryStart[i];

            for (int gy = cy - 1; gy <= cy + 1; gy++)
            {
                if (gy < 0 || gy >= g.rows) continue;

                for (int gx = cx - 1; gx <= cx + 1; gx++)
                {
                    if (gx < 0 || gx >= g.cols) continue;

                    int c = gy * g.cols + gx;
                    int end = CellEnd(c, cellStart, g.cellCount, totalCount);

                    for (int k = cellStart[c]; k < end; k++)
                    {
                        int packed = sortedByCell[k];

                        if (packed < fluidCount)
                        {
                            if (packed == i) continue;
                            if (math.distancesq(fluid[packed].position, posI) < radiusSq)
                            {
                                fluidList[fluidWrite++] = packed;
                            }
                        }
                        else
                        {
                            int b = packed - fluidCount;
                            if (math.distancesq(boundary[b], posI) < radiusSq)
                            {
                                boundaryList[boundaryWrite++] = b;
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>Cell index of a position, clamped into the grid.</summary>
    [BurstCompile]
    public static int CellOf(float2 position, in GridGeometry g)
    {
        return RowOf(position.y, g) * g.cols + ColumnOf(position.x, g);
    }

    [BurstCompile]
    public static int ColumnOf(float x, in GridGeometry g)
    {
        return math.clamp((int)math.floor((x - g.origin.x) / g.cellSize), 0, g.cols - 1);
    }

    [BurstCompile]
    public static int RowOf(float y, in GridGeometry g)
    {
        return math.clamp((int)math.floor((y - g.origin.y) / g.cellSize), 0, g.rows - 1);
    }

    /// <summary>
    /// One past the last slot of a cell's run. Because runs are laid out in cell
    /// order, cell c ends exactly where cell c + 1 begins, so no explicit end
    /// array is needed. An empty cell yields an empty range.
    /// </summary>
    [BurstCompile]
    public static int CellEnd(int cell, in NativeArray<int> cellStart, int cellCount, int totalCount)
    {
        return cell + 1 < cellCount ? cellStart[cell + 1] : totalCount;
    }
}
