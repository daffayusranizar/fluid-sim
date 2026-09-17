using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Uniform grid over the simulation domain, rebuilt every step. It turns the
/// neighbour search from O(N^2) into roughly O(N * neighboursPerParticle).
///
/// Cell size equals the smoothing length, so a neighbour can only live in a
/// particle's own cell or one of the 8 surrounding cells. Because the domain is
/// a bounded box, cells are located by direct index arithmetic rather than a
/// hash: no collisions, O(1) lookup. (Spatial hashing exists to represent
/// unbounded domains and pays for that with collisions. The GPU path does need
/// it -- see T-030 -- but the CPU path does not.)
///
/// Fluid and boundary particles share one grid, packed as:
///     [0, fluidCount)                 -> fluid particle
///     [fluidCount, fluidCount + B)    -> boundary particle (index - fluidCount)
///
/// Particles are grouped into cells with a counting sort (count, exclusive
/// prefix sum, scatter), so each cell occupies one contiguous run of
/// sortedByCell. That is cache friendly, and it is the same algorithm as the
/// GPU count sort and prefix scan in T-031.
/// </summary>
public class SPHGrid
{
    // --- Domain ---
    private Vector2 origin;
    private float cellSize = 1f;
    private int cols = 1;
    private int rows = 1;
    private int cellCount = 1;

    // --- Packed particle counts ---
    private int fluidCount;
    private int boundaryCount;
    private int totalCount;

    // --- Counting-sort storage ---
    private int[] packedCell;    // packed index -> cell index
    private int[] cellStart;     // cell -> first slot of its run in sortedByCell
    private int[] cellCursor;    // scratch cursor used while scattering
    private int[] sortedByCell;  // packed indices, grouped by cell

    /// <summary>Neighbour indices per fluid particle, fluid particles only.</summary>
    public List<int>[] FluidNeighbors { get; private set; }

    /// <summary>Neighbour indices per fluid particle, boundary particles only.</summary>
    public List<int>[] BoundaryNeighbors { get; private set; }

    /// <summary>
    /// Rebuilds the grid from the current positions and answers every fluid
    /// particle's neighbourhood query.
    /// </summary>
    public void Rebuild(
        Particle2D[] fluid,
        Vector2[] boundary,
        Vector2 domainOrigin,
        Vector2 domainSize,
        float smoothingLength)
    {
        fluidCount = fluid.Length;
        boundaryCount = boundary != null ? boundary.Length : 0;
        totalCount = fluidCount + boundaryCount;

        cellSize = Mathf.Max(0.0001f, smoothingLength);
        cols = Mathf.Max(1, Mathf.CeilToInt(domainSize.x / cellSize));
        rows = Mathf.Max(1, Mathf.CeilToInt(domainSize.y / cellSize));
        cellCount = cols * rows;
        origin = domainOrigin;

        EnsureStorage();

        // 1. Cell index for every packed particle
        for (int i = 0; i < fluidCount; i++)
        {
            packedCell[i] = CellOf(fluid[i].position);
        }

        for (int b = 0; b < boundaryCount; b++)
        {
            packedCell[fluidCount + b] = CellOf(boundary[b]);
        }

        // 2. Count particles per cell (cellStart doubles as the counter here)
        System.Array.Clear(cellStart, 0, cellCount);
        for (int p = 0; p < totalCount; p++)
        {
            cellStart[packedCell[p]]++;
        }

        // 3. Exclusive prefix sum. cellStart[c] becomes the first slot of cell
        //    c's run, and cellCursor starts at the same value to be consumed
        //    slot by slot during the scatter.
        int running = 0;
        for (int c = 0; c < cellCount; c++)
        {
            int count = cellStart[c];
            cellStart[c] = running;
            cellCursor[c] = running;
            running += count;
        }

        // 4. Scatter packed indices into their cell's run. Runs stay in cell
        //    order, so cell c ends exactly where cell c + 1 begins.
        for (int p = 0; p < totalCount; p++)
        {
            int c = packedCell[p];
            sortedByCell[cellCursor[c]++] = p;
        }

        QueryNeighbors(fluid, boundary, cellSize * cellSize);
    }

    private void QueryNeighbors(Particle2D[] fluid, Vector2[] boundary, float radiusSq)
    {
        for (int i = 0; i < fluidCount; i++)
        {
            List<int> fluidList = FluidNeighbors[i];
            List<int> boundaryList = BoundaryNeighbors[i];
            fluidList.Clear();
            boundaryList.Clear();

            Vector2 posI = fluid[i].position;
            int cx = ColumnOf(posI.x);
            int cy = RowOf(posI.y);

            for (int gy = cy - 1; gy <= cy + 1; gy++)
            {
                // Cells outside the grid simply do not exist; skip rather than
                // clamp, or distant edge cells would be pulled into range.
                if (gy < 0 || gy >= rows) continue;

                for (int gx = cx - 1; gx <= cx + 1; gx++)
                {
                    if (gx < 0 || gx >= cols) continue;

                    int c = gy * cols + gx;
                    int end = CellEnd(c);

                    for (int k = cellStart[c]; k < end; k++)
                    {
                        int p = sortedByCell[k];

                        if (p < fluidCount)
                        {
                            if (p == i) continue;

                            if ((fluid[p].position - posI).sqrMagnitude < radiusSq)
                            {
                                fluidList.Add(p);
                            }
                        }
                        else
                        {
                            int b = p - fluidCount;

                            if ((boundary[b] - posI).sqrMagnitude < radiusSq)
                            {
                                boundaryList.Add(b);
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>Cell index of a position, clamped into the grid.</summary>
    private int CellOf(Vector2 position)
    {
        return RowOf(position.y) * cols + ColumnOf(position.x);
    }

    private int ColumnOf(float x)
    {
        return Mathf.Clamp(Mathf.FloorToInt((x - origin.x) / cellSize), 0, cols - 1);
    }

    private int RowOf(float y)
    {
        return Mathf.Clamp(Mathf.FloorToInt((y - origin.y) / cellSize), 0, rows - 1);
    }

    /// <summary>One past the last slot of a cell's run. Empty cells give an empty range.</summary>
    private int CellEnd(int cell)
    {
        return cell + 1 < cellCount ? cellStart[cell + 1] : totalCount;
    }

    /// <summary>Allocates on first use and only reallocates when sizes grow.</summary>
    private void EnsureStorage()
    {
        if (packedCell == null || packedCell.Length < totalCount)
        {
            packedCell = new int[Mathf.Max(1, totalCount)];
            sortedByCell = new int[Mathf.Max(1, totalCount)];
        }

        if (cellStart == null || cellStart.Length < cellCount)
        {
            cellStart = new int[cellCount];
            cellCursor = new int[cellCount];
        }

        if (FluidNeighbors == null || FluidNeighbors.Length != fluidCount)
        {
            FluidNeighbors = new List<int>[Mathf.Max(1, fluidCount)];
            BoundaryNeighbors = new List<int>[Mathf.Max(1, fluidCount)];

            for (int i = 0; i < fluidCount; i++)
            {
                FluidNeighbors[i] = new List<int>();
                BoundaryNeighbors[i] = new List<int>();
            }
        }
    }
}
