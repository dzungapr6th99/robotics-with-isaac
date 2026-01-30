using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CuOptClientService.Common
{
    internal static class CuOptBuild
    {
        /// <summary>
        /// Convert a distance matrix (meters) to a travel time matrix (seconds).
        /// With speed = 1 m/s, time == distance.
        /// </summary>
        public static List<List<double>> BuildTimeMatrixFromDistance(
            List<List<double>> distanceMeters,
            double speedMetersPerSecond = 1.0)
        {
            if (distanceMeters == null)
                throw new ArgumentNullException(nameof(distanceMeters));

            if (speedMetersPerSecond <= 0)
                throw new ArgumentOutOfRangeException(nameof(speedMetersPerSecond), "Speed must be > 0.");

            int n = distanceMeters.Count;
            if (n == 0)
                return new List<List<double>>();

            // Validate square matrix
            for (int i = 0; i < n; i++)
            {
                if (distanceMeters[i] == null)
                    throw new ArgumentException($"Row {i} is null.", nameof(distanceMeters));

                if (distanceMeters[i].Count != n)
                    throw new ArgumentException(
                        $"Distance matrix must be NxN. Row {i} has {distanceMeters[i].Count}, expected {n}.",
                        nameof(distanceMeters));
            }

            var timeSeconds = new List<List<double>>(n);

            for (int i = 0; i < n; i++)
            {
                var row = new List<double>(n);
                for (int j = 0; j < n; j++)
                {
                    double d = distanceMeters[i][j];

                    if (double.IsNaN(d) || double.IsInfinity(d))
                        throw new ArgumentException($"Distance at [{i},{j}] is invalid: {d}", nameof(distanceMeters));

                    if (d < 0)
                        throw new ArgumentException($"Distance at [{i},{j}] is negative: {d}", nameof(distanceMeters));

                    row.Add(d / speedMetersPerSecond); // seconds
                }
                timeSeconds.Add(row);
            }

            return timeSeconds;
        }
    }
}
