using System;

namespace PDollarGestureRecognizer
{
    public class Geometry
    {
        /// <summary>
        /// Computes the Squared Euclidean Distance between two points in 2D
        /// </summary>
        public static float SqrEuclideanDistance(Point a, Point b)
        {
            return (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y);
        }

        /// <summary>
        /// Computes the Euclidean Distance between two points in 2D
        /// </summary>
        public static float EuclideanDistance(Point a, Point b)
        {
            return (float)Math.Sqrt(SqrEuclideanDistance(a, b));
        }
    }
}
