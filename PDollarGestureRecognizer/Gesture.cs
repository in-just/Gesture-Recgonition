using System;

namespace PDollarGestureRecognizer
{
    /// <summary>
    /// Implements a gesture as a cloud of points (i.e., an unordered set of points).
    /// Gestures are normalized with respect to scale, translated to origin, and resampled into a fixed number of 32 points.
    /// </summary>
    public class Gesture
    {
        public Point[] Points = null;            // gesture points (normalized)
        public string Name = "";                 // gesture class
        private const int SAMPLING_RESOLUTION = 32;

        /// <summary>
        /// Constructs a gesture from an array of points
        /// </summary>
        /// <param name="points"></param>
        public Gesture(Point[] points, string gestureName = "")
        {
            this.Name = gestureName;
            
            // normalizes the array of points with respect to scale, origin, and number of points
            this.Points = Scale(points);
            this.Points = TranslateTo(Points, Centroid(Points));
            this.Points = Resample(Points, SAMPLING_RESOLUTION);
        }

        #region gesture pre-processing steps: scale normalization, translation to origin, and resampling

        /// <summary>
        /// Performs scale normalization with shape preservation into [0..1]x[0..1]
        /// </summary>
        /// <param name="points"></param>
        /// <returns></returns>
        private Point[] Scale(Point[] points)
        {
            float minx = float.MaxValue, miny = float.MaxValue, maxx = float.MinValue, maxy = float.MinValue;
            for (int i = 0; i < points.Length; i++)
            {
                if (minx > points[i].X) minx = points[i].X;
                if (miny > points[i].Y) miny = points[i].Y;
                if (maxx < points[i].X) maxx = points[i].X;
                if (maxy < points[i].Y) maxy = points[i].Y;
            }

            Point[] newPoints = new Point[points.Length];
            float scale = Math.Max(maxx - minx, maxy - miny);
            for (int i = 0; i < points.Length; i++)
                newPoints[i] = new Point((points[i].X - minx) / scale, (points[i].Y - miny) / scale, points[i].StrokeID);
            return newPoints;
        }

        /// <summary>
        /// Translates the array of points by p
        /// </summary>
        /// <param name="points"></param>
        /// <param name="p"></param>
        /// <returns></returns>
        private Point[] TranslateTo(Point[] points, Point p)
        {
            Point[] newPoints = new Point[points.Length];
            for (int i = 0; i < points.Length; i++)
                newPoints[i] = new Point(points[i].X - p.X, points[i].Y - p.Y, points[i].StrokeID);
            return newPoints;
        }

        /// <summary>
        /// Computes the centroid for an array of points
        /// </summary>
        /// <param name="points"></param>
        /// <returns></returns>
        private Point Centroid(Point[] points)
        {
            float cx = 0, cy = 0;
            for (int i = 0; i < points.Length; i++)
            {
                cx += points[i].X;
                cy += points[i].Y;
            }
            return new Point(cx / points.Length, cy / points.Length, 0);
        }

        /// <summary>
        /// Resamples the array of points into n equally-distanced points
        /// </summary>
        /// <param name="points"></param>
        /// <param name="n"></param>
        /// <returns></returns>
        public Point[] Resample(Point[] points, int n)
        {
            Point[] newPoints = new Point[n];
            newPoints[0] = new Point(points[0].X, points[0].Y, points[0].StrokeID);
            int numPoints = 1;

            float I = PathLength(points) / (n - 1); // computes interval length
            float D = 0;
            for (int i = 1; i < points.Length; i++)
            {
                if (points[i].StrokeID == points[i - 1].StrokeID)
                {
                    float d = Geometry.EuclideanDistance(points[i - 1], points[i]);
                    if (D + d >= I)
                    {
                        Point firstPoint = points[i - 1];
                        while (D + d >= I)
                        {
                            // add interpolated point
                            float t = Math.Min(Math.Max((I - D) / d, 0.0f), 1.0f);
                            if (float.IsNaN(t)) t = 0.5f;
                            newPoints[numPoints++] = new Point(
                                (1.0f - t) * firstPoint.X + t * points[i].X,
                                (1.0f - t) * firstPoint.Y + t * points[i].Y,
                                points[i].StrokeID
                            );

                            // update partial length
                            d = D + d - I;
                            D = 0;
                            firstPoint = newPoints[numPoints - 1];
                        }
                        D = d;
                    }
                    else D += d;
                }
            }

            if (numPoints == n - 1) // sometimes we fall a rounding-error short of adding the last point, so add it if so
                newPoints[numPoints++] = new Point(points[points.Length - 1].X, points[points.Length - 1].Y, points[points.Length - 1].StrokeID);
            return newPoints;
        }

        /// <summary>
        /// Computes the path length for an array of points
        /// </summary>
        /// <param name="points"></param>
        /// <returns></returns>
        private float PathLength(Point[] points)
        {
            float length = 0;
            for (int i = 1; i < points.Length; i++)
                if (points[i].StrokeID == points[i - 1].StrokeID)
                    length += Geometry.EuclideanDistance(points[i - 1], points[i]);
            return length;
        }

        #endregion
    }
}