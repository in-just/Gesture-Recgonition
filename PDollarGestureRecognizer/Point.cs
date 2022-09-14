using System;

namespace PDollarGestureRecognizer
{
    /// <summary>
    /// Implements a 2D Point that exposes X, Y, and StrokeID properties.
    /// StrokeID is the stroke index the point belongs to (e.g., 0, 1, 2, ...) that is filled by counting pen down/up events.
    /// </summary>
    public class Point
    {
        public float X, Y;
        public int StrokeID;      

        public Point(float x, float y, int strokeId)
        {
            this.X = x;
            this.Y = y;
            this.StrokeID = strokeId;
        }
    }
}
