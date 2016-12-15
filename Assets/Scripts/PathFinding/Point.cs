using UnityEngine;

namespace Assets.Scripts.PathFinding
{
    public class Point
    {
        public int X;
        public int Y;

        public Point()
        {
            X = 0;
            Y = 0;
        }
        
        public Point(int x, int y)
        {
            X = x;
            Y = y;
        }

        public bool Belongs(StraightLine line)
        {
            var maxX = line.Finish.X;
            var minX = line.Start.X;
            if (line.Start.X > line.Finish.X)
            {
                maxX = line.Start.X;
                minX = line.Finish.X;

            }
            var maxY = line.Finish.Y;
            var minY = line.Start.Y;
            if (line.Start.Y > line.Finish.Y)
            {
                maxY = line.Start.Y;
                minY = line.Finish.Y;

            }
            bool belongs = X >= minX && X <= maxX && Y >= minY && Y <= maxY;
            return belongs;
        }
    }
}