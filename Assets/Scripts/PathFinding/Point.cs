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
            var belongs = false;
            foreach (var point in line.Points)
            {
                if (X == point.X && Y == point.Y) belongs = true;
            }
            return belongs;
        }
    }
}