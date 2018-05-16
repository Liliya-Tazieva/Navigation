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
            var idx = line.Points.FindIndex(arg => arg.X == X && arg.Y == Y);
            return idx != -1 && line.Points[idx].X != line.Start.X && line.Points[idx].Y != line.Start.Y;
        }
    }
}