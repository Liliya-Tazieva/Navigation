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
            bool belongs = X > line.Start.X && X < line.Finish.X && Y > line.Start.Y && Y < line.Finish.Y;
            return belongs;
        }
    }
}