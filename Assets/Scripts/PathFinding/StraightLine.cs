using System.Net.NetworkInformation;

namespace Assets.Scripts.PathFinding
{
    public class StraightLine
    {
        public Destinations Destination;
        public Point Start;
        public Point Finish;

        public StraightLine(int x, int y, Destinations destination)
        {
            Destination = destination;
            Start = new Point(x,y);
            if (Destination == Destinations.Right)
            {
                Finish = new Point(33, y);
            }
            else if (Destination == Destinations.Left)
            {
                Finish = new Point(0, y);
            }
            else if (Destination == Destinations.Up)
            {
                Finish = new Point(x, 34);
            }
            else if (Destination == Destinations.Down)
            {
                Finish = new Point(x, 0);
            }
            else if (Destination == Destinations.UpRight)
            {
                var delta1 = 33 - x;
                var delta2 = 34 - y;
                Finish = delta1 < delta2 ? new Point(33, y+delta1) : new Point(x + delta2, 34);
            }
            else if (Destination == Destinations.UpLeft)
            {
                var delta1 = x;
                var delta2 = 34 - y;
                Finish = delta1 < delta2 ? new Point(0, y+delta1) : new Point(x - delta2, 34);
            }
            else if (Destination == Destinations.DownRight)
            {
                var delta1 = 33 - x;
                var delta2 = y;
                Finish = delta1 < delta2 ? new Point(33, y-delta1) : new Point(x + delta2, 0);
            }
            else if (Destination == Destinations.DownLeft)
            {
                var delta1 = x;
                var delta2 = y;
                Finish = delta1 < delta2 ? new Point(0, y-delta1) : new Point(x - delta2, 0);
            }
        }

        public StraightLine(Point s, Point f)
        {
            Start = s;
            Finish = f;
            Destination = Destinations.Default;
        }

        public bool Belongs(StraightLine line)
        {
            bool belongs = line != null && Start.Belongs(line) && Finish.Belongs(line);
            return belongs;
        }

        public static Point Crossing(Point s1, Point f1, Point s2, Point f2)
        {
            var isCrossing = false;
            var crossPoint = new Point();
            var k1 = (f1.X - s1.X) / (f1.Y - s1.Y);
            var k2 = (f2.X - s2.X) / (f2.Y - s2.Y);
            var line1 = new StraightLine(s1, f1);
            var line2 = new StraightLine(s2,f2);

            if (k1 == k2)
            {
                return line1.Belongs(line2) || line2.Belongs(line1) ? new Point(-100, -100) : null;
            }
            crossPoint.X = ((s1.X * f1.Y - f1.X * s1.Y) * (f2.X - s2.X) - (s2.X * f2.Y - f2.X * s2.Y) * (f1.X - s1.X)) /
                ((s1.Y - f1.Y) * (f2.X - s2.X) - (s2.Y - f2.Y) * (f1.X - s1.X));
            crossPoint.Y = ((s2.Y - f2.Y) * crossPoint.X - (s2.X * f2.Y - f2.X * s2.Y)) / (f2.X - s2.X);

            if (crossPoint.Belongs(line1) && crossPoint.Belongs(line2))
                isCrossing = true;

            return isCrossing ? crossPoint : null;
        }
    }
}