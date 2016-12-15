using System.Net.NetworkInformation;
using UnityEngine;

namespace Assets.Scripts.PathFinding
{
    public class StraightLine
    {
        public Destinations Destination;
        public Point Start;
        public Point Finish;

        public StraightLine(Node node1, Node node2)
        {
            var start = new Point(node1.X(),node1.Y());
            var finish = new Point(node2.X(), node2.Y());
            Start = start;
            Finish = finish;
            Destination = Destinations.Default;
        }

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

        public static Point Crossing(StraightLine line1, StraightLine line2)
        {
            /*Debug.Log("Line1: " + line1.Start.X + " " + line1.Start.Y
                        + " finish " + line1.Finish.X + " " + line1.Finish.Y);
            Debug.Log("Line2: start " + line2.Start.X + " " + line2.Start.Y
                        + " finish " + line2.Finish.X + " " + line2.Finish.Y);*/
            return Crossing(line1.Start.X, line1.Start.Y, line1.Finish.X, line1.Finish.Y,
                line2.Start.X, line2.Start.Y, line2.Finish.X, line2.Finish.Y);
        }

        public static Point Crossing(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
        {
            var isCrossing = false;
            var crossPoint = new Point();
            var line1 = new StraightLine(new Point(x1,y1), new Point(x2,y2));
            var line2 = new StraightLine(new Point(x3, y3), new Point(x4, y4));

            if ((y2-y1)*(x4-x3) == (y4-y3)*(x2-x1))
            {
                return line1.Belongs(line2) || line2.Belongs(line1) ? new Point(-100, -100) : null;
            }
            if (y1-y2 == 0)
            {
                crossPoint.X = ((x1*y2 - x2*y1) + (x4*y3 - x3*y4)*(x2 - x1))/((x2 - x1)*(y3 - y4) + (x4 - x3)*(y2 - y1));
                crossPoint.Y = ((y2 - y1)*crossPoint.X + (x2*y1 - x1*y2))/(x2 - x1);

                if (crossPoint.Belongs(line1) && crossPoint.Belongs(line2))
                    isCrossing = true;
            }
            else
            {
                crossPoint.Y = ((x4*y3 - x3*y4)*(y1 - y2) + (x1*y2 - x2*y1))/((x1 - x2)*(y3 - y4) + (x4 - x3)*(y1 - y2));
                crossPoint.X = ((x1 - x2)*crossPoint.Y + (x2*y1 - x1*y2))/(y1 - y2);

                if (crossPoint.Belongs(line1) && crossPoint.Belongs(line2))
                    isCrossing = true;
            }
            

            return isCrossing ? crossPoint : null;
        }
    }
}