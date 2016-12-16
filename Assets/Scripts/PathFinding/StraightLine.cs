using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;
using Assets.Scripts.Core;
using UnityEngine;

namespace Assets.Scripts.PathFinding
{
    public class StraightLine
    {
        public Destinations Destination;
        public Point Start;
        public Point Finish;
        public List<Point> Points;

        public static List<Point> FindMiddlePoints(Node sNode, Node fNode)
        {
            var s = new Point(sNode.X(),sNode.Y());
            var f = new Point(fNode.X(), fNode.Y());
            var d = Extensions.DestinationInverse(fNode.DestinationFromPrevious);
            return FindMiddlePoints(s, f, d);
        }

        public static Destinations FindDestination(Point start, Point finish)
        {
            var destination = Destinations.Default;
            if (start.X > finish.Y)
            {
                if (start.Y > finish.Y) destination = Destinations.UpRight;
                else if (start.Y < finish.Y) destination = Destinations.DownRight;
                else destination = Destinations.Right;
            }
            else if (start.X < finish.X)
            {
                if (start.Y > finish.Y) destination = Destinations.UpLeft;
                else if (start.Y < finish.Y) destination = Destinations.DownLeft;
                else destination = Destinations.Left;
            }
            else
            {
                destination = start.Y > finish.Y ? Destinations.Up : Destinations.Down;
            }
            return destination;
        }

        public static List<Point> FindMiddlePoints(Point s, Point f, Destinations d)
        {
            var points = new List<Point>();
            if (d == Destinations.Right)
            {
                for(var i = s.X;i<=f.X;++i)
                    points.Add(new Point(i,s.Y));
            }
            else if (d == Destinations.Left)
            {
                for (var i = s.X; i >= f.X; --i)
                    points.Add(new Point(i, s.Y));
            }
            else if (d == Destinations.Up)
            {
                for (var i = s.Y; i <= f.Y; ++i)
                    points.Add(new Point(s.X, i));
            }
            else if (d == Destinations.Down)
            {
                for (var i = s.Y; i >= f.Y; --i)
                    points.Add(new Point(s.X, i));
            }
            else if (d == Destinations.UpRight)
            {
                var delta = f.X - s.X;
                for (var i = 0; i <= delta; ++i)
                    points.Add(new Point(s.X + i, s.Y + i));
            }
            else if (d == Destinations.UpLeft)
            {
                var delta = f.Y - s.Y;
                for (var i = 0; i <= delta; ++i)
                    points.Add(new Point(s.X - i, s.Y + i));
            }
            else if (d == Destinations.DownRight)
            {
                var delta = f.X - s.X;
                for (var i = 0; i <= delta; ++i)
                    points.Add(new Point(s.X + i, s.Y - i));
            }
            else if (d == Destinations.DownLeft)
            {
                var delta = s.X - f.X;
                for (var i = 0; i <= delta; ++i)
                    points.Add(new Point(s.X - i, s.Y - i));
            }
            return points;
        }
        
        public StraightLine(Node node1, Node node2)
        {
            var start = new Point(node1.X(),node1.Y());
            var finish = new Point(node2.X(), node2.Y());
            Start = start;
            Finish = finish;
            Destination = FindDestination(start, finish);
            Points = FindMiddlePoints(start, finish, Destination);
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
            Points = FindMiddlePoints(Start, Finish, Destination);
        }

        public StraightLine(Point s, Point f)
        {
            Start = s;
            Finish = f;
            Destination = FindDestination(s, f);
            Points = FindMiddlePoints(Start, Finish, Destination);
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
            var line1 = new StraightLine(new Point(x1,y1), new Point(x2,y2));
            var line2 = new StraightLine(new Point(x3, y3), new Point(x4, y4));
            var crossPoint = line1.Points.Intersect(line2.Points).ToList();

            /*var a1 = y1 - y2;
            var b1 = x2 - x1;
            var c1 = x1*y2 - x2*y1;
            var a2 = y3 - y4;
            var b2 = x4 - x3;
            var c2 = x3 * y4 - x4 * y3;
            if (a1*b2 - a2*b1 == 0)
            {
                return line1.Belongs(line2) || line2.Belongs(line1) ? new Point(-100, -100) : null;
            }
            else
            {
                crossPoint.X = -(c1*b2 - c2*b1)/(a1*b2 - a2*b1);
                crossPoint.Y = -(a1*c2 - a2*c1)/(a1*b2 - a2*b1);

                if (crossPoint.Belongs(line1) && crossPoint.Belongs(line2))
                    isCrossing = true;
            }
            Debug.Log("Cross point = " + crossPoint.X + " " + crossPoint.Y + " " + isCrossing);
            return isCrossing ? crossPoint : null;*/
            return crossPoint.Count != 0 ? crossPoint[0] : null;
        }
    }
}