using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;
using Accord.MachineLearning.DecisionTrees.Pruning;
using Assets.Scripts.Core;
using UnityEngine;
using UnityEngine.Networking;

namespace Assets.Scripts.PathFinding
{
    public class StraightLine
    {
        public Destinations Destination;
        public Point Start;
        public Point Finish;
        public List<Point> Points;

        public static bool OnOneLine(Node sNode, Node fNode, Node[,] array)
        {
            var destination = FindDestination(sNode, fNode);
            var line = new StraightLine(sNode.X(),sNode.Y(),destination);
            var obstacleIndex = -1;
            for(var i = 0; i<line.Points.Count; ++i)
            {
                if (array[line.Points[i].X, line.Points[i].Y].InformerNode.IsObstacle)
                {
                    obstacleIndex = i;
                    break;
                }
            }
            if(obstacleIndex!=-1) line.Points.RemoveRange(obstacleIndex,line.Points.Count-obstacleIndex-1);
            return line.Points.Exists(arg => arg.X == fNode.X() && arg.Y == fNode.Y());
        }

        public static List<Point> FindMiddlePoints(Node sNode, Node fNode)
        {
            var s = new Point(sNode.X(),sNode.Y());
            var f = new Point(fNode.X(), fNode.Y());
            var d = FindDestination(s, f);
            return FindMiddlePoints(s, f, d);
        }

        public static Destinations FindDestination(Node start, Node finish)
        {
            var sPoint = new Point(start.X(),start.Y());
            var fpoint = new Point(finish.X(),finish.Y());
            return FindDestination(sPoint,fpoint);
        }

        public static Destinations FindDestination(Point start, Point finish)
        {
            Destinations destination;
            if (start.X < finish.X)
            {
                if (start.Y < finish.Y) destination = Destinations.UpRight;
                else if (start.Y > finish.Y) destination = Destinations.DownRight;
                else destination = Destinations.Right;
            }
            else if (start.X > finish.X)
            {
                if (start.Y < finish.Y) destination = Destinations.UpLeft;
                else if (start.Y > finish.Y) destination = Destinations.DownLeft;
                else destination = Destinations.Left;
            }
            else
            {
                destination = start.Y < finish.Y ? Destinations.Up : Destinations.Down;
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
            var isCrossing = false;

            var crossPoint = new Point();
            foreach (var pointInLine1 in line1.Points)
            {
                if (pointInLine1.Belongs(line2))
                {
                    isCrossing = true;
                    crossPoint = pointInLine1;
                    break;
                }
            }

            return isCrossing ? crossPoint : null;
            /*return Crossing(line1.Start.X, line1.Start.Y, line1.Finish.X, line1.Finish.Y,
                line2.Start.X, line2.Start.Y, line2.Finish.X, line2.Finish.Y);*/
        }

        public static Point Crossing(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
        {
            var isCrossing = false;
            var line1 = new StraightLine(new Point(x1,y1), new Point(x2,y2));
            var line2 = new StraightLine(new Point(x3, y3), new Point(x4, y4));

            var crossPoint = new Point();
            
            var a1 = y1 - y2;
            var b1 = x2 - x1;
            var c1 = x1*y2 - x2*y1;
            var a2 = y3 - y4;
            var b2 = x4 - x3;
            var c2 = x3 * y4 - x4 * y3;
            if (a1*b2 - a2*b1 != 0)
            {
                crossPoint.X = -(c1*b2 - c2*b1)/(a1*b2 - a2*b1);
                crossPoint.Y = -(a1*c2 - a2*c1)/(a1*b2 - a2*b1);

                if (crossPoint.Belongs(line1) && crossPoint.Belongs(line2))
                    isCrossing = true;
            }
            else
            {
                return line1.Belongs(line2) || line2.Belongs(line1) ? new Point(-100, -100) : null;
            }

            return isCrossing ? crossPoint : null;
        }

        public void ReduceLine(Node[,] nodesArray)
        {
            foreach (var point in Points)
            {
                if (nodesArray[point.X, point.Y].InformerNode.IsObstacle)
                {
                    Finish = point;
                    Points = FindMiddlePoints(Start, Finish, Destination);
                    break;
                }
            }
        }
    }
}