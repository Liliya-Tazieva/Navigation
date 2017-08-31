using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace ConsoleApplication2
{
    class StraightLine
    {
        public Destinations Destination;
        public Vector2 Start;
        public Vector2 Finish;
        public List<Vector2> Points;

        public StraightLine(int x, int y, Destinations destination, NodeArray nodesArray)
        {
            Destination = destination;
            Start = new Vector2(x, y);
            if (destination == Destinations.Right)
            {
                Finish = new Vector2(nodesArray.widght-1, y);
            }
            else if (destination == Destinations.Left)
            {
                Finish = new Vector2(0, y);
            }
            else if (destination == Destinations.Down)
            {
                Finish = new Vector2(x, 0);
            }
            else if (destination == Destinations.Up)
            {
                Finish = new Vector2(x, nodesArray.height - 1);
            }
            else if (destination == Destinations.DownRight)
            {
                var delta1 = nodesArray.widght - 1 - x;
                var delta2 = y;
                Finish = delta1 < delta2 ? new Vector2(nodesArray.widght - 1, y - delta1) 
                    : new Vector2(x + delta2, 0);
            }
            else if (destination == Destinations.DownLeft)
            {
                var delta1 = x;
                var delta2 = y;
                Finish = delta1 < delta2 ? new Vector2(0, y - delta1) 
                    : new Vector2(x - delta2, 0);
            }
            else if (destination == Destinations.UpRight)
            {
                var delta1 = nodesArray.widght - 1 - x;
                var delta2 = nodesArray.height - 1 - y;
                Finish = delta1 < delta2 ? new Vector2(nodesArray.widght - 1, y + delta1) 
                    : new Vector2(x + delta2, nodesArray.height - 1);
            }
            else if (destination == Destinations.UpLeft)
            {
                var delta1 = x;
                var delta2 = nodesArray.widght - 1 - y;
                Finish = delta1 < delta2 ? new Vector2(0, y + delta1) : new Vector2(x - delta2, nodesArray.widght - 1);
            }
            Points = FindMiddlePoints(Start, Finish, destination);
        }

        public static bool OnOneLine(Node sNode, Node fNode, NodeArray nodesArray)
        {
            var destination = FindDestination(sNode, fNode);
            var finishPoint = new Vector2(Convert.ToInt32(fNode.Position.X), Convert.ToInt32(fNode.Position.Y));
            var line = new StraightLine(Convert.ToInt32(sNode.Position.X), Convert.ToInt32(sNode.Position.Y), 
                destination, nodesArray);
            if (line.Points.Exists(arg => arg == finishPoint))
            {
                for (var i = 0; i < line.Points.Count; ++i)
                {
                    if (nodesArray.Array[Convert.ToInt32(line.Points[i].X), Convert.ToInt32(line.Points[i].Y)]
                            .IsObstacle) return false;
                    if (line.Points[i] == finishPoint) return true;
                }
            }
            return false;
        }
        
        public static Destinations FindDestination(Node start, Node finish)
        {
            var sPoint = start.Position;
            var fpoint = finish.Position;
            return FindDestination(sPoint, fpoint);
        }

        public static Destinations FindDestination(Vector2 start, Vector2 finish)
        {
            Destinations destination;
            var startX = Convert.ToInt32(start.X);
            var startY = Convert.ToInt32(start.Y);
            var finishX = Convert.ToInt32(finish.X);
            var finishY = Convert.ToInt32(finish.Y);
            if (startX < finishX)
            {
                if (startY < finishY) destination = Destinations.UpRight;
                else if (startY > finishY) destination = Destinations.DownRight;
                else destination = Destinations.Right;
            }
            else if (startX > finishX)
            {
                if (startY < finishY) destination = Destinations.UpLeft;
                else if (startY > finishY) destination = Destinations.DownLeft;
                else destination = Destinations.Left;
            }
            else
            {
                destination = startY < finishY ? Destinations.Up : Destinations.Down;
            }
            return destination;
        }

        public static List<Vector2> FindMiddlePoints(Node s, Node f)
        {
            var d = FindDestination(s, f);
            return FindMiddlePoints(s.Position, f.Position, d);
        }

        public static List<Vector2> FindMiddlePoints(Vector2 s, Vector2 f, Destinations d)
        {
            var points = new List<Vector2>();
            var sX = Convert.ToInt32(s.X);
            var sY = Convert.ToInt32(s.Y);
            var fX = Convert.ToInt32(f.X);
            var fY = Convert.ToInt32(f.Y);
            if (d == Destinations.Right)
            {
                for (var i = sX; i <= fX; ++i)
                    points.Add(new Vector2(i, sY));
            }
            else if (d == Destinations.Left)
            {
                for (var i = sX; i >= fX; --i)
                    points.Add(new Vector2(i, sY));
            }
            else if (d == Destinations.Down)
            {
                for (var i = sY; i >= fY; --i)
                    points.Add(new Vector2(sX, i));
            }
            else if (d == Destinations.Up)
            {
                for (var i = sY; i <= fY; ++i)
                    points.Add(new Vector2(sX, i));
            }
            else if (d == Destinations.DownRight)
            {
                var delta = fX - sX;
                for (var i = 0; i <= delta; ++i)
                    points.Add(new Vector2(sX + i, sY - i));
            }
            else if (d == Destinations.DownLeft)
            {
                var delta = sX - fX;
                for (var i = 0; i <= delta; ++i)
                    points.Add(new Vector2(sX - i, sY - i));
            }
            else if (d == Destinations.UpRight)
            {
                var delta = fX - sX;
                for (var i = 0; i <= delta; ++i)
                    points.Add(new Vector2(sX + i, sY + i));
            }
            else if (d == Destinations.UpLeft)
            {
                var delta = sX - fX;
                for (var i = 0; i <= delta; ++i)
                    points.Add(new Vector2(sX - i, sY + i));
            }
            return points;
        }
    }
}
